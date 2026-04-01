#include <rclcpp/rclcpp.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <autoware_adapi_v1_msgs/srv/clear_route.hpp>
#include <autoware_adapi_v1_msgs/srv/set_route_points.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp> 
#include <nav_msgs/msg/odometry.hpp>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>
#include <mutex>
#include <string> // 为了支持 std::string 和 to_string
#include <atomic>

using namespace std::chrono_literals;

class InfiniteLoopNode : public rclcpp::Node
{
public:
    // 修改：将节点名统一为 infinite_loop_node
    InfiniteLoopNode() : Node("infinite_loop_node")
    {
        // 如果没有这一行，ros2 param get 就会报错 Parameter not set
        this->declare_parameter("use_external_topic", false); 
        this->declare_parameter("trigger_distance", 50.0);
        this->declare_parameter("waypoints_raw", std::vector<double>{});

        // ================== 2. 参数获取 ==================
        use_external_topic_ = this->get_parameter("use_external_topic").as_bool();
        trigger_distance_ = this->get_parameter("trigger_distance").as_double();
        
        RCLCPP_INFO(this->get_logger(), "--------------------------------");
        RCLCPP_INFO(this->get_logger(), "模式检查: use_external_topic = %s", use_external_topic_ ? "TRUE (外部话题)" : "FALSE (内部循环)");
        RCLCPP_INFO(this->get_logger(), "--------------------------------");

        // 解析 waypoints (如果不是外部模式才严格检查)
        std::vector<double> raw_points = this->get_parameter("waypoints_raw").as_double_array();
        if (!raw_points.empty() && raw_points.size() % 7 == 0) {
            for (size_t i = 0; i < raw_points.size(); i += 7) {
                geometry_msgs::msg::Pose p;
                p.position.x = raw_points[i];
                p.position.y = raw_points[i+1];
                p.position.z = raw_points[i+2];
                p.orientation.x = raw_points[i+3];
                p.orientation.y = raw_points[i+4];
                p.orientation.z = raw_points[i+5];
                p.orientation.w = raw_points[i+6];
                points_.push_back(p);
            }
            num_points_ = points_.size();
        }

        // ================== ROS 接口 ==================
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        client_set_ = this->create_client<autoware_adapi_v1_msgs::srv::SetRoutePoints>(
            "/api/routing/set_route_points", rmw_qos_profile_services_default, callback_group_);
        
        client_change_ = this->create_client<autoware_adapi_v1_msgs::srv::SetRoutePoints>(
            "/api/routing/change_route_points", rmw_qos_profile_services_default, callback_group_);
        
        client_clear_ = this->create_client<autoware_adapi_v1_msgs::srv::ClearRoute>(
            "/api/routing/clear_route", rmw_qos_profile_services_default, callback_group_);

        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = callback_group_;
        
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/localization/kinematic_state", 10,
            std::bind(&InfiniteLoopNode::odom_callback, this, std::placeholders::_1),
            sub_opts);

        const auto route_state_qos = rclcpp::QoS(1).reliable().transient_local();
        sub_route_state_ = this->create_subscription<autoware_adapi_v1_msgs::msg::RouteState>(
            "/api/routing/state", route_state_qos,
            std::bind(&InfiniteLoopNode::route_state_callback, this, std::placeholders::_1),
            sub_opts);

        if (use_external_topic_) {
            sub_external_route_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
                "/external/route_cmd", 10,
                std::bind(&InfiniteLoopNode::external_route_callback, this, std::placeholders::_1),
                sub_opts);
            RCLCPP_INFO(this->get_logger(), ">>> 外部指令模式启动: 等待 /external/route_cmd 话题...");
        } else {
            RCLCPP_INFO(this->get_logger(), ">>> 内部循环模式启动 (Trigger: %.1fm)", trigger_distance_);
        }

        current_pose_received_ = false;
        current_goal_idx_ = num_points_ > 0 ? num_points_ - 1 : 0;
        last_processed_goal_idx_ = -1;
        has_new_external_cmd_ = false;

        main_thread_ = std::thread(&InfiniteLoopNode::main_loop, this);
    }

    ~InfiniteLoopNode()
    {
        if (main_thread_.joinable()) main_thread_.join();
    }

private:
    bool use_external_topic_; 
    double trigger_distance_;
    std::vector<geometry_msgs::msg::Pose> points_;
    size_t num_points_ = 0;
    geometry_msgs::msg::Pose current_pose_;
    bool current_pose_received_;
    int current_goal_idx_;
    int last_processed_goal_idx_;
    bool has_new_external_cmd_;
    geometry_msgs::msg::PoseArray latest_external_cmd_;
    std::mutex external_cmd_mutex_;
    std::atomic<bool> route_state_received_{false};
    std::atomic<uint16_t> route_state_{autoware_adapi_v1_msgs::msg::RouteState::UNKNOWN};

    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Client<autoware_adapi_v1_msgs::srv::SetRoutePoints>::SharedPtr client_set_;
    rclcpp::Client<autoware_adapi_v1_msgs::srv::SetRoutePoints>::SharedPtr client_change_;
    rclcpp::Client<autoware_adapi_v1_msgs::srv::ClearRoute>::SharedPtr client_clear_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<autoware_adapi_v1_msgs::msg::RouteState>::SharedPtr sub_route_state_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_external_route_;
    std::thread main_thread_;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_ = msg->pose.pose;
        current_pose_received_ = true;
    }

    void external_route_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(external_cmd_mutex_);
        latest_external_cmd_ = *msg;
        has_new_external_cmd_ = true;
        RCLCPP_INFO(this->get_logger(), "收到外部路径指令，包含 %zu 个点", msg->poses.size());
    }

    void route_state_callback(const autoware_adapi_v1_msgs::msg::RouteState::SharedPtr msg) {
        route_state_.store(msg->state);
        route_state_received_.store(true);
    }

    double calc_dist(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2) {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    }

    bool send_route(const std::vector<geometry_msgs::msg::Pose> &wps, const geometry_msgs::msg::Pose &goal, bool is_initial) {
        auto req = std::make_shared<autoware_adapi_v1_msgs::srv::SetRoutePoints::Request>();
        req->header.frame_id = "map";
        req->option.allow_goal_modification = true;
        req->waypoints = wps;
        req->goal = goal;

        auto client = is_initial ? client_set_ : client_change_;
        if (!client->service_is_ready()) {
             RCLCPP_WARN(this->get_logger(), "服务未就绪，跳过发送");
             return false;
        }
        auto result_future = client->async_send_request(req);
        if (result_future.wait_for(2s) == std::future_status::ready) {
            auto result = result_future.get();
            return result->status.success;
        } else {
            RCLCPP_ERROR(this->get_logger(), "服务调用超时");
            return false;
        }
    }

    bool clear_route() {
        auto req = std::make_shared<autoware_adapi_v1_msgs::srv::ClearRoute::Request>();
        if (!client_clear_->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "clear_route 服务未就绪，跳过调用");
            return false;
        }

        auto result_future = client_clear_->async_send_request(req);
        if (result_future.wait_for(2s) == std::future_status::ready) {
            auto result = result_future.get();
            if (!result->status.success) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "clear_route 失败: code=%u, msg=%s",
                    result->status.code, result->status.message.c_str());
            }
            return result->status.success;
        }

        RCLCPP_ERROR(this->get_logger(), "clear_route 服务调用超时");
        return false;
    }

    void main_loop() {
        while (rclcpp::ok() && !client_set_->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "等待 Routing 服务...");
        }
        while (rclcpp::ok() && !client_change_->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "等待 change_route_points 服务...");
        }
        while (rclcpp::ok() && !client_clear_->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "等待 clear_route 服务...");
        }

        if (use_external_topic_) {
            run_external_mode_logic();
        } else {
            run_internal_loop_logic();
        }
    }

    void run_external_mode_logic() {
        rclcpp::Rate rate(5);
        while (rclcpp::ok()) {
            bool has_cmd = false;
            geometry_msgs::msg::PoseArray current_cmd;
            {
                std::lock_guard<std::mutex> lock(external_cmd_mutex_);
                if (has_new_external_cmd_) {
                    current_cmd = latest_external_cmd_;
                    has_cmd = true;
                    has_new_external_cmd_ = false;
                }
            }

            if (has_cmd) {
                if (current_cmd.poses.empty()) continue;
                geometry_msgs::msg::Pose goal_pose = current_cmd.poses.back();
                std::vector<geometry_msgs::msg::Pose> waypoints;
                if (current_cmd.poses.size() > 1) {
                    for (size_t i = 0; i < current_cmd.poses.size() - 1; ++i) {
                        waypoints.push_back(current_cmd.poses[i]);
                    }
                }

                RCLCPP_INFO(this->get_logger(), "开始下发外部路径...");
                bool success = false;
                while (rclcpp::ok() && !success) {
                    const auto state = route_state_.load();
                    const bool state_ready = route_state_received_.load();

                    if (!state_ready || state == autoware_adapi_v1_msgs::msg::RouteState::UNSET) {
                        success = send_route(waypoints, goal_pose, true);
                    } else if (state == autoware_adapi_v1_msgs::msg::RouteState::SET) {
                        success = send_route(waypoints, goal_pose, false);
                    } else if (state == autoware_adapi_v1_msgs::msg::RouteState::ARRIVED) {
                        if (clear_route()) {
                            success = send_route(waypoints, goal_pose, true);
                        } else {
                            success = false;
                        }
                    } else {
                        RCLCPP_WARN(
                            this->get_logger(),
                            "当前 routing state=%u，暂不下发路径，1秒后重试",
                            state);
                        success = false;
                    }

                    if (success) RCLCPP_INFO(this->get_logger(), "外部路径规划成功！");
                    else {
                        RCLCPP_WARN(this->get_logger(), "外部路径规划被拒绝，1秒后重试..."); //还有bug，要加判断是否路径是空的
                        std::this_thread::sleep_for(1s);
                    }
                }
            }
            rate.sleep();
        }
    }

    void run_internal_loop_logic() {
        if (num_points_ < 2) return;
        std::vector<geometry_msgs::msg::Pose> init_wps;
        for (size_t i = 0; i < num_points_ - 1; ++i) init_wps.push_back(points_[i]);
        geometry_msgs::msg::Pose init_goal = points_[num_points_ - 1];

        if (send_route(init_wps, init_goal, true)) {
            RCLCPP_INFO(this->get_logger(), "初始路径已发送 (Goal: P%d)", current_goal_idx_);
            RCLCPP_INFO(this->get_logger(), "请手动驾驶车辆，或者等待车辆移动...");
            std::this_thread::sleep_for(1s);
        }

        rclcpp::Rate rate(10);
        while (rclcpp::ok()) {
            if (!current_pose_received_) { rate.sleep(); continue; }
            int goal_idx = current_goal_idx_;
            int trigger_idx = (current_goal_idx_ - 1 + num_points_) % num_points_;
            double dist_trigger = calc_dist(current_pose_.position, points_[trigger_idx].position);
            double dist_goal = calc_dist(current_pose_.position, points_[goal_idx].position);

            // 判断逻辑拆分，以便打印 Trigger 源
            bool is_near_trigger = (dist_trigger < trigger_distance_);
            bool is_near_goal = (dist_goal < trigger_distance_);

            if ((is_near_trigger || is_near_goal) && (last_processed_goal_idx_ != current_goal_idx_)) {
                
                std::string source = is_near_trigger ? "P" + std::to_string(trigger_idx) + " (Pre)" 
                                                     : "P" + std::to_string(goal_idx) + " (Final)";
                RCLCPP_WARN(this->get_logger(), "\n⚡ 触发切换! 触发源: %s", source.c_str());

                int next_goal_idx = (current_goal_idx_ + 1) % num_points_;
                std::vector<geometry_msgs::msg::Pose> next_wps_list;
                for (size_t i = 1; i <= num_points_; ++i) next_wps_list.push_back(points_[(next_goal_idx + i) % num_points_]);
                geometry_msgs::msg::Pose next_goal = next_wps_list.back();
                next_wps_list.pop_back();

                if (send_route(next_wps_list, next_goal, false)) {
                    RCLCPP_INFO(this->get_logger(), "成功延展路径 -> 新终点 P%d", next_goal_idx);
                    last_processed_goal_idx_ = current_goal_idx_;
                    current_goal_idx_ = next_goal_idx;
                    std::this_thread::sleep_for(500ms);
                } else {
                    RCLCPP_ERROR(this->get_logger(), " 切换失败");
                }
            }
            rate.sleep();
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InfiniteLoopNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
