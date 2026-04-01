#include <rclcpp/rclcpp.hpp>
#include <autoware_adapi_v1_msgs/srv/set_route_points.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

class InfiniteLoopNode : public rclcpp::Node
{
public:
    InfiniteLoopNode() : Node("infinite_loop_route_only")
    {
        // ================== 参数读取 ==================
        this->declare_parameter("trigger_distance", 50.0);
        this->declare_parameter("waypoints_raw", std::vector<double>{});

        trigger_distance_ = this->get_parameter("trigger_distance").as_double();
        std::vector<double> raw_points = this->get_parameter("waypoints_raw").as_double_array();

        // 解析扁平数组为 Pose 对象
        if (raw_points.size() % 7 != 0 || raw_points.empty()) {
            RCLCPP_ERROR(this->get_logger(), "参数错误：waypoints_raw 的长度必须是 7 的倍数！当前长度: %zu", raw_points.size());
            rclcpp::shutdown();
            return;
        }

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
        RCLCPP_INFO(this->get_logger(), "已加载 %zu 个目标点", num_points_);

        // ================== ROS 接口 ==================
        // 使用 Reentrant Callback Group 允许并发，避免在等待服务时死锁
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        client_set_ = this->create_client<autoware_adapi_v1_msgs::srv::SetRoutePoints>(
            "/api/routing/set_route_points", rmw_qos_profile_services_default, callback_group_);
        
        client_change_ = this->create_client<autoware_adapi_v1_msgs::srv::SetRoutePoints>(
            "/api/routing/change_route_points", rmw_qos_profile_services_default, callback_group_);

        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = callback_group_;
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/localization/kinematic_state", 10,
            std::bind(&InfiniteLoopNode::odom_callback, this, std::placeholders::_1),
            sub_opts);

        // 初始状态
        current_pose_received_ = false;
        current_goal_idx_ = num_points_ - 1; // 初始目标设为最后一个点
        last_processed_goal_idx_ = -1;

        RCLCPP_INFO(this->get_logger(), ">>> 纯轨迹规划模式启动 (Trigger: %.1fm)", trigger_distance_);

        // 启动主逻辑线程
        main_thread_ = std::thread(&InfiniteLoopNode::main_loop, this);
    }

    ~InfiniteLoopNode()
    {
        if (main_thread_.joinable()) main_thread_.join();
    }

private:
    // 成员变量
    double trigger_distance_;
    std::vector<geometry_msgs::msg::Pose> points_;
    size_t num_points_;
    
    geometry_msgs::msg::Pose current_pose_;
    bool current_pose_received_;
    
    int current_goal_idx_;
    int last_processed_goal_idx_;

    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Client<autoware_adapi_v1_msgs::srv::SetRoutePoints>::SharedPtr client_set_;
    rclcpp::Client<autoware_adapi_v1_msgs::srv::SetRoutePoints>::SharedPtr client_change_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    std::thread main_thread_;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
        current_pose_received_ = true;
    }

    double calc_dist(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2)
    {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    }

    // 封装发送请求逻辑
    bool send_route(const std::vector<geometry_msgs::msg::Pose> &wps, const geometry_msgs::msg::Pose &goal, bool is_initial)
    {
        
        auto req = std::make_shared<autoware_adapi_v1_msgs::srv::SetRoutePoints::Request>();
        req->header.frame_id = "map";
        req->option.allow_goal_modification = true;
        req->waypoints = wps;
        req->goal = goal;

        auto client = is_initial ? client_set_ : client_change_;
        auto result_future = client->async_send_request(req);

        // 等待结果
        if (result_future.wait_for(5s) == std::future_status::ready) {
            auto result = result_future.get();
            return result->status.success;
        } else {
            RCLCPP_ERROR(this->get_logger(), "服务调用超时");
            return false;
        }
    }

    void main_loop()
    {
        // 等待服务
        while (rclcpp::ok() && !client_set_->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "等待 Routing 服务...");
        }
        std::this_thread::sleep_for(1s);

        // ================= 初始路径 =================
        std::vector<geometry_msgs::msg::Pose> init_wps;
        for (size_t i = 0; i < num_points_ - 1; ++i) {
            init_wps.push_back(points_[i]);
        }
        geometry_msgs::msg::Pose init_goal = points_[num_points_ - 1];

        if (send_route(init_wps, init_goal, true)) {
            RCLCPP_INFO(this->get_logger(), "初始路径已发送 (Goal: P%d)", current_goal_idx_);
            RCLCPP_INFO(this->get_logger(), "请手动驾驶车辆，或者等待车辆移动...");
            last_processed_goal_idx_ = -1;
            std::this_thread::sleep_for(1s);
        }

        // ================= 循环监控 =================
        rclcpp::Rate rate(10); // 10Hz
        while (rclcpp::ok()) {
            if (!current_pose_received_) {
                rate.sleep();
                continue;
            }

            // 1. 获取关键点索引
            int goal_idx = current_goal_idx_;
            // 处理负数取模: (idx - 1 + N) % N
            int trigger_idx = (current_goal_idx_ - 1 + num_points_) % num_points_;

            // 2. 计算距离
            double dist_trigger = calc_dist(current_pose_.position, points_[trigger_idx].position);
            double dist_goal = calc_dist(current_pose_.position, points_[goal_idx].position);

            // 3. 判断触发
            bool is_near_trigger = (dist_trigger < trigger_distance_);
            bool is_near_goal = (dist_goal < trigger_distance_);

            if ((is_near_trigger || is_near_goal) && (last_processed_goal_idx_ != current_goal_idx_)) {
                
                std::string source = is_near_trigger ? "P" + std::to_string(trigger_idx) + " (Pre)" 
                                                     : "P" + std::to_string(goal_idx) + " (Final)";
                RCLCPP_WARN(this->get_logger(), "\n⚡ 触发切换! 触发源: %s", source.c_str());

                // 计算下一段路径 (全量循环)
                int next_goal_idx = (current_goal_idx_ + 1) % num_points_;
                std::vector<geometry_msgs::msg::Pose> next_wps_list;
                
                // 循环 num_points 次
                for (size_t i = 1; i <= num_points_; ++i) {
                    int idx = (next_goal_idx + i) % num_points_;
                    next_wps_list.push_back(points_[idx]);
                }

                geometry_msgs::msg::Pose next_goal = next_wps_list.back();
                next_wps_list.pop_back(); // 移除最后一个作为 goal，剩下的作为 waypoints

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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InfiniteLoopNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}