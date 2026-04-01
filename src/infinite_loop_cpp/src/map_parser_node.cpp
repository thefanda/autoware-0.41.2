#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <dfcv_mining_msgs/msg/tbox_can_fd_msg.hpp>

// Autoware Map & Lanelet2
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>

//  引入 Autoware 扩展头文件
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>

using namespace std::chrono_literals;

class MapParserNode : public rclcpp::Node
{
public:
    MapParserNode() : Node("map_parser_node")
    {
        // 1. 订阅 Autoware 地图
        sub_map_ = this->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
            "/map/vector_map", rclcpp::QoS(1).transient_local(),
            std::bind(&MapParserNode::map_callback, this, std::placeholders::_1));

        // 2. 订阅 TBox 路径请求
        sub_tbox_route_ = this->create_subscription<dfcv_mining_msgs::msg::TboxCanFdMsg>(
            "/external/tbox_can_fd_msg", 10,
            std::bind(&MapParserNode::tbox_route_callback, this, std::placeholders::_1));

        // 3. 发布结果
        pub_poses_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "/external/route_cmd", 10);

        RCLCPP_INFO(this->get_logger(), " 等待 Autoware 地图话题 (/map/vector_map)...");
    }

private:
    lanelet::LaneletMapPtr lanelet_map_;
    bool map_loaded_ = false;

    rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr sub_map_;
    rclcpp::Subscription<dfcv_mining_msgs::msg::TboxCanFdMsg>::SharedPtr sub_tbox_route_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_poses_;

    void map_callback(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "接收到地图数据，正在解码...");
        lanelet_map_ = std::make_shared<lanelet::LaneletMap>();
        
        //  修复点：命名空间改为 lanelet::utils::conversion
        lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map_);
        
        map_loaded_ = true;
        RCLCPP_INFO(this->get_logger(), "地图解码成功！包含 %zu 个 Lanelet。", lanelet_map_->laneletLayer.size());
    }

    bool build_pose_from_lane_id(const lanelet::Id lane_id, geometry_msgs::msg::Pose & pose) const
    {
        if (!lanelet_map_->laneletLayer.exists(lane_id)) {
            RCLCPP_WARN(this->get_logger(), "地图中不存在 ID: %lld", (long long)lane_id);
            return false;
        }

        auto lanelet = lanelet_map_->laneletLayer.get(lane_id);
        const auto & centerline = lanelet.centerline();
        if (centerline.empty()) {
            return false;
        }

        const double length = lanelet::geometry::length(centerline);
        const double target_dist = length * 0.75;
        const auto target_point = lanelet::geometry::interpolatedPointAtDistance(centerline, target_dist);
        const double next_dist = std::min(target_dist + 1.0, length);
        const auto next_point = lanelet::geometry::interpolatedPointAtDistance(centerline, next_dist);

        const double dx = next_point.x() - target_point.x();
        const double dy = next_point.y() - target_point.y();
        const double yaw = std::atan2(dy, dx);

        pose.position.x = target_point.x();
        pose.position.y = target_point.y();
        pose.position.z = target_point.z();
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = sin(yaw / 2.0);
        pose.orientation.w = cos(yaw / 2.0);
        return true;
    }

    bool compute_goal_yaw_from_lane_id(const lanelet::Id lane_id, double & yaw, double & z) const
    {
        if (!lanelet_map_->laneletLayer.exists(lane_id)) {
            RCLCPP_WARN(this->get_logger(), "终点 ID 不存在: %lld", (long long)lane_id);
            return false;
        }

        const auto lanelet = lanelet_map_->laneletLayer.get(lane_id);
        const auto & centerline = lanelet.centerline();
        if (centerline.empty()) {
            return false;
        }

        const double length = lanelet::geometry::length(centerline);
        const double tail_dist = std::max(0.0, length - 1.0);
        const auto prev_point = lanelet::geometry::interpolatedPointAtDistance(centerline, tail_dist);
        const auto end_point = lanelet::geometry::interpolatedPointAtDistance(centerline, length);
        yaw = std::atan2(end_point.y() - prev_point.y(), end_point.x() - prev_point.x());
        z = end_point.z();
        return true;
    }

    void tbox_route_callback(const dfcv_mining_msgs::msg::TboxCanFdMsg::ConstSharedPtr msg)
    {
        if (!map_loaded_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                " 尚未收到地图，无法解析 ID！请确保 map_loader 正在运行。");
            return;
        }

        if (msg->move_id_pass_path_id.empty()) {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "⚡ 开始解析 TBox 路径，ID 数量: %zu", msg->move_id_pass_path_id.size());

        geometry_msgs::msg::PoseArray output_msg;
        output_msg.header.stamp = this->now();
        output_msg.header.frame_id = "map";

        const size_t id_count = msg->move_id_pass_path_id.size();
        for (size_t i = 0; i + 1 < id_count; ++i) {
            const auto lane_id = static_cast<lanelet::Id>(msg->move_id_pass_path_id[i]);
            try {
                geometry_msgs::msg::Pose via_pose;
                if (build_pose_from_lane_id(lane_id, via_pose)) {
                    output_msg.poses.push_back(via_pose);
                }
            } catch (const std::exception & e) {
                RCLCPP_ERROR(this->get_logger(), "解析途经 ID %lld 时发生错误: %s", (long long)lane_id, e.what());
            }
        }

        const auto goal_lane_id = static_cast<lanelet::Id>(msg->move_id_pass_path_id.back());
        double goal_yaw = 0.0;
        double goal_z = msg->move_end_z;
        if (!compute_goal_yaw_from_lane_id(goal_lane_id, goal_yaw, goal_z)) {
            RCLCPP_WARN(this->get_logger(), "终点姿态将使用默认朝向（yaw=0），终点 ID: %lld", (long long)goal_lane_id);
        }

        geometry_msgs::msg::Pose goal_pose;
        goal_pose.position.x = msg->move_end_x;
        goal_pose.position.y = msg->move_end_y;
        goal_pose.position.z = goal_z;
        goal_pose.orientation.x = 0.0;
        goal_pose.orientation.y = 0.0;
        goal_pose.orientation.z = sin(goal_yaw / 2.0);
        goal_pose.orientation.w = cos(goal_yaw / 2.0);
        output_msg.poses.push_back(goal_pose);

        if (!output_msg.poses.empty()) {
            pub_poses_->publish(output_msg);
            RCLCPP_INFO(this->get_logger(), " 已发布 %zu 个坐标点（含终点）。", output_msg.poses.size());
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapParserNode>());
    rclcpp::shutdown();
    return 0;
}
