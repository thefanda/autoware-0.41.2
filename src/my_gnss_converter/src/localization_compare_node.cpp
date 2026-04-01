#include <rclcpp/rclcpp.hpp>
#include <dfcv_mining_msgs/msg/location_imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <cmath>

class LocalizationCompareNode : public rclcpp::Node
{
public:
    LocalizationCompareNode() 
    : Node("localization_compare_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        RCLCPP_INFO(this->get_logger(), "Localization compare node started");

        // QoS 用 SensorDataQoS，兼容 Autoware
        auto qos = rclcpp::SensorDataQoS();

        imu_sub_ = this->create_subscription<dfcv_mining_msgs::msg::LocationIMU>(
            "/location_imu_info", qos,
            [this](dfcv_mining_msgs::msg::LocationIMU::ConstSharedPtr msg){
                last_imu_ = msg;
                try_compare();
            });

        kin_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/localization/kinematic_state", qos,
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg){
                last_kin_ = msg;
            });

        ndt_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/localization/pose_estimator/ndt_scan_matcher/pose_with_covariance", qos,
            [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg){
                last_ndt_ = msg;
            });
    }

private:
    // 最新消息缓存
    dfcv_mining_msgs::msg::LocationIMU::ConstSharedPtr last_imu_;
    nav_msgs::msg::Odometry::ConstSharedPtr last_kin_;
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr last_ndt_;

    rclcpp::Subscription<dfcv_mining_msgs::msg::LocationIMU>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr kin_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ndt_sub_;

    void try_compare()
    {
        if (!last_imu_ || !last_kin_ || !last_ndt_) return;

        // 时间戳对齐
        rclcpp::Time t_imu(last_imu_->header.stamp);
        rclcpp::Time t_kin(last_kin_->header.stamp);
        rclcpp::Time t_ndt(last_ndt_->header.stamp);

        double dt_imu_kin = std::fabs((t_imu - t_kin).seconds());
        double dt_imu_ndt = std::fabs((t_imu - t_ndt).seconds());

        // 超过阈值就不输出
        if (dt_imu_kin > 0.2 || dt_imu_ndt > 0.5){
          // RCLCPP_INFO(this->get_logger(), "Localization compare node started");
            return;
        } 

        // ---------- IMU yaw ----------
        double imu_yaw = last_imu_->yaw;

        // ---------- Kinematic ----------
        const auto &kp = last_kin_->pose.pose;
        double kin_x = kp.position.x;
        double kin_y = kp.position.y;
        double kin_z = kp.position.z;
        double kin_yaw = tf2::getYaw(kp.orientation)*180/3.1415926;

        // ---------- NDT ----------
        const auto &np = last_ndt_->pose.pose;
        double ndt_x = np.position.x;
        double ndt_y = np.position.y;
        double ndt_z = np.position.z;
        double ndt_yaw = tf2::getYaw(np.orientation)*180/3.1415926;

        // ---------- 对比输出 ----------
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 500,
            "\nIMU yaw: %.3f\n"
            "KIN: x=%.2f y=%.2f z=%.2f yaw=%.3f\n"
            "NDT: x=%.2f y=%.2f z=%.2f yaw=%.3f",
            imu_yaw,
            kin_x, kin_y, kin_z, kin_yaw,
            ndt_x, ndt_y, ndt_z, ndt_yaw);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalizationCompareNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
