#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "ublox_msgs/msg/nav_pvt.hpp"
#include "dfcv_mining_msgs/msg/location_imu.hpp"  // 你的自定义消息
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;

class GNSSConverterNode : public rclcpp::Node
{
public:
    GNSSConverterNode()
    : Node("gnss_converter_node")
    {
        // 发布三个话题
        fix_velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "/sensing/gnss/ublox/fix_velocity", 10);
        nav_sat_fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
            "/sensing/gnss/ublox/nav_sat_fix", 10);
        navpvt_pub_ = this->create_publisher<ublox_msgs::msg::NavPVT>(
            "/sensing/gnss/ublox/navpvt", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
    "/sensing/imu/tamagawa/imu_raw", 10);
        // 订阅自定义消息
        sub_ = this->create_subscription<dfcv_mining_msgs::msg::LocationIMU>(
            "/location_imu_info", 10,
            std::bind(&GNSSConverterNode::callback, this, _1));
    }

private:
    void callback(const dfcv_mining_msgs::msg::LocationIMU::SharedPtr msg)
    {
        // -------------------------------
        // 1. TwistWithCovarianceStamped
        // -------------------------------
        geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
        twist_msg.header = msg->header;
        twist_msg.header.frame_id = "gnss_link";
        // twist_msg.twist.twist.linear.x = msg->acc_x;
        // twist_msg.twist.twist.linear.y = msg->acc_y;
        // twist_msg.twist.twist.linear.z = msg->acc_z;

        // twist_msg.twist.twist.angular.x = msg->gyro_x;
        // twist_msg.twist.twist.angular.y = msg->gyro_y;
        // twist_msg.twist.twist.angular.z = msg->gyro_z;
        twist_msg.twist.twist.linear.x = 0;
        twist_msg.twist.twist.linear.y = 0;
        twist_msg.twist.twist.linear.z = 0;

        twist_msg.twist.twist.angular.x = 0;
        twist_msg.twist.twist.angular.y = 0;
        twist_msg.twist.twist.angular.z = 0;

        // 简单协方差示例，可按需要修改
        for (int i = 0; i < 36; ++i)
        {
            twist_msg.twist.covariance[i] = 0.0;
        }
        twist_msg.twist.covariance[0] = 0.05;
        twist_msg.twist.covariance[7] = 0.05;
        twist_msg.twist.covariance[14] = 0.05;

        fix_velocity_pub_->publish(twist_msg);

        // -------------------------------
        // 2. NavSatFix
        // -------------------------------
        sensor_msgs::msg::NavSatFix nav_msg;
        nav_msg.header = msg->header;
        nav_msg.header.frame_id = "gnss_link";
        nav_msg.latitude = msg->lat;
        nav_msg.longitude = msg->lon;
        nav_msg.altitude = msg->hil_z; // 可选使用 HIL 高度或 GPS 高度
        nav_msg.status.status = msg->gps_week_navigation == 4 ? sensor_msgs::msg::NavSatStatus::STATUS_FIX : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        nav_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

        nav_sat_fix_pub_->publish(nav_msg);

        // -------------------------------
        // 3. NavPVT
        // -------------------------------
        ublox_msgs::msg::NavPVT navpvt_msg;
        //navpvt_msg.header = msg->header;
        navpvt_msg.i_tow = msg->gps_millisecond;


        rclcpp::Time t = msg->header.stamp;
        std::time_t tt = static_cast<std::time_t>(t.seconds()); // 用 t.seconds() 代替 t.sec
        std::tm* gmtime_ptr = std::gmtime(&tt);

        navpvt_msg.year  = gmtime_ptr->tm_year + 1900; // tm_year 是从1900开始
        navpvt_msg.month = gmtime_ptr->tm_mon + 1;    // tm_mon 0~11
        navpvt_msg.day   = gmtime_ptr->tm_mday;
        navpvt_msg.hour  = gmtime_ptr->tm_hour;
        navpvt_msg.min   = gmtime_ptr->tm_min;
        navpvt_msg.sec   = gmtime_ptr->tm_sec;


        navpvt_msg.fix_type = msg->gps_week_navigation == 4 ? 3 : 0;
        navpvt_msg.num_sv = 10; // 示例值
        navpvt_msg.lon = static_cast<int32_t>(msg->lon * 1e7); // NavPVT 需要1e-7 度
        navpvt_msg.lat = static_cast<int32_t>(msg->lat * 1e7);
        navpvt_msg.height = static_cast<int32_t>(msg->hil_z * 1000);
        navpvt_msg.h_msl = static_cast<int32_t>(msg->hil_z * 1000);

        navpvt_pub_->publish(navpvt_msg);
            // -------------------------------
        // 4. Imu (tamagawa imu_raw)
        // -------------------------------
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header = msg->header;
         imu_msg.header.frame_id = "tamagawa/imu_link";
        // 1) orientation: roll/pitch/yaw -> quaternion
        tf2::Quaternion q;
        q.setRPY(msg->roll, msg->pitch, msg->yaw);
        imu_msg.orientation.x = 0;
        imu_msg.orientation.y = 0;
        imu_msg.orientation.z = 0;
        imu_msg.orientation.w = 1;

        // 2) angular velocity
        imu_msg.angular_velocity.x = msg->gyro_y * M_PI / 180.0;  // X 保持
        imu_msg.angular_velocity.y = -msg->gyro_x * M_PI / 180.0;  // Y 保持
        imu_msg.angular_velocity.z = msg->gyro_z * M_PI / 180.0;  // Z 取反


        // 3) linear acceleration
        imu_msg.linear_acceleration.x = msg->acc_y;
        imu_msg.linear_acceleration.y = -msg->acc_x;
        imu_msg.linear_acceleration.z = msg->acc_z;

        // 协方差可按需要设置，这里先全部设 0
        for (int i = 0; i < 9; i++) {
            imu_msg.orientation_covariance[i] = 0.0;
            imu_msg.angular_velocity_covariance[i] = 0.0;
            imu_msg.linear_acceleration_covariance[i] = 0.0;
        }

        imu_pub_->publish(imu_msg);

    }
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr fix_velocity_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_fix_pub_;
    rclcpp::Publisher<ublox_msgs::msg::NavPVT>::SharedPtr navpvt_pub_;

    rclcpp::Subscription<dfcv_mining_msgs::msg::LocationIMU>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GNSSConverterNode>());
    rclcpp::shutdown();
    return 0;
}
