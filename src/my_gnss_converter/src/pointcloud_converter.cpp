#include <memory>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include "dfcv_mining_msgs/msg/location_imu.hpp" 
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
class PointCloudConverter : public rclcpp::Node
{
public:
    PointCloudConverter()
    : Node("pointcloud_converter")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar/concatenated/pointcloud",
            rclcpp::SensorDataQoS().keep_last(1),
            std::bind(&PointCloudConverter::pointcloud_callback, this, std::placeholders::_1)
        );
        
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/sensing/lidar/points_autoware",
            rclcpp::SensorDataQoS().keep_last(1) // 保持和 Autoware 一致
        );
        sub_ = this->create_subscription<dfcv_mining_msgs::msg::LocationIMU>(
            "/location_imu_info_wrong",
            rclcpp::QoS(10),
            std::bind(&PointCloudConverter::callback, this, std::placeholders::_1)
        );

        pub_ = this->create_publisher<dfcv_mining_msgs::msg::LocationIMU>(
            "/location_imu_info",
            rclcpp::QoS(10)
        );
    this->declare_parameter("velocity_longitudinal", 5.0);
    this->declare_parameter("velocity_lateral", 0.0);
    this->declare_parameter("heading_rate", 0.0);
        velocity_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", 10);

    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);
        Eigen::Matrix4f T_lidartortk;
        T_lidartortk = Eigen::Matrix4f::Identity();
        T_lidartortk(0, 3) = 0;
        T_lidartortk(1, 3) = 0;
        T_lidartortk(2, 3) = 0;
        Eigen::AngleAxisd yawAngler(Eigen::AngleAxisd(0* M_PI / 180, Eigen::Vector3d::UnitZ()));
        Eigen::AngleAxisd pitchAngler(Eigen::AngleAxisd(0 * M_PI / 180, Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd rollAngler(Eigen::AngleAxisd(0* M_PI / 180, Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd rotation_vectorr;
        rotation_vectorr = yawAngler * pitchAngler * rollAngler;
        T_lidartortk.block<3, 3>(0, 0) = rotation_vectorr.matrix().cast<float>();
        pcl::transformPointCloud(*cloud, *source_cloud, T_lidartortk);
        sensor_msgs::msg::PointCloud2 out_msg2;
        pcl::toROSMsg(*source_cloud, out_msg2);
        sensor_msgs::msg::PointCloud2 out_msg;
        out_msg.header = msg->header;              // 保留 frame_id
        out_msg.header.stamp = this->get_clock()->now();  // 改为系统当前时间
        out_msg.height = 1;
        out_msg.width  = source_cloud->size();
        out_msg.is_bigendian = false;
        out_msg.is_dense = msg->is_dense;

        // 设置字段，和 Autoware 一致
        out_msg.fields.resize(6);
        out_msg.fields[0].name = "x";           out_msg.fields[0].offset = 0;  out_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32; out_msg.fields[0].count = 1;
        out_msg.fields[1].name = "y";           out_msg.fields[1].offset = 4;  out_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32; out_msg.fields[1].count = 1;
        out_msg.fields[2].name = "z";           out_msg.fields[2].offset = 8;  out_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32; out_msg.fields[2].count = 1;
        out_msg.fields[3].name = "intensity";   out_msg.fields[3].offset = 12; out_msg.fields[3].datatype = sensor_msgs::msg::PointField::UINT8;   out_msg.fields[3].count = 1;
        out_msg.fields[4].name = "return_type"; out_msg.fields[4].offset = 13; out_msg.fields[4].datatype = sensor_msgs::msg::PointField::UINT8;   out_msg.fields[4].count = 1;
        out_msg.fields[5].name = "channel";     out_msg.fields[5].offset = 14; out_msg.fields[5].datatype = sensor_msgs::msg::PointField::UINT16;  out_msg.fields[5].count = 1;

        // 每个点 16 字节
        out_msg.point_step = 16;
        out_msg.row_step = out_msg.point_step * out_msg.width;
        out_msg.data.resize(out_msg.row_step * out_msg.height);

        // 遍历原始点云，填充新点云
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(out_msg2, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(out_msg2, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(out_msg2, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(*msg, "intensity");

        sensor_msgs::PointCloud2Iterator<float> out_iter_x(out_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> out_iter_y(out_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> out_iter_z(out_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> out_iter_intensity(out_msg, "intensity");
        sensor_msgs::PointCloud2Iterator<uint8_t> out_iter_return_type(out_msg, "return_type");
        sensor_msgs::PointCloud2Iterator<uint16_t> out_iter_channel(out_msg, "channel");

        for (size_t i = 0; i < out_msg2.width * out_msg2.height; ++i,
             ++iter_x, ++iter_y, ++iter_z, ++iter_intensity,
             ++out_iter_x, ++out_iter_y, ++out_iter_z, ++out_iter_intensity, ++out_iter_return_type, ++out_iter_channel)
        {
            *out_iter_x = *iter_x;
            *out_iter_y = *iter_y;
            *out_iter_z = *iter_z;
            *out_iter_intensity = static_cast<uint8_t>(*iter_intensity); // 转为 uint8
            *out_iter_return_type = 0; // 默认填 0
            *out_iter_channel = 0;     // 默认填 0
        }

        publisher_->publish(out_msg);
    }
   void callback(const dfcv_mining_msgs::msg::LocationIMU::SharedPtr msg)
{
    // -----------------------------
    // 1️⃣ 发布 LocationIMU
    // -----------------------------
    auto out_msg = *msg;
    out_msg.header.stamp = this->get_clock()->now();
    pub_->publish(out_msg);

    // -----------------------------
    // 2️⃣ 发布 VelocityReport（参数化）
    // -----------------------------
    // 获取参数，如果没传入，则使用默认值

    double vel_longitudinal = this->get_parameter("velocity_longitudinal").as_double();
    double vel_lateral     = this->get_parameter("velocity_lateral").as_double();
    double heading_rate    = this->get_parameter("heading_rate").as_double();
    if (vel_longitudinal == 0.0) {
        return;
    }
    autoware_vehicle_msgs::msg::VelocityReport vel_msg;

    // header 时间戳 + frame
    vel_msg.header.stamp = this->get_clock()->now();
    vel_msg.header.frame_id = "base_link"; // 可根据需要修改

    // 填速度
    vel_msg.longitudinal_velocity = vel_longitudinal;
    vel_msg.lateral_velocity = vel_lateral;
    vel_msg.heading_rate = heading_rate;

    velocity_pub_->publish(vel_msg);
}

    rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_pub_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<dfcv_mining_msgs::msg::LocationIMU>::SharedPtr sub_;
    rclcpp::Publisher<dfcv_mining_msgs::msg::LocationIMU>::SharedPtr pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
