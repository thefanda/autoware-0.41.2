// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/cuda_pointcloud_preprocessor_v49/cuda_pointcloud_preprocessor.hpp"
#include "autoware/cuda_pointcloud_preprocessor_v49/memory.hpp"
#include "autoware/cuda_pointcloud_preprocessor_v49/point_types.hpp"
#include "autoware/pointcloud_preprocessor_v49/diagnostics/crop_box_diagnostics.hpp"
#include "autoware/pointcloud_preprocessor_v49/diagnostics/latency_diagnostics.hpp"
#include "autoware/pointcloud_preprocessor_v49/diagnostics/pass_rate_diagnostics.hpp"
#include "autoware/pointcloud_preprocessor_v49/utility/memory.hpp"

#include <autoware/point_types/types.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>

#include <autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp>
#include <autoware/point_types/types.hpp>
#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/ros/diagnostics_interface.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <cuda_blackboard/cuda_adaptation.hpp>
#include <cuda_blackboard/cuda_blackboard_publisher.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/transform_datatypes.h>

#include <agnocast/agnocast_subscription.hpp>
#include <autoware/cuda_utils/cuda_check_error.hpp>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cuda_runtime.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <chrono>
#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::cuda_pointcloud_preprocessor_v49
{
using sensor_msgs::msg::PointCloud2;

using autoware::point_types::PointXYZIRC;
using autoware::point_types::PointXYZIRCAEDT;
using autoware::point_types::PointXYZIRCAEDTGenerator;
using autoware::pointcloud_preprocessor_v49::utils::is_data_layout_compatible_with_point_xyzirc;
using point_cloud_msg_wrapper::PointCloud2Modifier;

// A lightweight "crop-only" wrapper node around CudaPointcloudPreprocessor.
// It disables undistortion/outlier features and only applies crop boxes.
class CudaPointcloudPreprocessorCropOnlyNode : public rclcpp::Node
{
public:
  explicit CudaPointcloudPreprocessorCropOnlyNode(const rclcpp::NodeOptions & node_options);

private:
  bool getTransform(
    const std::string & target_frame, const std::string & source_frame,
    tf2::Transform * tf2_transform_ptr);

  // Callback
  void pointcloudCallback(AUTOWARE_MESSAGE_UNIQUE_PTR(sensor_msgs::msg::PointCloud2)
                            input_pointcloud_msg_ptr);
  void twistCallback(const geometry_msgs::msg::TwistWithCovarianceStamped & twist_msg);
  void imuCallback(const sensor_msgs::msg::Imu & imu_msg);

  // Helper Functions
  [[nodiscard]] bool validatePointcloudLayout(
    const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg) const;
  std::pair<std::uint64_t, std::uint32_t> getFirstPointTimeInfo(
    const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg);

  void updateTwistQueue(std::uint64_t first_point_stamp);
  void updateImuQueue(std::uint64_t first_point_stamp);
  std::optional<geometry_msgs::msg::TransformStamped> lookupTransformToBase(
    const std::string & source_frame);
  std::unique_ptr<cuda_blackboard::CudaPointCloud2> processPointcloud(
    const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg,
    const geometry_msgs::msg::TransformStamped & transform_msg,
    const std::uint32_t first_point_rel_stamp);

  void publishDiagnostics(
    const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg,
    const std::unique_ptr<cuda_blackboard::CudaPointCloud2> & output_pointcloud_ptr);

  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  // Diagnostic
  std::unique_ptr<autoware_utils::DiagnosticsInterface> diagnostics_interface_;

  std::string base_frame_;
  bool use_3d_undistortion_;
  bool use_imu_;
  double processing_time_threshold_sec_;
  double timestamp_mismatch_fraction_threshold_;

  std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> twist_queue_;
  std::deque<geometry_msgs::msg::Vector3Stamped> angular_velocity_queue_;

  // Subscribers
  AUTOWARE_SUBSCRIPTION_PTR(sensor_msgs::msg::PointCloud2) pointcloud_sub_;

  // CUDA pub
  std::unique_ptr<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>> pub_;

  std::unique_ptr<CudaPointcloudPreprocessor> cuda_pointcloud_preprocessor_;

  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<autoware_utils::DebugPublisher> debug_publisher_;

  // Callback group for pointcloud subscription
  rclcpp::CallbackGroup::SharedPtr pointcloud_callback_group_;
};

CudaPointcloudPreprocessorCropOnlyNode::CudaPointcloudPreprocessorCropOnlyNode(
  const rclcpp::NodeOptions & node_options)
: Node("cuda_pointcloud_preprocessor_crop_only_v49", node_options),
  tf2_buffer_(this->get_clock()),
  tf2_listener_(tf2_buffer_)
{
  using std::placeholders::_1;

  // Set CUDA device flags
  // note: Device flags are process-wide
  CHECK_CUDA_ERROR(cudaSetDeviceFlags(cudaDeviceScheduleBlockingSync));

  // Parameters
  base_frame_ = declare_parameter<std::string>("base_frame");
  use_3d_undistortion_ = false;  // Not used in crop-only mode
  use_imu_ = false;  // Not used in crop-only mode
  processing_time_threshold_sec_ = declare_parameter<double>("processing_time_threshold_sec");
  timestamp_mismatch_fraction_threshold_ = 0.0;  // Not used in crop-only mode
  const bool crop_box_negative = declare_parameter<bool>("crop_box.negative", true);

  const auto crop_box_min_x_vector = declare_parameter<std::vector<double>>("crop_box.min_x");
  const auto crop_box_min_y_vector = declare_parameter<std::vector<double>>("crop_box.min_y");
  const auto crop_box_min_z_vector = declare_parameter<std::vector<double>>("crop_box.min_z");

  const auto crop_box_max_x_vector = declare_parameter<std::vector<double>>("crop_box.max_x");
  const auto crop_box_max_y_vector = declare_parameter<std::vector<double>>("crop_box.max_y");
  const auto crop_box_max_z_vector = declare_parameter<std::vector<double>>("crop_box.max_z");

  if (
    crop_box_min_x_vector.size() != crop_box_min_y_vector.size() ||
    crop_box_min_x_vector.size() != crop_box_min_z_vector.size() ||
    crop_box_min_x_vector.size() != crop_box_max_x_vector.size() ||
    crop_box_min_x_vector.size() != crop_box_max_y_vector.size() ||
    crop_box_min_x_vector.size() != crop_box_max_z_vector.size()) {
    throw std::runtime_error("Crop box parameters must have the same size");
  }

  std::vector<CropBoxParameters> crop_box_parameters;

  for (std::size_t i = 0; i < crop_box_min_x_vector.size(); i++) {
    CropBoxParameters parameters{};
    parameters.min_x = static_cast<float>(crop_box_min_x_vector.at(i));
    parameters.min_y = static_cast<float>(crop_box_min_y_vector.at(i));
    parameters.min_z = static_cast<float>(crop_box_min_z_vector.at(i));
    parameters.max_x = static_cast<float>(crop_box_max_x_vector.at(i));
    parameters.max_y = static_cast<float>(crop_box_max_y_vector.at(i));
    parameters.max_z = static_cast<float>(crop_box_max_z_vector.at(i));
    crop_box_parameters.push_back(parameters);
  }

  // Publisher
  pub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/output/pointcloud");
  diagnostics_interface_ =
    std::make_unique<autoware_utils::DiagnosticsInterface>(this, this->get_fully_qualified_name());

  // Subscriber
#ifdef USE_AGNOCAST_ENABLED
  agnocast::SubscriptionOptions sub_options{};
#else
  rclcpp::SubscriptionOptions sub_options;
  sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
#endif

  // Create mutually exclusive callback group for pointcloud subscription
  // Agnocast requires to be in a mutually exclusive callback group with no native ROS subscriptions
  pointcloud_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  sub_options.callback_group = pointcloud_callback_group_;

  // cppcheck-suppress unknownMacro
  pointcloud_sub_ = AUTOWARE_CREATE_SUBSCRIPTION(
    sensor_msgs::msg::PointCloud2, "~/input/pointcloud", rclcpp::SensorDataQoS{}.keep_last(1),
    std::bind(&CudaPointcloudPreprocessorCropOnlyNode::pointcloudCallback, this, std::placeholders::_1),
    sub_options);

  cuda_pointcloud_preprocessor_ = std::make_unique<CudaPointcloudPreprocessor>();
  cuda_pointcloud_preprocessor_->setCropBoxParameters(crop_box_parameters);
  cuda_pointcloud_preprocessor_->setCropBoxNegative(crop_box_negative);
  // Disable undistortion by setting Undistortion2D but passing empty queues.
  // The implementation checks queue size before applying distortion correction.
  cuda_pointcloud_preprocessor_->setUndistortionType(
    CudaPointcloudPreprocessor::UndistortionType::Undistortion2D);
  // Disable ring outlier filter
  cuda_pointcloud_preprocessor_->setRingOutlierFilterActive(false);

  // initialize debug tool
  {
    using autoware_utils::DebugPublisher;
    using autoware_utils::StopWatch;

    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ =
      std::make_unique<DebugPublisher>(this, "cuda_pointcloud_preprocessor_crop_only_v49");
    stop_watch_ptr_->tic("processing_time");
  }
}

bool CudaPointcloudPreprocessorCropOnlyNode::getTransform(
  const std::string & target_frame, const std::string & source_frame,
  tf2::Transform * tf2_transform_ptr)
{
  if (target_frame == source_frame) {
    tf2_transform_ptr->setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    tf2_transform_ptr->setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
    return true;
  }

  try {
    const auto transform_msg =
      tf2_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    tf2::convert(transform_msg.transform, *tf2_transform_ptr);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    RCLCPP_ERROR(
      get_logger(), "Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    tf2_transform_ptr->setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    tf2_transform_ptr->setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
    return false;
  }
  return true;
}

void CudaPointcloudPreprocessorCropOnlyNode::twistCallback(
  const geometry_msgs::msg::TwistWithCovarianceStamped & twist_msg)
{
  // Not used in crop-only mode
  (void)twist_msg;
}

void CudaPointcloudPreprocessorCropOnlyNode::imuCallback(const sensor_msgs::msg::Imu & imu_msg)
{
  // Not used in crop-only mode
  (void)imu_msg;
}

void CudaPointcloudPreprocessorCropOnlyNode::pointcloudCallback(
  // cppcheck-suppress unknownMacro
  AUTOWARE_MESSAGE_UNIQUE_PTR(sensor_msgs::msg::PointCloud2) input_pointcloud_msg_ptr)
{
  const auto & input_pointcloud_msg = *input_pointcloud_msg_ptr;

  // Validate layout and optionally convert PointXYZIRC -> PointXYZIRCAEDT
  if (!validatePointcloudLayout(input_pointcloud_msg)) {
    return;
  }

  // If the input is already PointXYZIRCAEDT, we can pass it through directly.
  // If it's PointXYZIRC, convert it to PointXYZIRCAEDT so that the CUDA
  // preprocessor can safely consume it.
  PointCloud2 pointcloud_to_process;
  const PointCloud2 * pointcloud_to_process_ptr = nullptr;

  if (is_data_layout_compatible_with_point_xyzircaedt(input_pointcloud_msg.fields)) {
    pointcloud_to_process_ptr = &input_pointcloud_msg;
  } else if (is_data_layout_compatible_with_point_xyzirc(input_pointcloud_msg)) {
    pointcloud_to_process = PointCloud2{};
    pointcloud_to_process.header = input_pointcloud_msg.header;

    const auto num_points = input_pointcloud_msg.width * input_pointcloud_msg.height;
    PointCloud2Modifier<PointXYZIRCAEDT, PointXYZIRCAEDTGenerator> modifier(
      pointcloud_to_process, input_pointcloud_msg.header.frame_id);
    modifier.reserve(num_points);

    sensor_msgs::PointCloud2ConstIterator<float> it_x(input_pointcloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(input_pointcloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(input_pointcloud_msg, "z");
    sensor_msgs::PointCloud2ConstIterator<std::uint8_t> it_i(
      input_pointcloud_msg, "intensity");
    sensor_msgs::PointCloud2ConstIterator<std::uint8_t> it_r(
      input_pointcloud_msg, "return_type");
    sensor_msgs::PointCloud2ConstIterator<std::uint16_t> it_c(
      input_pointcloud_msg, "channel");

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_i, ++it_r, ++it_c) {
      PointXYZIRCAEDT point{};
      point.x = *it_x;
      point.y = *it_y;
      point.z = *it_z;
      point.intensity = *it_i;
      point.return_type = *it_r;
      point.channel = *it_c;

      // Derive polar coordinates from Cartesian geometry
      const float distance =
        std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
      point.distance = distance;
      point.azimuth = std::atan2(point.y, point.x);
      point.elevation = distance > 0.0F ? std::asin(point.z / distance) : 0.0F;

      // In crop-only mode we don't use per-point time stamps, so set to zero.
      point.time_stamp = 0U;

      modifier.push_back(point);
    }

    pointcloud_to_process_ptr = &pointcloud_to_process;
  } else {
    // This should not happen because validatePointcloudLayout() filtered it,
    // but keep a guard just in case.
    RCLCPP_ERROR(
      get_logger(),
      "Unsupported pointcloud layout for cuda_pointcloud_preprocessor_crop_only_node_v49");
    return;
  }

  stop_watch_ptr_->toc("processing_time", true);

  const auto transform_msg_opt = lookupTransformToBase(pointcloud_to_process_ptr->header.frame_id);
  if (!transform_msg_opt.has_value()) return;

  const auto t0 = std::chrono::steady_clock::now();
  auto output_pointcloud_ptr =
    processPointcloud(*pointcloud_to_process_ptr, *transform_msg_opt, 0);
  const auto t1 = std::chrono::steady_clock::now();
  const double elapsed_ms =
    std::chrono::duration<double, std::milli>(t1 - t0).count();

  output_pointcloud_ptr->header.frame_id = base_frame_;

  const size_t input_pts =
    static_cast<size_t>(pointcloud_to_process_ptr->width * pointcloud_to_process_ptr->height);
  const size_t output_pts =
    static_cast<size_t>(output_pointcloud_ptr->width * output_pointcloud_ptr->height);

  RCLCPP_INFO(
    get_logger(),
    "[CUDA CropBox] filter() took %.3f ms  (%zu pts → %zu pts)",
    elapsed_ms, input_pts, output_pts);

  publishDiagnostics(input_pointcloud_msg, output_pointcloud_ptr);
  pub_->publish(std::move(output_pointcloud_ptr));

  // Preallocate buffer for the next run.
  // This is intentionally done after publish() to avoid adding latency to the current cycle.
  cuda_pointcloud_preprocessor_->preallocateOutput();
}

[[nodiscard]] bool CudaPointcloudPreprocessorCropOnlyNode::validatePointcloudLayout(
  const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg) const
{
  static_assert(
    sizeof(InputPointType) == sizeof(autoware::point_types::PointXYZIRCAEDT),
    "PointStruct and PointXYZIRCAEDT must have the same size");

  if (is_data_layout_compatible_with_point_xyzircaedt(input_pointcloud_msg.fields)) {
    return true;
  }

  if (is_data_layout_compatible_with_point_xyzirc(input_pointcloud_msg)) {
    return true;
  }

  RCLCPP_ERROR(
    get_logger(),
    "Input pointcloud data layout is not compatible with PointXYZIRCAEDT or PointXYZIRC");
  return false;
}

std::pair<std::uint64_t, std::uint32_t> CudaPointcloudPreprocessorCropOnlyNode::getFirstPointTimeInfo(
  const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg)
{
  sensor_msgs::PointCloud2ConstIterator<std::uint32_t> iter_stamp(
    input_pointcloud_msg, "time_stamp");
  auto num_points = input_pointcloud_msg.width * input_pointcloud_msg.height;
  std::uint32_t first_point_rel_stamp = num_points > 0 ? *iter_stamp : 0;
  std::uint64_t first_point_stamp = input_pointcloud_msg.header.stamp.sec * 1e9 +
                                    input_pointcloud_msg.header.stamp.nanosec +
                                    first_point_rel_stamp;
  return {first_point_stamp, first_point_rel_stamp};
}

void CudaPointcloudPreprocessorCropOnlyNode::updateTwistQueue(std::uint64_t first_point_stamp)
{
  // Not used in crop-only mode
  (void)first_point_stamp;
}

void CudaPointcloudPreprocessorCropOnlyNode::updateImuQueue(std::uint64_t first_point_stamp)
{
  // Not used in crop-only mode
  (void)first_point_stamp;
}

std::optional<geometry_msgs::msg::TransformStamped>
CudaPointcloudPreprocessorCropOnlyNode::lookupTransformToBase(const std::string & source_frame)
{
  try {
    return tf2_buffer_.lookupTransform(base_frame_, source_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    return std::nullopt;
  }
}

std::unique_ptr<cuda_blackboard::CudaPointCloud2> CudaPointcloudPreprocessorCropOnlyNode::processPointcloud(
  const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg,
  const geometry_msgs::msg::TransformStamped & transform_msg,
  const std::uint32_t first_point_rel_stamp)
{
  // Empty queues since we don't use distortion correction
  std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> empty_twist_queue;
  std::deque<geometry_msgs::msg::Vector3Stamped> empty_angular_velocity_queue;

  return cuda_pointcloud_preprocessor_->process(
    input_pointcloud_msg, transform_msg, empty_twist_queue, empty_angular_velocity_queue,
    first_point_rel_stamp);
}

void CudaPointcloudPreprocessorCropOnlyNode::publishDiagnostics(
  const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg,
  const std::unique_ptr<cuda_blackboard::CudaPointCloud2> & output_pointcloud_ptr)
{
  const auto processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
  const auto pipeline_latency_ms =
    std::chrono::duration<double, std::milli>(
      std::chrono::nanoseconds(
        (this->get_clock()->now() - input_pointcloud_msg.header.stamp).nanoseconds()))
      .count();

  const auto input_point_count =
    static_cast<int>(input_pointcloud_msg.width * input_pointcloud_msg.height);
  const auto output_point_count =
    static_cast<int>(output_pointcloud_ptr->width * output_pointcloud_ptr->height);
  const auto stats = cuda_pointcloud_preprocessor_->getProcessingStats();

  const auto skipped_nan_count = stats.num_nan_points;
  const auto num_crop_box_passed_points = stats.num_crop_box_passed_points;
  (void)num_crop_box_passed_points;

  auto latency_diag = std::make_shared<autoware::pointcloud_preprocessor_v49::LatencyDiagnostics>(
    input_pointcloud_msg.header.stamp, processing_time_ms, pipeline_latency_ms,
    processing_time_threshold_sec_ * 1000.0);
  auto pass_rate_diag = std::make_shared<autoware::pointcloud_preprocessor_v49::PassRateDiagnostics>(
    input_point_count, output_point_count);

  auto crop_box_diag =
    std::make_shared<autoware::pointcloud_preprocessor_v49::CropBoxDiagnostics>(skipped_nan_count);

  diagnostics_interface_->clear();

  std::vector<std::shared_ptr<const autoware::pointcloud_preprocessor_v49::DiagnosticsBase>>
    diagnostics = {latency_diag, pass_rate_diag, crop_box_diag};

  int worst_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string message;

  for (const auto & diag : diagnostics) {
    diag->add_to_interface(*diagnostics_interface_);
    if (const auto status = diag->evaluate_status(); status.has_value()) {
      worst_level = std::max(worst_level, status->first);
      if (!message.empty()) {
        message += " / ";
      }
      message += status->second;
    }
  }
  if (message.empty()) {
    message = "CudaPointcloudPreprocessorCropOnly operating normally";
  }
  diagnostics_interface_->update_level_and_message(static_cast<int8_t>(worst_level), message);
  diagnostics_interface_->publish(this->get_clock()->now());

  debug_publisher_->publish<autoware_internal_debug_msgs_v49::msg::Float64Stamped>(
    "debug/processing_time_ms", processing_time_ms);
  debug_publisher_->publish<autoware_internal_debug_msgs_v49::msg::Float64Stamped>(
    "debug/latency_ms", pipeline_latency_ms);
}

}  // namespace autoware::cuda_pointcloud_preprocessor_v49

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::cuda_pointcloud_preprocessor_v49::CudaPointcloudPreprocessorCropOnlyNode)

