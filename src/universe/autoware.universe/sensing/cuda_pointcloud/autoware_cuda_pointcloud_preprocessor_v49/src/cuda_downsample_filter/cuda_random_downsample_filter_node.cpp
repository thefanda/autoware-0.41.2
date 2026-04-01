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

#include "autoware/cuda_pointcloud_preprocessor_v49/cuda_downsample_filter/cuda_random_downsample_filter_node.hpp"

#include "autoware/pointcloud_preprocessor_v49/utility/memory.hpp"

#include <chrono>

namespace autoware::cuda_pointcloud_preprocessor_v49
{

CudaRandomDownsampleFilterNode::CudaRandomDownsampleFilterNode(
  const rclcpp::NodeOptions & node_options)
: Node("cuda_random_downsample_filter_v49", node_options)
{
  const size_t sample_num = static_cast<size_t>(declare_parameter<int64_t>("sample_num"));
  const int64_t max_mem_pool_size_in_byte =
    declare_parameter<int64_t>("max_mem_pool_size_in_byte", 1e9);

  if (max_mem_pool_size_in_byte < 0) {
    RCLCPP_ERROR(
      this->get_logger(), "Invalid pool size was specified. The value should be positive");
    return;
  }

  sub_ =
    std::make_shared<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/input/pointcloud",
      std::bind(
        &CudaRandomDownsampleFilterNode::cudaPointcloudCallback, this, std::placeholders::_1));

  pub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/output/pointcloud");

  cuda_random_downsample_filter_ = std::make_unique<CudaRandomDownsampleFilter>(
    sample_num, max_mem_pool_size_in_byte);
}

void CudaRandomDownsampleFilterNode::cudaPointcloudCallback(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr msg)
{
  if (!pointcloud_preprocessor_v49::utils::is_data_layout_compatible_with_point_xyzirc(msg->fields)) {
    RCLCPP_WARN(
      this->get_logger(),
      "Input pointcloud data layout is not compatible with PointXYZIRC. "
      "Output may preserve structure but field semantics could differ.");
  }

  const auto t0 = std::chrono::steady_clock::now();
  auto output_pointcloud_ptr = cuda_random_downsample_filter_->filter(msg);
  const auto t1 = std::chrono::steady_clock::now();
  const double elapsed_ms =
    std::chrono::duration<double, std::milli>(t1 - t0).count();

  const size_t input_pts = static_cast<size_t>(msg->width * msg->height);
  const size_t output_pts =
    static_cast<size_t>(output_pointcloud_ptr->width * output_pointcloud_ptr->height);

  RCLCPP_INFO(
    this->get_logger(),
    "[CUDA RandomDownsample] filter() took %.3f ms  (%zu pts → %zu pts)",
    elapsed_ms, input_pts, output_pts);

  pub_->publish(std::move(output_pointcloud_ptr));
}

}  // namespace autoware::cuda_pointcloud_preprocessor_v49

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::cuda_pointcloud_preprocessor_v49::CudaRandomDownsampleFilterNode)
