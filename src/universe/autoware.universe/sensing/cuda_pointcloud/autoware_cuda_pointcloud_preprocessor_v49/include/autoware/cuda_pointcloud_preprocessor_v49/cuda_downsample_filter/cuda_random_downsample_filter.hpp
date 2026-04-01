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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_DOWNSAMPLE_FILTER__CUDA_RANDOM_DOWNSAMPLE_FILTER_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_DOWNSAMPLE_FILTER__CUDA_RANDOM_DOWNSAMPLE_FILTER_HPP_

#include "autoware/cuda_pointcloud_preprocessor_v49/cuda_downsample_filter/thrust_custom_allocator.hpp"
#include "autoware/cuda_utils/cuda_check_error.hpp"

#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <cuda_blackboard/cuda_unique_ptr.hpp>

#include <cuda_runtime.h>

#include <cstdint>
#include <memory>

namespace autoware::cuda_pointcloud_preprocessor_v49
{

class CudaRandomDownsampleFilter
{
public:
  explicit CudaRandomDownsampleFilter(
    const size_t sample_num, const int64_t max_mem_pool_size_in_byte = 1e9);
  ~CudaRandomDownsampleFilter() = default;

  std::unique_ptr<cuda_blackboard::CudaPointCloud2> filter(
    const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_points);

  void set_sample_num(size_t sample_num) { sample_num_ = sample_num; }
  size_t get_sample_num() const { return sample_num_; }

private:
  template <typename T>
  T * allocateBufferFromPool(size_t num_elements);

  template <typename T>
  void returnBufferToPool(T * buffer);

  size_t sample_num_{0};

  cudaStream_t stream_{};
  cudaMemPool_t mem_pool_{};

  std::unique_ptr<ThrustCustomAllocator> thrust_custom_allocator_;
};

}  // namespace autoware::cuda_pointcloud_preprocessor_v49

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_DOWNSAMPLE_FILTER__CUDA_RANDOM_DOWNSAMPLE_FILTER_HPP_
