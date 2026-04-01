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

#include "autoware/cuda_pointcloud_preprocessor_v49/cuda_downsample_filter/cuda_random_downsample_filter.hpp"
#include "autoware/cuda_pointcloud_preprocessor_v49/cuda_downsample_filter/thrust_custom_allocator.hpp"

#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/random.h>
#include <thrust/sequence.h>
#include <thrust/shuffle.h>

#include <chrono>

namespace autoware::cuda_pointcloud_preprocessor_v49
{
namespace
{

__global__ void copyPointsByIndicesKernel(
  const uint8_t * __restrict__ input_data, uint8_t * __restrict__ output_data,
  const size_t * __restrict__ indices, size_t num_output_points, size_t point_step)
{
  size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_output_points) {
    return;
  }

  size_t src_idx = indices[idx];
  const uint8_t * src = input_data + src_idx * point_step;
  uint8_t * dst = output_data + idx * point_step;

  for (size_t i = 0; i < point_step; ++i) {
    dst[i] = src[i];
  }
}

}  // namespace

CudaRandomDownsampleFilter::CudaRandomDownsampleFilter(
  const size_t sample_num, const int64_t max_mem_pool_size_in_byte)
: sample_num_(sample_num)
{
  CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));

  int current_device_id = 0;
  CHECK_CUDA_ERROR(cudaGetDevice(&current_device_id));
  cudaMemPoolProps pool_props = {};
  pool_props.allocType = cudaMemAllocationTypePinned;
  pool_props.location.id = current_device_id;
  pool_props.location.type = cudaMemLocationTypeDevice;
  CHECK_CUDA_ERROR(cudaMemPoolCreate(&mem_pool_, &pool_props));

  uint64_t pool_release_threshold = max_mem_pool_size_in_byte;
  CHECK_CUDA_ERROR(cudaMemPoolSetAttribute(
    mem_pool_, cudaMemPoolAttrReleaseThreshold, static_cast<void *>(&pool_release_threshold)));

  thrust_custom_allocator_ = std::make_unique<ThrustCustomAllocator>(stream_, mem_pool_);
}

std::unique_ptr<cuda_blackboard::CudaPointCloud2> CudaRandomDownsampleFilter::filter(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_points)
{
  const size_t num_input_points = input_points->width * input_points->height;
  const size_t point_step = input_points->point_step;

  if (num_input_points == 0) {
    auto empty_output = std::make_unique<cuda_blackboard::CudaPointCloud2>();
    empty_output->header = input_points->header;
    empty_output->height = 1;
    empty_output->width = 0;
    empty_output->fields = input_points->fields;
    empty_output->is_bigendian = input_points->is_bigendian;
    empty_output->point_step = point_step;
    empty_output->row_step = 0;
    empty_output->is_dense = input_points->is_dense;
    empty_output->data = cuda_blackboard::make_unique<std::uint8_t[]>(0);
    return empty_output;
  }

  const size_t num_output_points = std::min(sample_num_, num_input_points);

  thrust::device_vector<size_t> indices(num_input_points);
  thrust::sequence(thrust::cuda::par.on(stream_), indices.begin(), indices.end(), 0UL);

  unsigned int seed = static_cast<unsigned int>(
    std::chrono::steady_clock::now().time_since_epoch().count());
  thrust::default_random_engine rng(seed);
  thrust::shuffle(thrust::cuda::par.on(stream_), indices.begin(), indices.end(), rng);

  auto output = std::make_unique<cuda_blackboard::CudaPointCloud2>();
  output->header = input_points->header;
  output->height = 1;
  output->width = num_output_points;
  output->fields = input_points->fields;
  output->is_bigendian = input_points->is_bigendian;
  output->point_step = point_step;
  output->row_step = point_step * num_output_points;
  output->is_dense = input_points->is_dense;
  output->data = cuda_blackboard::make_unique<std::uint8_t[]>(output->row_step * output->height);

  const dim3 block_dim(256);
  const dim3 grid_dim((num_output_points + block_dim.x - 1) / block_dim.x);

  copyPointsByIndicesKernel<<<grid_dim, block_dim, 0, stream_>>>(
    input_points->data.get(), output->data.get(), thrust::raw_pointer_cast(indices.data()),
    num_output_points, point_step);

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  return output;
}

template <typename T>
T * CudaRandomDownsampleFilter::allocateBufferFromPool(size_t num_elements)
{
  T * buffer{};
  CHECK_CUDA_ERROR(cudaMallocFromPoolAsync(&buffer, num_elements * sizeof(T), mem_pool_, stream_));
  CHECK_CUDA_ERROR(cudaMemsetAsync(buffer, 0, num_elements * sizeof(T), stream_));
  return buffer;
}

template <typename T>
void CudaRandomDownsampleFilter::returnBufferToPool(T * buffer)
{
  CHECK_CUDA_ERROR(cudaFreeAsync(buffer, stream_));
}

}  // namespace autoware::cuda_pointcloud_preprocessor_v49
