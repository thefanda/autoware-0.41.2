// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DEVICE_OPTIONAL_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DEVICE_OPTIONAL_HPP_

#include <cuda_runtime.h>

namespace autoware::cuda_pointcloud_preprocessor_v49
{

// A tiny, device-friendly optional replacement used to support CUDA toolchains
// that do not provide libcudacxx headers such as <cuda/std/optional>.
// This implementation is intentionally minimal and only supports the operations
// required by this package (value storage, has_value check, and comparisons).

struct nullopt_t
{
  explicit constexpr nullopt_t(int) {}
};

inline constexpr nullopt_t nullopt{0};

template <typename T>
class DeviceOptional
{
public:
  __host__ __device__ constexpr DeviceOptional() : has_value_(false), value_{} {}
  __host__ __device__ constexpr DeviceOptional(nullopt_t) : has_value_(false), value_{} {}
  __host__ __device__ constexpr DeviceOptional(const T & v) : has_value_(true), value_(v) {}

  __host__ __device__ constexpr bool has_value() const { return has_value_; }
  __host__ __device__ constexpr explicit operator bool() const { return has_value_; }

  __host__ __device__ constexpr T & value() { return value_; }
  __host__ __device__ constexpr const T & value() const { return value_; }

private:
  bool has_value_;
  T value_;
};

template <typename T>
__host__ __device__ constexpr bool operator==(const DeviceOptional<T> & lhs, nullopt_t)
{
  return !lhs.has_value();
}

template <typename T>
__host__ __device__ constexpr bool operator==(nullopt_t, const DeviceOptional<T> & rhs)
{
  return !rhs.has_value();
}

template <typename T>
__host__ __device__ constexpr bool operator!=(const DeviceOptional<T> & lhs, nullopt_t)
{
  return lhs.has_value();
}

template <typename T>
__host__ __device__ constexpr bool operator!=(nullopt_t, const DeviceOptional<T> & rhs)
{
  return rhs.has_value();
}

// Comparison semantics follow cuda::std::optional:
// - nullopt is considered less than any value
// - if both have values, compare contained values
template <typename T>
__host__ __device__ constexpr bool operator<(const DeviceOptional<T> & lhs, const DeviceOptional<T> & rhs)
{
  if (!lhs.has_value() && !rhs.has_value()) {
    return false;
  }
  if (!lhs.has_value() && rhs.has_value()) {
    return true;
  }
  if (lhs.has_value() && !rhs.has_value()) {
    return false;
  }
  return lhs.value() < rhs.value();
}

template <typename T>
__host__ __device__ constexpr bool operator<=(const DeviceOptional<T> & lhs, const DeviceOptional<T> & rhs)
{
  return !(rhs < lhs);
}

template <typename T>
__host__ __device__ constexpr bool operator>(const DeviceOptional<T> & lhs, const DeviceOptional<T> & rhs)
{
  return rhs < lhs;
}

template <typename T>
__host__ __device__ constexpr bool operator>=(const DeviceOptional<T> & lhs, const DeviceOptional<T> & rhs)
{
  return !(lhs < rhs);
}

template <typename T>
struct DeviceOptionalLess
{
  __host__ __device__ constexpr bool operator()(const DeviceOptional<T> & lhs, const DeviceOptional<T> & rhs) const
  {
    return lhs < rhs;
  }
};

}  // namespace autoware::cuda_pointcloud_preprocessor_v49

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DEVICE_OPTIONAL_HPP_


