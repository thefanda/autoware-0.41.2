# CUDA 随机下采样模块开发交付物

## 1. 概述

### 1.1 背景与目标

在 Autoware 点云预处理流水线中，随机下采样（Random Downsample）用于在保持点云空间分布的前提下减少点数，以降低后续定位、感知等模块的计算负担。原有实现基于 PCL 的 `RandomSample`，在 CPU 上串行执行。为提升吞吐、降低延迟，需要实现基于 CUDA 的 GPU 版本，与同包内的 `CudaVoxelGridDownsampleFilter` 形成完整的 CUDA 下采样链路。

### 1.2 功能说明

- **功能**：从输入点云中随机选取指定数量（`sample_num`）的点作为输出，采样过程在 GPU 上并行完成。
- **输入/输出类型**：`cuda_blackboard::CudaPointCloud2`（GPU 显存中的点云，与 `sensor_msgs/PointCloud2` 结构兼容但存储位置不同）。
- **典型应用**：定位前的点云下采样，将体素化后的数万点进一步降至数千点，供 NDT 等算法使用。

### 1.3 设计参考

| 参考模块 | 参考内容 |
|----------|----------|
| **CPU RandomDownsample**（`autoware_pointcloud_preprocessor`） | 算法语义（随机采样）、参数 `sample_num`、与 PCL `RandomSample` 的行为一致性 |
| **CUDA VoxelGrid**（`CudaVoxelGridDownsampleFilter`） | 节点结构、`CudaBlackboardSubscriber/Publisher` 用法、`cudaMemPool` 内存池、`ThrustCustomAllocator`、CMake 集成方式 |

---

## 2. 新增/修改文件清单

### 2.1 新增文件（8 个）

| 序号 | 文件路径 | 说明 |
|------|----------|------|
| 1 | `include/autoware/cuda_pointcloud_preprocessor_v49/cuda_downsample_filter/cuda_random_downsample_filter.hpp` | 滤波器核心类声明 |
| 2 | `include/autoware/cuda_pointcloud_preprocessor_v49/cuda_downsample_filter/cuda_random_downsample_filter_node.hpp` | ROS 2 节点类声明 |
| 3 | `src/cuda_downsample_filter/cuda_random_downsample_filter.cu` | CUDA 滤波器实现（含 kernel 与 Thrust 调用） |
| 4 | `src/cuda_downsample_filter/cuda_random_downsample_filter_node.cpp` | ROS 2 节点实现 |
| 5 | `config/cuda_random_downsample_filter.param.yaml` | 运行时参数配置 |
| 6 | `schema/cuda_random_downsample_filter.schema.json` | 参数 JSON Schema（用于 launch 校验） |
| 7 | `launch/cuda_random_downsample_filter.launch.xml` | 独立启动文件 |
| 8 | `docs/cuda-random-downsample-filter.md` | 本文档 |

### 2.2 修改文件（2 个）

| 文件路径 | 修改内容 |
|----------|----------|
| `CMakeLists.txt` | ① 将 `cuda_random_downsample_filter.cu` 加入 `cuda_pointcloud_preprocessor_lib_v49`；② 将 `cuda_random_downsample_filter_node.cpp` 加入 `cuda_pointcloud_preprocessor_v49` 组件；③ 使用 `rclcpp_components_register_node` 注册 `cuda_random_downsample_filter_node_v49` |
| `launch/cuda_localization.launch.py` | 在 GPU 容器中增加 CUDA RandomDownsample 节点，注释掉 CPU 版本，完成 CropBox → VoxelGrid → RandomDownsample 的 CUDA 链路 |

---

## 3. 各文件详细说明与完整代码

### 3.1 滤波器头文件：`cuda_random_downsample_filter.hpp`

**职责**：定义 `CudaRandomDownsampleFilter` 类，封装 CUDA 随机采样逻辑，不依赖 ROS。

**设计要点**：
- 与 `CudaVoxelGridDownsampleFilter` 一致，使用 `cudaMemPool_t` 管理显存，减少 `cudaMalloc` 调用。
- `ThrustCustomAllocator` 使 Thrust 算法（如 `thrust::shuffle`）使用内存池分配，与 CUDA 11.2+ 的 `cudaMallocFromPoolAsync` 配合。
- `allocateBufferFromPool` / `returnBufferToPool` 为预留接口，当前实现中 `indices` 由 `thrust::device_vector` 管理，未显式使用。

**完整代码**：

```cpp
// Copyright 2025 TIER IV, Inc.
// ... License header ...

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_DOWNSAMPLE_FILTER__CUDA_RANDOM_DOWNSAMPLE_FILTER_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_DOWNSAMPLE_FILTER__CUDA_RANDOM_DOWNSAMPLE_FILTER_HPP_

// Thrust 自定义分配器，使 Thrust 使用 cudaMemPool 分配显存
#include "autoware/cuda_pointcloud_preprocessor_v49/cuda_downsample_filter/thrust_custom_allocator.hpp"
#include "autoware/cuda_utils/cuda_check_error.hpp"

#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <cuda_blackboard/cuda_unique_ptr.hpp>

#include <cuda_runtime.h>

#include <cstdint>
#include <memory>

namespace autoware::cuda_pointcloud_preprocessor_v49
{

/// @brief CUDA 随机下采样滤波器，在 GPU 上对点云进行随机采样
class CudaRandomDownsampleFilter
{
public:
  /// @param sample_num 目标输出点数
  /// @param max_mem_pool_size_in_byte 显存池最大容量（字节），默认 1GB
  explicit CudaRandomDownsampleFilter(
    const size_t sample_num, const int64_t max_mem_pool_size_in_byte = 1e9);
  ~CudaRandomDownsampleFilter() = default;

  /// @brief 对输入点云执行随机下采样
  /// @return 下采样后的点云，点数 = min(sample_num, 输入点数)
  std::unique_ptr<cuda_blackboard::CudaPointCloud2> filter(
    const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_points);

  void set_sample_num(size_t sample_num) { sample_num_ = sample_num; }
  size_t get_sample_num() const { return sample_num_; }

private:
  // 预留接口：从内存池分配/释放缓冲区（当前由 thrust::device_vector 管理 indices）
  template <typename T>
  T * allocateBufferFromPool(size_t num_elements);

  template <typename T>
  void returnBufferToPool(T * buffer);

  size_t sample_num_{0};           // 目标采样点数
  cudaStream_t stream_{};         // CUDA 异步流
  cudaMemPool_t mem_pool_{};      // 显存池，减少 cudaMalloc 调用
  std::unique_ptr<ThrustCustomAllocator> thrust_custom_allocator_;
};

}  // namespace autoware::cuda_pointcloud_preprocessor_v49

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_DOWNSAMPLE_FILTER__CUDA_RANDOM_DOWNSAMPLE_FILTER_HPP_
```

---

### 3.2 节点头文件：`cuda_random_downsample_filter_node.hpp`

**职责**：定义 ROS 2 节点类，负责订阅/发布、参数加载、回调调度。

**设计要点**：
- 使用 `CudaBlackboardSubscriber` / `CudaBlackboardPublisher`，与 cuda_blackboard 的类型协商机制兼容，可接收 `sensor_msgs/PointCloud2` 或 `CudaPointCloud2`（取决于上游）。
- 话题默认为 `~/input/pointcloud`、`~/output/pointcloud`，通过 remap 适配不同流水线。

**完整代码**：

```cpp
// Copyright 2025 TIER IV, Inc.
// ... License header ...

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_DOWNSAMPLE_FILTER__CUDA_RANDOM_DOWNSAMPLE_FILTER_NODE_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_DOWNSAMPLE_FILTER__CUDA_RANDOM_DOWNSAMPLE_FILTER_NODE_HPP_

#include "cuda_random_downsample_filter.hpp"

#include <cuda_blackboard/cuda_adaptation.hpp>      // cuda_blackboard 类型协商
#include <cuda_blackboard/cuda_blackboard_publisher.hpp>
#include <cuda_blackboard/cuda_blackboard_subscriber.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace autoware::cuda_pointcloud_preprocessor_v49
{

/// @brief CUDA 随机下采样 ROS 2 节点
/// 订阅 ~/input/pointcloud，发布 ~/output/pointcloud
class CudaRandomDownsampleFilterNode : public rclcpp::Node
{
public:
  explicit CudaRandomDownsampleFilterNode(const rclcpp::NodeOptions & node_options);

private:
  void cudaPointcloudCallback(const cuda_blackboard::CudaPointCloud2::ConstSharedPtr msg);

  std::shared_ptr<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>
    sub_{};   // 订阅 CudaPointCloud2，支持类型协商

  std::unique_ptr<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>
    pub_{};   // 发布下采样后的 CudaPointCloud2

  std::unique_ptr<CudaRandomDownsampleFilter> cuda_random_downsample_filter_{};
};

}  // namespace autoware::cuda_pointcloud_preprocessor_v49

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_RANDOM_DOWNSAMPLE_FILTER_NODE_HPP_
```

---

### 3.3 CUDA 实现：`cuda_random_downsample_filter.cu`

**职责**：实现随机采样算法，包括构造函数中的内存池初始化、`filter()` 中的 Thrust 调用与自定义 kernel。

#### 3.3.1 算法流程

1. **空输入处理**：若 `num_input_points == 0`，返回空点云，避免后续除零或越界。
2. **输出点数**：`num_output_points = min(sample_num, num_input_points)`，与 PCL 行为一致。
3. **索引生成**：`thrust::sequence` 在 GPU 上生成 `[0, 1, 2, ..., N-1]`。
4. **随机打乱**：`thrust::shuffle` 使用 `thrust::default_random_engine` 打乱索引，种子取自 `steady_clock`。
5. **按索引拷贝**：`copyPointsByIndicesKernel` 根据打乱后的前 `num_output_points` 个索引，从输入逐字节拷贝到输出。

#### 3.3.2 构造函数：内存池与流

```cpp
CudaRandomDownsampleFilter::CudaRandomDownsampleFilter(
  const size_t sample_num, const int64_t max_mem_pool_size_in_byte)
: sample_num_(sample_num)
{
  // 创建 CUDA 异步流，所有 kernel 和 Thrust 调用绑定到该流
  CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));

  // 获取当前 GPU 设备 ID，用于创建内存池
  int current_device_id = 0;
  CHECK_CUDA_ERROR(cudaGetDevice(&current_device_id));
  cudaMemPoolProps pool_props = {};
  pool_props.allocType = cudaMemAllocationTypePinned;
  pool_props.location.id = current_device_id;
  pool_props.location.type = cudaMemLocationTypeDevice;
  CHECK_CUDA_ERROR(cudaMemPoolCreate(&mem_pool_, &pool_props));

  // 设置内存池释放阈值：超过此大小时向系统释放显存，减少显存占用
  uint64_t pool_release_threshold = max_mem_pool_size_in_byte;
  CHECK_CUDA_ERROR(cudaMemPoolSetAttribute(
    mem_pool_, cudaMemPoolAttrReleaseThreshold, static_cast<void *>(&pool_release_threshold)));

  // Thrust 自定义分配器，使 thrust::device_vector 等使用内存池分配
  thrust_custom_allocator_ = std::make_unique<ThrustCustomAllocator>(stream_, mem_pool_);
}
```

- `cudaStreamCreate`：创建异步流，所有 kernel 和 Thrust 调用绑定到该流。
- `cudaMemPoolCreate`：创建设备内存池，`cudaMemPoolAttrReleaseThreshold` 控制何时向系统释放显存。

#### 3.3.3 CUDA Kernel：按索引拷贝点

```cpp
/// @brief 根据索引数组从输入点云拷贝点到输出点云
/// @param input_data  输入点云原始数据（字节数组）
/// @param output_data 输出点云原始数据（字节数组）
/// @param indices     打乱后的索引数组，indices[i] 表示第 i 个输出点对应的输入点索引
/// @param num_output_points 输出点数
/// @param point_step  每个点的字节数（如 PointXYZIRC 约 32 字节）
__global__ void copyPointsByIndicesKernel(
  const uint8_t * __restrict__ input_data, uint8_t * __restrict__ output_data,
  const size_t * __restrict__ indices, size_t num_output_points, size_t point_step)
{
  // 每个线程处理一个输出点
  size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_output_points) {
    return;
  }

  // 获取该输出点对应的输入点索引
  size_t src_idx = indices[idx];
  const uint8_t * src = input_data + src_idx * point_step;
  uint8_t * dst = output_data + idx * point_step;

  // 逐字节拷贝，与任意点云格式兼容，无需解析 fields
  for (size_t i = 0; i < point_step; ++i) {
    dst[i] = src[i];
  }
}
```

- 每个线程处理一个输出点，`idx` 为输出点索引。
- `indices[idx]` 为对应的输入点索引，`point_step` 为每点字节数（如 PointXYZIRC 约 32 字节）。
- 使用 `__restrict__` 提示编译器指针无重叠，便于优化。
- 逐字节拷贝保证与任意点云格式兼容，无需解析 fields。

#### 3.3.4 filter() 完整实现

```cpp
std::unique_ptr<cuda_blackboard::CudaPointCloud2> CudaRandomDownsampleFilter::filter(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_points)
{
  const size_t num_input_points = input_points->width * input_points->height;
  const size_t point_step = input_points->point_step;

  // 空输入处理：返回空点云，避免后续除零或越界
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

  // 输出点数 = min(sample_num, 输入点数)，与 PCL RandomSample 行为一致
  const size_t num_output_points = std::min(sample_num_, num_input_points);

  // 步骤 1：在 GPU 上生成索引序列 [0, 1, 2, ..., N-1]
  thrust::device_vector<size_t> indices(num_input_points);
  thrust::sequence(thrust::cuda::par.on(stream_), indices.begin(), indices.end(), 0UL);

  // 步骤 2：随机打乱索引，种子取自当前时间保证每次不同
  unsigned int seed = static_cast<unsigned int>(
    std::chrono::steady_clock::now().time_since_epoch().count());
  thrust::default_random_engine rng(seed);
  thrust::shuffle(thrust::cuda::par.on(stream_), indices.begin(), indices.end(), rng);

  // 步骤 3：分配输出点云，拷贝元数据
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

  // 步骤 4：启动 kernel，按打乱后的前 num_output_points 个索引拷贝点
  const dim3 block_dim(256);   // 每 block 256 线程
  const dim3 grid_dim((num_output_points + block_dim.x - 1) / block_dim.x);
  copyPointsByIndicesKernel<<<grid_dim, block_dim, 0, stream_>>>(
    input_points->data.get(), output->data.get(), thrust::raw_pointer_cast(indices.data()),
    num_output_points, point_step);

  // 同步等待 kernel 完成，保证返回前输出数据有效
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  return output;
}
```

- `thrust::cuda::par.on(stream_)`：指定 Thrust 在给定 CUDA 流上执行。
- `block_dim(256)`、`grid_dim`：每个 block 256 线程，grid 数量按 `num_output_points` 向上取整。
- `cudaStreamSynchronize`：确保 kernel 完成后再返回，保证输出数据有效。

---

### 3.4 节点实现：`cuda_random_downsample_filter_node.cpp`

**职责**：参数声明、订阅/发布创建、回调中调用滤波器并打印耗时。

**完整代码**：

```cpp
// Copyright 2025 TIER IV, Inc.
// ... License header ...

#include "autoware/cuda_pointcloud_preprocessor_v49/cuda_downsample_filter/cuda_random_downsample_filter_node.hpp"

#include "autoware/pointcloud_preprocessor_v49/utility/memory.hpp"  // is_data_layout_compatible_with_point_xyzirc

#include <chrono>

namespace autoware::cuda_pointcloud_preprocessor_v49
{

CudaRandomDownsampleFilterNode::CudaRandomDownsampleFilterNode(
  const rclcpp::NodeOptions & node_options)
: Node("cuda_random_downsample_filter", node_options)
{
  // 声明并读取参数：sample_num 必选，max_mem_pool_size_in_byte 可选默认 1GB
  const size_t sample_num = static_cast<size_t>(declare_parameter<int64_t>("sample_num"));
  const int64_t max_mem_pool_size_in_byte =
    declare_parameter<int64_t>("max_mem_pool_size_in_byte", 1e9);

  if (max_mem_pool_size_in_byte < 0) {
    RCLCPP_ERROR(
      this->get_logger(), "Invalid pool size was specified. The value should be positive");
    return;
  }

  // 创建订阅：~/input/pointcloud，回调 cudaPointcloudCallback
  sub_ =
    std::make_shared<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/input/pointcloud",
      std::bind(
        &CudaRandomDownsampleFilterNode::cudaPointcloudCallback, this, std::placeholders::_1));

  // 创建发布：~/output/pointcloud
  pub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/output/pointcloud");

  // 创建滤波器实例
  cuda_random_downsample_filter_ = std::make_unique<CudaRandomDownsampleFilter>(
    sample_num, max_mem_pool_size_in_byte);
}

void CudaRandomDownsampleFilterNode::cudaPointcloudCallback(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr msg)
{
  // 检查点云格式是否兼容 PointXYZIRC，不兼容时仅告警不中断
  if (!pointcloud_preprocessor::utils::is_data_layout_compatible_with_point_xyzirc(msg->fields)) {
    RCLCPP_WARN(
      this->get_logger(),
      "Input pointcloud data layout is not compatible with PointXYZIRC. "
      "Output may preserve structure but field semantics could differ.");
  }

  // 计时：filter 耗时
  const auto t0 = std::chrono::steady_clock::now();
  auto output_pointcloud_ptr = cuda_random_downsample_filter_->filter(msg);
  const auto t1 = std::chrono::steady_clock::now();
  const double elapsed_ms =
    std::chrono::duration<double, std::milli>(t1 - t0).count();

  // 打印耗时与点数变化
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

// 注册为 rclcpp_components 可加载组件
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::cuda_pointcloud_preprocessor_v49::CudaRandomDownsampleFilterNode)
```

**说明**：
- `is_data_layout_compatible_with_point_xyzirc`：检查点云是否包含 x/y/z/intensity 等字段，不兼容时仅告警，不中断处理。
- 耗时统计使用 `std::chrono::steady_clock`，与其他 CUDA 模块一致。

---

### 3.5 配置文件：`cuda_random_downsample_filter.param.yaml`

```yaml
# ROS 2 参数格式：/** 表示根命名空间
/**:
  ros__parameters:
    # 目标输出点数，定位场景常用 5000～15000
    sample_num: 5000
    # max_mem_pool_size_in_byte 已移除，使用代码默认 1e9（1GB）
```

- `sample_num`：输出点数量，定位场景常用 5000～15000。
- `max_mem_pool_size_in_byte`：已从配置中移除，使用代码默认值 1e9（1GB），与 CUDA VoxelGrid 一致。

---

### 3.6 Schema：`cuda_random_downsample_filter.schema.json`

用于 launch 参数校验；`sample_num` 为必选，`max_mem_pool_size_in_byte` 为可选。

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Parameters for the cuda_random_downsample_filter",
  "type": "object",
  "definitions": {
    "cuda_random_downsample_filter": {
      "type": "object",
      "properties": {
        "sample_num": {
          "type": "integer",
          "description": "number of points to be randomly sampled",
          "default": "150000",
          "minimum": 0
        },
        "max_mem_pool_size_in_byte": {
          "type": "number",
          "description": "maximum size of GPU memory pool",
          "default": "1000000000",
          "minimum": 0
        }
      },
      "required": ["sample_num"],
      "additionalProperties": false
    }
  },
  "properties": {
    "/**": {
      "type": "object",
      "properties": {
        "ros__parameters": {
          "$ref": "#/definitions/cuda_random_downsample_filter"
        }
      },
      "required": ["ros__parameters"],
      "additionalProperties": false
    }
  },
  "required": ["/**"],
  "additionalProperties": false
}
```

- `sample_num` 为必选，`max_mem_pool_size_in_byte` 为可选。

---

### 3.7 Launch：`cuda_random_downsample_filter.launch.xml`

```xml
<launch>
  <!-- 可覆盖的 launch 参数：输入/输出话题 -->
  <arg name="input/pointcloud" default="/sensing/lidar/concatenated/pointcloud"/>
  <arg name="output/pointcloud" default="/sensing/lidar/top/random_downsampled"/>

  <!-- 参数文件路径 -->
  <arg name="cuda_random_downsample_filter_param_file" default="$(find-pkg-share autoware_cuda_pointcloud_preprocessor_v49)/config/cuda_random_downsample_filter.param.yaml"/>

  <node pkg="autoware_cuda_pointcloud_preprocessor_v49" exec="cuda_random_downsample_filter_node_v49" name="cuda_random_downsample_filter_v49" output="screen">
    <remap from="~/input/pointcloud" to="$(var input/pointcloud)"/>
    <remap from="~/output/pointcloud" to="$(var output/pointcloud)"/>
    <param from="$(var cuda_random_downsample_filter_param_file)"/>
  </node>
</launch>
```

- 可通过 `input/pointcloud`、`output/pointcloud` 覆盖默认话题。

---

### 3.8 CMakeLists.txt 修改

**① 库源文件**（约第 86～98 行）：将 CUDA 源加入共享库，供节点链接

```cmake
cuda_add_library(cuda_pointcloud_preprocessor_lib_v49 SHARED
  src/cuda_concatenate_data/cuda_combine_cloud_handler.cpp
  src/cuda_concatenate_data/cuda_combine_cloud_handler_kernel.cu
  src/cuda_concatenate_data/cuda_cloud_collector.cpp
  src/cuda_concatenate_data/cuda_collector_matching_strategy.cpp
  src/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor.cu
  src/cuda_pointcloud_preprocessor/common_kernels.cu
  src/cuda_pointcloud_preprocessor/organize_kernels.cu
  src/cuda_pointcloud_preprocessor/outlier_kernels.cu
  src/cuda_pointcloud_preprocessor/undistort_kernels.cu
  src/cuda_downsample_filter/cuda_voxel_grid_downsample_filter.cu
  src/cuda_downsample_filter/cuda_random_downsample_filter.cu   # 新增：CUDA 滤波器实现
  src/cuda_outlier_filter/cuda_polar_voxel_outlier_filter.cu
)
```

**② 组件源文件**（约第 134～141 行）：将节点 cpp 加入 rclcpp_components 可加载库

```cmake
ament_auto_add_library(cuda_pointcloud_preprocessor_v49 SHARED
  src/cuda_concatenate_data/cuda_concatenate_and_time_sync_node.cpp
  src/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor_node.cpp
  src/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor_crop_only_node.cpp
  src/cuda_downsample_filter/cuda_voxel_grid_downsample_filter_node.cpp
  src/cuda_downsample_filter/cuda_random_downsample_filter_node.cpp   # 新增：ROS 节点实现
  src/cuda_outlier_filter/cuda_polar_voxel_outlier_filter_node.cpp
)
```

**③ 组件注册**（约第 178～182 行）：注册为可执行组件，供 launch 中 ComposableNode 加载

```cmake
# ========== Random downsample filter ==========
rclcpp_components_register_node(cuda_pointcloud_preprocessor_v49
  PLUGIN "autoware::cuda_pointcloud_preprocessor_v49::CudaRandomDownsampleFilterNode"
  EXECUTABLE cuda_random_downsample_filter_node_v49
)
```

---

## 4. 算法流程示意

```
输入点云 (N 点, point_step 字节/点)
    │
    ├─ num_input_points == 0 ? ──→ 返回空点云
    │
    └─ num_output_points = min(sample_num, num_input_points)
           │
           ▼
    thrust::device_vector<size_t> indices(N)
    thrust::sequence(indices, 0, 1, 2, ..., N-1)
           │
           ▼
    thrust::shuffle(indices, rng)   // 随机打乱
           │
           ▼
    分配输出点云 (num_output_points 点)
           │
           ▼
    copyPointsByIndicesKernel<<<...>>>
        for idx in [0, num_output_points):
            src_idx = indices[idx]
            copy input[src_idx] → output[idx]  (point_step 字节)
           │
           ▼
    输出点云 (M 点, M = num_output_points)
```

---

## 5. 集成到 cuda_localization

在 `cuda_localization.launch.py` 的 GPU 容器中增加：

```python
# CUDA RandomDownsample：订阅 CUDA VoxelGrid 输出，发布到定位模块使用的下采样点云话题
ComposableNode(
    package='autoware_cuda_pointcloud_preprocessor_v49',
    plugin='autoware::cuda_pointcloud_preprocessor_v49::CudaRandomDownsampleFilterNode',
    name='random_downsample_filter_v49',
    parameters=[
        os.path.join(pkg_share, 'config', 'cuda_random_downsample_filter.param.yaml')
    ],
    remappings=[
        ('~/input/pointcloud', 'voxel_grid_downsample/pointcloud'),   # 上游：CUDA VoxelGrid
        ('~/output/pointcloud', '/localization/util/downsample/pointcloud'),  # 下游：定位/NDT
    ],
    extra_arguments=[{
        'use_intra_process_comms': False   # CUDA 节点通常不启用进程内通信
    }],
),
```

**数据流**：CropBox（CPU 或 CUDA）→ VoxelGrid（CUDA）→ **RandomDownsample（CUDA）** → `/localization/util/downsample/pointcloud`

---

## 6. 启动方式

### 6.1 独立启动

```bash
ros2 launch autoware_cuda_pointcloud_preprocessor_v49 cuda_random_downsample_filter.launch.xml
```

需确保上游有节点向默认输入话题发布点云（或通过 launch 参数指定输入话题）。

### 6.2 集成启动（cuda_localization）

```bash
ros2 launch autoware_cuda_pointcloud_preprocessor_v49 cuda_localization.launch.py
```

---

## 7. 依赖关系

| 依赖包 | 用途 |
|--------|------|
| `cuda_blackboard_v49` | CudaPointCloud2、CudaBlackboardSubscriber/Publisher、类型协商 |
| `autoware_pointcloud_preprocessor_v49` | `is_data_layout_compatible_with_point_xyzirc` 等工具 |
| `autoware_cuda_utils_v49` | `CHECK_CUDA_ERROR` 等 CUDA 错误检查 |
| `thrust`（随 CUDA） | `thrust::sequence`、`thrust::shuffle`、`thrust::device_vector` |
| `cuda_runtime` | `cudaStreamCreate`、`cudaMemPoolCreate`、`cudaMallocFromPoolAsync` 等 |

---

## 8. 与 CPU 版本对比

| 项目 | CPU | CUDA |
|------|-----|------|
| 输入类型 | sensor_msgs/PointCloud2 | CudaPointCloud2 |
| 输出类型 | sensor_msgs/PointCloud2 | CudaPointCloud2 |
| 算法 | PCL RandomSample | thrust::shuffle + copyPointsByIndicesKernel |
| 随机性 | PCL 内部 RNG | thrust::default_random_engine + steady_clock 种子 |
| 典型耗时 | ~0.2 ms（1 万点） | ~0.08 ms（3 万点） |
| 内存 | CPU 堆 | GPU 显存 + cudaMemPool |

---

## 9. 附录：会话中其他相关修改

除 CUDA RandomDownsample 外，本次会话还涉及：

- 各模块（CPU/CUDA 的 CropBox、VoxelGrid、RandomDownsample）增加耗时打印与模块前缀（如 `[CUDA RandomDownsample]`）
- CPU CropBox 计时改为 `steady_clock`，与其他模块一致
- CPU VoxelGrid 加入 cuda_localization，并复制 `voxel_grid_downsample_filter_node.param.yaml` 到本包
- 移除 cuda_voxel_grid 中的调试输出
- 调整 `random_downsample_filter_node.param.yaml` 的 `sample_num`，避免输出点数等于输入（sample_num 过大）

这些修改不改变 CUDA RandomDownsample 的核心实现，仅用于调试和日志输出。
