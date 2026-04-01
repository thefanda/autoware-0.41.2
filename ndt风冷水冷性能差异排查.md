# 排查文档（系统层）

## 1. 背景
1. 现象：同一套 Autoware 与同一 Docker 编译方式下，`ndt` 单帧耗时差异明显。
2. 对比：水冷控制器约 `20ms`，风冷控制器约 `300ms`。
3. 本次排查范围：仅系统层，不讨论代码和业务参数。

## 2. 关键对象
1. 可执行文件：[autoware_ndt_scan_matcher_node](/data/autoware/build/autoware_ndt_scan_matcher/autoware_ndt_scan_matcher_node)
2. 主库：[libautoware_ndt_scan_matcher.so](/data/autoware/build/autoware_ndt_scan_matcher/libautoware_ndt_scan_matcher.so)
3. OMP库：[libmultigrid_ndt_omp.so](/data/autoware/build/autoware_ndt_scan_matcher/libmultigrid_ndt_omp.so)
4. 功耗配置文件：[nvpmodel.conf](/etc/nvpmodel/nvpmodel.conf)

## 3. 已完成排查与结果
| 项目 | 风冷 | 水冷 | 结论 |
|---|---|---|---|
| 运行二进制路径 | `/data/autoware/build/.../autoware_ndt_scan_matcher_node` | 相同 | 运行对象一致 |
| 三个核心文件 SHA256 | `142458...`,`f3fe1b...`,`e986ee...` | 相同 | 编译产物一致 |
| 运行时 maps 关键库 | `libgomp/libtbb/libpcl_*` 路径一致 | 一致 | 关键库加载路径一致 |
| CPU governor / 频率 | `performance` / `2009600` | `performance` / `2009600` | 未见明显降频 |
| CPU 在线核 | `0-9` (10核) | `0-11` (12核) | 核数不同但非主因 |
| EMC 频率 | `3199000000` / `max 3199000000` | 相同 | 内存频率未见被压制 |
| 温度（tegrastats） | CPU约 `41C` | 未见异常证据 | 非热限频场景 |
| 线程负载（pidstat） | 总CPU约 `259%~271%` | 未给同口径数据 | 非单线程，约2-3核有效负载 |
| 内核版本 | `5.15.163-rt-tegra` | `5.10.120-rt70-tegra` | 系统栈不一致 |
| nvpmodel状态 | `nvpmodel -q` 解析失败 | 配置同样老模板风格 | 功耗配置体系异常/不标准 |
| `/etc/nv_tegra_release` | 不存在 | 不存在 | 系统发行信息不标准 |

## 4. 已排除项
1. 非“代码差异”导致：核心二进制和库哈希一致。
2. 非“明显CPU降频/热限频”导致：governor、CPU频率、EMC、温度均正常。
3. 非“运行错库路径”导致：`/proc/<pid>/maps` 关键库路径一致。

## 5. 当前高概率原因（系统层）
1. 两台控制器底层系统栈不一致（内核/BSP/RT补丁/驱动），导致每次迭代计算成本不同。
2. `nvpmodel` 体系异常，配置文件模板与平台代际不匹配（`CPU_A57/CPU_DENVER` 风格），存在系统镜像历史遗留问题。
3. NDT实际并行效率仅约2-3核，`10核 vs 12核` 不足以解释 `20ms vs 300ms`，主因更可能是“单核吞吐 + 系统调度/内存路径”。

## 6. 原理说明（简版）
1. NDT耗时核心是：`迭代次数 × 每次迭代成本`。
2. 在点数、二进制、表面频率都接近时，差距通常来自系统层的“每次迭代成本”。
3. 每次迭代成本受内核调度、RT补丁、驱动栈、缓存与内存访问路径影响明显。
4. 因此系统镜像/BSP不一致时，出现数量级差距是可能的。

## 7. 建议下一步（只做系统层）
1. 统一两台控制器到同一系统镜像（内核+BSP+驱动+rootfs），再复测 NDT。
2. 导出并比对系统包清单：`dpkg-query -W -f='${Package} ${Version}\n' | sort`。
3. 同口径抓 `perf stat`（cycles/instructions/cache-misses/context-switches/cpu-migrations）做风冷/水冷对比。
4. 修复 `nvpmodel` 配置一致性，再复测。

如果你要，我可以下一版直接给你一份“可执行的系统对比脚本（一次跑完输出报告）”。
