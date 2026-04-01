# Autoware 诊断链路说明（Word版，纯文字流程图）

版本：基于当前工作区代码整理
目的：回答“/diagnostics 出现 ERROR 后会触发什么逻辑、谁来处理”

------------------------------------------------------------
1. 一句话结论
------------------------------------------------------------

/diagnostics 的 ERROR 不会直接触发停车。
它先进入 diagnostic graph，再影响 /system/operation_mode/availability，
最后由 mrm_handler 决定是否进入 MRM（紧急处置）。

只有命中 graph 配置的诊断项才会影响这条主链。
未命中的 ERROR 会进 /diagnostics_graph/unknowns，通常不影响主控制逻辑。

------------------------------------------------------------
2. 主链路（纯文字图）
------------------------------------------------------------

[模块发布 /diagnostics]
          |
          v
[autoware_diagnostic_graph_aggregator]
    |                         \
    |(status.name 命中)         \(status.name 未命中)
    v                           v
[/diagnostics_graph/status]    [/diagnostics_graph/unknowns]
    |
    +--> [ModesAvailability]
    |         |
    |         v
    |   [/system/operation_mode/availability]
    |         |
    |         v
    |     [mrm_handler]
    |       |     |      \
    |       |     |       \-> 调用 MRM operator（pull_over/comfortable_stop/emergency_stop）
    |       |     v
    |       | [/system/emergency/gear_cmd]
    |       v
    | [/system/fail_safe/mrm_state]
    |
    +--> [hazard_status_converter] --> [/system/emergency/hazard_status]
    |
    +--> [default_adapi diagnostics] --> [/api/system/diagnostics/status]

------------------------------------------------------------
3. 主链路的关键判定
------------------------------------------------------------

1) graph 命中条件：
   - 匹配键是 DiagnosticStatus.status.name
   - 必须与 graph 中的 diag 叶子名称一致

2) mode availability 判定：
   availability.autonomous = (level("/autoware/modes/autonomous") == OK)

3) mrm_handler 紧急判定：
   isEmergency = !operation_mode_availability.autonomous
                 || is_emergency_holding
                 || availability_timeout

4) 默认参数行为（当前默认配置）：
   - use_pull_over = false
   - use_comfortable_stop = false
   => 常见会走 emergency_stop 分支

------------------------------------------------------------
4. localization 场景专用链路
------------------------------------------------------------

/autoware/modes/autonomous 依赖 /autoware/localization。
/autoware/localization 由 localization.yaml 中多项诊断组成。

[例子：EKF 发布 ERROR]
"localization: ekf_localizer" = ERROR
      |
      v
命中 leaf: /autoware/localization/sensor_fusion_status
      |
      v
/autoware/localization 变坏
      |
      v
/autoware/modes/autonomous 变坏
      |
      v
/system/operation_mode/availability.autonomous = false
      |
      v
mrm_handler 判定 emergency -> 进入 MRM

------------------------------------------------------------
5. 为什么有的 ERROR 不触发 MRM
------------------------------------------------------------

常见原因：

1) 名字不匹配 graph
   - 例如发布了 "localization: ekf_localizer: callback_xxx"
   - 但 graph 配的是 "localization: ekf_localizer"
   - 结果：进 unknown，不影响主链

2) 命中节点不在 autonomous 依赖链上

3) 你观察的是状态展示支路，而非控制主链

------------------------------------------------------------
6. 与 localization 初始化接口的区别
------------------------------------------------------------

A. /api/localization/initialization_state
   - 只有 UNKNOWN/UNINITIALIZED/INITIALIZING/INITIALIZED
   - 不是 diagnostics 错误码通道

B. /api/localization/initialize 服务
   - 会返回 ERROR_UNSAFE / ERROR_GNSS / ERROR_ESTIMATION 等
   - 这是“初始化调用结果”错误，直接返回给服务调用者
   - 不等于 /diagnostics 主链路

C. /autoware/state
   - autoware_state 会参考 localization 初始化状态
   - localization 未 INITIALIZED 时，系统状态保持 INITIALIZING

------------------------------------------------------------
7. 另一条常混淆链：component_state_monitor
------------------------------------------------------------

component_state_monitor 也订阅 /diagnostics，
但它主要看 hardware_id == "topic_state_monitor" 的项。
它发布 /system/component_state_monitor/component/launch/*，
供 autoware_state 在启动阶段判断模块就绪。

这条链不等同于 graph -> availability -> MRM 主链。

------------------------------------------------------------
8. 快速排查命令
------------------------------------------------------------

1) 看原始诊断：
   ros2 topic echo /diagnostics

2) 看是否没命中 graph：
   ros2 topic echo /diagnostics_graph/unknowns

3) 看 graph 汇总：
   ros2 topic echo /diagnostics_graph/status

4) 看模式可用性：
   ros2 topic echo /system/operation_mode/availability

5) 看 MRM 状态：
   ros2 topic echo /system/fail_safe/mrm_state

------------------------------------------------------------
9. 核心代码与配置（索引）
------------------------------------------------------------

- system/autoware_diagnostic_graph_aggregator/src/node/aggregator.cpp
- system/autoware_diagnostic_graph_aggregator/src/node/availability.cpp
- system/autoware_mrm_handler/src/mrm_handler/mrm_handler_core.cpp
- system/autoware_hazard_status_converter/src/converter.cpp
- system/autoware_default_adapi/src/diagnostics.cpp
- system/autoware_system_diagnostic_monitor/config/autoware-main.yaml
- system/autoware_system_diagnostic_monitor/config/localization.yaml
- localization/autoware_ekf_localizer/src/ekf_localizer.cpp
- localization/autoware_ndt_scan_matcher/src/ndt_scan_matcher_core.cpp
- localization/autoware_localization_error_monitor/src/localization_error_monitor.cpp
- system/autoware_system_diagnostic_monitor/script/component_state_diagnostics.py
- system/autoware_default_adapi/src/compatibility/autoware_state.cpp

