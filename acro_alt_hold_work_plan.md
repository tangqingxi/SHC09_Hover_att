# Acro 定高模式工作计划

这份文档用于整理 `acro-alt-hold` 模式的代码阅读顺序和实现路线。目标是：垂向保持定高闭环，横滚、俯仰、偏航摇杆输入直接跟踪角速度指令。

## 目标行为

```text
roll/pitch/yaw 摇杆 -> 角速度 setpoint
throttle 摇杆       -> 高度 / 爬升率控制
高度控制器          -> thrust setpoint
角速度控制器        -> torque setpoint
控制分配            -> 电机 / 舵面输出
```

这是 PX4 Acro 模式和 Altitude 模式的混合：

- Acro 模式：横滚、俯仰、偏航摇杆映射到机体系角速度 setpoint。
- Altitude 模式：垂向是闭环控制，但横滚、俯仰通常映射到姿态 / 倾角 setpoint。

我们想要的模式应该保留 Altitude 模式的高度控制链路，同时把横滚、俯仰、偏航的姿态指令链路替换成角速度指令链路。

## 建议先看的 PX4 官方文档

- Acro Mode: https://docs.px4.io/main/en/flight_modes_mc/acro
- Altitude Mode: https://docs.px4.io/main/en/flight_modes_mc/altitude
- VehicleControlMode flags: https://docs.px4.io/main/en/msg_docs/VehicleControlMode
- Multicopter controller diagrams: https://docs.px4.io/main/en/flight_stack/controller_diagrams
- Control allocation: https://docs.px4.io/main/en/concept/control_allocation

## 代码阅读顺序

### 1. 模式控制 flag

先看：

```text
src/modules/commander/ModeUtil/control_mode.cpp
```

重点看 `NAVIGATION_STATE_ALTCTL` 分支。这个分支会打开：

- `flag_control_manual_enabled`
- `flag_control_altitude_enabled`
- `flag_control_climb_rate_enabled`
- `flag_control_attitude_enabled`
- `flag_control_rates_enabled`
- `flag_control_allocation_enabled`

对 acro 定高来说，关键问题是：是否要关闭姿态控制，或者在保留定高和角速度控制的同时绕过姿态 setpoint 生成。

### 2. 模式需要哪些估计量

然后看：

```text
src/modules/commander/ModeUtil/mode_requirements.cpp
```

`NAVIGATION_STATE_ALTCTL` 的模式要求包括：

- 角速度估计
- 姿态估计
- 本地高度估计
- 手动控制输入

这些大概率也是 acro 定高模式应该继承的安全要求。

### 3. FlightTask 选择逻辑

然后看：

```text
src/modules/flight_mode_manager/FlightModeManager.cpp
```

找到 manual altitude control 相关逻辑。`NAVIGATION_STATE_ALTCTL` 会切换到下面的任务之一：

```text
src/modules/flight_mode_manager/tasks/ManualAltitude/
src/modules/flight_mode_manager/tasks/ManualAltitudeSmoothVel/
```

这些任务负责生成高度 / 垂向 setpoint。acro 定高应该尽量保留这条垂向控制链路。

### 4. Acro 摇杆到角速度 setpoint 的现有逻辑

导管风扇已有的 acro-rate 逻辑在：

```text
src/modules/df_hover_rate_control/DfHoverRateControl.cpp
src/modules/df_hover_rate_control/DfHoverRateControl.hpp
src/modules/df_hover_rate_control/module.yaml
```

重点看 `DfHoverRateControl.cpp` 里的逻辑：手动摇杆输入经过 `math::superexpo(...)`，再乘以 `_acro_rate_max`，最后发布为 `vehicle_rates_setpoint`。

相关参数在 `module.yaml` 中定义：

- `DF_ACRO_R_MAX`
- `DF_ACRO_P_MAX`
- `DF_ACRO_Y_MAX`
- `DF_ACRO_EXPO`
- `DF_ACRO_EXPO_Y`
- `DF_ACRO_SUPEXPO`
- `DF_ACRO_SUPEXPOY`

这部分是实现 acro 定高时最值得复用的角速度指令生成逻辑。

### 5. 普通 MC 姿态到角速度的路径

作为对照，再看：

```text
src/modules/mc_att_control/mc_att_control_main.cpp
src/modules/mc_att_control/mc_att_control.hpp
src/modules/mc_att_control/mc_att_control_params.yaml
```

普通 Altitude 模式会使用姿态 setpoint。在 `mc_att_control_main.cpp` 中重点看：

- `generate_attitude_setpoint(...)`
- 发布 `vehicle_rates_setpoint` 的代码块

这里展示了 PX4 正常情况下如何从姿态误差生成角速度 setpoint。acro 定高要避免由摇杆生成横滚 / 俯仰姿态 setpoint，而是直接提供机体系角速度 setpoint。

### 6. 高度控制器输出

垂向控制输出相关代码：

```text
src/modules/mc_pos_control/MulticopterPositionControl.cpp
src/modules/mc_pos_control/MulticopterPositionControl.hpp
src/modules/mc_pos_control/multicopter_altitude_mode_params.yaml
src/modules/mc_pos_control/multicopter_position_control_params.yaml
```

目标是保留高度控制器生成的 thrust setpoint，并把它和手动生成的角速度 setpoint 组合起来。

### 7. 控制分配

等 setpoint 链路理清楚之后，再看：

```text
src/modules/control_allocator/ControlAllocator.cpp
src/modules/control_allocator/module.yaml
src/modules/control_allocator/VehicleActuatorEffectiveness/
```

控制分配不是这个任务最先应该改的地方。它负责接收期望力矩 / 推力，再映射到电机或舵面。acro 定高的主要工作在上游：正确生成角速度 setpoint 和 thrust setpoint。

## 推荐实现路线

### 方案 A：小步实验参数开关

这是建议优先做的 SITL 验证方案。

可以增加一个参数，例如：

```text
DF_ALT_ACRO_EN
```

当这个参数在 Altitude 模式下启用时：

1. 保持 `NAVIGATION_STATE_ALTCTL` 不变。
2. 保留现有高度 FlightTask。
3. 用已有 acro-rate 映射逻辑，根据手动摇杆生成 roll、pitch、yaw rate setpoint。
4. thrust 仍然来自高度控制器。
5. 发布 `vehicle_rates_setpoint`，其中：
   - roll / pitch / yaw 来自 acro 摇杆映射
   - thrust 来自高度控制

这条路线改动最小，适合先回答核心飞行问题：飞机能不能在闭环定高的同时跟踪角速度指令。

### 方案 B：正式新增内部模式

方案 A 飞通之后，再考虑这个方案。

可以新增一个内部模式，例如：

```text
ACRO_ALTCTL
```

预期控制 flag 类似：

```text
manual: true
altitude: true
climb_rate: true
attitude: false 或显式绕过
rates: true
allocation: true
```

这个方案架构上更干净，但工作量更大，需要处理：

- 新增或复用 navigation state
- 更新 commander 模式选择
- 更新模式 requirements
- 更新 flight_mode_manager 的 task 选择
- 必要时更新模式显示 / MAVLink 映射
- 添加 SITL 验证

## SITL 验证清单

建议在 SITL 中观察或记录这些 topic：

```text
manual_control_setpoint
vehicle_control_mode
vehicle_local_position
vehicle_attitude
vehicle_attitude_setpoint
vehicle_rates_setpoint
vehicle_thrust_setpoint
actuator_motors
actuator_servos
```

重点检查这些行为：

- throttle 居中时能够保持高度。
- throttle 上下拨动时对应上升 / 下降速度。
- roll 摇杆改变 `vehicle_rates_setpoint.roll`。
- pitch 摇杆改变 `vehicle_rates_setpoint.pitch`。
- yaw 摇杆改变 `vehicle_rates_setpoint.yaw`。
- roll / pitch 摇杆不再生成普通倾角姿态 setpoint。
- `vehicle_rates_setpoint.thrust_body` 仍然来自高度控制。
- 控制分配输出没有明显饱和或异常跳变。

## 第一个实际里程碑

第一个目标应该尽量窄：

> 在 `gazebo-classic_SHC09` 或 `gazebo-classic_SHC09_d` 中，以 Altitude 模式飞行，横滚 / 俯仰 / 偏航输入跟踪 acro 角速度，同时保持高度闭环。

在日志和 SITL 飞行中确认这个行为之前，不建议先大规模调参，也不建议立刻做正式新模式。
