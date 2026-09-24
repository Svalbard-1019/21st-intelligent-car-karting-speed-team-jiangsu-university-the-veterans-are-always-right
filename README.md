# 21 届智能车卡丁快跑组：科目一、科目二与代码说明

## 项目概览

本工程运行在 Infineon AURIX TC264 卡丁车平台上，使用 C 语言开发。核心任务由后轮编码器、IMU 航向角、前轮舵机和后轮差速电机共同完成。

代码的主执行入口是 `user/cpu0_main.c` 的 `core0_main()`。系统上电后读取 Flash 中保存的参数和路线，随后由 IPS200 菜单选择记录、科目一自动驾驶或科目二模式。

## 科目一：惯导路线自动驾驶

### 功能

科目一先由人工推车记录路线，再让车辆按照保存的惯导路线自动行驶。路线由位置 `(x, y)`、航向 `theta` 和路线点索引组成，单位分别是米、米和度。

科目一支持：

- 按后轮编码器和 IMU 航向角记录路线点。
- 将路线和停车点保存到 Flash。
- 使用纯追踪算法计算目标转向角。
- 使用速度规划器根据直线、弯道、累计转角和横向误差调整目标速度。
- 在指定停车点停车，并执行教学倒车路线。
- 通过 GPS 辅助停车和串口日志检查运行状态。

### 运行流程

```text
记录路线
  -> guandao_recode()
  -> update_state() / recode_waypoint()
  -> Flash 保存
  -> 菜单选择科目一
  -> portion_1()
  -> guandao_trace(&INS)
  -> pursuit_contral_mode()
  -> out_servo + out_v_l/out_v_r
  -> 前轮舵机和后轮电机
```

### 关键代码

| 文件 | 作用 |
| --- | --- |
| `user/cpu0_main.c` | 主循环；`Guandao_Recode_Mode` 调用记录，`Guandao_portion_1` 调用 `portion_1()`。 |
| `code/guandao.c` | 路线记录、位姿更新、纯追踪、速度规划、停车和倒车状态机。 |
| `code/guandao.h` | 路线结构体、容量、轮距、轴距和公开接口。 |
| `code/flash.c` / `code/flash.h` | 保存和读取科目一路线、科目三路线、停车点及参数。 |
| `code/IMU.c` / `code/IMU.h` | IMU 初始化、陀螺仪数据和航向角。 |
| `code/angle_control.c` | 前轮舵机角度控制和不同科目的转向参数。 |
| `code/rear_motor/rear_motor.c` | 后轮 PID、差速前馈、PWM 限幅和里程采样。 |
| `code/portion1_precoast.h` | 科目一进入停车区前的渐进减速。 |
| `code/portion3_reverse_tracker.h` | 独立的科目三倒车追踪模块，本说明不展开。 |

### 主要入口和数据

- `guandao_recode(&INS)`：记录主路线。
- `portion_1()`：科目一自动驾驶入口。
- `guandao_trace(&INS)`：根据 `route_setting_choice` 选择路线并更新车辆状态。
- `pursuit_contral_mode()`：计算目标点、预瞄、转向和左右轮速度。
- `update_state()`：使用编码器增量和同一时刻的 IMU 航向更新车辆位姿。
- `out_servo`：前轮舵机目标角度。
- `out_v_l`、`out_v_r`：左右后轮目标速度，随后由后轮驱动模块换算成电机控制量。

### 菜单和参数

科目一在 `code/display.c` 的 `Menu_Mode_Choice()` 中选择，选择后会执行：

```c
main_mode = Guandao_portion_1;
route_setting_choice = 0;
angle_control_select_route(route_setting_choice);
portion_1_reset();
```

控制参数页面 `Menu_Control_P()` 提供基础速度、倒车速度、预瞄点数和运行速度参数。参数写入 Flash 后，启动流程只读取参数，不会无条件覆盖用户设置。

## 科目二：通道路线组合追踪

### 当前代码中的定义

菜单中的 `Guandao_Voice` 是当前工程对科目二模式的入口。它将 `route_setting_choice` 设为 `3`，并在主循环中调用：

```c
portion2_points_trace(0, 0, port2_flag);
```

代码注释中也把该模式称为“管道语音模式”。当前实现的运动主体是通道路线组合和惯导追踪，语音或外部触发信号通过 `port2_flag` 等状态量影响阶段切换。

### 四阶段状态机

`portion2_points_trace()` 位于 `code/guandao.c`，内部使用静态变量 `p2p_state`：

1. **阶段 0：组合前半段路线**
   - 从 `passage.recode_map` 提取多段路线。
   - 生成 `portion_2` 的前三段路线。
2. **阶段 1：追踪前三段路线**
   - 调用 `guandao_trace(&INS)`。
   - 当 `state` 非零时进入下一阶段。
3. **阶段 2：组合返回段路线**
   - 反向复制一段通道路线。
   - 再拼接最后一段路线，扩展 `portion_2.length_index`。
4. **阶段 3：继续追踪组合路线**
   - 再次调用 `guandao_trace(&INS)` 完成后半段。

这种设计把通道路线拆成若干固定区段，再按比赛阶段拼接到 `portion_2` 中。路线点来源主要是 `passage.recode_map`，而实际追踪仍复用 `guandao_trace()` 和纯追踪控制器。

### 关键代码

| 文件 | 作用 |
| --- | --- |
| `user/cpu0_main.c` | `Guandao_Voice` 分支调用 `portion2_points_trace()`。 |
| `code/guandao.c` | `portion2_points_trace()` 四阶段状态机和通道路线复制。 |
| `code/guandao.c` | `guandao_trace()` 负责路线节点选择、位姿更新和底层追踪。 |
| `code/guandao.h` | `portion2_points_trace()` 接口、`passage` 和 `portion_2` 路线结构。 |
| `code/RemteControl.c` | 遥控器/SBUS 输入、通道状态和触发量。 |
| `code/display.c` | 菜单模式选择和 `route_setting_choice` 设置。 |

### 与科目一的关系

科目二复用科目一的基础运动控制链路：编码器和 IMU 更新位姿，`guandao_trace()` 选择追踪状态，`pursuit_contral_mode()` 生成转向和速度输出。区别在于科目二先通过 `portion2_points_trace()` 组合路线，再分阶段调用追踪器。

## 控制输出链路

```text
编码器 + IMU
  -> update_state()
  -> current_state / current_point_index
  -> pursuit_contral_mode()
  -> out_servo, out_v_l, out_v_r
  -> angle_control / rear_motor
  -> 舵机、左右后轮电机
```

后轮驱动模块还包含 PID、前馈、左右轮差速补偿、PWM 变化率限制和刹车控制。串口调试输出在 `user/cpu0_main.c` 的 `Serial_Debug_Update()`，科目一日志以 `AUTO,cfg=p1spd8` 开头；其他路线会输出对应的 `REC` 或 `P3AUTO` 标识。

## 编译、测试和烧录

### 主机侧源码测试

在仓库根目录执行：

```powershell
python -m unittest discover -s tests -v
```

这些测试主要检查 C 源码接口、参数契约、路线保存格式、速度规划和后轮控制边界。

### AURIX 工程构建

1. 使用 AURIX Development Studio 打开工程。
2. 执行 Clean Project。
3. 执行 Build Project 和 Link。
4. 确认没有编译错误、链接溢出或未解析符号。
5. 烧录 TC264 后连接串口，确认日志中的配置标识与预期一致。

本仓库从提交 `24b6c22` 提取；工程操作记录见 [`PROJECT_LOG.md`](PROJECT_LOG.md)。

## 维护建议

- 修改路线追踪前先保留一份实车串口 JSON。
- 速度、预瞄、路线点阈值和停车逻辑一次只改一组参数。
- 修改后同时运行主机侧测试和 ADS 构建。
- 提交代码时保留串口配置标识，方便确认烧录版本。
