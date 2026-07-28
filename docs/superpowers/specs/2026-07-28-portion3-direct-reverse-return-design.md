# Portion3 保存后直接倒车回程设计

## 目标

在 `portion3` 分支中，让科目三路线记录在 Flash 保存成功后，无需 Reset、无需再次按键、无需掉头，车辆保持当前车头方向，立即沿刚保存的路线倒序循迹回到原始起点并停车。

## 当前行为

- `guandao_recode()` 在 Portion3 长按保存时先制动，随后补写当前端点并调用 `Flash_Store_Mode(2)`。
- 保存成功后 `guandao_record_saved` 阻止继续记录，但不会自动切换到回程控制。
- 现有 `portion3_points_switch()` 会倒序、平移、旋转路线，并由 `portion3_return_reset()` 清零里程和重新校准 Yaw；它适用于重新进入科目三后按前进方向复现，不满足“原地直接倒车且不 Reset”。
- Portion1 已有真正倒车循迹所需的运动航向、倒车转向符号、速度限制和终点停止保护，可复用其设计原则，但 Portion3 必须使用自己的路线和状态，避免污染 Portion1 停车流程。

## 方案

### 保存到回程的状态转换

Portion3 的保存流程保留“先制动、再写 Flash”，避免车辆运动时写 Flash。Flash 写入成功后：

1. 保留 `portion_3.current_state`、`Yaw_1` 和后轮里程状态，不调用 `portion3_return_reset()`，也不启动 IMU 重校准。
2. 在调用现有 Flash 保存流程前，借用已有规划缓冲区暂存原始路线和实时端点位姿；Flash 继续保存兼容旧固件的局部回程路线。写入后恢复原始坐标路线与端点位姿，再仅在 RAM 中按点序倒序。
3. 初始化 Portion3 独立倒车状态，起始索引指向倒序路线的第一个有效前瞻点。
4. 把 `main_mode` 切换为 `Guandao_portion_3`，把 `conrtol_mode` 和 `daoche_flag` 切换为倒车。
5. 制动释放后开始输出受限倒车速度，不要求人工输入。

### 倒车循迹

新增 Portion3 专用倒车更新函数，使用 `portion_3.current_state` 和倒序后的 `portion_3.recode_map`：

- 每次循环先调用现有 `update_state()`，倒车运动航向使用 `Yaw_1 + 180°`。
- 从当前索引向路线末端搜索最近点，只允许索引单调前进，防止弯道上跳回旧点。
- 沿倒车运动方向选择满足前瞻距离的目标点。
- 用自行车模型计算转角，并按现有倒车机构对舵机命令取正确符号；限制转角幅度与变化率。
- 弯道和接近终点时降低倒车速度；不调用任何“原地旋转到航向”逻辑。

### 停车与故障保护

- 目标为原始路线第一个记录点。
- 同时满足末端索引、目标距离和合理航向误差时启动制动并保持零输出。
- 增加基于记录路线长度的最大允许行驶距离以及超时保护，避免定位漂移时持续倒车。
- 路线少于 3 点、路线长度非法或倒序初始化失败时保持制动，不进入倒车。
- 保存仍只发生一次；倒车期间不再次写 Flash。

## 诊断

P3 串口日志增加倒车状态、路线索引、终点距离和倒车转向命令，版本标识更新，便于实车区分新固件并确认没有发生 Yaw 重置或里程清零。

## 测试

主机侧测试覆盖：

- Portion3 保存完成后自动进入倒车，不依赖 Reset 或菜单按键。
- Flash 写入发生在直接倒车路线生成之前；写入后恢复原始路线和实时端点位姿，保证现有 Flash 格式兼容且直接倒车仍使用同一记录坐标系。
- 回程初始化不调用 `portion3_return_reset()`、`rear_motor_reset_odometry()` 或 `IMU_yaw_rezero_start()`。
- 倒序路线以保存端点为首、原始起点为终点，坐标不做平移旋转。
- 倒车追踪使用 `Yaw_1 + 180°`、倒车模式和受限速度。
- 到达原始起点或触发越程/超时保护时制动停车。
- 全量现有 Python 测试继续通过；由于本机没有 TASKING 工具链，最终 ADS 编译、烧录和实车验证仍需在开发环境完成。

## 联网审核依据

- [Nav2 Regulated Pure Pursuit](https://github.com/ros-navigation/navigation2/blob/main/nav2_regulated_pure_pursuit_controller/README.md)：目标点应位于机器人实际运动方向，并应在曲率和目标接近阶段调低线速度。
- [Nav2 reverse behavior issue #3086](https://github.com/ros-navigation/navigation2/issues/3086)：倒车路径中方向符号处理错误会导致车辆旋转或异常循迹，说明倒车运动航向与转向符号必须成套处理。
- [Autoware Lanelet2 direction-change design](https://github.com/autowarefoundation/autoware_lanelet2_extension/blob/main/autoware_lanelet2_extension/docs/lanelet2_format_extension.md)：正向和反向行驶方向应明确建模，而不是依赖临时掉头逻辑。

## 不在本次范围

- 不改变 Portion1 倒车停车流程。
- 不修改 Flash 格式和已保存路线兼容性。
- 不实现掉头、前进回程或开放式轨迹规划。
- 不调整用户现有 `.cproject`、Eclipse 设置和其他未提交文件。