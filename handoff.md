# Final 分支项目交接文档

> 更新日期：2026-07-26
> 项目：第 21 届智能车竞赛卡丁快跑组 TC264 工程
> 仓库：`Svalbard-1019/kart_508`
> 目标分支：`final`

## 1. 当前正在做什么

当前任务是把已经分别验证过的两套程序整合为一套比赛固件，并继续提高稳定性：

- 科目一使用 KMY 的 INS 路线记录、自动追踪、停车点状态机和倒车停车。
- 科目三使用 KMS Return 的 `portion_3` 遥控记录、反向路线生成和自动返程。
- 两个科目共用 TC264、IMU、左后轮编码器、转向电机、后轮驱动、菜单、串口和 Flash。
- 对容易互相影响的转向 PID、后轮速度 PID、转弯降速比例做科目隔离。
- 后续重点不再是盲调 PID，而是修正路线长度、物理距离预瞄、路线进度、单编码器速度闭环和动态制动等结构问题。

目前融合已经完成，下一阶段是按本文优先级逐项优化和上车验证。

## 2. 当前权威版本

### 2.1 Final 功能代码基线

```text
分支：final
本文描述的功能代码基线：83b9546
名称：Integrate latest KMY and KMS return profiles
提交时间：2026-07-24 17:00:23 +0800
```

### 2.2 融合来源

```text
KMY 分支：kmy
KMY 提交：b7448b7
名称：Stop steering output in idle mode

KMS Return 分支：codex/kms-return-heading-fix
KMS Return 提交：325e311
名称：Use symmetric rear PWM rate limits
```

交接文档提交会位于 `83b9546` 之后，但只新增文档，不改变上述功能代码。

`final` 不是把两个分支的文件整体覆盖，而是保留 KMY 主体，再把 KMS Return 的科目三功能按路线隔离整合进去。

### 2.3 本机工作树

主工程目录当前仍可能停留在 `kmy`。`final` 使用独立工作树：

```text
D:\kadingkuaipao\imitation\lsddd\kart_508\kart_508-final-integration
```

查看当前版本：

```powershell
git -C D:\kadingkuaipao\imitation\lsddd\kart_508\kart_508-final-integration status
git -C D:\kadingkuaipao\imitation\lsddd\kart_508\kart_508-final-integration log -1 --oneline
```

## 3. Final 目前包含什么

### 3.1 科目一 KMY

科目一使用 `INS` 路线，入口为：

```text
Menu -> Mode Choice -> Guandao_portion_1
main_mode = Guandao_portion_1
route_setting_choice = 0
```

当前保留：

- `INS` 路线记录与 Flash 保存。
- 默认路线点距 `recode_threshold = 0.3m`，该值也可从 Flash 参数读取。
- 自动驾驶直接追踪记录路线，科目一当前不启用平滑规划路线。
- 10ms 编码器脉冲与同时刻 Yaw 成对缓存，主循环逐样本积分位姿。
- 第一停车点进场状态机。
- 停车附近 GPS 采样与局部偏差估计。
- 教学倒车路线、局部 SE(2) 对齐和自动倒车规划兜底。
- 到位条件同时检查位置、航向、路线截面和超时保护。
- 停止后启动非阻塞主动制动。
- 进入 IDLE 后停止转向电机输出，避免停车后方向盘持续大幅动作。

主要文件：

- `code/guandao.c`
- `code/auto_park_plan.c`
- `code/parking_se2.h`
- `code/gps.c`
- `code/flash.c`

### 科目一当前控制参数

转向控制：

| 参数 | KMY |
|---|---:|
| Kp | 500 |
| Ki | 18 |
| Kd | 27 |
| 前馈 | 65 |

后轮速度控制：

| 参数 | KMY |
|---|---:|
| Kp | 4.0 |
| Ki | 0.6 |
| Kd | 0.12 |
| 前馈 | 8.0 |
| 2.5m/s 以上附加前馈 | 500 |
| PWM 每次变化上限 | 600 |

科目一转弯速度比例：

| 类型 | 比例 |
|---|---:|
| 普通弯 | 0.75 |
| 急弯 | 0.60 |
| 累计小弯 | 0.85 |
| 累计中弯 | 0.75 |
| 累计大弯 | 0.75 |

### 3.2 科目三 KMS Return

科目三使用 `portion_3` 路线：

```text
记录：Menu -> Recode Points -> portion_3
返程：Menu -> Mode Choice -> Guandao_portion_3
route_setting_choice = 2
```

当前包含：

- 遥控器跟随记录 `portion_3`。
- 固定记录点距 `0.20m`。
- 保存时反转路线顺序。
- 以记录终点为返程局部坐标原点。
- 根据记录终点 Yaw 建立返程坐标方向。
- 返程终点裁掉 `0.5m`。
- 路线容量扩展到 800 点。
- Flash 使用两个 EEPROM 页保存一条长路线。
- 科目三专用转向 PID 和后轮 PID。
- 科目三专用转弯降速比例。
- 科目三末段越点保护：最后几个点允许根据线段投影、横向误差和航向误差推进路线索引。
- 科目三最终停车距离为 `0.15m`。
- 只对 `portion_3` 使用左轮里程到车身中心里程的 IMU 航向增量修正。

### 科目三当前控制参数

转向控制：

| 参数 | KMS Return |
|---|---:|
| Kp | 1000 |
| Ki | 15 |
| Kd | 40 |
| 前馈 | 80 |

后轮速度控制：

| 参数 | KMS Return |
|---|---:|
| Kp | 10.0 |
| Ki | 0.3 |
| Kd | 0.8 |
| 前馈 | 13.0 |
| 2.5m/s 以上附加前馈 | 0 |
| PWM 每次变化上限 | 1000 |

科目三转弯速度比例：

| 类型 | 比例 |
|---|---:|
| 普通弯 | 0.70 |
| 急弯 | 0.55 |
| 累计小弯 | 0.80 |
| 累计中弯 | 0.70 |
| 累计大弯 | 0.70 |

### 3.3 两个科目共用的底层能力

- 四路 17kHz PWM 驱动两个 HIP4082 后轮全桥。
- 后轮速度闭环目前只读取左后轮 TIM2 编码器。
- 后轮 PWM 硬限幅为 9500。
- 自动驾驶将旧速度单位乘 `0.1` 换算为 m/s。
- 左右目标轮速用于生成差动 PWM 前馈。
- 主动制动按实时速度选择 2500、1800、1000 三档反向 PWM。
- IMU 在定时中断中更新，后轮编码器每 10ms 采样。
- 里程缓存保存 `{脉冲增量, 同时刻 Yaw}`，避免主循环堵塞后整段脉冲使用最后一个 Yaw 投影。
- GPS 中断只接收数据，解析放在主循环中。
- 串口诊断采用缓冲式非阻塞发送。
- 屏幕和串口诊断已经限频。
- 屏幕损坏时可以使用串口菜单输入模拟按键操作。
- 电池电压通过 A11 分压读取并显示。

## 4. 关键数据流

### 4.1 记录路线

```text
遥控/人工推车
    -> 10ms 左轮编码器 + Yaw 采样
    -> update_state()
    -> current_state(x, y, theta)
    -> recode_waypoint()
    -> recode_map[]
    -> 长按保存
    -> Flash_Store_Mode(route_setting_choice)
```

### 4.2 自动驾驶

```text
Flash 读取路线
    -> update_state()
    -> 最近点和前方点搜索
    -> pursuit_contral_mode()
    -> out_v_l / out_v_r / out_servo
    -> Guandao_Rear_Motor_Update()
    -> 路线对应后轮 PID
    -> 四路后轮 PWM

out_servo
    -> 11ms 左右的转向控制中断
    -> 路线对应转向 PID
    -> 前轮转向电机 PWM
```

### 4.3 科目一停车与倒车

```text
前进追踪
    -> 第一停车入口局部坐标判断
    -> 主动制动
    -> 等待 GPS / 建立局部倒车路线
    -> 教学路线优先，否则自动倒车规划
    -> 低速精调位置和 Yaw
    -> 主动制动
    -> IDLE，转向输出清零
```

## 5. 当前已经验证的内容

源码测试：

```powershell
python -m unittest discover -s tests -p "test_*.py" -v
```

当前结果：

```text
Ran 38 tests
OK
```

测试覆盖：

- KMY/KMS 转向和后轮参数隔离。
- 科目一停车模块保留。
- 科目三 800 点和双页 Flash 入口。
- 科目三终点越点保护。
- 10ms 脉冲/Yaw 成对积分代码存在。
- GPS 和串口非阻塞调度代码存在。
- 主动制动方向、停止条件和编码器采样。
- IDLE 模式停止转向输出。
- 左轮到中心里程换算公式通过独立 C99 编译测试。

注意：多数测试属于源码结构测试，不能替代实车测试。

当前 Codex 终端没有 TASKING `amk/cctc`，因此尚未在命令行完成完整 TC264 固件链接。烧录前必须在 AURIX Development Studio 中 Clean/Build。

## 6. 当前已知风险和优化空间

以下按优先级排序。每次只改一项并测试，不要一次全部修改。

### P0：修复科目三保存后与重启后路线长度不一致

位置：`code/flash.c` 的 `Flash_Read_portion_3points()`。

当前保存完整 `stored_length`，但读取时执行：

```c
portion_3.length_index = stored_length - 1;
```

影响：

- 保存后直接跑和断电重启后跑的路线终点不同。
- 最终减速、停车和索引状态可能差一个点。
- 同一代码可能表现出偶发终点差异。

下一步应先建立“保存 -> 立即读取 -> 点数与每个点完全一致”的测试，再决定是否删除减 1。若最后一个按键保存点确实不应追踪，应在保存前明确删除，而不是只在重启读取时删除。

### P1：预瞄从按点数改为按物理距离

当前位置：`code/guandao.c` 的 `pursuit_midhandle()`。

当前逻辑：

```c
preview_index = current_point_index + preview_steps;
```

问题：

- KMY 默认 0.3m/点，KMS 固定 0.2m/点。
- 同样 5 个预瞄点分别约为 1.5m 和 1.0m。
- 实际点间距受打点速度、转弯、编码器和主循环节拍影响，并不完全均匀。

建议：

```text
Ld = clamp(L0 + Kv * 实际车速, Lmin, Lmax)
```

从当前路线进度沿路线累计弧长，找到距离达到 `Ld` 的插值目标点。不要简单把 KMS 设置为更多点数代替物理距离。

### P1：弯道识别改为距离窗口内的平滑有向曲率

当前 `guandao_accumulated_route_turn()` 在未来 12 个点内把所有航向变化绝对值相加。

问题：

- 直线打点轻微左右抖动也会累计成大转角。
- 会产生不必要的降速和速度忽快忽慢。
- 固定 12 点在 0.2m 和 0.3m 点距下代表不同距离。

建议：

- 使用固定米数窗口。
- 先对路线切线或曲率做轻度滤波。
- 保留左/右转符号，减少左右交替噪声。
- 弯道进入和退出增加滞回，避免速度命令反复切换。

### P1：修正单左轮速度反馈和车身中心目标不一致

当前实际情况：

- 后轮 PID 只读取左轮编码器。
- 上层目标是 `(out_v_l + out_v_r) / 2`，即车身中心速度。
- 输出端又给左右轮叠加差动 PWM。

转弯时左轮本来就不等于中心速度，PID 会把正常轮速差当成速度误差，并与差动前馈互相影响。

建议优先方案：

```text
v_center = v_left - side_sign * yaw_rate * TRACK_WIDTH / 2
```

使用 IMU 角速度把左轮速度换算为中心速度，再闭环中心目标。实施前必须分别做左圆、右圆、顺时针 180°、逆时针 180° 测试，确认 `side_sign` 和有效轮距。不要未经验证直接同时应用到 KMY/KMS。

### P1：路线索引推进增加几何约束

当前 `guandao_find_closest_index()` 只在未来 8 个点中按欧氏距离选择最近点。

风险：

- 绕桶、回头弯、路线相邻处可能跳到尚未经过的点。
- 高速下单周期跨越距离更大，更容易跳过圆弧。
- 当前投影和航向保护只覆盖科目三最后几个点，不覆盖全路线。

建议：

- 以路线线段投影计算连续进度，而不只使用离散点索引。
- 进度只能单调增加。
- 候选线段必须满足车头/切线方向约束。
- 单周期允许推进的最大弧长与实际速度和 `dt` 对应。
- 终点附近使用独立的横向、纵向和航向门限。

### P2：科目三返程起始朝向需要实际对齐

当前 `portion3_points_switch()`：

- 使用记录终点 `origin.theta + 180°` 建立返程坐标系。
- `portion3_return_reset()` 直接令 `Yaw_1 = 0`。

这等价于假设人工原地掉头后，车头与返程第一段完全一致。人工误差几度就会让整条返程路线产生旋转偏差。

建议：

- 用终点前约 0.6~1.0m 的路线切线估计返程方向，避免只依赖最后一个点的 Yaw。
- 启动时计算实际车头与返程第一段切线的误差。
- 误差过大时不高速起步，先低速对齐或提示重新摆正。
- 串口增加 `StartYawErr`。

### P2：末段减速改为动态制动距离

当前使用固定 `final_dsts` 线性降速，且最低速度仍为旧单位 4，即约 0.4m/s；到终点后才启动主动制动。

高速时固定距离无法覆盖负载、电池电压、地面摩擦和驱动状态变化。

建议：

```text
stop_distance = v_actual^2 / (2 * a_brake) + reaction_margin
```

- `a_brake` 使用 Rack Test 实测值。
- 直线可以保持高速，只在预计制动距离内开始减速。
- 终点附近允许目标速度继续降到比 0.4m/s 更低。
- 主动制动用于最后消除滑行，不替代提前速度规划。

### P2：KMY 末段航向不应永久写死为 -90°

科目一 `azimuth_adjust()` 当前使用 `CORRECT_ANGLE_1 = -90°`。

若每次路线方向、起点摆放或 Yaw 清零存在差异，固定角度会在最后阶段把车辆拉向错误方向。

建议从以下数据生成目标航向：

- 停车入口前若干路线点的平均切线；或
- 记录停车点保存的车身 Yaw；或
- 两者加权并做有效性检查。

该修正只能在确认已经进入停车直线后启用，不能提前影响最后一个锥桶。

### P2：多速度窗口时 PID 时间仍固定为 0.1s

后轮模块能积累多个 100ms 编码器窗口，避免主循环延迟时丢脉冲；但一次取出多个窗口时：

- 测量值按窗口数求平均。
- 积分和微分仍固定使用 `0.1s`。

建议使用：

```text
dt = 0.1s * window_count
```

或者逐窗口执行 PID。这样主循环偶发延迟时，PID 的积分和微分不会使用错误时间尺度。

### P3：两科目运行参数仍未完全隔离

已经隔离：

- 转向 PID。
- 后轮速度 PID。
- 部分转弯速度比例。

仍然共用：

- `base_speed`
- `preview_spets`
- `final_dsts`
- `persuit_threshold`
- 部分转向增益、限幅和制动参数

建议建立：

```c
typedef struct {
    float base_speed;
    float lookahead_min;
    float lookahead_gain;
    float final_decel;
    float pursuit_threshold;
    ...
} guandao_profile_t;
```

进入模式时一次性选择并冻结 KMY/KMS 配置，不要在每轮控制中继续读取可变的全局 `route_setting_choice`。

### P3：验证左轮中心里程修正的符号与轮距

`final` 已把 KMS 的左轮中心里程修正合入，但该公式依赖：

- 编码器安装在左轮还是右轮。
- Yaw 正方向定义。
- 实际有效轮距。
- 轮胎侧滑情况。

公式通过了数学单元测试，不代表实车符号一定正确。必须通过左右对称圆测试确认；如果左右弯误差方向相反，优先检查符号和有效轮距，不要继续调预瞄。

## 7. 推荐实施顺序

严格一次只做一项：

1. 修复科目三 Flash 路线长度一致性。
2. 保存、断电、重启并逐点校验 `portion_3`。
3. 将预瞄改成按米计算，仅在一个实验分支测试。
4. 将弯道识别改成固定距离平滑曲率。
5. 加全路线投影进度和防跳段保护。
6. 做左右圆测试，确定单编码器中心速度换算符号和有效轮距。
7. 修正后轮中心速度闭环。
8. 加动态制动距离。
9. 改科目三返程起始朝向对齐。
10. 最后才重新微调转向 PID、速度 PID 和降速比例。

每一步正常后单独提交，提交名说明唯一变量。若某一步失败，直接回退该提交，不要继续叠加补丁。

## 8. 推荐实车测试矩阵

### 8.1 烧录前

1. `git log -1 --oneline` 确认目标提交。
2. AURIX Development Studio 执行 Clean Project。
3. 重新 Build，不使用旧 ELF。
4. 确认没有 RAM overflow、Flash 页冲突、重复符号或 unresolved external。
5. 烧录后串口打印提交短哈希，避免忘记烧录。

### 8.2 基础硬件

1. Rack Test 检查 Yaw 静止不漂。
2. 左右各转 90°，Yaw 应接近实际角度。
3. 推行 1m、2m，检查编码器距离。
4. 固定 PWM 检查左右桥输出。
5. 检查 A11 电池电压与万用表误差。
6. 检查驱动公共地、12V、5V 和 HIP4082 使能。

### 8.3 科目一

1. 原路线先跑 1.5m/s。
2. 正常后测试 2.0m/s、2.5m/s。
3. 对比同一路线三种速度的入弯位置。
4. 检查第一停车入口纵向、横向和 Yaw 误差。
5. 连续倒车至少 5 次，记录是否到位、是否超时、制动距离和最终横向误差。
6. 停车后确认方向盘不再持续动作。

### 8.4 科目三

1. 记录新路线，确认约每 0.2m 增加一个点。
2. 保存后立即记录 Len 和最后三个点。
3. 断电重启，再次读取 Len 和最后三个点。
4. 两组数据必须完全一致后再自动返程。
5. 低速返程确认起步方向、每个锥桶入弯点和最终停车。
6. 再逐步提高速度。
7. 测试一次超过 500 点的路线，验证第二页 Flash。

### 8.5 必须保存的串口字段

公共：

```text
commit, mode, route, dt, dtMax, baseSpeed
```

追踪：

```text
idx, len, targetX, targetY, x, y, yaw,
previewDistance, curve, crossTrack, headingError,
outServo, actualSteer, outVl, outVr
```

后轮：

```text
targetMps, centerActualMps, rawLeftMps,
pwmCenter, pwmL, pwmR, speedError, integral
```

停车：

```text
reason, entryLong, entryLat, entryYaw,
reverseState, reverseIndex, finalDistance,
brakePwm, brakeElapsed, brakeExitReason
```

## 9. 是否需要重新打点

- 仅修改控制算法、PID、预瞄或制动，一般可先使用原路线对照。
- 修改编码器米/脉冲系数、中心里程换算符号、Yaw 坐标定义后，记录和自动必须使用同一算法，建议重新打点。
- 修复 `portion_3` Flash 长度规则后，应重新保存一次科目三路线，并验证断电前后逐点一致。
- 修改科目三返程坐标变换后必须重新保存路线。
- 对比测试时不要同时重新打点和改算法，否则无法判断变化来源。

## 10. 不要这样改

- 不要把 KMY 或 KMS Return 的 `guandao.c` 整文件覆盖到 `final`。
- 不要用一个 PID 参数强行兼顾两个科目。
- 不要一次修改预瞄、PID、降速、里程和终点判断。
- 不要仅凭一次成功运行就合入 `final`。
- 不要把 GPS 单点瞬间写入当前 `x/y`。
- 不要在绕桶急弯中直接做大幅 GPS 坐标修正。
- 不要删除或覆盖用户未提交的 `.cproject`、`.settings`、附件和测试数据。
- 不要使用旧 Release 目录里的 ELF 判断新代码效果。

## 11. Git 操作建议

拉取当前整合版：

```powershell
git fetch origin
git switch final
git pull origin final
git log -1 --oneline
```

每个优化建立独立分支：

```powershell
git switch final
git pull origin final
git switch -c codex/final-<single-change-name>
```

测试通过后再合回 `final`。不要直接在 KMY、KMS Return 和 Final 三个分支同时改同一功能。

## 12. 关键文件索引

| 文件 | 作用 |
|---|---|
| `code/guandao.c` | 位姿积分、路线记录、纯追踪、转弯减速、科目一停车倒车、科目三返程 |
| `code/guandao.h` | 路线结构、车辆尺寸、点数容量和公共接口 |
| `code/flash.c` | 参数、INS、portion_3、GPS 和停车标记的 Flash 保存读取 |
| `code/rear_motor/rear_motor.c` | 左轮编码器采样、后轮速度 PID、差动 PWM、主动制动 |
| `code/rear_motor/rear_motor.h` | KMY/KMS 后轮参数和制动参数 |
| `code/rear_motor/rear_left_wheel_odometry.h` | 左轮里程换算车身中心里程 |
| `code/angle_control.c` | 前轮角度闭环和路线 PID 选择 |
| `code/angle_control.h` | KMY/KMS 转向 PID 参数 |
| `code/display.c` | 屏幕菜单、串口菜单和模式切换 |
| `user/cpu0_main.c` | 主循环调度、后轮目标速度、GPS 主循环解析和串口诊断 |
| `user/isr.c` | IMU、转向控制和 10ms 编码器采样中断 |
| `code/auto_park_plan.c` | 自动倒车路径规划兜底 |
| `code/parking_se2.h` | 局部 SE(2) 坐标变换 |
| `tests/` | 源码结构、制动、驱动映射和里程公式测试 |

## 13. 接手后的第一件事

不要先调 PID。先完成下面这个最小闭环：

```text
修复 portion_3 长度读取规则
    -> 自动化保存/读取一致性测试
    -> 上车记录一条短路线
    -> 保存前记录 Len/末点
    -> 断电重启
    -> 再次读取 Len/末点
    -> 完全一致后低速返程
```

这个问题解决后，再开始物理距离预瞄。这样每次测试的路线数据本身是稳定的，后续结论才可信。
