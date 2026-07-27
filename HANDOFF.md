# Handoff

- 更新时间：2026-07-27 23:37
- 项目：`D:\kadingkuaipao\imitation\lsddd\kart_508\kart_508`
- 当前分支：`final`
- 状态：可交接；源码与自动化测试完成，等待 ADS 编译、烧录和实车验证

## 当前目标

让 Portion1 和 Portion3 自动驾驶速度连续、转弯更快且不出现“一停一走”；保证 Portion3 保存后第一次启动就按正确路线运行，无需再次 Reset，同时保留路线精度和终点停车能力。

完成标准：ADS Clean/Build/Link 成功，烧录后日志版本正确；同一路线连续多次实车测试中，Portion3 第一次启动正常、路线中速度连续、转弯速度可接受、终点停车准确。

## 当前状态

- `final` 已包含今天全部功能代码，结束工作前本地与 `origin/final` 同步至 `fc8c74b`。
- Portion1 和 Portion3 已共用同一个连续速度规划器；按用户决定，切换科目时不重置规划速度状态。
- Portion3 首次启动会静止约 1 秒采集 125 次陀螺仪 Z 轴零偏；完成后重新清空里程队列和编码器基线再追踪。
- Portion3 保存和重新读取使用完全相同的路线长度，不再读取时减掉一个点。
- 尚未用 TASKING/ADS 编译，也尚未烧录或实车验证今天的最终版本。
- 主工作区存在用户原有的 `.cproject`、`.settings/org.eclipse.core.resources.prefs` 和多个未跟踪附件/临时目录；未纳入今天的功能提交，不要清理或 `git add -A`。

## 今日完成

- Portion1 弯道速度和平滑减速：
  - 累计弯道比例为 `0.85 / 0.75 / 0.65`。
  - 普通弯、急弯、发卡弯比例为 `0.80 / 0.65 / 0.60`。
  - 速度加减限制仍为旧速度单位 `35 / 40`，对应约 `3.5 / 4.0 m/s²`。
  - 后轮普通 PID 减速改为逐步释放 PWM，不再直接掉到零 PWM。
  - 提交：`278c2a6`、`98081d7`。
- Portion3 保存一致性：
  - 修复 Flash 读取路线时少一个点的问题。
  - 保存和读取均保留精确 `stored_length`，800 点跨页格式继续保留。
  - 提交：`360cf29`。
- Portion3 第一次运行必须 Reset 的问题：
  - 日志证明第一次异常不是 800 点导致，而是仅清零 `Yaw_1` 没有重新估计陀螺仪零偏，且里程采样队列/编码器基线未完全清理。
  - 增加非阻塞 125 样本零偏校准，校准期间输出为零；完成后再次清空里程状态。
  - 串口增加 `p3Init`、`gzRaw`、`gzOff`，版本标识为 `P3AUTO,cfg=p3save2`。
  - 提交：`c50814f`。
- Portion1/Portion3 共用速度行为：
  - 两者直接共用一个 `guandao_speed_planner_t` 和更新时间状态。
  - Portion3 使用与 Portion1 相同的 2.4 m 距离转弯窗口、2.5° 抖动过滤、档位滞回、转弯比例和加减速限幅。
  - 删除 Portion3 原来的 `0.70 / 0.55` 低速弯道比例。
  - 按用户选择，进入 Portion1 或 Portion3 时都不重置共享规划器到最低速度。
  - Portion3 专属路线、转向预瞄、首次校准和最终 0.15 m 停车判定保持独立。
  - 提交：`fc8c74b`。

## 验证

- 已运行：

```powershell
python -m unittest discover -s tests -v
```

- 合并到 `final` 前后均通过，最终结果：`Ran 61 tests`，`OK`。
- 新测试覆盖共享速度规划器、相同比例、距离窗口、档位滞回、不重置共享状态、Portion3 首次零偏校准和里程复位。
- `git diff --check` 通过。
- 今天修改的 C、Python 和 Markdown 文件均通过 UTF-8 解码检查。
- 推送后 `HEAD` 与 `origin/final` 均为 `fc8c74b9ea851ec03ece6b33a6132c4ac5db021a`。
- 未验证：TASKING/ADS 编译、链接、烧录和实车表现。

## 关键决策

- Portion3 第一次运行问题按 IMU 零偏和里程基线根因处理，不通过修改 800 点容量或伪造路线终点掩盖。
- 首次 Portion3 校准必须让车辆保持静止约 1 秒；这是启动保护，不是路线中的停顿。
- Portion3 与 Portion1 使用同一速度规划器，而不是只提高几个弯道比例。
- 用户明确选择共享规划器跨科目保持状态，不在进入科目时重置到 `MIN_SPEED`。
- Portion3 仍保留独立最终停车距离，避免为了速度一致而失去终点保护。
- 用户要求以后每次功能更改都提交并上传目标分支。

## 失败尝试

- 仅在 `portion3_return_reset()` 中设置 `Yaw_1 = 0`：不能消除第一次启动时陀螺仪零偏跳变；MCU Reset 后正常是因为完整 IMU 初始化重新标定了零偏。
- 怀疑 800 点保存导致第一次路线错误：日志中两次运行的 `len=393` 和起始目标一致，排除路线内容变化；根因是第一次运行姿态在约 0.2 秒内异常跳变。
- 只提高 Portion3 弯道比例：只能提高弯速，不能解决缺少档位滞回和连续限速造成的速度突变，因此未采用。

## 阻塞与风险

- 当前环境没有 TASKING `amk/cctc` 工具链，必须在 AURIX Development Studio 中完成实际构建。
- 共享速度状态不在科目切换时重置是用户明确选择。若车辆在高规划速度状态下中途切换科目，校准结束后会从保留状态继续限速计算；实车需重点确认起步是否过猛。
- Portion3 启动校准时车辆若移动，采集到的零偏会失真，可能再次导致航向漂移。
- 自动化测试是源码契约和主机侧算法测试，不能替代 TC264 编译、实时中断时序及实车附着条件。
- Portion1 倒车终点精度和高菜单速度 `-40` 的安全性仍未在今天最终固件上重新实测。

## 下一步

1. 在 ADS 打开本地 `kart_508`，执行 Clean Project、Build 和 Link；记录所有 warning/error，确认无 RAM overflow、重复符号、Flash 冲突或 unresolved external。
2. 烧录后先架空后轮，确认串口出现 `AUTO,cfg=p1spd8` 和 `P3AUTO,cfg=p3save2`。
3. Portion3 启动时保持车辆静止至少 1 秒，确认 `p3Init` 从 `1` 变 `0`，`gzOff` 稳定后车辆才开始输出。
4. 用同一路线连续测试 Portion3 至少 3 次，每次都从首次启动直接运行，不按第二次 Reset；保存完整 JSON。
5. 对比 `req100/cmd100/tgt100/act100/pwm/turnLv`，确认路线中没有指令突降、转弯速度达到预期且出弯连续加速。
6. 记录终点横向、纵向和航向误差；若第一次仍偏，只分析 `p3Init/gzRaw/gzOff/yaw10/pRel/idx`，不要同时修改速度和转向。

## 恢复入口

- 首先阅读：`HANDOFF.md`。
- 设计与计划：
  - `docs/superpowers/specs/2026-07-27-portion3-start-rezero-design.md`
  - `docs/superpowers/specs/2026-07-27-portion3-shared-speed-planner-design.md`
  - `docs/superpowers/plans/2026-07-27-portion3-shared-speed-planner.md`
- 关键代码：
  - `code/guandao.c`：共享速度规划、Portion3 启动门控和最终停车。
  - `code/IMU.c`：非阻塞陀螺仪零偏校准。
  - `code/rear_motor/rear_motor.c`：里程复位和渐进 PWM 释放。
  - `code/flash.c`：Portion3 精确路线长度保存/读取。
  - `user/cpu0_main.c`：`p1spd8`、`p3save2` 串口字段。
- 当前关键提交：`fc8c74b`；当前分支：`final`。
- 恢复检查命令：

```powershell
git status -sb
git log -5 --oneline
python -m unittest discover -s tests -v
```
