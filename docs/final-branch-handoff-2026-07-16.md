# final 分支换机测试交接

## 当前版本

- GitHub 仓库：`Svalbard-1019/kart_508`
- 当前分支：`final`
- 整合提交：`516d60b Integrate KMY and KMS route modes`
- KMY 基线：`8b9e735 Restore local SE2 parking alignment`
- KMS 参考：`88c38d5 Align KMS return smoothing with KMY`

`final` 不是把两个分支整文件强制覆盖，而是以 `8b9e735` 为底座，选择性移植
`88c38d5` 及 KMS 后续已经验证过的科目三功能。这样保留 KMY 的科目一和倒车停车代码，
同时加入 KMS 的返程能力。

## KMS 昨天的两个对照版本

### 版本一：dc1061c

```text
dc1061c Revert "Test master style KMS return tracking"
```

- 恢复 KMS 原来的返程追踪逻辑，撤销 master 风格追踪实验。
- 已包含此前的路线下标保护、Start 路径诊断和非阻塞串口 Bus Error 修复。
- 不包含最后加入的 KMY 式物理距离预瞄和平滑改进。
- 适合作为“原KMS返程基线”对照。

换电脑建立该版本的本地测试分支：

```powershell
git fetch origin
git switch -c kms-yesterday-baseline dc1061c
```

### 版本二：88c38d5

```text
88c38d5 Align KMS return smoothing with KMY
```

- 这是当前远程 `kms` 分支的最新版本。
- 在 `dc1061c` 基础上加入KMY式转向时间限速。
- 按实际路线距离换算科目三预瞄，适配 `0.2m` 点距。
- 移除追踪循环中不限频的屏幕刷新。
- 后轮加入左右差动 PWM 前馈。
- 适合作为“平滑KMS返程”对照。

换电脑切换该版本：

```powershell
git fetch origin
git switch kms
git pull origin kms
git log -1 --oneline
```

最后一条应显示 `88c38d5`。测试完两个KMS版本后，使用下面命令回到整合固件：

```powershell
git switch final
git pull origin final
```

## 已整合内容

### 科目一 KMY

- 保留 INS 路线记录和自动追踪。
- 保留 GPS 辅助、转向 PID、后轮闭环和差动 PWM。
- 保留停车点状态机、教学倒车路线和局部 SE(2) 停车对齐。
- 科目一普通路线点距保持 `0.3m`。

### 科目三 KMS

- 保留 `portion_3` 遥控记录和反向返程。
- 科目三记录点距为 `0.2m`。
- 路线容量扩展到 800 点。
- 返程方向使用终点前约 `0.8m` 的路线基线确定，减少末端两个点噪声造成的整条路线旋转。
- 科目三把 KMY 的预瞄点数换算成实际物理距离，避免 `0.2m` 和 `0.3m` 点距产生不同预瞄距离。
- 科目三追点阈值和终点停车距离均为 `0.15m`。

### 公共部分

- Flash 路线改成双页存储，支持最多 800 点。
- 旧的单页路线仍可读取；保存后会写成新格式。
- GPS 继续在主循环解析，不放回控制中断。
- 调试串口采用非阻塞逐字节发送。
- 串口使用固定 ASCLIN 模块地址，保留 KMS 的 Start Bus Error 修复。
- 屏幕自动驾驶诊断页会根据当前模式显示 INS 或 `portion_3` 数据。

## 换电脑拉取

已有仓库：

```powershell
cd <kart_508工程目录>
git fetch origin
git switch final
git pull origin final
git log -1 --oneline
```

最后一条应显示：

```text
516d60b Integrate KMY and KMS route modes
```

没有仓库时：

```powershell
git clone -b final git@github.com:Svalbard-1019/kart_508.git
cd kart_508
```

## 烧录前检查

1. 在 AURIX Development Studio 中执行完整 Clean/Build。
2. 确认没有 RAM overflow、Flash 页冲突或 unresolved external。
3. 当前 Codex 终端没有 TASKING `amk`，因此 `516d60b` 只完成了源码检查和 5 项自动测试，尚未在这里完成固件链接。
4. 确认烧录的是 `final` 分支生成的新 ELF，不要继续使用旧 Release 目录中的 ELF。

## 建议测试顺序

1. 先进入 Rack Test，检查转向、后轮、编码器、Yaw、遥控器和电池电压。
2. 使用原来的 KMY 科目一路线低速自动跑，确认前进和倒车状态与 `8b9e735` 一致。
3. 科目一先用 `1.5m/s`，正常后再测试 `2.0m/s` 和 `2.5m/s`。
4. 进入科目三记录模式，确认 `route=2`、Len 按约 `0.2m` 增加。
5. 保存、断电重启，确认 `portion_3` 的 Len 能正常恢复。
6. 科目三先用约 `1.0m/s` 返程，检查转弯位置和终点停车。
7. 最后测试超过 400 点的路线，验证第二个 Flash 页。

## 串口重点数据

- 记录：`route`、`len`、`x/y`、`yaw`、`encL/encR`。
- 自动：`idx/len`、`D/A`、`reason`、`x/y/yaw`、`Vl/Vr`、`servo`、`PWM`。
- 科目一应使用 INS；科目三应使用 `portion_3`。
- Start 后没有输出时，先确认串口波特率和烧录 ELF；出现 Bus Error 时保存完整的 `BUS ADDR/TIN/CLS/STAGE`。

## 是否需要重新打点

- 原来不超过 400 点的 KMY/KMS 路线理论上可以直接读取，先用低速验证。
- 测试科目三 800 点存储必须重新记录并保存一次。
- 如果旧路线读取后的 Len、起点方向或停车标记异常，应重新打点，不能继续高速测试。

## 本次未提交内容

原电脑工作区中的 `.cproject`、`.settings`、附件、压缩包和硬件文档没有纳入 `final` 提交，
它们不会通过 GitHub 自动同步到新电脑。
