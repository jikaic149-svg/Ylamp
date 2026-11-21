# Ylamp V1.2 五轴机械臂（J1–J5）｜DM-MC-Board02

先读 [当前进度](process.md)，再按需查本页源码入口和某一天的历史。历史中的“当前”“进行中”和旧参数只适用于原记录，不能覆盖当前进度。

## 当前版本

- 整机控制对象为V1.2五轴，不是旧双轴或单电机台架。机械依据为 `../Ylamp-V1.2/装配体2.step`。
- 源码基线 `a01b475` 加当前未提交修改：已删除项目CAN3显示/记录链路；观测统一回传五轴反馈与双Pitch，不再切换J2/J3。J2候选补差、J3原补偿、J1/J4/J5及名义动态项不变；J1辨识暂后置。
- 当前实板 `build/v12-j4-raw-range-20261121/ylamp_v12_five_axis.bin`，155204字节，SHA-256 `44AF05C033AEA28724797A6DADE36F43606786571ACAA9DF504AE53FF656EA66`，**2026-11-21已烧录**；UID、完整烧前备份、写后及普通运行中全应用回读均通过。J2四段采集完成，J4首段振动后fault32退出；之后最新实读五轴反馈全部过期、DEGRADED/fault97/submitted0，支撑不能确认。主动测试已停止，先按用户要求查振动。完整状态和限制见 [当前进度](process.md)。
- 本版仅将J4原始行程改为−180°～350°，保留55.7024265°零点；31/31主机、16/16五轴Python、参考角自测、ARM/门禁通过。旧2AA07A27…版本保留在 `build/v12-usb-unified-20261120/`，不是当前板端版本；先前待承托和J3缺反馈记录均为历史。
- 当前修改此前已通过31项主机、16项Python、观测/USB/Pitch零点自测、C/Python统一帧对照、隐藏GUI及ARM构建/镜像门禁；控制/参考/支撑配置408字节与之前J2候选一致。本次重核构建/门禁并完成部署，未重跑上述全部软件回归；软件测试和镜像回读不等于新公式或USB吞吐已实机验收。没有Git远端，通信修改尚未提交/推送。
- 本次在用户确认托稳后完成一次刷写/必要复位，显式原旧J3取数和原ACK各一次，随后按用户要求开始测试；不增加自动恢复或故障策略，不调J1，不删除历史数据。既有CAN3移除、统一观测等修改保留；原厂SDK不改，芯片启动表中未启用中断的弱名称不等于CAN3功能仍运行。

详细逻辑见 [完整源码说明HTML](../output/ylamp-code-logic-20261120.html)：21节，含启动、控制周期、坐标/公式、旧J3、通信、故障与采样辨识的源码定位。

## 文件职责

| 位置 | 内容及维护边界 |
|---|---|
| [App/](App/) | 应用入口、RTOS、串口DMA及异常处理 |
| [BSP/](BSP/) | CAN、USB、UART、KEY、电源等板级接口 |
| [Device/](Device/) | 电机协议、BMI088、YummyIMU及遥测设备层 |
| [Model/](Model/) | 五轴几何、工作区及名义动力学 |
| [Mode/](Mode/) | 五轴轨迹采样与纯计算支撑公式 |
| [Task/](Task/) | 唯一控制所有者 `FiveControl`、发送及USB接口 |
| [Algorithm/](Algorithm/) | 项目既有算法；不能将旧双轴公式直接当五轴结果 |
| [config/](config/) | 电机映射、软件零点、CAD偏置、行程及补偿配置 |
| [tests/](tests/) | HAL替身行为测试及离线数值回归 |
| [tools/](tools/) | 上位机、离线辨识、图形/几何复核和镜像门禁 |
| [cmake/](cmake/) / [CMakeLists.txt](CMakeLists.txt) | 原生主机及GNU ARM构建 |
| [history/](history/) | 原文按日归档，及保留的 `mit_20261107/` 早期材料 |
| `build/` | 构建、实测、拟合、烧前备份混合存储；不能整体当缓存清理 |

`result.json`是已忽略的无关切片导出，不是固件或测试结果；本轮保留原位。不改工作区的CAD、URDF、分享包和工具状态目录。

## 五轴公式与坐标

权威参数为 [joint_references.def](config/joint_references.def)、[five_coordinates.h](config/five_coordinates.h)、[five_mit.c](config/five_mit.c)。原始反馈不归零；J4保留原始多圈坐标，最新用户范围−180°～350°已部署，不折成±180°。

```text
theta_i = raw_reference_rad[i] - zero_rad[i]
qCAD_i  = theta_i + CAD_offset_deg[i] * pi / 180
G2      = G_CAD2(qCAD) + [Yg(qCAD) * delta]_J2
G3      = 1.18 * G_CAD3(qCAD) - 0.01 * cos(theta_3)
```

公式入口是 [FiveSupport_GravityModel](Mode/five_support.c)，静态残差计算在 [five_dynamics.c](Model/five_dynamics.c)。每次使用同一份五轴实际姿态，只采用新残差的J2输出；不按零点两侧、速度方向或采样段切换系数，不把固定姿态的正弦切片硬编码为通用公式。

J3额外加力证据不足，保留原倍率/余弦项且只施加一次。有效势能组合不是重新测得的物理质量/质心，名义M/c未更新；选定输出的支撑修正也不是一套重新辨识的完整保守刚体模型。预算在公式之后独立处理，纯G硬件/软件Kp/Kd仍为0。

## 上位机与控制边界

| 入口 | 用途 |
|---|---|
| [five_control.py](tools/five_control.py) | 五轴控制会话、状态、显式旧协议J3入口和采样协议 |
| [j3_pitch_monitor.py](tools/j3_pitch_monitor.py) / [.cmd](tools/j3_pitch_monitor.cmd) | 统一五轴反馈＋BMI088/Yummy双Pitch，两个对照图、一个CSV；开始/暂停只采集，不启停电机，不再支持--joint切换 |
| [usb_monitor.py](tools/usb_monitor.py) | USB普通五轴快照和公共解码/组包；原专用FD适配器工具已移除 |
| [five_gravity_sweep.py](tools/five_gravity_sweep.py) | 显式主动采样，同步记录全部五轴实际姿态 |
| [fit_pitch_zero.py](tools/fit_pitch_zero.py) | 统一CSV默认分别拟合J2/BMI与J3/Yummy，按源计数去重；兼容历史单轴CSV |
| [five_gravity_residual_report.py](tools/five_gravity_residual_report.py) | 既有数据的离线重力/摩擦分析，不授权部署 |
| [urdf_collision/](tools/urdf_collision/) | CAD/仿真/工作区验证工具，不能代替实物或碰撞验收 |

J3是旧协议，失能静默不等于通信坏；仅走唯一所有者的显式旧协议流程，不使用新版0xCC/0x33读取，不自动唤醒、ACK或复位。每boot的单次限制、独立冷启动和故障处理保持。

统一观测版本为 `0x594C5002`，模式0正常/1开始/2暂停，旧3/4拒绝；100Hz目标、40ms独立新鲜阈值，213字节一帧（206字节payload），保留每路源时间/计数，不声称硬同步。新J4版本实采800帧/100Hz，五轴和Yummy全部新鲜，BMI预热后286帧新鲜，暂停静默通过；这是通信/静态观测验收，不是物理零点或重力模型验证。本次未启动/关闭旧GUI，COM36已正常取得；主动采样时由单一工具独占。CSV沿用 `build/j3-pitch-capture/five_pitch_*.csv`。

主动采样期间不要另开COM36状态/控制会话：会话交接可能取消轨迹。`POSITION`只表示轨迹计时结束并继续反馈保持，不是实际到位证明。删除小角度/Move静止门槛的源码已随本次镜像部署，但没有新增运动验收；行程、力矩、失联及温度处理仍保留。

## 构建与验证

在 `code/` 运行；需要CMake 3.22+、Ninja和可运行的本机GCC。Windows须保证编译器及其运行库在PATH。Python数值测试需要NumPy，硬件工具另需串口依赖；本轮不安装依赖。

```powershell
cmake -S . -B build/host -G Ninja -DBSP_HOST_TEST=ON -DYLAMP_J3_REVERSE_TEST=ON
cmake --build build/host
ctest --test-dir build/host --output-on-failure
python -B -m unittest discover -s tests -p "test_five*.py"
```

整机ARM构建使用 [arm-gcc.cmake](cmake/arm-gcc.cmake)，并为 `BSP_CUBE_ROOT` 指定既有、未经修改的 `H7_BSP-HEAD`。使用新构建名，保留旧镜像和烧前备份；不得回刷 `ylamp_firmware.*`、旧双轴/通信检查版来“恢复”五轴。

任何整机烧录前必须对同版ELF/BIN运行（先替换下列占位路径）：

```powershell
python tools/check_v12_image.py <候选ELF> <同版BIN>
```

GNU ARM工具须可用；含临时反向测试的构建加 `--j3-reverse-test` 核对相应链路。结构门禁通过不代表获准烧录、使能或运动；仍需核对UID、现场出力/承托、烧前备份、全镜像写后回读与实物验收。

## 实测与构建产物

以下是本地已有证据入口，`build/` 被Git忽略；不能假称Git已经备份这些原始记录。这里的存在也不表示每个样本或整段序列均已合格。

| 本地位置（均在 `build/`） | 归属 |
|---|---|
| `j2-pitch-capture/` / `j3-pitch-capture/` | 原始手扫CSV；J2旧Roll误选文件保留但不用于Pitch标定 |
| `pitch-zero-ls-20261119_01/fit.json` | J2/J3零点拟合、条件置信区间及假设 |
| `v12-gravity-identify-tracking-20261119/j2_sweep_05.jsonl` | J2双向健康窗口及回扫故障，序列无完整end |
| `v12-gravity-sweep-unlocked-20261119/j345_sweep_01.jsonl` | J345已完成采样及同目录事后无接触确认 |
| `v12-no-motion-rest-gate-20261119/gravity_residual_02.json` | 探索性共享势能拟合；不能按文件名当已部署模型 |
| `v12-no-motion-rest-gate-20261119/gravity_sine_quality_20261120_01.json` | 条件正弦质量及幅相区间，不是正确概率 |
| `v12-j2-gravity-candidate-20261120/` | 之前未烧录J2候选，保留作对照 |
| `v12-usb-unified-20261120/` | 当前已部署2AA07A27…ELF/BIN；`deploy_20261121_01/`含烧前备份和普通运行中验收，3C728955…旧镜像仍留原目录 |

不可重建的CSV/JSONL、用户确认、标定结果、板端回读和烧前备份均保持原位。是否清理可重建的CMake/目标文件，须另行明确范围，不能用目录名或体积推断可删除。

## 历史索引

以下为原 `process.md` 的330条记录：按标题明确日期归档，保留原时区、正文及同日原顺序，不按修改时间猜日期。旧相对路径仍以 `code/` 为基准。最新有效状态只看 [process.md](process.md)。

| 日期 | 迁入原条目数 | 原文 |
|---|---:|---|
| 2026-11-08 | 78 | [process.md](history/2026-11-08/process.md) |
| 2026-11-09 | 40 | [process.md](history/2026-11-09/process.md) |
| 2026-11-12 | 6 | [process.md](history/2026-11-12/process.md) |
| 2026-11-13 | 7 | [process.md](history/2026-11-13/process.md) |
| 2026-11-14 | 47 | [process.md](history/2026-11-14/process.md) |
| 2026-11-15 | 32 | [process.md](history/2026-11-15/process.md) |
| 2026-11-16 | 6 | [process.md](history/2026-11-16/process.md) |
| 2026-11-17 | 31 | [process.md](history/2026-11-17/process.md) |
| 2026-11-18 | 36 | [process.md](history/2026-11-18/process.md) |
| 2026-11-19 | 41 | [process.md](history/2026-11-19/process.md) |
| 2026-11-20 | 6 | [process.md](history/2026-11-20/process.md)，同日继续追加本轮归档记录 |
| 2026-11-21 | 新增部署记录 | [process.md](history/2026-11-21/process.md) |

整理前3298行README全文也保存在 [2026-11-20日档](history/2026-11-20/process.md) 的原文快照区；这是本次归档时间，不是把旧章节重新标成今天。原文SHA-256及保全核对记录一并保留，旧流程只用于历史查证。

原 [mit_20261107/](history/mit_20261107/) 保留原样。不创建第二套 `archive/`、报告或临时目录；后续详细记录只追加至 `history/YYYY-MM-DD/process.md`，根 `process.md` 保持60行以内。
