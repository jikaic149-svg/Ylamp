# 过程历史｜2026-11-19

> Source: code/process.md; original dates, timezones, wording and within-day order are preserved.
> Paths inside original entries remain relative to code/. Historical states are not current instructions.
> Current entries: [process.md](../../process.md) / [README.md](../../README.md).
> Original SHA-256: 4C7CCD31CA17676A9FF4F418397FD5164063B5933E65F67640396A409DCB073B; preamble: # 过程日志 followed by two LF; original records: 41; payload bytes: 160095.

<!-- original-process:begin -->
## 2026-11-19 00:10–00:22（北京时间）｜五轴写板、上电与第一次静态重力试验

- 已按用户授权在唯一目标 STM32H723（UID `00420030-31345106-38373332`）写入五轴镜像。原板 1 MiB 备份在 `%LOCALAPPDATA%\Ylamp\board-backups\v12_20261118_161011_314Z`；首次上电旧双轴镜像不再运行。`build/v12-control/flash_v12.log` 记录 704528 字节验证和运行，全量 `readback_v12.bin` 与 `ylamp_v12_five_axis.bin` 的 SHA-256 同为 `264BFCB9A3804CBD4F280698375ECFBD98C3199DA27E05F2E8597729F999EAAC`；五轴主机回归当时为 24/24。板上 RTOS 和五轴控制周期计数递增，烧后电机未上电时反馈 0/5；未宣称这证明承重。
- 用户报告电机已上电并要求实测。`build/identification/five_powered_20261118_161958409Z.jsonl`：COM36 实收 1953 组连续五槽快照，板时跨度 1952 ms，序号无缺口。J1/J2/J4/J5 各自电机反馈计数增加 195，均为 state=0；J3 旧协议从未回包，并非实测零角。J5 原始角 −80.051°，低于已写板标定下限 −79.657593° 约 0.393°。J4 软件角约 +41.594°，不在本版主动轨迹域。没有放宽限位、改零点或写电机。
- `tools/five_control.py` 使用五轴 USB op16；实板会话确认协议 `0x594C3501`，实际发出一次 `field=7` 返回成功。`build/identification/five_gravity_20261118_162201093Z.jsonl` 中板端仍为 `WAIT`、fresh=`0x1B`、enabled/submitted=`0`、无有效 G。field7 不发使能，不是冷启动。此次未声明独立卸载承重，未发冷使能，且 J5 处于已标定范围之外；未发送运动、失能或复位。**这不是通过的带载重力测试**，没有用四轴失能状态的小幅力矩读数拟合惯性或摩擦。本次上电测试未修改或重烧固件。
- 后续：保持承托，将 J5 原始角移回标定内后，按五轴支持下的冷启动流程取得 J3 新反馈，先核验五轴实际力矩、限幅与位移，再决定运动辨识。当前记录不证明可松手。
- 按电机反馈序号去重：四台各有 196 个不同样本（含窗口起始一帧），不是每台 1953 次编码器读数。重力请求后的 17 个板端状态样本横跨 2014 ms，全部为 `WAIT/0x1B/0/0`，未见非零故障或有效重力向量。独立回归 24/24 及 USB 组包自测通过，不计作带载通过。
- 2026-11-18 11:30（纽约）：在 COM36 再次实际发送五轴重力请求并收到回执。`build/identification/five_gravity_real_20261118_163020866Z.jsonl` 中，之后 2014 ms 的 17 个状态均为 `WAIT`，fresh=`0x1B`，enabled/submitted=`0`；J3 收包计数为 0，J5 原始角 −80.051°。此次板端无可用重力向量，未驱动轨迹、未复位、未失能，不能据此填写承重或摩擦测试通过。手扶不是冷启动要求的独立承重。

## 2026-11-19 08:40（America/New_York）｜J3 重力模型零位测试预检，等待主板 USB 连接

- 需求与范围：用户要求开始测试重力补偿零点。承接上一轮机械基准方案，本轮按 J3 模型角度基准核对准备，不改编码器硬件零位或现有软件零点；接受用户已确认的方向，不扩展其余四轴主动扫动或调参，后续记录五轴实际姿态用于计算 G3。
- 已交付预检结果：电脑当前未枚举到串口，COM36 不在可用列表，尚不能读取实时关节角；已通过进度消息说明该阻碍。未由旧健康记录推断当前板端模式、反馈或使能状态。
- 实际检查：读取根 AGENTS、README 顶部、process 最新机械基准与 build 清理记录、声明式零点、CAD 偏置配置、USB 被动采集工具及会话结束路径；`python code/tools/usb_monitor.py --list` 正常退出但无端口输出，`Win32_SerialPort` 同样为空，Windows 当前 PnP Ports/USB VID_0483 筛选无匹配。未打开串口或 SWD。
- 已核对配置：J3 `zero=27.880494097628134°`、`cad_offset=-95.25529552672751°`，当前契约仍为 `qCAD=qReference-zero+cad_offset`。后续需选 J3 直接父/子刚体的一对独立可测机械基准，取得同姿态 qCAD_ref 与 qSoftware_ref，再检验偏置常量；没有把静止、悬停或力矩过零当作几何零位依据，也没有实测出新偏置。
- 版本与文件边界：`code/build` 实际不存在，与上一条清理记录相符；未恢复回收站、重建 build 或伪造旧实测文件。当前源码已撤除 J3×2，而板上最后记录仍为 C3F4857… 两倍对照；本轮未回读 Flash，不宣称实时核实了镜像。唯一文本修改为追加本条日志，未改固件、模型、标定或工作区。
- 实际未执行：未切模式、按 KEY、发电机命令、中性唤醒、使能/失能、清故障、复位、烧录、主动运动或重复同姿态阻抗采样；未运行与本次预检无关的构建或主机回归，未把配置检查算作实物零位验收通过。
- 当前状态与下一步：预检结果已交付，实时测试暂停等待主板原生 USB 数据连接。提示用户保持现有供电/使能状态，不因连接数据线自行按 KEY 或重启；端口出现后先安排被动角度读取，再确认机械测量基准和可达姿态。现场基准、参考角、重复就位与独立姿态验证尚未完成。

## 2026-11-19 08:46（America/New_York）｜用户授权撤除 J3×2 后重刷，候选通过验证但 ST-Link 未连接

- 需求与授权：用户明确当前板上是两倍补偿，随后要求“重刷”；按上下文执行撤除 J3×2、恢复五轴名义 G 的部署准备，包含必要烧录复位，不扩大为自动使能、中性唤醒、清故障、主动运动或模型零点修改。已提醒烧录/复位会中断补偿，需可靠承托机械臂；现场承托状态尚未得到本轮确认。
- 当前结果：新候选的完整 ARM 构建、ELF/BIN 结构门禁和 27/27 主机回归均通过，已通过进度消息交付；实际写板因调试器连接失败而暂停。没有连上 MCU，未读取实时 UID、运行状态或 Flash，未备份、停核、复位、擦除、写入或进行烧后验收。板上仍按用户确认的 J3×2 及最后 C3F4857… 历史镜像记录处理，不能把离线候选称为已部署。
- 构建产物：使用当前未提交工作树，在新建 `build/v12-gravity-restored-20261119/` 生成 `ylamp_v12_five_axis.{elf,bin,hex,map}`，未从 Git 回退或恢复回收站旧 build。BIN 为 705344 字节，SHA-256 `F06C9E2B5AF6976BB1FC6B949356F5270F3D8BB3229616CCEDDC84137F6155F4`；这是本轮当前源码重新构建的结果，恰与原 J3×2 之前的反向测试版归档哈希一致，不是拿旧台架镜像替代整机。
- 构建配置与行为：Release、GNU ARM 10.3.1、现有 H7_BSP-HEAD、`BSP_HOST_TEST=OFF`、`YLAMP_CAN3_TELEMETRY=ON`、`YLAMP_J3_REVERSE_TEST=ON`，保留最后实板的可选测试入口能力，但不触发反向测试。源码 `FiveControl::observe` 直接使用名义逆动力学 G，`FiveSupport_Sample` 的重力底量也是名义 G，已无 J3×2 配置/乘法；硬件 MIT 和正式 GRAVITY 软件 Kp/Kd 仍为零，阻抗增益、其他四轴、零点、工作区和既有失效行为未改。
- 实际验证：`python tools/check_v12_image.py build/v12-gravity-restored-20261119/ylamp_v12_five_axis.elf build/v12-gravity-restored-20261119/ylamp_v12_five_axis.bin --j3-reverse-test` PASS。在 `build/v12-gravity-restored-host-20261119/` 用 UCRT64 GCC 16.1.0 完整构建并执行 CTest，27/27 通过（0.73 秒），包括五轴控制、KEY、冷启、无声 J3、轨迹准入、参考角和反向入口；不将这些主机测试当作真实带载承重或零位验收。
- 构建接入问题：首次 ARM CMake 参数因 PowerShell 拆分未加引号的工具链路径而失败，改为完整引用 `-D` 参数后成功；首次 host 配置因 UCRT64 运行库未在 PATH 而不能启动编译器，在该命令进程前置既有 `C:/msys64/ucrt64/bin` 后成功。未安装新工具、修改全局环境或更改源码规避检查。
- 连接证据：`usb_monitor.py --list` 无串口输出；Windows 当前 PnP Ports/ST-Link/J-Link/CMSIS-DAP/STM32 筛选无匹配。按 aixprobe 的分层接入原则，本会话没有对应硬件 MCP，复用现有 OpenOCD 0.12.0 开发版，禁用网络调试端口、默认 examine-end 回调和目标 RAM 工作区，`reset_config none`，仅尝试普通只读身份连接。在 init 阶段报 `Error: open failed`，退出 1，证据为 `build/v12-gravity-restored-20261119/preflash_readonly.log`；尚未执行 UID 读取，不能解释成烧后校验失败。不因探针未枚举改用复位连接或重复擦写，结束时 OpenOCD 进程数 0。
- 文件与剩余事项：本轮未改任何固件源码或配置；`joint_references.def`、`five_scene.inc`、`five_workspace.inc`、`five_mit.c`、`Task/five_control.c`、`Mode/five_support.c` 六份 SHA-256 均与清理前记录相同。仅追加本日志并生成上述新构建目录。需接回 ST-Link、主板原生 USB，并确认机械臂可靠承托；随后重新核对实际 UID/出力状态、保存烧前完整 Flash，再使用候选门禁及现有受检烧录入口写入、全应用回读哈希和板端验收，不自动使能电机。

## 2026-11-19 08:54（America/New_York）｜继续撤除 J3×2 部署，运行中备份与身份核对完成

- 需求与当前状态：用户回复“继续”，承接上一条 ×1 镜像烧录授权；已交付连接、UID、完整 Flash 备份及实时五轴出力结果。当前仍在健康重力出力，烧录暂停在写板前，等待现场可靠承托确认；本轮日志与只读检查已收尾。仍不自动使能、中性唤醒、清故障或移动电机；尚未擦写。
- 连接与备份：Windows 已枚举 ST-Link 0483:3748，主板原生 USB 串口仍未枚举。使用既有 OpenOCD/ST-Link 普通运行中连接，禁用默认 examine-end 回调、目标 RAM 工作区和网络调试端口，不 halt/reset；IDCODE `0x10016483`、UID `00420030-31345106-38373332` 匹配目标板，ST-Link V2J43S7、目标电压 3.297377 V（不是电机动力电压证明），核心 Cortex-M7 r1p2。
- 原始备份：`build/v12-gravity-restored-20261119/before_20261119_02/before_flash_1MiB.bin` 为 1048576 字节，SHA-256 `C5592C69B72FB7C3E6000D90EBA8D150F984C29AFF384C19F537BB643080019E`。前 705360 字节 SHA-256 `C3F4857FB1CC574539CD45782438A36FF8A2F839BE3AED2283F72286A23C7D80`，与已记录 J3×2 应用完全一致；只读日志 `preflash_readonly_20261119_03.log` 含正确 TARGET/UID、`BACKED_UP_RUNNING`，退出 0。
- 工具与边界：本轮采用嵌入式工作流、aixprobe 分层核对及 ponytail 复用现有入口；无该技能硬件 MCP，实际仍用项目已有 OpenOCD。首次多条命令经 PowerShell 传参发生 `Unexpected command line argument: $id`，在初始化前退出，未连目标；改用现有 `tools/v12_read_board.tcl` 的检查后回调及新建的 6 行 `preflash_capture_20261119_03.tcl`，避免内联引号解析。未修改生产烧录/读板脚本。
- 候选与 ABI 检查：本轮重新执行当前 ELF/BIN 的 `check_v12_image.py --j3-reverse-test`，仍 PASS、哈希 F06C9E2B…6155F4。准备读取旧板控制状态前，已在实际 C3F4857… Flash 中唯一匹配当前编译 `FiveControl_StatusNow` 的完整 20 字节函数（旧地址 `0x08001F38`），确认其控制对象基址 `0x20002AF0`、初始化标志偏移 476、状态对象偏移 20；不是直接把新 ELF 的函数地址当旧版地址。
- 状态 ABI 与实时只读结果：以相同 ARM 编译器对 `control_abi_20261119.c` 的静态断言实际编译通过，确认短枚举、24 字节反馈、184 字节状态及字段偏移；实际旧 Flash 还唯一匹配 `FiveControl_KeyPress` 的前 32 字节（旧地址 `0x08002158`），含 known_enabled 偏移 472 的读取。随后第二次普通非停机连接，通过现有读板入口及 `preflash_status_20261119_04.tcl` 采集三份 480 字节控制对象 `before_20261119_02/control_{0,1,2}.bin`，日志为 `preflash_readonly_20261119_04.log`，退出 0。CPUID `0x411FC272`、DHCSR `0x01010001`，三份均 `phase=GRAVITY、fresh/enabled/submitted=31、fault/limited=0、gravity_valid=1、cold=0、known_enabled=31、init=1`；控制 ticks 348660→348677、重力时间 1744564→1744649 ms。明确是仍在运行并提交五轴支撑，不是 WAIT，也不将旧无反馈记录覆盖当前事实。
- 当前姿态记录：末份只读控制对象按声明式软件零点解算约 `[56.128609,9.158095,1.222003,-140.583837,-16.600345]°`，五台反馈 state 均为 1。J3 板端支撑 G 为 `-0.2859511 N·m`（仍是两倍版本字段），驱动等效回报约 `-0.2857143 N·m`；这些读数不是独立轴端测力，也没有无外力标签，不能拟合新零位或补偿模型。本轮未主动调整任何轴姿态。
- 文件与剩余边界：除日志及本次 build 内只读脚本/ABI 检查产物外未改生产文件。已有烧录授权保留，但用户“继续”不被冒称为已独立核实机械承托；由于实际五轴仍在出力，不直接停核/复位使其突然失去补偿，不自动失能替代承托。需用户确认机械臂已可靠外部承托、不依赖电机支撑后，再对当前候选重新门禁/核对旧应用、停核备份、刷写、全应用回读与板端验收。当前实板仍为已回读确认的 C3F4857… J3×2，未取得新零位或新的物理承重验收。

## 2026-11-19 09:04（America/New_York）｜用户明确要求执行，J3 名义 ×1 镜像已刷入并全量回读一致

- 需求与授权：向用户说明五轴当前仍在重力出力、烧录会中断支撑并要求可靠承托后，用户再次明确回复“刷”；按该现场继续指令执行本次停核、刷写与软件复位，开始前再次提示保持可靠承托。不把该指令扩大为电机使能/失能、中性唤醒、清故障、主动运动或重新标定授权，不冒称 AI 独立验证了机械承托。
- 当前成果：已通过进度消息交付刷写、完整字节核验、复位运行及烧后 WAIT 状态。部署与只读板端验收已完成，当前版本记录已同步；没有将镜像回读通过视为补偿承重或 J3 零位已校准。
- 烧前检查：当前目录 `cmake --build` 报无待构建项，重新执行 `check_v12_image.py <当前ELF> <同版BIN> --j3-reverse-test` PASS；实际候选哈希仍为 `F06C9E2B5AF6976BB1FC6B949356F5270F3D8BB3229616CCEDDC84137F6155F4`。该候选为本日当前五轴源码重建的名义 G 镜像，并非从旧 Git 或旧台架归档回刷。先前同配置主机回归 27/27 通过，本条不冒称重新跑过。
- 烧录与备份：复用 `tools/v12_flash_checked.tcl`，`V12_CONNECT_RESET=0` 普通 SWD 连接，IDCODE/UID 再次匹配 `0x10016483 / 00420030-31345106-38373332`，目标电压 3.292535 V。停核后在新目录 `build/v12-gravity-restored-20261119/before_write_20261119_01/` 保存完整 1 MiB Flash；该文件 SHA-256 与上一条运行中备份同为 `C5592C69B72FB7C3E6000D90EBA8D150F984C29AFF384C19F537BB643080019E`。擦写前还由 OpenOCD 校验完整旧备份，输出 `V12_OLD_IMAGE_VERIFIED`，其应用前缀对应 C3F4857… 两倍版。
- 实际写入：按候选覆盖扇区擦写并 verify，未全片擦除或修改 Option Bytes。日志 `build/v12-gravity-restored-20261119/flash_v12_gravity_restored_20261119_01.log` 记录 `V12_VERIFIED_BYTES 705344`、`AFTER_FLASH running`，退出 0；仅执行本次授权的 SYSRESETREQ 复位运行，没有使用复位连接。
- 回读证据：`build/v12-gravity-restored-20261119/readback_v12.bin` 与候选 BIN 均为 705344 字节；完整 SHA-256 均为 `F06C9E2B5AF6976BB1FC6B949356F5270F3D8BB3229616CCEDDC84137F6155F4`，另在主机逐字节比较通过。板上已由 C3F4857… J3×2 更新为该名义 ×1 版本，不能继续写成“源码恢复但实板未刷”。
- 行为边界：只撤除临时 J3 支撑倍率；J1/J2/J4/J5、软件零点/CAD 映射、工作区、故障策略和原阻抗参数不变，正式 GRAVITY 硬件/软件 Kp/Kd 仍为零。只保留原已启用的可选反向入口能力，不触发它；未发送任何额外电机或 USB 控制指令。烧后真实使能/输出状态仍以接下来的非停机只读验收为准，不先声称 WAIT、全部失能或五轴健康出力。
- 普通重连验收：使用匹配当前 ELF 的地址与此前 ARM ABI 断言，经 `tools/v12_read_board.tcl` 和本次 `postflash_acceptance_20261119_01.tcl` 回调普通运行中连接，不 halt/reset、不写控制 RAM、不执行运行态 CRC 算法。UID 再次一致，CPUID `0x411FC272`，首尾 DHCSR `0x01010001`；480 MHz、应用 stage=12，CFSR/HFSR=0。任务迭代 150941→151034、HAL 板时 152204→152297 ms，栈剩余 1946 words，控制 ticks 30188→30206。未读取要求暂停解释的完整异常记录，也未把 CFSR/HFSR=0 扩展成全部硬件验收。
- 最新板端控制事实：三份 `postflash_20261119_01/control_{0,1,2}.bin` 均为 WAIT、fresh/enabled/submitted=0、limited/fault=0、gravity_valid=0、known_enabled=0、init=1；五路 received_count 都为 0。当前没有电机反馈，也没有提交支撑力矩，不能把烧前健康 GRAVITY 套到烧后，不能将未收到反馈等同于电机真实失能、动力已断开或无残留输出。USB 串口枚举仍为空，未打开 USB 控制会话，不自动重启、恢复反馈、使能、发 J3 中性帧或清故障；该状态不作为零位测试或承重通过。
- 运行中应用再回读：普通重连取得 `postflash_20261119_01/application_running.bin`，完整 705344 字节 SHA-256 仍为 F06C9E2B…6155F4，与候选/写后回读一致；日志 `postflash_status_20261119_01.log` 含 `POSTFLASH_ACCEPTANCE_RUNNING`，退出 0。全部 OpenOCD 会话已正常退出，剩余进程数 0；软件复位之后未再停核或复位主板。
- 文件与后续：同步根 `AGENTS.md`、`code/README.md` 当前镜像、备份与最新 WAIT 状态，把旧两倍段落标为历史；本轮只增加日志和部署证据/只读验收脚本，六份关键源码/配置 SHA-256 全部与前记录相同。后续测零位仍需按既有流程核实现场供电、反馈、承托及独立已知失能状态，不能盲按 KEY 或把本次刷写当作已重新使能；J3 几何基准、实际新偏置、多姿态与带载补偿均未新增验收，未做独立断电启动或物理按键往返测试。
- 用户后续决定：刷写收尾期间用户表示“可能零点歪了”，要求之后重新标定零点位置。记录为待验证假设与后续 J3 机械基准/角度映射标定计划，不记成已查实根因；保留现有零点及原始记录作对照，其余四轴不追加主动复验或调参。本轮不因此开始动臂、采零、编码器硬件置零、改软件零点/CAD 偏置或再次刷写，待用户准备好后另行推进实测。

## 2026-11-19 09:17（America/New_York）｜双 IMU 零位标定启动，YummyIMU 实时可读、BMI088 采样等待原生 USB

- 需求与授权：用户要求“阅读两个IMU，重新标定零点，允许你驱动电机运动”。已记录本次标定目的下的显式运动授权，不再以完全没有运动授权为阻碍；仍按当前 J3 范围，不扩大为其余四轴主动扫动/调参、工作区旁路、新增自动使能或自行复位重刷。当前尚未发运动，零位标定未完成；已交付实际读数与采样入口缺失结果。
- 已有机械定义：读取并沿用 2026-11-15 已确认的板载 BMI088 服务 J2、J4 附近 YummyIMU 服务 J3，以及两节均以俯仰方向竖直朝上为软件零位，不重复询问朝上/朝下或复验已确认的电机方向。当前用 J2 IMU 作为上游姿态参照，只求 J3 新零位；安装轴与刚体的精确对应仍未有已验收配置，不能凭 Pitch=0° 直接标零，也不能在任意姿态下直接相减两个 Pitch。
- 实际入口与 ABI：完整读取当前 App/五轴 USB/标定模块、两路 IMU 及 UART/供电实现；核对匹配 ELF 的 `BMI088_GetSample`、`YummyIMU_GetStatus`、`BSP_UART_GetStatus` 反汇编。BMI 对象在 `0x200032E0`/124 字节，样本偏移 92、源序号偏移 120；Yummy 对象在 `0x2000336C`/5288 字节，姿态状态偏移 4，RX 计数/溢出用对象的实际 ISR 计数，不误读未更新的显示缓存；UART10 使用第 7 槽，796 字节步长、状态偏移 16，地址 `0x2000691C`。本会话无 aixprobe 硬件 MCP，沿用现有 OpenOCD；没有用 GDB 停核调用、直接写私有标志或重刷来开启采样。
- 原始记录：在 `build/imu-zero-20261119_01/` 增加只读 `read_sensors_01.tcl`，通过现有 `tools/v12_read_board.tcl` 先核对目标 ID/UID 和 CPU running，再采 `application.bin` 及三组 `control_*.bin`、`bmi_*.bin`、`yummy_*.bin`、`uart10_*.bin`。应用完整 705344 字节 SHA-256 为 F06C9E2B…6155F4，匹配当前 ELF/BIN 后才解释 RAM。日志 `read_sensors_01.log` 退出 0；首尾 DHCSR `0x01010001`，未暂停/复位或写目标，GPIOC ODR `0xE009` 只证明 PC13/14/15 使能锁存为高，不冒称测过模块端实际电压。
- 最新五轴状态：三组均 GRAVITY、fresh/enabled/submitted=31、fault=0，五台反馈 state=1、源计数持续增长；该实测取代刚刷后的 WAIT 作为最新状态。AI 没有发使能、中性帧或按键，不能推断是谁或哪一入口恢复了控制。末组软件姿态约 `[93.329149,1.158400,-8.154566,-209.149074,5.999723]°`，J3 名义支撑 G `+0.1661466 N·m`、驱动等效回报 `+0.1587307 N·m`；只记录五轴实际姿态，不将回报当独立测力或据此拟合新零位。
- 外接 YummyIMU 实测：源序号 36362→36373→36383，源时间 473272→473382→473482 ms，观察到约 100 Hz 更新；三组 Pitch 约 −0.640/−0.647/−0.664°、Roll 约 −12.970/−13.004/−13.053°、Yaw 约 −88.595/−88.669/−88.641°，近端原始 USE 文本与这些值对应。RX 字节 837154→837637、RX running=1、续接 HAL_OK；坏帧累计 1、溢出 0，UART 累计错误位为 0x4（FE），未清除或重启 RX。位掩码不增加不代表此间绝无重复 FE，不宣称链路零错误或绝对姿态精度已验收。诊断快照非严格同步高频标定数据，未直接拿它们计算零点。
- 板载 BMI088 实测：initialized=1、failed=0、enabled=0、power_transition=0、sequence=0，表示已初始化且休眠，并非取得了零加速度/零角速度测量。现有 USB op10 可调用 `JointCalibration_SetIMU` 显式开采样；当前实现仅在 FIVE_MOTION 拒绝开启，正常 GRAVITY 可采，不需为此失能或重启。旧 `JointCalibration_Mark` 仍只支持 J1/J2，不能拿它假装一键完成 J3 标定。
- 当前阻碍与下一步：pyserial 枚举无端口，Windows 当前只见 ST-Link、没有主板 Ports；现有安全传感器采样入口尚无数据连接。需接主板自身 Type-C 原生 USB 数据口（保留 ST-Link，不重启），再通过现有正常接口开启 BMI088、取得双 IMU/编码器新鲜稳定配对数据；运动前使用真实当前姿态做原生行程/联动工作区预检，保留正式纯重力零增益，不把开环轨迹计时到位当标定完成。安装映射、真正参考姿态、独立重复就位和新偏置尚未完成；未改变零点、模型、控制策略或标定参数。
- 文件与收尾边界：本轮仅增加上述 build 内只读采集脚本和原始证据、追加本日志并更新 README/AGENTS 最新状态及授权；未改生产源码，未构建、烧录、停止/失能/使能电机、驱动运动、发模块命令、重启接收或清故障。OpenOCD 已正常退出、剩余进程数 0；运动授权保留，当前等待的是采样数据口而不是再次索取运动权限。

## 2026-11-19 09:28（America/New_York）｜原生 USB 接通，双 IMU 已采集，J3 反馈缺失阻止标零

- 需求与范围：用户表示主板原生 USB 已插上，继续上一条双 IMU/J3 零位标定及已授权运动任务。COM36 已枚举为 USB VID:PID 0483:5740、序列 387933623134；不再把接线作为当前阻碍。沿用嵌入式工作流和 aixprobe 普通非停机连接；本会话无 aixprobe 硬件 MCP，复用 OpenOCD。按 ponytail 复用 `five_control.Board`、`usb_monitor.serial_port` 与现有受检读板入口，不改生产接口或固件。
- 先发现的实际控制变化：第一次 USB 状态 `dual_02.jsonl` 于 14:24:49 UTC 已为 DEGRADED、fresh=27、enabled/submitted=0、fault=67，J3 源时 922649 ms/计数 180156，板时 1049205 ms，过期约 126.6 秒；其余四轴 state=0。诊断脚本最初以健康 GRAVITY 阻止只读采样，在发 op10 前退出；确认原生采样明确独立于 CAN 健康后，移除该临时主机门槛，没有修改板端准入或重复发送电机操作。缺失/驱动状态/TX 三个锁存位已在采样前存在，不能归因于本次 BMI 开启或 USB 插线；未定位更早故障原因。
- 实际采集：新增 build 内 `capture_dual_02.py`、`read_dual_02.tcl`、`read_stopped_02.tcl`。唯一采样会话 `dual_02_acquire.jsonl` 于 14:25:28 UTC 经正常 op10 field=1 开启 BMI，一次请求获得 ACK；持续泵 USB，预热 6 秒后以普通 SWD 运行中采完整应用和 100 份控制/标定/双 IMU 诊断块，同时约 4 Hz 记录控制状态。采样结束经正常 op10 field=0 一次请求停止，泵 USB 再等 1 秒完成已有异步收尾。未使用旧版不支持的 op8，未暂停核心调用函数或写私有 RAM 标志。
- 身份与固件：`read_dual_02.log` 目标 IDCODE 0x10016483、UID 00420030-31345106-38373332，首尾 DHCSR 0x01010001；`application_02.bin` 全 705344 字节 SHA-256 仍为 F06C9E2B5AF6976BB1FC6B949356F5270F3D8BB3229616CCEDDC84137F6155F4，匹配当前 ELF/BIN 后才解释 RAM。OpenOCD 两次只读连接均退出 0，没有 halt/reset、目标 RAM 写入或 Flash 写入；BMI 仅经正常接口执行现有功耗寄存器转换，不向 Yummy 模块发命令或更改持久化配置。
- 双 IMU 实际数据：100 份 BMI 均 initialized=1、failed=0、enabled=1、power_transition=0，源序号 1037→1633、源时间 1098499→1104459 ms，约 100 Hz；加速度中位 `[0.2702455,0.1472434,9.7095351] m/s²`、模长中位 9.7146213 m/s²，角速度中位 `[0.1220703,-0.0610352,0] °/s`。Yummy 源序号 96775→97370、源时间 1098500→1104450 ms，同为约 100 Hz；Pitch/Roll/Yaw 中位 `[4.3750005,-0.1340000,-0.1730000]°`，Pitch 范围 4.373…4.378°。这是传感器原轴读数，未完成安装映射或独立机械精度校验，不把近似静止或 Z 向约 g 直接当作机械零位。
- 采集质量边界：两 IMU 源时间差为 −1…9 ms，但 100 份长块的前后源序号括号均跨更新，不是严格原子/同步标定配对，未据此拟合安装相位或新零位。外接在正常 USB 冻结快照中的累计坏帧为 2（早先为 1）、溢出 0、UART 累计 FE=0x4；末尾 RX running=1、rearm=HAL_OK、live RX bytes=2107371。未自动清错、重启 UART 或模块、修改格式/波特率；累计错误掩码不变不证明没有重复 FE。
- 末次控制与资源验收：USB 末次板时 1105523 ms，仍 DEGRADED/fresh=27/enabled=submitted=0/fault=67，J3 源时/计数不变、年龄 182874 ms；各 during_capture 状态亦无恢复，USB malformed/incomplete 均 0。停止后 `read_stopped_02.log` 于板时 1139899…1139906 ms 核对 BMI initialized=1、failed=0、enabled=0、power_transition=0，标定 initialized=1、active/stop_requested/stop_failed=0，序号停在 1639；控制仍 DEGRADED、fault=67，J1/J2/J4/J5 新鲜 state=0，J3 旧 state=0 不作当前独立失能证明。没有把正常停止 ACK 单独冒称硬件休眠完成。
- 交付与文件：已通过进度消息交付双 IMU 实际读数、恢复休眠及 J3 反馈阻碍；同步 README/AGENTS 当前状态，保留更早健康 GRAVITY 和等待 USB 记录为历史。Python 诊断脚本 `py_compile` 通过，采集实际退出 0、完整应用哈希通过；本轮未改生产源码、零点、CAD、工作区、增益或调度，不运行与本次只读诊断无关的全量构建/27 项回归。
- 未完成与下一步：J3 实时编码器缺失，且控制器锁存故障，尚无可接受的零位候选；未运动、改零点或烧录。正常 Acknowledge 需要全部新鲜使能，ColdStart 拒绝锁存故障，不能通过反复 KEY/清错/唤醒或绕过输出所有者假装恢复。需用户确认可靠外部承托、五台电机独立已知失能，并明确本次复位及受检冷启动的恢复授权，才推进恢复和工作区预检；运动授权本身继续保留，不再重复询问一般运动许可。恢复后仍需完成安装映射/独立机械参考、严格配对及重复复测，不能把当前诊断静态姿态写成新零位。
- 收尾检查：`git diff --check -- README.md process.md` 通过，仅提示 LF/CRLF 转换；上述 Python 诊断脚本再次编译检查通过。`five_scene.inc`、`five_workspace.inc`、`joint_references.def`、`five_mit.c`、`Task/five_control.c`、`Mode/five_support.c` 六份 SHA-256 均与本轮开始前相同；全部硬件/USB 会话已退出，OpenOCD 剩余进程数 0。本条日志已保存，采样与记录收尾完成；零位标定本身仍暂停等待上述现场条件及恢复授权。

## 2026-11-19（America/New_York）｜明确 J3 必须使能后读数，核对现有冷启动握手

- 用户澄清：“J3要使能才能读数”。按该已知旧协议行为推进，不将失能静默本身当作 J3 通信故障证据，不再要求 J3 在失能时先提供新反馈；不把这句硬件事实说明扩展为已确认现场承托、五台独立失能或授权整板复位。
- 源码核对与已交付结论：`Task/five_control.c` 的 `FiveControl_ColdStart` 只要求其余四轴新鲜失能，允许缺 J3；提交中性 MIT 后进入 COLD_OBSERVE，静默达到既有等待时间即单次使能 J3，并等待 post-enable 新反馈，再预装真实 G、启控其余轴。没有“必须先在失能状态读到 J3 才能使能”的死等。`tests/five_mit_checks.inc::test_cold(true)` 对此已有专门用例。
- 另一个独立限制：上轮最后读到的 fault=67 属于已锁存的反馈/驱动状态/TX 位，不能据 J3 静默推断这些位的全部起因。`ColdStart` 在 fault 非零时拒绝，`Acknowledge` 仍要求五轴新鲜使能；`Task/motor_usb.c` 当前没有独立 J3 enable 操作。`Mode/j3_feedback_mode.c` 是旧单轴模式实现，不是当前五轴 USB 可调用入口，不借旧版或 SWD 调用绕过唯一输出所有者。
- 实际动作与文件：本轮仅按嵌入式技能核对生命周期/协议源码与测试，更新 README/AGENTS 以防以后重复误判，并追加本日志。没有打开硬件会话、发送任何电机/采样请求、清故障、复位、改零点或构建新固件；不把上轮状态冒称本轮实时重读。
- 验证与收尾：已在现有 host 构建目录实际执行 CTest 的 five-cold/five-cold-silent，2/2 通过（0.03 秒），确认主机模型中的中性预装、静默 J3 单次使能、反馈后交接契约；不是本轮实板使能验收。`git diff --check -- README.md process.md` 通过，仅 LF/CRLF 提示；没有改生产代码或新增构建。日志已保存，说明与记录收尾完成。
- 未完成事项：零位任务仍待现场承托、已知失能及必要复位/冷启动恢复确认；一般标定运动授权继续保留，不重复询问电机方向或朝上定义。本轮未新增任何硬件状态观察或恢复动作。

## 2026-11-19 09:36（America/New_York）｜用户允许承托条件下复位/冷启动，J3 新反馈与五轴重力交接成功

- 用户授权：针对上一条完整确认问题，用户回复“允许”；按其对可靠外部承托、五台独立已知失能及一次复位后原有冷启动的整体确认执行，不再重复索取这次恢复授权。仍不扩大为新故障策略、反复复位/使能、重刷、其他四轴主动扫动或零点直接改写。
- 复位前检查：COM36 `before_reset_03.jsonl` 再读 DEGRADED/fresh=27/enabled=submitted=0/fault=67；普通 SWD 经现有 UID/运行状态门禁取得 `application_before_reset_03.bin`，完整 705344 字节 SHA-256 F06C9E2B…6155F4，匹配当前 ELF/BIN。`pre_reset_03.log` 为 IDCODE 0x10016483、UID 00420030-31345106-38373332、DHCSR 0x01010001，cold/known_enabled=0、init=1，未在健康带载状态贸然复位。
- 一次授权复位：新增 `reset_only_03.tcl`，再次核对原 UID、init、enabled/submitted、cold/known_enabled 后，仅执行一次 SYSRESETREQ 的 reset run；没有写 Flash、RAM 清 fault 或调用旧单轴函数。`reset_only_03.log` 退出 0，复位前板时 0x1a1219，后板时 0x76a、CPU running/DHCSR 0x01010001。`control_after_reset_03.bin` 为 WAIT、fresh=27、enabled=submitted=0、fault=0、known_enabled=0、init=1，COM36 正常重枚举。
- 原生冷启动与实际结果：执行现有 `five_control.py --seconds 2 --log .../cold_start_03.jsonl cold --supported-all-off`，只发一次正常冷启动请求、不回放。退出 0，末次板时 27201 ms 为 GRAVITY、fresh/enabled/submitted=31、fault/limited=0、G age=2 ms；五台均有新 state=1，J3 计数 401、源时 27199 ms，确认使能后已恢复真实新反馈。J3 原始角 0.33970326 rad、名义 G +0.17006335 N·m、驱动等效回报 +0.16849875 N·m；不当作新零点或独立轴端测力。
- 当前进度：已向用户交付复位和实际启控成功。继续用原生 BMI op10 做双 IMU/新编码器采集并预检 J3 小幅路径；准备短数据窗口、前后源序号及重复读核对，不把上一条长 RAM 快照当作严格同步数据。此时尚未发送主动运动、修改零点/CAD/工作区/增益，采集与机械零位复测仍未完成。
- 后续实际采集：复用 `capture_dual_02.py`，仅增加 02/03 运行号参数以保护历史证据，正常 op10 开/关各一次；新增 `read_dual_03.tcl` 对 BMI 32 字节、Yummy 20 字节分别做前后源序号及重复数据读，150 个短窗口全部留在日志。`dual_03_analysis.json`：BMI 一致窗口 123、Yummy 129，两者一致且源差≤20 ms 共 102；实际源差 −6…5 ms。BMI 源 1037→1493、板时 235443→240003 ms，100 Hz；Yummy 源 23437→23893、板时 235449→240008 ms，约 100.02 Hz。应用再回读 `application_03.bin` 全哈希匹配 F06C9E2B…6155F4，首尾 DHCSR 0x01010001，未停核。
- 读数与可辨识边界：上述 102 组 BMI 加速度中位 `[0.27473465,0.14320318,9.70818901] m/s²`、角速度中位 `[0.00213053,-0.00106526,0] rad/s`；Yummy 姿态中位 `[4.50299978,0.53800005,-0.63000005]°`。34 个 USB 控制状态全部 GRAVITY/fresh=enabled=submitted=31/fault=limited=0；软件角中位约 `[93.329149,1.158400,-8.395074,-209.149074,5.999724]°`，五轴编码器跨度均 0。只是有一致性筛选的同姿态数据，不是形式化原子发布、运动辨识或独立机械零位证明；未据 Pitch 数值求零点。
- 新姿态主动预检：`workspace_preflight_03.json` 从 cold 末段五轴真实反馈和原生读回零点计算软件角，先断言各 raw 已在记录分支内，再向既有 host `bsp_tests.exe five-wire` 提交纯离线 P 请求，调用真实 `FiveMIT_Prepare→FiveWorkspace_Line`。5 秒原地轨迹、J3 −1°、+1°、J3 软件零位全部 REJECT，未发送任何板端 staged target/move。原因是非零 J1 适用既有 yaw-box，当前 J3/J4/J5 组合不在该已验证包络；不是证明原地已经碰撞，也不作为停止重力的门槛。未把 J1 假作 0、J4 折算另一圈或修改 workspace 以放行动作。
- 资源与链路实测：`read_stopped_03.log` 板时 0x4333b…0x43342，BMI initialized=1/failed=0/enabled=0/power_transition=0、末序号 1498，标定 active/stop_requested/stop_failed=0；五轴仍 GRAVITY、全新鲜使能与提交、fault=0。本次复位后正常 op14 冻结快照在采集前和末验收都为坏帧 0、溢出 2、UART error=0，末尾 RX running=1、rearm=HAL_OK；不能冒称零丢帧或旧 FE 根因已修复。未自动清错或重启接收；复位本身按原启动流程重置了累计诊断，不能跨复位直接比较累计错误。
- 末次板端验收：`final_status_03.json` 于 14:42:30 UTC，板时 401644 ms，GRAVITY/fresh=enabled=submitted=31/fault=limited=0、G age=4 ms，J3 源时 401639 ms、计数 75289，五台 state=1；Yummy 新序号 40059、age=6 ms、姿态 `[4.583,0.595,-0.829]°`，bad_frames=0/overflows=2/uart_error=0，USB malformed/incomplete=0。最终状态查询未发电机命令，所有会话正常关闭，主板继续名义纯重力，不再复位、失能或重新使能。
- 用户后续提出“同方向扫动，同时记录 IMU 与电机编码器变化，是否就能得到零点坐标”。按标定方案讨论处理，尚未授权绕过工作区或改控制模式。单轴运动可以用于识别编码器—IMU变化映射/转轴，但在安装偏角未知时，绝对机械零位仍需已知安装几何或一次独立的已知参考姿态；二维俯仰简化模型中的截距混合了安装偏角与编码器零点，不把两路原始 Pitch 直接相减。查阅 Bosch BMI088 官方数据手册 §8.3 及原始研究 Sensors 2022,22(24),9850 §4.5（绝对零位需额外已知参考），用于解释该辨识边界，不引入外部实现或改变参数。
- 当前未完成：J3 静默/启控阻碍已解除，不再重复索取本次恢复授权；新的主动路径预检拒绝和机械参考/安装映射尚未解决。没有主动扫动或零位候选，不通过改其余四轴、工作区、纯重力零增益或新恢复策略规避。已交付恢复、双 IMU 读数及预检结果，正在核对文档和保持生产源码未变；零位重标本身尚未完成。

## 2026-11-19 09:58（America/New_York）｜J3/Pitch 串口面板完成，按“自动重刷”部署并实测开始/暂停

- 最新需求与范围：用户改为临时 USB 仅采 J3 编码器和 YummyIMU Pitch、提供开始/暂停，安装偏角暂按 0°；随后明确“自动重刷”并要求加快。按该请求实现与部署，不继续等待安装参考，不把零偏假设记为物理标定；未拟合/写入新零点、改 G、工作区、增益或新增恢复策略。刷前已明确提醒补偿会中断并保持既有可靠承托；该次授权包含烧录复位，不自动重新使能或扫动。
- 实现：`Task/motor_usb.c/.h` 在原 USB v1+CRC 下增加只读 type13（8B nonce+mode）、type93 ACK（18B）和 type94 双角度样本（36B）；不建立控制 SESSION，不调用控制动作。模式 0 原五轴 USB、1 双通道运行、2 观测暂停；100 Hz/40 ms 新鲜度在 `config/usb_config.h`，数据为 J3 raw rad→deg 和 Yummy 原 Pitch，无零点减法或安装角修正。源时/序号/新鲜度和 state 为质量元数据，不发送其余四轴角或 Roll/Yaw。暂停抑制周期 USB，控制与传感器仍运行；DTR/RX epoch 失效恢复原五轴遥测，既有控制会话退出契约不改。无阻塞/堆/新任务，单帧 43B，忙丢本次、不追发。
- PC 工具：新增 `tools/j3_pitch_monitor.py`/`.cmd`，复用 pyserial、现有 CRC/Stream，Tkinter 原生界面有选口、连接、开始/继续、暂停、两角度曲线及自动 CSV。连接先观测暂停并校验版本，只有 ACK 后才进入运行；nonce 匹配，无自动重放/重连/使能，MCU 时间回退停止采集。CSV 保存原始度数、源时间/计数、有效性和 `imu_mount_offset_deg=0` 假设，缺失以空值而非伪造 0°。不自动求零位。扩展 `dm_serial.py`/`usb_monitor.py` 以复用既有分包/CRC，而非另建传输框架。
- 实际检查：新增 `tests/test_usb_focus.c` 并注册 CTest，覆盖正常默认、开始/暂停/继续/恢复、过期/缺源、CRC、DTR、背压不重发和无控制调用。首轮新测试遗漏 `bsp_usb.h` 导致编译失败，补 include 后完整重建成功；首次顺序命令在构建失败后仍跑到旧 27 项加未构建新项，不将其当作本次通过。重建后的实际 28/28 通过（0.41 秒）。Python 协议自测、既有 USB Stream 自测与编译检查通过；Tk 隐藏窗口构造/关闭通过。GUI检查初次对 Tcl_Obj 直接与字符串比较导致测试断言失败，改用原生 `instate` 后通过，未修改界面状态绕过检查。
- 候选：新目录 `build/v12-j3-pitch-usb-20261119/` 用当前工作树完整 Release/ARM GCC10.3.1 构建，CAN3 与既有可选 reverse 入口保持 ON（未触发）；`check_v12_image.py --j3-reverse-test` PASS。BIN 706152 字节、SHA-256 D0C2DA27D374DFAD6B159D5ADFFD9FBEC090004D8C9433475D9BA4B48B4143C2，DTCM 42800 B、FLASH 706152 B；只增加观测入口，仍为完整五轴发布而非台架。
- 烧录与备份：刷前 `before_flash_status.jsonl` 实读健康 GRAVITY、fresh/enabled/submitted=31、fault=0，未冒称全部失能；按新明确刷写授权中断。复用现有 `v12_flash_checked.tcl` 普通 SWD 连接，UID 00420030-31345106-38373332 正确，停核后保存全 1 MiB `before_flash/before_flash_1MiB.bin`，SHA-256 0AFCF1DE78CFDC852CB62C9270147C4BEDFC571B871282DE57257827C54AB944，擦写前完整旧 F06C9E2B… 应用 verify。日志 `flash.log` 有 V12_OLD_IMAGE_VERIFIED、V12_VERIFIED_BYTES 706152、AFTER_FLASH running，退出 0；没有全片擦除、Option Bytes 改写或电机命令。
- 全量回读与运行：`readback_v12.bin` 与候选全哈希相同；普通非停机重连 `running_application.bin` 706152 字节也完全匹配 D0C2DA27…143C2，`read_running.log` 首尾 DHCSR 0x01010001，UID 正确、退出 0。未沿用旧版 RAM 地址来解释新固件。
- USB 实板验收：`python tools/j3_pitch_monitor.py --port COM36 --probe-seconds 2` 实收 200 帧、100 Hz，暂停 ACK 后观测静默通过；末样本板时 56384 ms，Yummy 源时 56379 ms/序号5540/Pitch −0.813° 新鲜，J3 source_count=0、角度空值、state255，未把空值当零。串口关闭后原五轴完整遥测恢复，正常 five_control status 可读，工具没发任何电机/采样开关操作。
- 重要刷后状态：`postflash_status.jsonl` 末次板时 56767 ms 为 DEGRADED、fresh/enabled=27、submitted=0、fault=1、gravity_valid=0；J1/J2/J4/J5 新鲜 state=1，J3 无任何新反馈、真实状态未知。驱动使能不能由 MCU 重刷视作清除；当前没有提交重力支撑，已立即告知用户继续承托，没有自动唤醒 J3、使能/失能、清故障、反复复位或盲按 KEY。恢复启控不包含在此次纯观测工具开始/暂停中。
- 交付：已通过进度消息交付实际刷入/全哈希、100 Hz 与暂停验收及工具入口；README/AGENTS 已更新当前镜像和刷后状态。当前双通道工具已完成，J3 实角度采集仍受其原有启控条件限制；不将通道存在或传输通过冒称 J3 新反馈、物理零位或补偿承重通过。
- 收尾：`git diff --check` 对本次已跟踪修改通过，仅 LF/CRLF 提示；28/28 主机、Python 编译/协议/既有 USB 流自测及隐藏 Tk 构造均已实际通过。未进一步复位或恢复出力，Flash/备份/运行中哈希证据保留于上述候选目录。

## 2026-11-19 10:01（America/New_York）｜用户手动扫动，启动可见上位机并自动记录

- 用户要求“我来扫，你启动上位机”；将主动扫动留给用户，本轮不发任何电机指令。仅给已有工具增加 `--start`：显示界面后连接 COM36，经观测暂停 ACK/版本校验才开始采集；只触发一次，不会在用户后来按暂停时自动恢复，不属于电机自动使能。
- 实际启动：通过 pythonw 以可见交互窗口启动 `tools/j3_pitch_monitor.py --port COM36 --start`，PID 20068、窗口标题 `Ylamp · J3 编码器 / Yummy Pitch`、Responding=True；该窗口持有 COM36，不再从其他客户端抢占或打开控制会话。Python 编译及既有 self-test 再次通过，GUI stderr 日志为空。
- 实际记录：`build/j3-pitch-capture/j3_pitch_20261119_150141_970990.csv` 已创建并持续写入；读尾行 host UTC 15:01:44、MCU 302647/302657 ms，Pitch 157.792° 新鲜（flags=12），J3 count=0/角度空白/state255。只能证明 Pitch 在变，不能用来拟合 J3 零点；已立即告知用户正式扫点前先恢复 J3 新反馈，未把空白当 0°或自动唤醒。
- 当前状态与边界：上位机按用户要求继续运行/记录，暂停按钮由用户控制，安装偏角仍暂按 0°假设；没有再次刷写、使能/失能、清故障、运动或改零位。前述烧后 DEGRADED/缺 J3 事实仍未获新的有效反馈证据，不再打开串口验证以免打断当前窗口。

## 2026-11-19 10:08（America/New_York）｜只读定位 J3 无反馈与 Pitch 不显示

- 用户要求立即读取 J3，随后补充 Pitch 也不显示；按诊断处理，没有自动恢复采集、重新使能或增加电机命令。沿用嵌入式工作流与 aixprobe 的运行中只读诊断要求，不占用 GUI 已持有的 COM36。
- 实板证据位于 `build/v12-j3-pitch-usb-20261119/diagnose_j3_pitch_01*`。使用既有 `tools/v12_read_board.tcl` 普通 SWD 连接，关闭调试网络端口、examine-end 与 RAM work area，未 halt/reset；UID 为 `00420030-31345106-38373332`，首尾 DHCSR 均 `0x01010001`，板时从 654560 ms 前进至 655017 ms。完整 706152 字节应用回读 SHA-256 仍为 `D0C2DA27D374DFAD6B159D5ADFFD9FBEC090004D8C9433475D9BA4B48B4143C2`，按同版 ELF 的 RAM 布局解读，未沿用旧版 IMU/UART 地址。
- 三次运行中快照均为 DEGRADED、fresh/enabled/known_enabled=27、submitted=0、fault=1、gravity_valid=0。J3 底层驱动 `feedback_valid=0`、`received_count=0`，FiveControl 的 J3 缓存计数也为 0，排除了仅上位机丢弃 J3 数值的解释；J3 真实使能/故障状态仍未知，不能把无反馈直接等同于已失能或 CAN 损坏。现有旧协议需要显式中性/使能握手，普通反馈轮询不包含 J3；当前 fault 与其余四轴已使能使正常 ColdStart 门槛不满足，未绕过控制所有者或私改 RAM。
- Yummy 的源时间为 654559→654709→654869 ms，序号 65360→65375→65391，Pitch 为 +4.049→+4.049→+4.047°；UART10 running=1、error=0、rearm=HAL_OK，bad_frames=0、overflows=2 未增长。说明本次观察期间 IMU 与接收链路仍在更新，不是 Pitch 传感器停更。
- USB 观测状态为 mode=2（暂停）、focus_sequence=2469，三次均不变；因此当前没有周期角度样本发往上位机。GUI PID 20068 仍响应，CSV 尾部仍为旧板时 302657 ms / Pitch157.792°，不能把窗口旧值或空白当作当前 IMU 读数。已立即告知用户板上真实 Pitch 约 +4.05°、J3 仍为零接收，以及当前没有重力提交，先保持可靠承托、不要正式扫点。开始/继续只恢复 USB 采集，不会解决 J3 握手或启控条件。
- 本轮仅新增只读证据回调与过程记录；没有修改固件、上位机、零点、重力模型或工作区，没有刷写、自动开始、使能/失能、清故障或运动。J3 恢复需要后续明确授权及可靠承托条件，本轮不将诊断请求扩大为恢复操作。

## 2026-11-19 10:20（America/New_York）｜显式旧版 J3 取数并在上位机验收

- 用户要求按旧版本电机读取编码器并显示，随后明确“允许”。本次在该授权下完成显式 J3 取数，不沿用旧台架、不独立发 CAN、不把观测开始变成自动使能。完整阅读本地厂家 `DM4310_V1.1_Manual.pdf` 第 5–9 页和驱动协议 V1.4 第 37 页，核对 MIT、反馈布局及 FC/FD/FE 含义；J3 配置仍为 CAN1/命令 0x03/反馈 0x13，未写电机参数或零点。
- `Task/five_control.c/.h` 增加唯一所有者的 `FiveControl_ReadLegacyJ3`；`Task/motor_usb.c/.h` 增加 op16/field253 能力及 scope3/field11+显式支持令牌入口。只接受 J3 从未回报、其余四轴新鲜使能静止、无 G、无其它故障的受托状态：零增益/零力矩预装，20ms 后必要时单次 FC，再由中性 MIT 命令应答获取反馈，交回既有 G。每次启动至多一次、500ms 终止，无开机自动使能、清错、失能、复位或自动重试；取消及发送失败停止握手，最后取数命令仍为中性。原全 OFF 冷启门槛与普通失效处理不变。
- `tools/five_control.py` 增加 `read-j3 --supported`，能力/初态检查后只发一次请求，接收 ACK 后继续验收真实 J3 新反馈及五轴提交；不将 ACK 冒充完成。`tools/j3_pitch_monitor.py` 清理重连旧值，暂停时数字明确显示暂停，避免零帧仍显示旧 Pitch；原始角度及 USB 观测契约不变。保留工作树原有独立 `Mode/j3_feedback_mode.*`，未将其台架输出接入整机。
- 增加四个真实控制/驱动测试：成功交接、无回复超时、显式取消及发送失败，验证零增益中性帧、单次 FC、无 J3 新版查询/其它轴使能/失能、无自动清错重试；连同原回归 32/32 通过。首次主机 gcc 缺 DLL 搜索路径而无诊断退出，加入原 UCRT 工具目录后重建通过，不计先前失败为测试成功。ARM 完整构建、既有镜像门禁、Python 协议/流自测、编译及隐藏 Tk 检查通过。
- 部署 `build/v12-j3-legacy-read-20261119/`，706824 字节、SHA-256 `DF7783AB58C30B1C44B99128F345E61F89B932C8AF026A279449DA21B6B039F6`。关闭 PID20068 时正常退出并释放串口；烧前实读 4 路使能、缺 J3、无 G，不假称全 OFF。UID 正确，写前 verify D0C2… 初版，1MiB 备份 SHA-256 `F6273E3D9109E6EE81E93192D860D3C383156C927048D625118FCFCFBA084787`；完整写后及普通非停机运行中回读均匹配 DF7783…。没有重刷旧 Git 或其它台架。
- 实板 `read_j3_01.jsonl` 验收至板时30964ms：fresh/enabled/submitted=31，J3 源时30959ms、计数401、原始角16.294355°，故障1仍保留，未自动清错。重开可见 GUI PID38344 并显式开始观测后，`build/j3-pitch-capture/j3_pitch_20261119_152105_906428.csv` 连续4143帧、100Hz、全部 flags15；J3 随人工扫动变化，Pitch 也更新，GUI stderr为空。进度消息已交付两路实际角度与可用事实。后续进程检查 PID38344 已不在，未擅自重开。

## 2026-11-19 10:27（America/New_York）｜最新人工扫动数据的 Pitch 过零参考

- 用户要求按最新数据计算 J3 零点；沿用用户暂定安装偏角0°，只计算、不先写入。输入上节 CSV 共41.42秒/4143帧，SHA-256 `4A1FADE41C4B1D7A6253C7CA0E68231CBB08A3C75124C365CEB30E6279B82443`，两源时间差4–6ms。原始 J3 范围−58.980946°…111.437614°，Pitch范围−95.240997°…74.490997°。
- 全程简单直线拟合 RMSE 约1.72°、斜率约−0.9488，不能强行用编码器+Pitch常量描述整个大角度过程。选实际符号翻转的相邻两帧对 Pitch=0 作线性插值，三次得到16.6149183908°、17.3833483623°、16.9916448128°，均值16.9966371886°、中位16.9916448128°，重复跨度0.7684299715°；不将跨度冒充绝对精度或置信区间。插值自检和帧数/过零次数断言通过，完整结果在 `build/j3-pitch-capture/j3_zero_20261119_152105.json`。
- 已向用户交付约17.0°候选与0.77°重复跨度，并明确其含义是“当次上游姿态下 Yummy Pitch=0 对应的编码器坐标”。本次 CSV 没有同步 J2；不把它称为已校准的相对关节机械零点。用户质疑 J2 关系时，解释 IMU 世界姿态与电机壳体相对编码器的区别，不自动追加 J2 扫动或修正。

## 2026-11-19 10:33（America/New_York）｜用户确认写入临时参考并启用新零点纯重力

- 用户明确“写入”“测试效果”，随后收敛为“写入然后我来测试效果”“根据新零点做重力补偿”“修改代码完了之后，烧录，我来测试”。按其明确选择写入，不再等待同一授权；主动测试留给用户，AI 未发扫动或持姿轨迹。
- `config/joint_references.def` 仅将 J3 软件参考由27.880494097628134°改为16.996637188634534°，保留 `USER_APPROVED` 并注明临时 Pitch 参考及未验证机械零位；同步 `tests/test_joint_references.c` 的零点与软件端点预期。其余四轴零点、五个原始行程、CAD 偏置、工作区、增益、力矩预算和模型未修改。新旧整应用同为706824字节，相对 DF7783… 只改0x08025690–0x08025692三个字节，属于 J3 参考浮点数；没有发送电机 FE 硬件设零命令。
- 完整目标构建及32/32主机测试重新通过；`check_v12_image.py --j3-reverse-test` 门禁通过。当前候选/实板为 `build/v12-j3-zero-reference-20261119/ylamp_v12_five_axis.bin`，SHA-256 `025425911A8AFB85B5C43C226921957746A7006B8C01FFD6CD35644B732A14EF`，DTCM42816B。烧前 status 证实原五轴均使能且正在提交 G，已提前告知刷写中断补偿并保持承托；不将复位等同于失能。
- 烧写 UID `00420030-31345106-38373332` 核对正确；旧 DF7783… 应用写前 verify，完整1MiB备份 SHA-256 `F0C42F2701BE2FD397CF061C1779E8F84896ACAF9826E64F931F812ED13A3FE5`，路径为新目录 `before_flash/before_flash_1MiB.bin`。`flash.log` 记载 V12_VERIFIED_BYTES706824、AFTER_FLASH running，`readback_v12.bin` 全哈希相同；未全片擦除或改 Option Bytes。
- 刷后仅显式执行一次 `read-j3 --supported` 恢复旧版反馈，`read_j3_01.jsonl` 板时33865ms已为fresh/enabled/submitted31、J3计数201、G age1ms。随后依据本次明确恢复纯重力的要求，在五轴新鲜、已提交且静止后单次调用原 `acknowledge`，不是新增自动清错策略；`gravity_acceptance_01.jsonl` 最终板时46715ms为GRAVITY、31/31/31、fault/limited0，J3模型支撑−0.0958732441N·m、驱动等效回报约−0.105006N·m。用户可测试手感，回报力矩不等于外置传感器或物理承重已验证。
- 板端标定表读回 J3 为0.2966472804546356rad（16.9966371741°），其它四轴及五组端点逐项匹配。新参考通过原 `observe` 中 raw−zero+CAD 直接用于全模型；名义五轴 G、硬件/软件 Kp/Kd=0，不加倍、不加阻抗/摩擦。耦合模型的其它输出会自然随新q3改变，不把“其余轴参数未改”误称为“其余力矩恒定不变”。
- 普通SWD运行中重连全应用回读仍匹配025425…14EF，首尾DHCSR0x01010001、板时前进；`running_control.bin` 仍为GRAVITY、31/31/31、fault0、有效G，J3计数19244，随用户姿态变化的G3约−0.674374N·m。没有为最终核对再次停核、复位、刷写或发运动。
- 验收状态检查首次把异步ACK后的第一帧也强制要求新 phase 而断言失败：该帧已 fault0/31/31/31，但 phase 尚是上个5ms周期的DEGRADED。保留该真实过渡记录，后继8帧均GRAVITY、0故障0限幅；末5帧连续健康检查通过。没有修改控制逻辑或删掉过渡数据掩盖失败。
- 已通过进度消息交付“新零点已刷入且正在纯重力、用户自行测试”，未重复烧录。README/AGENTS同步当前实板、数据定义及权限边界。本次软件部署和板端提交验收完成；未验证事项为用户手感/无承托补偿效果、独立机械零位和更换上游姿态后的物理一致性，不据此宣称正式标定通过。

## 2026-11-19 10:46–10:51（America/New_York）｜J2 同流程手扫观测；按用户纠正使用 Pitch

- 用户明确切到J2零点测试，并要求“还是和刚才一样的流程”：上位机开始/暂停、用户手扫、之后计算候选，不自动写入。最初准备普通运行中静态配对，在用户收敛流程后取消该方案；`build/j2-zero-check-20261119_01/` 仅创建目录，没有在其中采样、使能IMU或发电机请求。沿用嵌入式/最小实现要求，复用现有USB帧、GUI、原生工具链和BMI会话，不增加线程、主动运动或标定写入。
- `Task/motor_usb.c/.h` 在原type13下增加模式3/4（J2运行/暂停），与J3原模式1/2保持兼容。仍用36字节样本及源时/序号/有效标记；BMI数据经现有Getter获取，既有5秒预热未完成时不置IMU-fresh。首次显式运行才取得临时BMI会话，暂停保留采样；离开该通道/DTR结束时单次请求既有异步STOP，不抢占外部active/stopping/failed会话。此入口不调用任何电机动作、零点捕获或写入，不改变五轴重力。
- `tools/j3_pitch_monitor.py` 复用原界面，增加 `--joint 2`：对应版本/模式协商、J2标签和CSV目录 `build/j2-pitch-capture/`，保留开始/暂停/单次自动开始及无重试。预热明确提示，J2 CSV字段与J3独立命名；J3默认入口不变。扩展主机USB测试覆盖J2角度、预热、开始/暂停/继续、外部会话BUSY及DTR收尾，验证无电机控制调用；Python增加两种通道版本协商/分帧测试。
- 首版错误沿历史记录选了Roll，实际部署 `build/v12-j2-imu-usb-20261119/`，708528字节，SHA-256 `60133A37BE3131E3AA59D0C9D0665A5041971AEDF8B01C148354D59EC8FBB0B6`。门禁、32/32、整镜像回读与健康五轴G通过，烧前备份SHA-256 `87ACDFD756B3738F7D879BDB007D9C61F0BB74EC4B8EAA5C7BFA875F47937805`；GUI PID13456记录 `j2_pitch_20261119_154612_405836.csv` 的列是`bmi_roll_deg`。用户明确“不是roll，是pitch”，该首版及数据不作为本次Pitch零位依据；保留历史，不重命名或假称选轴验证通过。用户手扫时其编码器与Roll回归近乎不相关，也未据此拟合零点。
- 立即改用标准加速度倾角 `Pitch=atan2(-ax,hypot(ay,az))`，度数范围−90°…+90°，检测零加速度向量无效，不加安装/传感器偏置。界面与CSV改为BMI088 Pitch/`bmi_pitch_deg`，J2协议能力由0x594C3201升为0x594C3202，旧Roll客户端不能误读新Pitch。这个量不是陀螺融合姿态或动态加速度补偿；用户需缓慢手扫和近零静止复核。本轮没有选择/写入任何J2候选零位。
- 修正后的目标构建、镜像门禁、32/32主机、Python协议自测及隐藏Tk的Pitch标题/CSV字段检查均重新通过。当前 `build/v12-j2-pitch-usb-20261119/ylamp_v12_five_axis.bin` 为708568字节，SHA-256 `36083A9F6417461AF1E697C55C42FC6FD1AACFD767F465C106B66734CD726C4A`，DTCM42824B。五个零点/原始行程、五轴模型、CAD偏置、增益、工作区均未变；J3仍16.996637188634534°，J2仍146.82395937988517°。
- 正常关闭旧GUI PID13456释放串口，刷前实读健康GRAVITY、31/31/31、fault0，提醒刷写中断补偿并保持承托。目标UID正确，写前verify旧60133…，完整1MiB备份SHA-256 `881C929B123F7AEE12A46BBDD773254088803A774CDCA4ED891CA7C091601D3D`，保存在当前构建 `before_flash/before_flash_1MiB.bin`。写后全BIN与普通运行中重连的708568字节应用回读均匹配36083…6C4A；首尾DHCSR0x01010001、板时推进，没有为最终回读停核或复位。
- 按同一部署交接显式单次 `read-j3 --supported` 恢复旧电机取数，再通过原 `acknowledge` 一次确认；`gravity_acceptance_01.jsonl` 最终板时53485ms为健康GRAVITY、fresh/enabled/submitted31、fault/limited0。BMI开启后运行中RAM仍为GRAVITY/31/31/31/fault0；BMI initialized1/failed0/enabled1/power_transition0、样本序号429。没有追加任何主动轨迹、其它轴调参或自动恢复策略。
- 已启动新的可见GUI PID10944，标题“Ylamp · J2 编码器 / BMI088 Pitch”，Responding=True、stderr空；该窗口持有COM36，后续不抢会话。新CSV `build/j2-pitch-capture/j2_pitch_20261119_155101_652414.csv` 已从预热进入双路flags15。实读末行seq2170/板时90117ms、J2源时90109ms/计数12725、BMI源时90116ms/计数2155，原始角149.818405°、Pitch−3.705052°；源序号随时间前进，不是界面旧值。已向用户交付“已纠正为Pitch、可手扫后暂停”，保持传感器临时会话供用户采集，不自动停采/重启或改零点。
- 收尾范围：软件与部署/数据通道验收完成，等待用户扫完、暂停后计算J2候选；旧Roll数据排除，动态Pitch精度和机械绝对零位未验证。README/AGENTS同步当前镜像、正确Pitch通道及串口所有者，现有未提交实现和全部历史数据保留。

## 2026-11-19 10:52–11:08（America/New_York）｜J2/J3 全样本最小二乘与新零点模型复核

- 用户追加要求两份有效Pitch数据全程最小二乘、置信率/区间、求新零点后复核历史模型并优化；不再只取过零平均。使用embedded-firmware-workflow及ponytail，复用NumPy、五轴RNE、原构建/烧录入口；没有新增依赖、主动扫动或自动恢复策略。当前时间按机器换算America/New_York；用户环境日期另报11月20日，但本次实际采样文件时间为11月19日UTC。
- 新增只读离线 `tools/fit_pitch_zero.py`，严格识别J2 BMI Pitch/J3 Yummy Pitch表头，不接受Roll；检查flags15、state1、有限值、安装偏置0、源年龄及单调源计数。两份原始CSV保留，J2 5666行中515行预热排除，5151有效配对/51.50s；J3 4143有效配对/41.42s。源SHA256分别为52E18CC0E953E4AD1C0B3D08E91D54CE9346E3F706917E8304B30469B2057D80、4A1FADE41C4B1D7A6253C7CA0E68231CBB08A3C75124C365CEB30E6279B82443。
- 全有效样本等权拟合Pitch=a*(raw-zero)：J2 a=-1.0142411969223166、zero=146.18099179755507°、RMSE0.494150°、R²0.999755532；J3 a=-0.948798142944831、zero=17.291374640739114°、RMSE1.719489°、R²0.997245032。与用户提出的raw+Pitch对应的固定斜率-1 LS也已计算，分别146.14790475388244°/17.81118785541254°，RMSE0.664126°/2.464434°；未偷偷将两种估计混用。
- 以2秒连续块pairs bootstrap、4000次、固定种子计算条件95%区间：J2 [146.13693203109634,146.21702849982722]°，J3 [16.803991925013968,17.883463485623004]°。同时报告1/4秒分块敏感性、双方向拟合、2°角箱均衡及五折时间块验证；4秒J3区间扩大至[16.74927315445498,18.19887887842133]°。J3残差有角度相关性、方向/停留权重影响，区间只属于本次仿射拟合，不是机械零点正确概率或独立复测。结果 `build/pitch-zero-ls-20261119_01/fit.json`，工具deploy=false且不连接串口。
- 已通过进度消息交付全样本零点、R²及条件95%区间。按用户要求将共享J2/J3参考更新为上述自由斜率LS零点（仍USER_APPROVED临时候选），保留J1/J4/J5、原始端点、CAD偏置、五轴刚体表、Kp/Kd/Fc/B和失效策略；不发送硬件FE，不把回归斜率当编码器比例或力矩倍率。
- 当前模型链复核：Task/five_control.c的原地G与Mode/five_mit.c的动态RNE均使用(raw-zero)+CAD；故共享零点一次更新同时移动G、M和速度耦合的输入相位，不能再把CAD偏置同量移动抵消或双加。D:/MuJoCo/Ylamp/gravity_formula_review.md、gravity_model.py与源码已阅读；历史双轴sin(-0.579557...)、旧摩擦及output/arm5_dof理想化模型均不是当前V1.2装配，且新CSV无力矩/加速度标签，不能声称重新辨识其质量/质心/惯量/摩擦。
- 发现Model/five_workspace.c的“实测行程内缩1°”原为旧零点硬编码：现在增加由joint_references.def直接生成的实测行程交集，并裁剪J3Band输出；冻结模型域/表格/路径/yaw箱均不扩大。旧J3 -86.73°/-93.18°路径已越过新实测下界，回归改为验证拒绝，并用同片域内-80°到-82°继续验证正常轨迹/协议/速度预算。此变更仅维护原行程约束，不进行实机轨迹。
- 快速及主机验证：离线LS合成自测通过；dm_monitor（修正其陈旧J3参考期望）、j3_pitch_monitor、usb_monitor自测通过；3项five_identify离线回归通过；32/32 CTest重新通过。当前处于收尾进行中，目标新构建、模型数值对照、写板/回读/板端验收尚未执行；实板仍36083…Pitch观测版，不能把源码更新当作已刷入。
- 后续已完成：新增 `tools/urdf_collision/check_zero_model.py`，复用D:/MuJoCo/Ylamp既有公式、场景与原生库，并将当前C算法编译成离线DLL，在103姿态验证G、完整逆动力学、Python回归器、势能梯度、M对称正定。最大C重力误差4.53404e-7N·m、C动态4.86536e-7N·m、Python RNE8.44e-15N·m、势能梯度7.33e-11N·m，最小M特征值0.002594567kg·m²；没有修改外部模型或积分/驱动硬件。首次调用不存在的JointReference.reference抛AttributeError，立即改用已有angle()+zero后重跑通过；未把失败当通过。结果 `build/pitch-zero-ls-20261119_01/model_review.json` 同时保存历史sin/cos/n次谐波相位换算式及不适用旧双轴拟合的原因。
- 同一实读原始姿态下，新模型预测G2/G3为−0.293124/−0.159187N·m，旧参考预测−0.268180/−0.153405N·m；差值是零点相位改变的模型预测，不是独立载荷测量。本次只改变两轴参考和轨迹行程交集，没有凭Pitch斜率缩放编码器或力矩。
- 新目标构建与ELF/BIN门禁通过：`build/v12-pitch-ls-gravity-20261119/ylamp_v12_five_axis.bin`，708448字节，SHA-256 `3023FD67CD7EF7E87DC56FBFF1A5672C6521E3262EA0E26B39C5F7A2AB02A175`，DTCM42824B。沿用CAN3与可选反向测试编译项但未触发反向入口。新代码未引用旧台架镜像。
- 按之前“代码改完烧录、用户测试”授权流程，先检查GUI原PID10944已退出、不再占COM36，再通过原status实读健康五轴G，提醒刷写中断支撑并承托。flash.log核对UID00420030-31345106-38373332、旧36083…应用写前verify，完整1MiB烧前备份SHA-256 `E5EEBAF1DEBE29F087AB909E2AAC7AEA3188C36A2E621C64E437D2989A5E87DF`，位于新目录before_flash；写后全708448字节readback匹配候选并运行。未全片擦除或改Option Bytes。
- 刷后显式一次 `read-j3 --supported` 获新J3计数201并交回G，再显式一次原ack确认，`gravity_acceptance_01.jsonl` 末五帧连续健康，最终板时34025ms：GRAVITY、fresh/enabled/submitted31、fault/limited0、G age1ms。回读J2=146.18099269217686°、J3=17.29137520469598°（float32），匹配新参考，其余参考/端点未改。G2/G3约−0.293136/−0.159198N·m；这证明板端在提交新模型，不证明无承托物理补偿已验收。
- 已进度交付“新零点已刷入、健康五轴G、用户现在可测试”。随后按新ELF地址进行普通只读SWD重连，无halt/reset，DHCSR首尾0x01010001，板时61352→65945ms推进；`running_application.bin` 全应用哈希再次匹配3023…A175，3份control快照均GRAVITY/31/31/31/fault0/limited0。没有重复烧录、自动使能/恢复、硬件FE设零或实机轨迹，未自动重启已关闭GUI。
- 收尾：README和AGENTS同步当前实板、两份LS结果、置信区间限制、模型及串口状态。Python语法检查通过；全树git diff --check发现既有config/motor_config.c:44末尾新空行（本任务未改该文件，不顺手修复）；本任务相关文件单独检查结果在下条补记。软件实现/部署及板端提交验收完成，未覆盖独立机械校准、J3系统残差来源、真实无承托手感/承重；没有力矩标签的物理参数重新辨识未实施，不能称已完成。
- 最终补检完成：本任务已跟踪文件单独diff --check及全部12个相关文件尾随空白检查通过；离线LS自测再次通过，真实旧Roll CSV已验证被新工具明确拒绝。保留全树既有无关空行问题。成果、必要测试、实际写板/回读及本次日志收尾均完成；后续由用户测试实物，不自动追加采集或运动。

## 2026-11-19 11:14（America/New_York）｜用户要求略加强J3，支撑G×1.10

- 用户要求“加强一点J3的力”。按小幅调整解释为当前J3重力支撑+10%，进度已明确告知倍率、其它四轴/新零点不变和刷写中断需承托；沿此前改代码后烧录、用户测试流程，使用embedded-firmware-workflow和ponytail最小修改，不主动扫动。
- `config/five_mit.c` 增加声明式 `g_five_j3_gravity_scale=1.10f`，`Mode/five_support.h` 声明；唯一输出所有者 `Task/five_control.c` 在新鲜完整五轴模型G计算之后、原有预算/故障处理之前仅乘J3一次。缓存降级G不重复相乘，GRAVITY/IMPEDANCE共用同一增量基底，原1.8N·m预算及FIVE_LIMIT语义不变；动态M/c和主动轨迹前馈、模型参数、CAD和零点不改。状态gravity_nm含实际支撑倍率，头文件注明，不冒称新模型辨识结果。
- 回归 `tests/five_mit_checks.inc` 验证同一姿态J3精确×1.10、其余四轴等于名义G、零PD及真实MIT帧一致，测量速度反向不引入阻尼；既有缓存、阻抗、反向、单次旧J3恢复及失效测试继续通过。主机32/32已实跑通过。
- 新目标 `build/v12-j3-gravity-x110-20261119/ylamp_v12_five_axis.bin` 已构建/门禁通过，708456字节，SHA-256 `B003DCD66476EB094F9A1E36B22D27B2E6E7C21032DB1B191661565E00F1176C`，DTCM42824B。当前仅候选完成，板上仍3023…名义版，写板/全回读和板端提交验收待执行。
- 烧前status已实读原板健康GRAVITY，板时514430ms、fresh/enabled/submitted31、fault/limited0，J3旧支撑−0.198584646N·m；GUI进程不在，COM36可用。没有清错、重新使能或其它主动运动。
- 部署完成：UID00420030-31345106-38373332核对正确，旧3023…应用写前verify，完整1MiB备份SHA-256 `614EFDFE15635220052DB9A4D64F0CC48870F9AA40B876AEF6FBBD8785D93A9A`；新708456字节写后readback与候选完全一致。显式一次旧J3取数后五轴fresh/enabled/submitted31、limited0，J3支撑−0.218443111N·m，相对同姿态旧−0.198584646即×1.10。五组参考/端点回读未改；由实测姿态复算名义G3−0.198584568，倍率1.100000434，其余四轴误差低于1.3e-7N·m。
- 重要未完成：本次仅一次原ack确认被rc2拒绝，启动缺反馈锁存fault1尚在，不能宣称健康GRAVITY。请求附近J3反馈速度−0.051282rad/s超过原still阈值0.04，符合静止条件未满足的现象；没有原子记录拒绝分支，不能排除瞬时提交等其它NOT_READY原因。保留失败 `gravity_acceptance_01.jsonl`，没有删除/重写或放宽门槛、自动重发ACK/FC/复位。后续只读 `after_rejected_ack_status.jsonl`末次板时35186ms为DEGRADED/fresh31/enabled31/submitted31/fault1/limited0，持续实际支撑；已向用户明确报告。
- 新ELF确认c和uwTick地址后普通运行中SWD全708456字节回读仍匹配B003…176C，首尾DHCSR0x01010001、板时124091→128708ms推进，没有halt/reset。RAM快照校验首次断言要求所有异步c快照submitted31失败，第二份读到submitted0（Tick每周期清零再提交，长结构转储非原子），fault仍1且没有新TX位；保留真实快照，不将单个周期内读数当新的故障或健康证明。协议完整帧已证实持续31提交，未为此重发控制。
- 用户随后问“你确定对零点两侧的力都做了不同方向的处理吗”。按只读验证处理，核对observe带符号G、正倍率和发送路径，没有abs(G)或按软件零点强制切正负。固定最新完整协议记录的其余轴（软件角约[93.219866,21.822372,*,−240.732333,−12.644258]°），离线G3=−0.3364323731*cos(q3)−0.9034120518*sin(q3)，q3为软件弧度；软件−5°/0°/+5°对应×1.10支撑−0.282056/−0.370076/−0.455279N·m，均负，最近模型力矩过零软件−20.425427°。结果保存新构建zero_side_check.json，未发送这些姿态、不绕过工作区。
- 向用户进度明确“不能确认新Pitch零点两侧已经正确反向”，没有把模型数值一致性当实物方向验收。Pitch参考、CAD相位及下游质心/其它关节姿态的关系仍需核验，不从单轴标定直接假定所有姿态G3(软件0)=0，也不擅自硬切符号或再次刷写。理论对照读取MIT Underactuated Robotics多体动力学原文，补偿为势能梯度，耦合关节的力矩零位不必等于单个编码器参考零点。
- 状态：倍率部署/全应用回读和只读两侧计算完成；健康模式锁存确认、实物两侧方向及手感验收未完成。README/AGENTS同步当前B003…镜像与DEGRADED，未覆盖或回滚其它五轴工作树实现。没有新增自动恢复、实机运动或参数重辨识。

## 2026-11-19 11:42（America/New_York）｜新Pitch零点的CAD偏置修正及代码复核

- 用户明确“修改，然后重新烧录”“检查所有的代码”，并再次指出新零点必须同步CAD偏置。使用embedded-firmware-workflow和ponytail，范围仍为整机五轴中的J3相位，保留J3×1.10、原预算、零PD、原输出所有者和失效/恢复流程，不追加主动扫动。源码成果已进度交付，收尾进行中；板上此时仍B003…旧CAD版。
- 从J3采集期间 `build/v12-j3-legacy-read-20261119/running_control.bin`（UTC15:21:10，CSV开始后约5秒）取回五轴原始姿态约[4.600901,154.758085,16.294355,-141.862469,-68.204562]°；近采集结束后的status中J2只差约1编码步。没有全程同步五轴记录，仍以该记录作为本次固定上游基准，不冒称全程同步验证。
- 新J2参考146.18099179755507°使上述J2软件角为+8.57709290713535°。沿原模型“关节中心连线最大高度”的upright定义，J3 Pitch零位对应CAD应为−95.25529552672751−8.57709290713535=−103.83238843386283°，不是旧同时双轴软件0的−95.255296°。此前“更新参考时CAD总应不变”的无条件表述已被本次校正；这里按独立上游记录确定映射，不是同量抵消J3新参考或强制按软件角翻符号。
- 新增共享 `config/five_coordinates.h`；硬件/仿真profile及Python模型统一消费，G/M/c使用同一新相位。`Model/five_workspace.c`公共入口先检查当前实测行程，再按 `q_table=q_live+CAD_live−CAD_table` 查询冻结证据；J3区间转换回当前软件角并交集行程。表格、路径、yaw箱在CAD空间不扩大；测试fixture显式转换坐标，不放宽原门槛。
- 快速模型核对：该上游基准下J3软件−5/0/+5°的×1.10支撑为+0.141638/+0.049545/−0.042925N·m。零处小非零值来自原CAD下游质心/腕部，不把Pitch零位伪称任意腕姿下的全子树质心平衡。最初将“最大高度”误当“整条中心连线完全竖直”的检查失败；保留真实结论，改为验证该定义的高度一阶导为0且为极大，已通过，未因此改模型参数。
- 当前生产调用链、设备协议/USB/控制及模型已复核，GCC16.1 `-fanalyzer` 主机目标构建通过、更新映射后32/32 CTest通过。不是每个历史台架文件/第三方库逐行审计通过。发现旧离线 `dm_fit_identification.py --self-test` 因已移除双轴配置 `.friction_smooth_rad_s` 查找失败；旧 `test_calibration.c`未注册当前CTest，不能声称其覆盖。均非当前整机已链接路径，本次不顺手重构旧双轴。
- 工具同步：修复check_five_wire旧CAD硬编码、check_yaw_box软件中心、workspace_sim陈旧四轴参考断言及重新生成时的CAD来源；check_zero_model增加前版CAD参数以准确比较实际旧/新映射。历史模型几何证据不重生成或覆盖。已追加新映射数据回归，Python回归、目标构建、门禁、写板/回读及板端验收待执行。
- 补充本轮只读证据：`build/v12-code-review-host-20261119/pose_imu_01.json`三组Yummy新鲜Pitch约+1.961°，同时J3软件角约−13.586595°，说明软件相对参考不是随上游变化保持的世界Pitch；不能用当前软件角正负冒充实物Pitch两侧。未发运动/中性帧/使能。上条“读取MIT原文”未取得可用网页正文，不能作已验证来源，本轮结论以本地CAD势能/动力学源码和实测记录为据。
- 收尾验证已完成：4/4 Python unittest（含上游基准最大高度及两侧连续G回归）、dm_monitor/j3_pitch_monitor/fit_pitch_zero/usb_monitor自测通过，所改Python语法检查通过。103姿态C/RNE/CAD/势能梯度/质量正定对照通过，最大C重力4.53404e−7N·m、动态4.86536e−7N·m；4组真实MIT量化帧离线闭环各6000帧，最大误差0.0332909°。结果在新构建model_review.json/wire_review.json；没有驱动物理机构，不称实物标定验收。
- 新目标 `build/v12-j3-cad-pitch-20261119/ylamp_v12_five_axis.bin` 全量编译/链接及门禁通过，709216字节，DTCM42824B，SHA-256 `9317BB0D17B777D555756A863B195FF7B5ED8AD4FD4D1BEEDCCA6A7F9210A183`。沿用CAN3和可选反向测试构建项，未调用反向入口。板上参考不变，新CAD3相位使qCAD3=raw3−121.123763074602°。
- 烧前只读status实见B003…板仍DEGRADED/fault1/31/31/31/limited0；Python GUI未运行，不抢COM36。已向用户进度提醒承托和刷写支撑中断，随后按“修改重刷”授权执行一次写板，UID00420030-31345106-38373332核对，旧B003…写前verify，完整1MiB备份位于新构建before_flash/before_flash_1MiB.bin，SHA-256 `13170786E431303DB689C659AC82DE3C5A176F57CE5B8F172544DD4A60B545B2`。写后709216字节回读全等候选；未全片擦除或改Option Bytes。
- 刷后显式一次read-j3 --supported取得旧J3新计数201并交回G，一次原ack成功（没有放宽阈值或重试）。gravity_acceptance_01.jsonl末五帧均健康，最终板时29877ms为GRAVITY/fresh31/enabled31/submitted31/fault0/limited0；五轴参考回读与新共享值一致。最终支撑向量约[0,−0.474507272,+0.085073963,−0.002193355,−0.046868101]N·m，按记录编码器及新CAD复算[ G1,G2,1.1G3,G4,G5 ]最大误差5.17e−8N·m。其它轴参数不动不等于冻结耦合G：修正下游姿态后G2/J4/J5数值随模型正常改变。
- 已进度交付“已烧录、健康五轴纯G、用户可测试手感”。按新ELF确认c=0x20002af0/496B、uwTick=0x20007e3c后普通只读SWD重连，不halt/reset；板时84568→88865ms推进，DHCSR首尾0x01010001。running_application.bin全709216字节哈希再次匹配9317…A183，profile CAD回读[0,37.131618499756,−103.832389831543,0,0]°；三份RAM均GRAVITY/31/31/31/fault0/limited0。
- README/AGENTS已同步新版、CAD基准、工作区换算、健康状态、备份与旧版历史；保留其它用户五轴工作树。状态：实现/测试/目标构建/写板/全回读/板端实际提交及本次日志已完成。未覆盖独立机械零位/安装偏置复测、全程上游姿态同步、无承托实物两侧和手感、真实参数重新辨识；留给用户实测，未追加运动/自动恢复/重复使能或GUI重启。

## 2026-11-19 11:50（America/New_York）｜J3正侧不足、负侧过强的诊断建议

- 用户实测反馈“J3正轴的补偿力不够，负轴力偏大”，询问想法。本轮仅分析，未修改源码/控制配置、未连接串口/SWD、未发电机命令、使能/清错/复位或烧录。沿embedded-firmware-workflow只读诊断及最小改动原则；当前镜像仍上一条已验收的9317…A183，不将上次健康状态冒充本轮新实读。
- 核对现用five_coordinates.h、five_mit.c、FiveControl.observe及FiveSupport_Sample：纯重力基底是同一有符号G3×1.10，没有按角度两侧分倍率，没有额外恒定偏置或速度摩擦项；原预算/零PD不变。
- 离线复用原记录五轴标定基准，固定上游/腕部，仅改变J3软件角：−20/−10/−5/0/+5/+10/+20°的支撑分别+0.408691/+0.232652/+0.141638/+0.049545/−0.042925/−0.135068/−0.315577N·m。该固定姿态曲线可写为0.0495448785433*cos(q3)−1.0588085969566*sin(q3)，单位N·m和弧度；±5/10/20°成对偶/奇分量核对通过。最近模型力矩零点约软件+2.679089773°，只是该腕部姿态的模型质心平衡，不是新的实测编码器零点或应直接写入的CAD修正。
- 暂将用户正/负理解为角度两侧时，残余相位/质心或外部偏置是优先检查项，而不是继续统一增大倍率；现有模型本身有偶分量，但没有真实所需力矩标签，不能据此认定偶分量错误或直接删掉。若用户指同一角度往两个方向运动的手感，则需优先区分方向相关摩擦/线缆滞回，不把运动方向混作角度分区。
- 建议先确认正/负含义，再由用户在可行姿态固定其余轴做两侧成对停点及同角度双向慢扫观测，记录五轴实际姿态、Pitch、反馈/指令力矩和漂移。手持时回报力矩不能冒称真实重力标签；原Pitch角度CSV不能独立辨识重力幅值/质心。未安排或执行主动轨迹、不绕过原工作区，不为调手感任意改已标定参考。
- 状态：只读源码核对、离线模型奇偶分量计算和建议已完成；无源码变化，未重复无关全量构建/硬件测试。根因仍待“角度两侧/运动方向”的定义和成对实测区分，未宣称物理故障定位完成。网页检索未返回有效正文，不用作本轮证据。

## 2026-11-19 11:54（America/New_York）｜确认正/负为J3零点两侧的角度

- 用户明确正侧不足、负侧过强指角度两侧，不是同角度双方向拖动。本轮沿embedded-firmware-workflow只读诊断，保留新参考和现用CAD/×1.10/预算/零PD；未改固件、连接设备、运动、重刷或恢复控制。实板未重新实读，仍以上次9317…验收为历史事实。
- 按上一条固定上游/腕部的局部曲线验证修正方向：仅为数学示例，加入−0.02*cos(q3)N·m后，−10°支撑从+0.232652变+0.212956，+10°从−0.135068变−0.154764，确实同时减小负角度侧幅值、增大正角度侧幅值。Python符号及幅值断言通过；0.02不是辨识值、未写入，不将该固定姿态示例外推为全部腕姿/行程的正确补偿。
- 建议优先检查/拟合重力曲线的偶分量及幅值/相位（质心或映射误差），不继续统一放大、不在0°硬切两套倍率，也不凭两侧手感修改已标定编码器零点。当前偶分量可能有真实CAD质心贡献，仅凭方向描述还不能认定其错误、删除或确定修正量；仍需成对姿态的有效力矩/平衡信息，并验证其它姿态。
- 状态：定义澄清、离线修正方向检查和本次日志完成。无源码改变，不重复编译或实机测试；物理根因及修正幅度未确定。网页检索未取得有效正文，无外部引用作为证据。

## 2026-11-19 11:58（America/New_York）｜用户授权试调J3两侧补偿，余弦项−0.02N·m

- 用户在明确“正负为零点两侧角度”后要求“你修正试试看”。沿此前改代码/烧录/用户手感测试流程，使用embedded-firmware-workflow与ponytail，仅试调J3支撑；已进度说明新参考、CAD、1.1倍、零PD、原1.8N·m预算、其它轴及原失效策略保留。修正是用户手感对照，不是新COM/力矩辨识，不扩展为主动扫动。
- `config/five_mit.c`新增 `g_five_j3_gravity_cosine_nm=-.02f`（N·m），`Mode/five_support.h`声明。唯一所有者observe在新鲜完整G3×1.10之后、原预算之前加 `-.02*cos(J3软件弧度)` 一次；缓存不反复叠加、过零连续。GRAVITY及IMPEDANCE的G基底包含试调，主动运动M/c/RNE前馈不改，CAD/工作区/零点未改；|增量|≤0.02N·m，不把固定姿态改善外推为全行程/所有腕姿已验证。
- 回归已更新真实反馈→模型→帧比较，覆盖−10/−0.1/0/+0.1/+10/+90°：标定其它轴姿态下负侧幅值减少、正侧增加，其它四轴名义G不变，硬件/软件PD为0，速度反转不改变支撑，缓存/阻抗/缺反馈/失效处理继续验收。GCC16.1静态分析构建、32/32 CTest、4/4 Python单元测试通过。
- 候选 `build/v12-j3-cosine-trim-20261119/ylamp_v12_five_axis.bin`，709264字节，DTCM42824B，SHA-256 `4A507966EFF3F08818F95216EFD31CDD000BB2A369E7E2B7B3C1C6F443D34344`，目标全量构建和ELF/BIN门禁通过。源码成果及验证已进度交付；此时尚未刷入，实板仍9317…CAD版。
- 烧前COM36被新开的pythonw PID5028“五电机控制台”（dm_monitor.py）占用，status打开串口报PermissionError(13)/系统5，未发送电机/会话请求。保留空的before_flash_status.jsonl失败痕迹；没有杀进程、强关窗口、抢占GUI会话或自动重连。已请用户在控制台点“断开”，同时继续只读SWD核对，不将串口失败作为复位/烧录/自动使能理由。写板/新镜像全回读与板端验收待串口交接。
- 串口交接已解决：先普通只读SWD全量回读确认旧9317…镜像、3份旧RAM为健康G，未halt/reset；随后第二次status可打开COM36，板时936224ms为GRAVITY/31/31/31/fault0/limited0。未强关或重启PID5028；用户继续明确“烧录啊”，已进度报告正在烧录及承托提醒。
- 一次部署已完成：目标UID00420030-31345106-38373332核对、旧9317…应用写前verify，完整1MiB备份新构建before_flash/before_flash_1MiB.bin，SHA-256 `1A5899ED8000BC7A511B7B56E717B3B584714401C88EFBCD2273DC7C8A99627B`。写后全709264字节readback与候选4A50…4344完全一致，仅原脚本必要应用扇区擦写及一次SYSRESETREQ运行，未改Option Bytes/全片擦除。
- 刷后显式一次read-j3 --supported，旧J3新计数201、五轴提交G；一次原ack成功，无阈值放宽或重试。gravity_acceptance_01.jsonl末五帧均健康，最终板时23683ms为GRAVITY/fresh31/enabled31/submitted31/fault0/limited0。J3软件−8.144215°时名义G3=0.254380241N·m，余弦增量−0.019798293N·m，支撑预测0.260019973、板读0.260019988N·m；五轴预测最大差2.14e−8N·m。其它四轴按相同CAD原模型，不加新项；J4约一编码步变化正常按实际姿态重算。
- 已进度交付“已烧录、健康五轴纯G、用户可测试手感”。按新ELF地址确认c=0x20002af0/496B、uwTick=0x20007e3c后普通运行中只读SWD回读全应用，哈希再次匹配4A50…4344，DHCSR首尾0x01010001、板时79356→83656ms推进，无再次halt/reset。实读余弦系数−0.0199999996N·m、倍率1.100000024。首次要求所有长结构RAM掩码31的断言失败：第二份读到observe周期内fresh/enabled/submitted=0，phase仍G、fault/limited0；保留真实样本，不冒称原子状态。另两份为31/31/31，随后完整协议after_running_status_01末板时142238ms重新确认健康31/31/31、fault/limited0；未因中间掩码重发任何控制。
- README/AGENTS已更新当前镜像、公式、预算/坐标不变、备份及验收、GUI未自动重连和经验试调限制。状态：源码试调、必要回归、目标构建/门禁、实际烧录、完整回读、板端支撑及过程记录已完成；不宣称真实质心/幅值已辨识或全姿态补偿改善，剩余为用户正负角度两侧手感测试，未追加主动运动或自动恢复。

## 2026-11-19 12:05（America/New_York）｜用户反馈偏移过多，J3余弦试调减半

- 用户在4A50…版本实测后说“让曲线略向负力矩方向偏移，刚才偏移多了”。按正在进行的试调/烧录流程，将余弦系数从−0.02减半为−0.01N·m；已明确进度告知具体调整及重新烧录。只改config/five_mit.c参数及对应回归常量，最大新增改变量0.01N·m，其余坐标、1.10倍率、预算、零PD、其它轴、工作区、主动RNE和失效策略不变，不追加主动运动。
- 源码及快速检查已交付：32/32 CTest通过，候选 `build/v12-j3-cosine-trim-half-20261119/ylamp_v12_five_axis.bin`，709264字节，DTCM42824B，目标全量构建及ELF/BIN门禁通过，SHA-256 `F1641FBD3B056BCD7115DAE7D97B367F67AEE068D13CEE2B28A7140F4C7C10CF`。这是经验试调幅度减半，不冒称新拟合结果。
- 烧前COM36可用，原4A50…板实读GRAVITY/fresh31/enabled31/submitted31/fault0/limited0，板时287421ms。已进度提醒承托和刷写中断支撑，准备按原受检流程一次烧录/回读/显式旧J3交接及验收；此条记录时尚未部署，后续结果再补记。
- 部署/回读已完成：UID00420030-31345106-38373332，旧4A50…应用写前verify，完整1MiB备份在新构建before_flash，SHA-256 `A75704AEAF4681A4E6E68E2628BD289ED0968F653F65EB253146D0AA8030FF60`；写后全709264字节readback与F164…10CF一致。显式一次旧J3取数取得新计数201并交回支撑，没有主动轨迹、硬件设零或额外使能。
- 健康模式验收未完成：单次原ack被rc2拒绝，保留gravity_acceptance_01失败记录。请求前完整状态为DEGRADED/31/31/31/fault1/limited0，J3速度+0.0219798rad/s；该帧在静止门槛内不能证明RPC执行瞬间全部条件成立，未原子捕获具体拒绝分支，不能归咎确定的速度或提交原因。未放宽still、重试清锁/使能/复位；已进度明确报告“减半版已刷入并提交支撑，但非健康模式”。
- 随后只读after_rejected_ack_status_01末板时78512ms持续DEGRADED/31/31/31/fault1/limited0，五轴继续提交新支撑。按新ELF确认原RAM/参数地址后普通运行中SWD全应用回读再次匹配F164…10CF，DHCSR首尾0x01010001、板时135034→139332ms推进，未再次halt/reset；板端余弦系数−0.00999999978N·m、倍率1.100000024。三份RAM均DEGRADED/31/31/31/fault1/limited0，新支撑与各自参考角复算最大差1.21e−7N·m。
- README/AGENTS更新当前减半版、备份和实际DEGRADED状态，旧4A50…健康验收明确降为历史。状态：参数减半、32项回归、目标构建/门禁、烧录及全回读已完成；健康模式确认因单次ack拒绝未完成，保持承托，等待进一步显式清锁指示，不自动恢复。真实手感效果仍由用户实测，不宣称本次已完成物理辨识或补偿验收。

## 2026-11-19 12:12（America/New_York）｜用户要求J3加力，倍率1.10→1.20

- 用户进一步要求“加力”。按当前J3试调任务解释为整体支撑倍率提高到1.20，已进度明确告知保留减半后的−0.01*cos(q3软件弧度)项，不再加大偏移；原零点、CAD、其它轴、预算、零PD、工作区、动态RNE和失效处理不变。config/five_mit.c及对应回归系数已修改，不把倍率调整称作辨识，不追加运动。
- 源码及快速检查已交付：32/32 CTest通过，候选 `build/v12-j3-gravity-x120-cosine-half-20261119/ylamp_v12_five_axis.bin` 全量构建/门禁通过，709264字节、DTCM42824B，SHA-256 `172E91E6037C282C63A46E213F03BFD7AAF518A9525C1A930F53C728F0639896`。此条记录时尚未刷入。
- 烧前COM36可用，旧F164…板实读DEGRADED/fresh31/enabled31/submitted31/fault1/limited0，板时351004ms，G3支撑约−0.878392N·m。没有重试旧boot的ack或为清锁单独复位；这次烧录由新加力参数任务触发。已提醒托住并停止手扫，后续一次写板/完整回读及板端验收结果再补记。
- 部署完成：核对UID00420030-31345106-38373332，旧F164…应用写前verify；完整1MiB备份在新构建before_flash，SHA-256 `CDE7B4F7F1065DEF1B0E2EFBA7777C20563D0157738F61377083A235423B08ED`。写后709264字节readback全等172E…9896。没有全片擦除/Option Bytes修改、重试旧boot清锁或额外运动。
- 新boot显式一次read-j3 --supported获J3新计数201并交回支撑，显式一次原ack成功，无重试。gravity_acceptance_01末五帧均健康，末板时39421ms为GRAVITY/fresh31/enabled31/submitted31/fault0/limited0。实读G3−0.961674511N·m，同姿态新公式复算−0.961674401N·m，五轴最大差1.11e−7N·m；旧倍率同姿态预测−0.881537189N·m，只是公式对照，不冒称前后完全同姿态实测。
- 已进度交付“已1.20倍烧录、0.01偏移保留、健康五轴G、用户可测试”。按新ELF地址核对，普通运行中SWD全应用回读仍匹配172E…9896，DHCSR首尾0x01010001，板时83456→87756ms推进，无再次halt/reset；实板参数倍率1.200000048、余弦系数−0.00999999978N·m，三份RAM均GRAVITY/31/31/31/fault0/limited0。
- README/AGENTS同步本版与健康验收，旧F164…锁存未确认及4A50…大偏移均明确为历史。状态：本次加力代码/32项回归、目标构建/门禁、烧录、全量回读、板端支撑和日志已完成；未覆盖用户无承托手感/所有姿态补偿，仍不宣称物理辨识或自动追加扫动、使能和恢复。

## 2026-11-19 12:20（America/New_York）｜用户要求再稍减轻，J3倍率1.20→1.18

- 用户要求“再稍微减轻一点点”。按现有手感试调流程将J3整体G倍率从1.20小幅降到1.18，已进度告知明确数值和烧录，保留−0.01*cos(q3软件弧度)修正、原零点/CAD、1.8N·m预算、零PD、其它四轴、工作区、主动RNE及原失效流程。使用embedded-firmware-workflow和ponytail，只改config/five_mit.c参数及对应回归常量，不追加实机扫动或新恢复策略。
- 源码成果及快速检查已交付：32/32 CTest通过，候选 `build/v12-j3-gravity-x118-cosine-half-20261119/ylamp_v12_five_axis.bin` 全量构建/门禁通过，709264字节、DTCM42824B，SHA-256 `7DD2DE766434557D1ECFE135EBFFB7E6993E980FA0979BBAD5E5CA9A7F8A17E9`。本条记录时尚未刷入，不称已完成部署。
- 烧前COM36可用（控制台PID5028窗口仍在，未强关/重连），旧172E…板实读GRAVITY/fresh31/enabled31/submitted31/fault0/limited0，板时435934ms。已提醒托住、停止手扫与刷写支撑中断，沿原受检单次烧录、完整备份/回读和显式旧J3交接验收，后续实际结果补记。
- 已完成一次部署：UID00420030-31345106-38373332核对，旧172E…应用写前verify；完整1MiB备份在新构建before_flash，SHA-256 `7EBF525B4FB74B1A58852ED056B0AAB0D97C379AEAC2C0D1F938FC4ECB93CFFE`。写后全709264字节readback与7DD2…17E9完全一致，未全片擦除/改Option Bytes或额外复位。
- 显式一次read-j3 --supported取得J3新计数201并交回五轴支撑，一次原ack成功，无重试。gravity_acceptance_01末五帧均健康，末板时67126ms为GRAVITY/fresh31/enabled31/submitted31/fault0/limited0。G3板读−0.079171672N·m，新公式同姿态预测−0.079171500N·m、五轴最大误差4.39e−7N·m；旧1.20公式同姿态预测−0.080351188N·m。只作新旧公式和板端计算对照，不冒称独立实物力矩辨识。
- 已进度交付“1.18倍已刷入、五轴健康G、用户可继续试”。按新ELF确认c/uwTick/参数地址后普通运行中只读SWD全应用回读仍匹配7DD2…17E9；DHCSR首尾0x01010001、板时101432→105736ms推进，无再次halt/reset。板端倍率1.179999948、余弦系数−0.00999999978N·m，三份RAM均GRAVITY/31/31/31/fault0/limited0。
- README/AGENTS已更新当前镜像/参数/备份/实际验收，1.20版移为历史，保留其它用户工作树。状态：本次微调源码、32项回归、目标构建/门禁、实际烧录/全回读、板端支撑和日志完成；未追加主动运动、GUI重连或自动恢复，实物手感仍由用户继续测试，不宣称全姿态物理补偿已验收。

## 2026-11-19 12:31（America/New_York）｜暂停补偿调参，梳理历史并建议下一里程碑

- 需求与范围：用户表示“暂时就这样”，要求阅读所有process并判断下一步。工作区含忽略目录的文件枚举仅发现本份实际过程日志；按2026-11-08至11-19的章节时间线梳理双轴拖动/阻抗、V1.2重建、通信/标定、离线工作区、五轴实控与本日零点/手感试调，并重点回读当前基线及对应源码。使用embedded-firmware-workflow与ponytail，只分析和记录，不实施新控制或继续调参。
- 当前基线：以README/AGENTS最新7DD2…17E9镜像和J3支撑`1.18*G3(qCAD)-0.01*cos(q3软件弧度)`为暂存基线；最新健康GRAVITY为上一条实测，本轮未连接硬件确认当前状态。保留J1/J2/J4/J5用户认可的补偿与方向，不重新要求照片、重标或全轴扫动。现有倍率/余弦仍是经验试调，不是物理模型完成辨识。
- 当前源码复核一：`config/five_mit.c:13`的实机profile仍为五轴零Kp/Kd；`Task/five_control.c:298`主动轨迹直接使用该profile，`Mode/five_mit.c:58–96`只有轨迹参考与名义逆动力学/摩擦前馈，没有另外的位置误差纠偏。原地IMPEDANCE的独立软件PD不能算作move已有闭环；应先明确并实现与纯G隔离的主动位置反馈，不直接用当前move开展定位验收。
- 当前源码复核二：`Task/five_control.c:301`仍按elapsed达到duration设at_target，`tools/five_control.py`仍以POSITION作为终点确认，未按位置误差、实际速度和稳定时间确认物理到位。此前主机反例属于历史，本轮没有重新运行；本轮确认的是该源码路径仍存在，不能称本次已复现实板不到位。
- 当前源码复核三：J3倍率/余弦仅在`FiveControl.observe`的新鲜G上施加；主动`FiveMIT_Sample`另算未含试调的RNE，切入/退出运动可能改变同姿态支撑基底。下一阶段应明确连续交接和共同G基底的使用契约，保留经验trim与物理模型的区别，不把1.18连带乘到M/c，也不以此宣称完成统一物理参数辨识。
- 当前源码复核四：`Task/motor_usb.c:312–329`在待发响应存在时尝试TX后直接返回，持续BUSY时不消费后续RX，取消请求仍可能等待；注释不代表该历史问题已修复。主动运动前需修正并覆盖背压下取消的定向测试，不新增自动恢复/重新使能或更改故障输出策略。现有状态分项也不是严格同周期快照；优先在原记录中补齐必要的目标/实际、G/PD/总指令、限幅与时间信息，不重建遥测框架。
- 已交付建议：下一里程碑为“补齐J3小范围主动位置闭环→到点保持→实板验收”，不是继续凭手感改重力或重做已有离线模型。当前五轴主动轨迹实际为同步五次曲线；Ruckig仅是历史已有算法资源，不能说当前五轴生产入口已经使用它。先完成上述最小控制/到位/取消/支撑交接修正及回归，再在真实姿态通过原生行程和联动工作区预检后，由用户在场进行J3低速双向小步与固定中心回正，量化跟踪误差、超调、收敛、双向重复性及力矩限幅。其余四轴不自动扫动/调参；后续扩展五轴联动、辨识或示教回放另行确认范围。
- 边界与剩余事项：不把旧双轴超调、摩擦系数或P16自动恢复重捕中心问题直接认作当前五轴故障；旧控制恢复路径不在当前生产链接。保留冻结工作区，预检拒绝就不发动作，不以折回J4、假设J1=0或放宽边界取得通过；此前build清理导致原始证据缺失的限制仍有效。新的主动反馈、模式交接和到位判据尚未设计确认/实施，本轮建议不构成烧录或运动授权。
- 操作、验证与状态：实际仅读取日志/README/AGENTS、当前配置与FiveControl/FiveMIT/FiveSupport/USB/CLI调用链、测试注册和Git状态；唯一写入为本条process记录，保留全部用户既有工作树。未修改源码、参数、固件或文档基线，未打开COM/SWD、切模式、采样、复位、使能/失能、清故障、烧录或运动。无功能变更，不重跑无关主机回归/ARM构建或仿真，既有32/32不计作本轮结果。分析结论已交付；本条保存/空白检查完成后收尾，实施及实板运动验收均未开始。

## 2026-11-19（America/New_York）｜用户将下一阶段明确为五轴重力残差系统辨识

- 用户纠正与有效目标：下一步应由AI在完整运动范围内驱动五轴，辨识原模型未补偿的量，用数学方法分离正弦重力补差函数并加回公式。按此将下一里程碑从“J3定位验收”改为“J1–J5多姿态重力残差辨识、独立验证及回写”；位置闭环、到位判据和取消链修正只作为可靠采集手段，不替代用户目标。本任务范围已明确扩大至五轴，不再以此前J3收敛范围阻止五轴采样规划；本条承接下一步讨论，本轮尚未开始运动或修改控制实现。
- 运动域契约：覆盖现有实测行程内可通过原生联合工作区及整段轨迹检查的姿态，报告覆盖与空白；不把五个独立行程的笛卡尔积称作全组合合法，不绕过未知域、J4连续圈或非零J1的既有约束。某处拒绝时如实报告，不能承诺有限数据覆盖全部连续姿态或自动扩大已证范围。
- 采集与分离方案（尚未执行）：在多组上游/腕部构型下做受控低速正反配对和双向接近停点，记录五轴实测q/速度/源时间、有效反馈/电流估矩、目标、G基底、反馈纠偏及实际总指令、限幅与温度。准静态段降低惯性/速度耦合影响，按实际姿态和速度匹配；用方向相关项辅助分离摩擦，检验不对称/线束/温漂残差，不假定简单双向平均能无条件消除静摩擦。有效段不能存在未知外部承力接触；不能只重复记录已发送的G作为真实所需力矩。驱动估矩未独立标定时，结果标记为驱动等效重力修正，不冒称绝对轴端力矩。
- 数学方案（尚未实现）：优先以现有五轴几何生成质量一阶矩/势能导数的共享三角基底，`delta_G(q)=Yg(q)*theta`，或等价的`delta_G=gradient(delta_U)`，用加权最小二乘及可辨识性检查求系数；固定其余姿态时体现sin/cos，相邻/下游角改变时保留耦合，不用五条互不相关的一元sin外推全机构。摩擦作为独立方向/速度项拟合，不全部吸收到重力函数。重力实验不能唯一辨识惯性二阶矩，不因本任务给M/c套统一倍率。
- 已核对现有工具：读取`tools/five_identify.py`的regressors/load_run/参数声明；当前固定CAD质心及归一化惯性，仅拟合J2–J5四个整体link_scale及五轴armature/Fc/B，共19参数。该接口不等于独立一阶矩重力残差拟合，不能声称直接运行即可得到用户要求的新补差。当前零PD主动轨迹、计时到位、USB背压与支持/主动G基底差异仍须在采集实现中解决，正式GRAVITY继续零硬件/软件PD，不新增自动恢复/重新使能。
- 基底与验收：保留当前1.18倍/−0.01cos版本作为比较基线；训练目标必须注明相对名义CAD G还是含经验trim的现用支撑G。若残差已吸收旧trim，回写时合并替换，不能再重复叠加1.18与余弦；当前未撤除任何项。按独立整段记录/姿态留出验证，给出各轴覆盖、回归秩/条件数、残差RMSE及条件置信区间，并做补差前后同条件实板对照；不只报训练R²，不把它说成零点/模型正确概率。
- 实际验证与状态：已交付用户目标修正及上述方法，查阅MIT Underactuated Robotics系统辨识章节的结构化线性回归/激励/可辨识性，以及原始重力与摩擦辨识研究；只作方法依据，不是本机数据或性能证明。日志Git空白检查已执行通过（仅既有LF/CRLF提示）；本轮唯一写入仍为process.md。未产生新测量/拟合系数，未开端口/SWD、驱动、重刷、改固件/参数或运行无关构建；采样闭环/轨迹覆盖/拟合器及新补差部署均未实施。此前J3定位验收的优先级建议以本条用户目标为准。

## 2026-11-19 12:39（America/New_York）｜五轴重力辨识开始预检，实际起点不满足主动运动域

- 用户明确“计划好之后立刻开始”。已开始五轴辨识任务的实板预检，范围为J1–J5，不继续将其限定为J3。计划顺序为实时状态/标定读取→覆盖与轨迹预检→采集闭环及必要回归→合法域内多姿态双向低速采样→重力/摩擦分离拟合与独立验证→合并补差并按授权部署；正式纯G保持零PD。当前未把仅有计划、既有模型或电机回报等于指令当作已完成系统辨识。
- 实际只读采集：确认原pythonw控制台PID5028仍在，COM36可正常独占打开，未强关/抢占GUI。运行`usb_monitor.py --port COM36 --seconds 2 --log build/five-gravity-id-20261119_01/passive_preflight.jsonl`，PC发送0字节，实收1957组完整五槽快照、约977.4组/秒，序号缺口/坏帧/重同步/冲突均0，五台新鲜使能。后经原`five_control.py status`读取标定和1秒状态，记录`control_preflight.jsonl`；会话建立/结束沿现有回G语义，未发运动/使能/清错请求。末板时929676ms为GRAVITY、fresh/enabled/submitted31、fault/limited0，G age2ms。
- 当前实际原始角约`[-25.845820,125.382302,52.882825,-176.418281,-67.745573]°`；按本次板端回读标定与当前源码核对，软件角约`[62.882428,-20.798690,35.591450,-232.120708,-10.720836]°`。J4原始角低于记录下端−171.697172°约4.721109°；这不证明机械硬碰撞，记录区间继续不妨碍原地G，但主动运动仍须遵守行程及联动工作区。J4不折回360°，不把J1假作0。
- 原生预检：以当前`Model/five_workspace.c`和原生成表/坐标/标定编译独立只读`five_workspace.dll`，调用真实`FiveWorkspace_Line`，没有访问设备输出。当前姿态原地、每轴±1°合计10条试探以及当前→候选起点的直达均拒绝。结果保留`workspace_preflight.json`与补齐参考圈/源码身份后的`workspace_preflight_02.json`，含采集与DLL的SHA256、四份工作区/配置源码SHA256和当前五轴坐标。CLI按契约返回2表示拒绝，第二次调用显式断言该返回值，不能把它当作运动已执行或代码异常。
- 通过模型预检的候选起点：保持当前J1，软件`[62.882428,0,8.577093,0,0]°`，对应原始`[-25.845820,146.180992,25.868468,55.702427,-57.024737]°`。只证明该姿态在当前冻结域内，不证明从实际姿态到该点的直达/任意人工路径或实物间隙安全；前述直达实际被拒，未发送目标。需要现场人工在承托并检查实际间隙/线束的条件下摆入可测试姿态，再取得新数据复检，不能自动从域外搬运或绕过门槛。
- 本轮工具成果：新增`tools/five_gravity_preflight.py`，仅离线读取现有capture及编译的工作区库，不打开端口或发电机命令；验证板端标定/五台身份与健康末状态，报告逐轴小步、候选起点及直达可行性。复用现有JOINT_REFERENCES与CAD映射，不复制零点常数；窄行程按既有参考圈，J1/J4保留实际多圈。内置自检实际通过，覆盖无原地/小步/直达放行、候选姿态、无输入修改、NaN拒绝及J2换圈/J4不换圈。首个GCC进程因UCRT DLL路径未加入PATH退出1，补齐本次进程PATH后`-Wall -Wextra -Werror`编译成功；未修改系统环境。
- 当前交付与限制：已向用户交付五轴在线、健康G与真实工作区拒绝。没有重刷、暂停/复位MCU、使能/失能、清故障、反向或发主动轨迹；原1.18/−0.01cos与其余轴补偿不动。不在当前无合法起点时盲试零PD move。原生预检工具完成，不等于采样闭环/正弦耦合拟合器已实现；没有合格运动数据和新系数，本次静态/无free-space声明记录不用于辨识。后续实板推进需要用户手动摆入可测试姿态；完整联合范围覆盖、控制采集实现、拟合验证和补差回写仍未完成。
- 本轮收尾检查：新工具自检再次通过，新Python文件无尾随空白；`git diff --check -- process.md`通过，仅既有LF/CRLF提示，新增记录已读回。当前本地基线BIN哈希仍为7DD2…17E9，未重新写板；这不是一次新的实板镜像回读。工具及预检记录收尾完成，正式采样/拟合任务因实际起始姿态未获主动运动放行而待现场调整，不标记系统辨识完成。

## 2026-11-19 12:45（America/New_York）｜复核J4完整454.799°行程，纠正机械越界与软件拒绝的混淆

- 用户质疑J4本有400多度行程，明确不需要小范围辨识。本轮按embedded-firmware-workflow与ponytail只核查范围、参考圈和门控，不把质疑视为放宽行程或修改工作区授权。五轴完整合法联合范围的辨识目标保持；上一轮±1°只是离线路径许可探针，不是拟议辨识幅度。
- 数值复核：`config/joint_references.def:15`确为原始连续角`[-171.697172,283.102025]°`，宽454.799197°，零点55.7024265°对应软件区间约±227.3995985°；没有按±180°或一个360°裁剪。上次实读原始−176.418281°比旧记录下端低4.721109°，算术无误，但只能说读数落在旧记录区间之外，不能将其概括成实体已超机械行程。端点为记录极值、并非硬止挡，本次不擅自扩大、重定义或否定完整行程。
- 参考圈核查：`JointCoordinates_Reference`对跨度大于一圈的J4保留真实原始多圈读数；不存在自动按当前范围选择±360°分支。假设+360°后数值183.581719°确在旧记录内，但本次没有独立物理圈数/线束证据，不能直接修改读数或命令来取得放行，也不能据此确诊了重启圈号错位。连续参考圈/旧端点是否与当前实物对应仍待核实。
- 独立的软件门控：`Model/five_workspace.c:129`在任一起止J1非零时仅走`in_yaw_box`；`five_yaw_box.def`为J1±200°且其余四轴表坐标各±5°，并非J4机械行程。当前J1软件约62.88°，因此即使单独处理J4的记录端点差异，也不会自动取得大范围联动放行。这是已有冻结证据的适用范围，不应以“J4超范围”一句遮盖。
- 本轮离线原生验证：先核对四份当前源码/配置及DLL哈希与上轮报告一致，再调用`FiveWorkspace_Line`，不访问设备。当前姿态拒绝，假设J4+360°后仍拒绝；同一候选下游构型取J4软件100°时，非零J1拒绝、仅将离线J1设0后通过。进一步在J1=0、J3软件8.577093°、J5=0以及J2软件0°/10°两组切片，J4软件−226°到+226°的227个姿态点均通过；每组226条2°直段有5条拒绝，整条452°直段也拒绝。这说明不能由姿态点通过推断整段通过，亦不能由小段保守拒绝推断J4只允许小角度。各拒绝段为[-114,-112]/[-86,-84]/[-58,-56]/[84,86]/[112,114]°，尚未定位为真正模型空档还是表格跨单元验证保守性，未据此修改门控。
- 结论与后续：纠正前一回复把“旧记录区间之外”简写为“超范围”的表述；撤回以该概括直接要求用户回小盒起点作为唯一推进办法。先核实J4参考圈与完整联合域的门控/路径覆盖，再安排五轴全范围辨识，不将±1°探针或±5°yaw-box冒充完整目标；当前合法路径条件及不绕过工作区的边界不变。
- 验证和状态：范围差值已用PowerShell decimal独立复算；当前C源码和原生库诊断完成，结论已先交付。本轮唯一写入为本process记录，未修改生产代码、预检工具、标定、限位、补偿、固件或冻结证据；未连接COM/SWD、发动作、复位、使能/失能或重刷。静态检查不是新的机械圈号确认或运动验收，五轴辨识/采集闭环/新系数仍未完成。

## 2026-11-19 13:02（America/New_York）｜五轴耦合重力残差拟合实现与大范围路径复核

- 用户要求立即开始、大范围测试并催促加快。沿embedded-firmware-workflow与ponytail复用现有五轴模型、记录审计和线性求解，不将“大胆测试”解释为绕过冻结工作区、改变连续圈或自动复位/使能。五轴辨识数学工具为本次实现；实际运动仍未下发，不将软件成果冒称实物系统辨识已执行。
- 实板预检：COM36可正常打开，原控制台PID5028保留，未强关/抢占。一次原status记录在`build/five-gravity-id-20261119_01/control_preflight_02.jsonl`，建立/结束会话沿原有回G语义；末板时1696807ms为GRAVITY/fresh31/enabled31/submitted31/fault0/limited0、G age3ms。原始角约`[-25.845820,125.382302,50.762734,-180.964537,-64.204762]°`，软件角`[62.882428,-20.798690,33.471360,-236.666964,-7.180025]°`。J4读数比旧记录下端低9.267365°，仍不据此断言已碰机械限位或可自动换圈。无持续采样记录，不从两次快照差值推断运动原因。
- 离线预检`workspace_preflight_latest.json`以该新记录及原生库复查，实际起点/±1°探针/直达候选起点均拒绝，未发这些目标。继续解析原冻结表的精确单元边界，在软件切片`[J1=0,J2=0,J3=8.577092907135324,J5=0]°`找到J4路径`[-226,-113.69979924999998,-85.27484943749998,-56.849899624999985,85.27484943750001,113.69979925000001,226]°`；6段全部经未修改的`FiveWorkspace_Line`通过，总跨452°。这修正了原整段/等间隔跨单元的保守拒绝，不缩行程、不改C门控，也不是从当前实物姿态出发的合法路径证明。首次用np.fromstring解析Pair数组因尾逗号产生奇数长度而失败；改为提取数值并核对31393/29241单元后完成上述检查，没有修改表或库。原生DLL SHA256仍`ac04a62711a6afff3cb0aa9e61da5bead731983ae79ee9b218cd9c4ce408c02e`。
- 新增`tools/five_gravity_fit.py`：用现有五轴CAD几何生成20个质量/一阶矩势能导数列，另加名义载荷在基座X/Z单位重力下的两列以允许保守的偏航相关残差。先消除结构性参数不可辨识组合，再检查实测激励满秩；有效组合不是唯一质量/质心或实测倾斜。联合估计五轴独立Fc/B奇速度项，按方向/关节均衡加权；仅纳入|实测速率|0.02–0.20rad/s、名义动态项≤0.03N·m且每轴两方向各≥15条方程。实测速度局部微分提供加速度，不使用轨迹计划加速度冒充测量。
- 参数及验证契约：拟合基线是名义CAD逆动力学，不是当前J3 1.18倍率/−0.01cos支撑；以后部署必须合并替换已吸收的经验trim，不能重复叠加。M/c不改、不由重力实验估二阶惯量。强制独立训练/验证记录，拒绝重复会话/哈希/重叠时间；要求至少8个真实MCU时间块，默认400次块重采样，给出条件95%区间、逐轴留出RMSE/覆盖/条件数和固定其余姿态的sin/cos示例。输出始终deploy=false，不自行写配置或设备；无外力/充分激励不足不产生系数。
- 最小兼容修改`tools/five_identify.py:load_run`增加可选`with_time=True`，默认仍返回原4数组；新选项保留排除反馈缺口后的真实MCU时间，不能把删除缺口后的数组下标冒充连续时间。README加入新离线入口及当前未执行主动采样的边界，没有修改顶部实板版本/参数。
- 首次交付及快速验证：新增`tests/test_five_gravity_fit.py`最初6项合成/离线测试通过，覆盖203姿态名义RNE一致、共享势能交叉偏导、单轴条件sin/cos、独立合成数据重力/摩擦恢复以及激励/非法角拒绝；原`test_five_identify.py`4项通过。已向用户交付拟合代码与真实未运动状态；以上测试不是硬件采样或物理系数验收。随后补入第7项记录时钟/缺口及默认API兼容回归，补检与真实静态记录拒绝检查待执行，收尾进行中。
- 未完成与边界：现有move仍是零PD前馈、按时长认POSITION、与J3经验G底量不一致；USB待发响应背压下取消可能延后，均未在本次离线工具中修复。当前姿态不在原生主动域，J4连续参考圈未得到现场确认。未烧录、halt/reset、使能/失能、清故障或追加电机运动；原五轴补偿继续保留。尚无合格训练/独立验证数据、补差系数或新模型，更未完成全行程实物辨识。进一步主动采样需要明确采样用位置纠偏/到位行为及合法起点，不能用重复静态取数或小范围探针代替用户目标。
- 收尾实测结果：补入记录时钟测试后新工具7/7、原辨识4/4 unittest再次实际运行通过；时钟回归验证删除缺口窗口后仍保留真实非连续时间、旧4数组API不变。预检工具自检通过；3份相关Python的AST语法和5份本次修改文件尾随空白检查通过，README/process的git diff --check通过，仅既有LF/CRLF提示。没有重跑无关32项CTest/ARM构建，不引用旧结果冒充本轮。
- 对两份真实preflight记录实际运行新CLI，输出`build/five-gravity-id-20261119_01/gravity_fit_preflight_rejected.json`，返回1与`status=insufficient_data,deploy=false`，两份均因未排除外力/手托而在审计阶段拒绝，没有系数或置信区间；没有替记录补写free-space标签。离线成果与本次日志已完成，实物采集/新公式/烧录仍未完成。继续实际运动需要采样专用位置反馈行为确认和合法起点，不把现有无闭环move用于大范围试扫，不擅自改变纯G、退出/自动恢复策略。

## 2026-11-19 13:07（America/New_York）｜说明当前行程、起点拒绝轴及采样位置闭环含义

- 用户要求给出当前运动范围、指出哪轴没过起点检查，并解释“采样专用位置”。本轮是只读说明，按embedded-firmware-workflow核对现有配置、真实反馈及调用链；没有修复/实现/烧录授权的推定，不改变控制、补偿、模式、限位、工作区或恢复策略。先向用户交付J4单轴行程不符及独立联动限制，随后补记本条。
- 最新被动实读：`usb_monitor.py --port COM36 --seconds 1 --log build/five-gravity-id-20261119_01/passive_range_status_20261119_1307.jsonl`仅收、不发应用命令，未建立控制SESSION；957组完整五槽快照、954.7组/秒，序号缺口/坏帧/重同步/冲突均0。最后快照UTC时间1795111633.3851576（本地2026-11-19 13:07:13）、板时2728507ms，五轴均state1、fresh、age3ms。未读取控制phase/fault，不由被动反馈冒称本轮已重新验收健康GRAVITY；未强关或抢占GUI端口。
- 最新原始角`[-25.845820,125.382302,55.265280,-179.565673,-64.204762]°`；按现有共享参考的软件角`[62.882428,-20.798690,37.973906,-235.268100,-7.180025]°`。单轴记录的软件区间分别为J1±201.6521075°、J2[-35.377223798,46.564378202]°、J3[-86.610663641,115.456767359]°、J4±227.3995985°、J5±22.632856°；主动工作区另内缩1°。只有J4不在单轴记录/内缩区间，其余四轴均在。J4原始角比记录下端-171.697172°低7.868501°，比内缩后的主动下端再多1°；与上一份读数不同，不继续沿用9.267365°差值。不据此断言已碰机械止挡、不自动加360°。
- 原生离线复核：当前四份工作区/坐标/参考源码及DLL SHA与前次报告逐一匹配。以最新实测姿态调用未改`FiveWorkspace_Line(q,q)`仍拒绝。`FiveControl_Move→observe(true)`的原始行程检查可先因J4拒绝，之后公共工作区又有独立约束：只要起止J1任一个不为0，就只接受旧yaw-box。换到当前软件角，该盒为J1±200°、J2±5°、J3[3.577092907,13.577092907]°、J4±5°、J5±5°。当前J1自身在行程内，但非零触发此分支，J2–J5均不满足盒；只在离线把J4设0、保持其余实测角也仍拒绝。此诊断不是真实move拒绝回执，本轮未发任何目标；也不表示四轴实体行程只有±5°。
- “采样专用位置”措辞澄清：不是新增编码器参考/机械零点，不是必须回某个固定位置，而是只在自动辨识扫动时使用的位置/速度误差反馈，拟议总指令为模型前馈加Kp*(目标角-实测角)+Kd*(目标速率-实测速率)，以跟踪慢扫轨迹、记录实际姿态/速度/驱动估矩。正式纯G继续零PD；采样退出回原G，不自动使能/恢复。现有move实机profile的Kp/Kd确为0、无外层误差反馈，现有KEY阻抗只保持进入姿态，不等于已经支持任意慢扫目标。上述采样闭环尚未实现或刷入，本轮仅解释，不将它记作现有功能或新确认。
- 验证与状态：实际完成一秒被动硬件取数、配置区间换算、原生离线起点/单改J4反例和源码/库身份检查，原生起点仍拒绝。没有源码修改、板端命令、运动、复位、使能/失能、清错或写板，不重复无关主机回归/目标构建；唯一文档修改为本条process记录。已交付J4与联动限制及采样闭环的说明，采样闭环与合法起始路径仍未完成；日志保存/空白检查后收尾。
- 用户在本轮进行中追加“J4改成-500到10”。这属于新的区间变更要求，尚未指明原始编码器角还是软件角；两者相差现用零点55.7024265°，对应的原始端点和受影响运动域不同，不据此默改零点或连续圈。已告知先保持固件/参考不动，待一句坐标口径确认后再按指定区间实施；当前`joint_references.def`仍为原值，没有将待确认范围写入生产配置/板端。

## 2026-11-19（America/New_York）｜按用户确认更新J4原始角范围−500°～10°，未部署

- 用户确认“原始编码器角”，随后催促加快。沿embedded-firmware-workflow和ponytail做最小范围/兼容修改，已明确告知保留J4零点55.7024265°、不烧录、不发运动。当前任务为指定范围落入源码及候选验证，不将确认原始角解释为更改零点/CAD、放宽几何表、启用新采样闭环或自动恢复授权。
- `config/joint_references.def`：J4 raw下/上限从[-171.697172,283.102025]°改为[-500,10]°，跨度510°；零点55.7024265°不变，软件角[-555.7024265,-45.7024265]°，保留原1°内缩后为[-554.7024265,-46.7024265]°。不是重新居中到-245°，不改编码器读数/加减360°，其它四轴、CAD、G/M/c、J3 1.18/−0.01cos、零PD及原力矩预算不改。新端点是用户明确指定，不冒称AI已测得机械止挡。
- 兼容修正：原上位机`JointReference.__post_init__`要求零点落在区间内，且MIDPOINT必须等于当前端点中点；只改两端点会导致所有导入该模块的五轴工具启动失败。按用户明确保留参考的语义将J4标记从MIDPOINT改为USER_APPROVED，并仅对此类型允许有限、次序正确但不含零点的区间；CANDIDATE/MIDPOINT原约束及NaN/反序拒绝保留。`dm_monitor.py`自检和`tests/test_joint_references.c`校验新端点/原零点、零点位于区间外、原始多圈不重映射；`tests/test_five_gravity_fit.py`合成时钟fixture使用范围中点而非零点作为有效测量输入，不改变实际记录。
- 首次主机回归29/32：旧five-mit/five-control/five-motion-admission用J4软件0/正角当可运动fixture，已被新区间明确排除。按新契约修正`tests/five_mit_checks.inc`：旧正J4几何盒/轨迹改为明确拒绝；正向FiveMIT/CheckBox/PrepareInBox/传输检查使用原已证负J4−160°构型；历史yaw样本仅保留为独立sampler/编码单元测试，显式不是获准路径。原地G在范围外仍有效、不允许由该G姿态直接运动、更新行程内仍须联合工作区的断言已补齐；不改生产工作区让测试过关。five-control的该外域fixture不再声称覆盖GRAVITY→MOTION正路径，仍覆盖纯G真实帧及部分总线失败交接。
- 后续实际验证：更新fixture后32/32 CTest通过（含GCC16.1主机构建）；`dm_monitor.py --self-test`完整通过（均离线模拟，无真实端口）；重力拟合7/7和原辨识4/4 Python unittest通过。测试不是新范围机械安全/承载或实板主动跟踪证明。保留第一次29/32的真实结果，不称所有历史姿态仍可动。
- 新ARM候选：`build/v12-j4-raw-range-20261119/ylamp_v12_five_axis.bin`，709264字节，SHA-256 `42942688362A98E33D216ACF5358914CDD644D1CFC79E6CF127278C4B05EC1D8`，DTCM42824B；原GNU10.3.1/Cube/Release/CAN3与可选J3对照ON配置全76项构建并完整链接通过，`check_v12_image.py <ELF> <BIN> --j3-reverse-test`通过。仅沿用编译项，未触发对照/使能/运动；旧7DD2构建及备份保留，没有覆盖为新BIN。
- 原生离线复查：用13:07:13被动记录的原始反馈按新本地参考换算，五轴全部落在新单轴1°内缩区间。通过重新构建的`bsp_tests five-wire`实际调用`FiveMIT_Prepare→FiveWorkspace_Line`，当前完整姿态仍REJECT；理想软件姿态[0,0,8.577092907,-160,0]°原地OK，J4软件0及-554.7024265°两处仍REJECT。这是新源码对旧被动读数的离线检查，不是新板端标定回读或已发送目标。
- 约束及文档：冻结表/yaw-box和公共工作区代码未改。当前J1非零仍走旧yaw盒，新J4原始范围与其软件±5°区域无交集；J1=0冻结大表也仅取新行程交集，不能据510°区间宣称全行程已获运动许可。README顶部、AGENTS同步源码/实板区别、候选和该限制，历史452°路径不再当作本配置可执行路线。
- 交付/剩余：已向用户交付配置完成、32项/上位机/Python/目标构建/门禁通过及仍未烧录。当前实际实板仍为7DD2…17E9旧范围，本轮未连接COM/SWD、halt/reset、备份/烧写/回读、清故障、使能/失能或发动作；没有新采样数据、拟合系数或自动扫动。范围实现和离线验证完成，日志及最终空白/候选参数复核进行中；部署、联动工作区覆盖及采样位置闭环尚未完成。
- 13:23收尾复核：从新旧ELF定位并解包实际80字节标定表，仅J4下限/上限/kind三字段变化，五轴zero均逐位相同、其余四轴全字段相同。相关8份文件尾随空白检查、README/process/dm_monitor的git diff --check通过，仅LF/CRLF提示。候选尺寸709264字节与门禁报告一致，本条已保存；本次指定范围修改、兼容处理、必要离线验证与日志完成，没有部署或扩大工作区。

## 2026-11-19（America/New_York）｜用户授权烧录J4新区间，完整回读通过；旧J3重启后缺反馈

- 用户明确“烧录，为什么还没通过”，随后催促加快。已提醒烧录中断支撑、需托稳停止手扫；本轮沿embedded-firmware-workflow、aixprobe的身份/ELF核对及工程原受检脚本，仅部署上一条4294…候选，不扩大联动工作区或新增采样闭环。当前无aixprobe MCP，沿用本机ST-Link/OpenOCD2026-09-24快照；没有临时修改控制/恢复策略或套用其它芯片寄存器。
- 烧前门禁再次实跑通过，候选709264字节/SHA-256 `42942688362A98E33D216ACF5358914CDD644D1CFC79E6CF127278C4B05EC1D8`不变。`five_control.py status`打开COM36报PermissionError13/系统5，未发SESSION或控制请求，失败空记录`before_flash_status_01.jsonl`保留。原pythonw控制台PID5028仍在；未强关/抢占或自动重连。先告知用户点断开，随后确认ST-Link独立于COM，可在不取得串口的情况下完成本次明确授权的烧录，未把串口错误当作自动恢复/使能理由。
- 烧前非停机SWD：用新建`read_before_no_halt.tcl`及原`v12_read_board.tcl`核对ID0x10016483、UID00420030-31345106-38373332、CPUID0x411FC272、3.290921V；DHCSR首尾0x01010001，tick0x3d2cf1→0x3d3dc2推进，无halt/reset。全709264字节应用回读SHA与7DD2…17E9归档逐字节相同。旧ELF确认c=0x20002af0/496B、uwTick=0x20007e3c；首次RAM解析误按4字节枚举断言失败，检查ELF的Tag_ABI_enum_size=small后按1字节枚举重解，3份均GRAVITY/fresh31/enabled31/submitted31/fault0/limited0/G有效。未因此操作目标或重试烧录。
- 已按用户烧录授权执行一次原`v12_flash_checked.tcl`：再次UID核对，短暂停核，完整1MiB备份至`build/v12-j4-raw-range-20261119/before_flash/before_flash_1MiB.bin`，写前verify旧7DD2…应用，随后仅按新应用覆盖区段擦写/校验并全应用dump。未全片擦除、改Option Bytes、发电机命令或使用connect-under-reset。原脚本必要的一次SYSRESETREQ恢复运行，日志`V12_VERIFIED_BYTES 709264`、`AFTER_FLASH running`，退出0；没有追加复位。
- 主机验证：烧前备份1MiB完整、应用前缀等于旧7DD2…；备份SHA-256 `5609687EC794A4659F6D63B111B6CE17122CC7A8685751EE6A0675E1A238145C`。写后readback_v12.bin全部709264字节等于候选4294…C1D8，不仅比较零点字段。
- 普通重连验收：原read工具加载`read_running_no_halt.tcl`，不halt/reset；DHCSR首尾0x01010001、tick0xb741→0xc809推进。运行中全应用回读仍逐字节匹配4294…C1D8。实际J4表回读约[55.702426153,-500.000009360,9.999999709]°、kind3，符合float32新范围及原零点。该读回证明新配置确实在板，不证明物理行程/工作区或重力恢复。
- 当前重要未完成：刷后3份RAM均DEGRADED/fresh27/enabled27/submitted0/limited0/fault1/gravity_valid=false；四轴计数约4992→4993、J3计数0，旧协议J3尚无反馈，控制器未恢复五轴G提交。已第一时间进度告知“已烧录、继续托稳、G尚未恢复”；未自动中性唤醒、FC、原ack、冷启动、失能或发轨迹，不将历史单次恢复授权无限沿用。需要用户明确的J3取数/交接及必要原ack确认，不能让用户据镜像校验成功就撤承托或开始测试手感。
- 对“为什么还没通过”的有效解释：新区间只解决单轴检查。`Model/five_workspace.c:129`在J1非零时仅接受旧yaw-box，J4需软件±5°；现J4软件区间[-555.7024265,-45.7024265]°与之完全不相交，所以不能由改上下限/烧录推出联合起点已放行。大角度几何表只覆盖J1=0且还受冻结J4域约束，这是软件联动验证覆盖不足，不是实体电机行程只有±5°，本轮不擅删门控或伪造J1=0。刷后J3反馈缺失又是独立运行条件，需分别处理。
- 文档及状态：README/AGENTS同步4294…实际部署、完整备份/回读、当前DEGRADED及原7DD2历史；生产源码/配置未再改，本轮新增的两份TCL仅为构建目录内只读证据钩子。32项/11项测试为上轮，不在本轮假称重跑；实际新验证为门禁、身份/旧镜像、备份/写入/全回读/普通运行重连和状态检查。烧录与配置回读完成，健康G交接/主动采样和工作区覆盖未完成；日志与必要最终检查收尾中。
- 最终收尾：README/process的git diff --check及5份本次文档/证据钩子的尾随空白检查通过，只有LF/CRLF提示；本条保存完成。烧录/完整备份及双重全应用回读已完成，尚无健康G交接，不自动追加任何恢复；向用户明确要求继续承托，后续一次旧J3取数及一次原ack需显式确认。

## 2026-11-19（America/New_York）｜按用户再次要求读取旧J3，继续移除旧小角度准入规则

- 用户先明确删除“J1非零时J4软件角须±5°”及类似小角度规则，随后补充“J3是旧版本，请用旧版本的读法去读”。按embedded-firmware-workflow及ponytail复用原所有者入口，优先完成真实旧J3取数；不将此要求扩为自动清故障、重复使能、复位或发轨迹的许可。首次取数结果已进度交付，收尾进行中。
- 实际在COM36执行一次`five_control.py --seconds 1 --log build/v12-j4-raw-range-20261119/read_j3_01.jsonl read-j3 --supported`，退出0。打开串口成功，没有关闭/抢占GUI。记录含唯一rpc_accepted，既有中性MIT→必要时单次FC→命令回包→原G交接流程成功；未发新版0xCC/0x33，当前boot单次入口已消耗，不重复调用。
- 取数前DEGRADED/27/27/0、J3 count0；末板时880687ms为DEGRADED/fresh31/enabled31/submitted31/fault1/limited0/Gage3ms。J3协议字段1、CAN1 cmd3/fb19、state1、raw0.22411644458770752rad（约12.841°）、count201、age3ms；其余四轴均新鲜使能。记录解析malformed/incomplete均0，未将支撑提交成功当成物理补偿验收。原反馈缺失锁存仍1，未执行acknowledge。
- J5读数在记录下端点附近，仍独立受主动端点余量约束；不因J3恢复或小角度规则修改声称当前可主动扫动。README/AGENTS已同步最新实板状态，实板仍为4294…C1D8镜像；没有重刷/复位/失能或轨迹。
- 小角度规则修改及必要回归尚待完成：拟仅将原固定J1=0/±5°冻结表准入从当前运动入口移除，保留共享原始行程、1°端点余量、协议速度/力矩、真实反馈/失效及5°跟随误差处理；明确范围验证不等于碰撞证明。不得假作J1=0或把J4换圈让旧几何证据通过。完成后补记实际源码、测试与未部署状态。
- 该部分后续完成：`five_workspace.c/.h`改为逐轴范围准入，旧冻结表/yaw-box保留离线，公共注释不再称碰撞证明；`five_mit.c/.h`同步范围语义。预检移除固定staging/回零建议，历史`check_yaw_box.py`仅接受实存SHA-256 AC04A627…02E的归档几何库，避免把新范围查询冒充几何证据。32/32及完整ARM/门禁通过，C16CA986…候选155032字节未部署；新增归档库守卫的首次临时验证命令因PowerShell引号导致SyntaxError，没有执行审计，未声称该检查通过。该候选之后已被下述主动辨识版本替代。

## 2026-11-19（America/New_York）｜用户要求立即实动及全部五轴辨识；真实扫动和当前进度

- 用户多次明确要求立即驱动、五轴全部辨识，否决以小角度/额外锁存拖住测试；此明确指令取代历史仅J3诊断范围。按embedded-firmware-workflow/ponytail复用FiveControl唯一所有者及原旧J3入口，不绕过CAN所有者、不用SWD调用电机函数。源/实物成果已多次进度交付，本条收尾与测试继续进行中。
- 主动采样实现：新增`FiveMIT_SampleFeedback`，保留`FiveMIT_Sample`的名义G+M*qdd+c，在生产路径用真实支撑G替换名义静态G（包含J3原1.18/−0.01cos），再以原保持增益添加软件位置/速度误差反馈；所有实际MIT Kp/Kd仍0、正式纯G不变。`FiveMIT_SendSample`显式接收可选支撑G，仿真传NULL保留原行为。配置`five_motion.h`仅给端点量化/浮点余量；原始角不裁剪/不换圈，目标仍保留原1°余量。
- 首版9AD8C510…8B53、155968字节：32/32、完整ARM/门禁通过，写前verify4294…、完整1MiB备份SHA-256 `9DBD4CDBEEECB93AAFE1CF26632CCF3490BFEAF1B6DD76638E824C1BC02DB6E8`，写后全应用回读相同。旧J3一次显式读取/原ACK后健康G；J5第一条向内目标被rc1拒绝，未发运动。定位为读数约−79.679450°比记录下限低一个编码步，严格一步容差未包含float32舍入；不是新版协议或目标越界。补0.0001°数值余量及真实起点回归，保留机械端点及目标余量。
- 6411CC40…9D46、155968字节：32/32、ARM/门禁、UID/旧9AD…verify、完整备份/写后回读通过，再次原旧J3读取/ACK各一次。`j5_inward_01.jsonl`实际−79.679450→约−70.412120°，位移约9.267°；目标raw约−67.025°未达到，新增“误差>1°、轨迹后2秒仍未到位就FIVE_TRACK锁存”造成fault128。明确返回G后31/31/31、limited0，未失能。随后按用户要求继续J3，原ACK一次；`j3_lower_01.jsonl`实际+13.212489→−66.740169°、位移−79.952658°，距raw目标−68.2°约1.46°，又由该新增到位判据锁存128。用户明确否决，该判据不能继续作为整机推进门槛。
- 当前部署8A8A3E8A…11DDA、155192字节：删除上述1°/2秒锁存及固定5°跟随误差锁存，保留真实失联、过温、原始行程、预算和传输故障处理。计时结束进入POSITION并继续软件反馈保持，明确不声称实际到位；`five_control.py`保存target/actual/error及physical_arrival_claimed=false。新增host回归确认实际未到2°也不会时间锁死；其它零PD支撑、旧J3、部分总线失败等32/32均通过，完整ARM及门禁通过。20个五轴全范围候选waypoint经原生five-wire规划20/20通过，均不是实物轨迹/碰撞证明。
- 当前烧录证据在`build/v12-gravity-sweep-unlocked-20261119/`：UID00420030-31345106-38373332、写前verify6411…，完整1MiB备份SHA-256 `2D1F6C68D81C270FB76D3402D2A4ED944F1ED311DDC0EAF2265F318B9E4D72DC`；原受检脚本一次必要复位，写后全部155192字节与8A8A…相同。未全片擦除或改Option Bytes，普通运行中再次全应用回读待做；不能套用旧镜像的运行回读。每次新boot均仅一次已授权旧J3中性/必要时单FC取数和一次原ACK，没有自动循环恢复。
- 新`tools/five_gravity_sweep.py`显式单SESSION顺序扫动各轴low→high→low→进入时位置；默认五轴，支持显式`--joints`继续未测轴，绝不自动使能/ACK/复位或重放目标。同步记录五轴原始位置/速度/驱动估矩、目标/实际误差及时间，峰值计划0.16rad/s；失败显式交回原G/保留故障。开始运动前已告知用户松开承托、手离开活动关节。当前free_space仍false，不能把“要求无接触”冒充全程独立确认；尚无合格模型/置信区间或新系数。
- 真实五轴尝试进度：`five_axis_sweep_01.jsonl`中J1在原0.4N·m预算内实际不动、反馈力矩约升至−0.31N·m后，预测请求触预算而fault32；不是运动准入或角度锁存。只读确认31/31/31、驱动state1、温度正常、G limited0后，原ACK一次恢复，显式继续J2–J5。`j2345_sweep_01.jsonl`中J2已有负/正/负三段，约114.563104°～186.363160°，返回进入姿态时原主动预算触发32；该批未全部完成，未把time POSITION当物理到位。再次确认仅预算故障、健康G/反馈和原ACK一次后，显式继续J3/J4/J5，正在记录`j345_sweep_01.jsonl`，运行时不得另占COM36/SESSION导致取消。
- 未完成及下一步：J1/J2原保持预算与温和增益不能据此视为全行程主动辨识能力，正继续其余轴实测并分析主动参数；未改纯G预算/公式，尚无五轴完整双向数据、独立验证或拟合部署。用户要求扩大J5，但没有新的明确原始端点，本轮未擅套J4范围或用协议量程冒充机械行程；目前J5已在旧区间内，不以此阻止J3/J4测试。先取得真实运动覆盖与驱动力标签，再给出可辨识组合和条件置信区间，不将失败或外力未知数据说成识别通过。

## 2026-11-19（America/New_York）｜J345实扫完成、独立主动跟踪版烧录；用户要求先跳过J1

- 最新范围：用户确认上一轮扫动全程无手托或外物接触，随后明确“J1不需要重力补偿”“J1重点是摩擦力前馈和离心力和加速度”“先不管J1”。后续不主动扫J1、不做J1调参或新增重力补差，摩擦/速度耦合/加速度辨识后置；本轮先补J2，已采J345保留。整机仍维持五轴控制和实际姿态记录，不将本轮J1静止作为其它轴必须重扫的理由。
- J345实际完成：`build/v12-gravity-sweep-unlocked-20261119/j345_sweep_01.jsonl`共83174个反馈快照、3370个sample、12个waypoint及12个trajectory_result；全部3371个before/sample状态健康，末交接GRAVITY/fresh31/enabled31/submitted31/fault0/limited0，malformed/incomplete均0。原始角实测范围J3[−66.762025552,123.852339314]°、J4[−495.573281797,5.409595200]°（跨度500.982877°）、J5[−76.685054152,−39.899830697]°。反馈时间跨度463708ms，不将计时POSITION冒充精确到位。其余轴仍有真实漂移，拟合必须用同步实测五轴姿态。
- 数据溯源：原173789699字节JSONL保持不变，SHA-256 `DE64C35C8DE485F9D38C789892394E7FA9B32ADE1101817674147E3EE01542B3`。原header的free_space=false表示采集时尚无独立确认；将用户本次事后确认、问题原文、限定范围及原SHA写入同目录`j345_sweep_01.operator_confirmation.json`，不静默重写采集标签，不把此次确认扩为后续使能时已承托。现有拟合器还未读取该旁注，不能冒称审核已通过。
- 上版8A8A…普通运行中无halt/复位回读已完成，`running_image.bin`的全部155192字节SHA与8A8A…一致；读取时DHCSR前后0x01010001且tick递增，三份RAM快照均POSITION/31/31/31、fault0/limited0/Gvalid。此事实是上一boot验收，不覆盖新镜像重启状态。
- 为解决此前J1/J2原保持预算阻止主动跟踪，已新增独立`g_five_mit_tracking`：Kp[20,30,15,10,10]N·m/rad、Kd[1,1.5,1,.5,.5]N·m·s/rad、总预算[2,5,1.8,.8,.4]N·m，仅主动MOTION/POSITION使用，实际MIT Kp/Kd仍0。原g_five_mit_hw、g_five_support_gains、g_joint_reference_config和J3倍率/余弦系数由旧新ELF/BIN比对逐字节相同；纯G/手动阻抗未改。此参数表在用户最新跳过J1要求前已烧入，后续不发J1扫动；不是J1或其它轴已识别模型，不用协议tmax冒充机构额定力矩。
- 当前实际镜像`build/v12-gravity-identify-tracking-20261119/ylamp_v12_five_axis.bin`，155344字节，SHA-256 `3C728955AA9042A05FB555DFFFEE2A83EE5C2BF0C8F2C2687200C15AAFCF8418`。32/32主机测试、76步完整ARM构建和镜像门禁通过；UID00420030-31345106-38373332，写前旧8A8A…verify通过，一次受检烧录/必要复位，1MiB烧前备份SHA-256 `702EE315FE88D6FC093392287039FEBB87DA93273F975363ADC1CD70AA01CBAA`。OpenOCD写后verify及全部155344字节readback_v12.bin与候选SHA一致；本次又实算主机SHA确认，无额外重刷。
- 当前状态实读：本次仅执行`five_control.py status`，`before_j2_only_status_01.jsonl`末板时280942ms为DEGRADED/fresh27/enabled27/submitted0/fault1/limited0、无有效G，旧J3 count0，其余四轴state1/新鲜。此boot未执行read-j3/ACK/冷启/失能或轨迹，不把缺J3反馈冒充健康G，也不假填read-j3的--supported。恢复原旧J3取数须短时承托，当前用户只确认扫动无接触，未确认这次重启后承托。
- 收尾/未完成：本次同步README和AGENTS最新范围/实板状态，没有因“跳过J1”再改固件或重刷。J2新序列、当前镜像普通运行二次全回读、离线多waypoint时间跨度审核（现单段425000ms上限）、旁注审核接入、排除J1的可辨识参数组合/分轴激励聚合、独立验证及新系数部署仍未完成。不将J345完成或用户无接触确认说成重力模型拟合成功。
- 本次文档/旁注收尾验证：JSON解析成功，旁注capture_sha256与原始文件重新实算一致，original_capture_modified=false且automatic_fitting_override=false；`git diff --check -- README.md process.md`退出0（仅Git换行提示）。未因纯文档和溯源旁注重复编译/发运动。推进J2前需用户确认此刻已短时托稳，以满足已有旧J3取数入口条件；这不是要求重做无接触扫动确认。

## 2026-11-19（America/New_York）｜继续J2请求的实时状态检查

- 用户要求“继续采样J2”；范围仍只主动扫J2，J1不扫动、不调参。沿用embedded-firmware-workflow及现有采样/旧协议入口，无固件源码或参数修改。
- 实际执行一次COM36只读status，日志`build/v12-gravity-identify-tracking-20261119/before_j2_only_status_02.jsonl`，退出0。末板时631419ms仍DEGRADED/fresh27/enabled27/submitted0/fault1/limited0、无有效G，旧J3 received=false/count0，其余四轴均state1且age5ms；J2原始位置2.0002663135528564rad，未发主动轨迹。
- 原因不是J2目标或行程拒绝，而是新boot旧J3尚未完成一次显式中性/必要时单次FC/反馈/G交接。核对原`read-j3 --supported`与`FiveControl_ReadLegacyJ3`流程未变；未假填现场承托确认，未自动使能、清错、复位或重刷。
- 状态检查已完成，J2双向采样仍未开始；推进前只需确认当前已短时托稳机械臂，恢复原旧J3反馈交接后再松手采样。用户的继续指令确定任务授权，但未据此冒称现场已有物理承托。本轮无代码更改，不重复构建或无关回归。

## 2026-11-19（America/New_York）｜确认承托后真实采J2；换段拒绝和回扫中断分开诊断

- 用户明确“是的”确认当前已短时托稳。按embedded-firmware-workflow复用原所有者，实际一次`read-j3 --supported`，`read_j3_supported_01.jsonl`退出0，末板时779802ms为31/31/31、J3 count201、fault1/limited0/Gage3ms；一次原ACK后`ack_after_j3_read_01.jsonl`末板时788421ms为健康GRAVITY/31/31/31/fault0/limited0。本boot旧J3单次入口已用，未发新协议CC/33、冷启、失能或复位。随后明确通知用户松手，等待5秒再仅扫J2；J1无主动扫动/调参。
- 第一轮`j2_sweep_01.jsonl`以峰值计划0.16rad/s实际动约143.851→133.251°（反馈全局极值），522反馈快照/22sample后fault32，控制器与工具取消到原支撑；五轴新鲜使能、limited0。只读`j2_budget_status_01.jsonl`确认仅软件LIMIT、31/31/31、正常state/温度和有效G后，原ACK一次（`ack_j2_first_budget_01.jsonl`）恢复健康；不重刷、不加预算，改用0.08rad/s降低耦合扰动。这是本轮唯一预算ACK；最后回扫故障未再清除。
- 主机采集问题独立记录：`j2_sweep_02.jsonl`运动已ACK，但读状态时客户端100ms时间戳过期，取消也未确认；立即用原gravity入口`j2_after_capture_timeout_gravity_01.jsonl`确认健康G，不重放move。首个仅pump一次的修复未解决`j2_sweep_04.jsonl`的完整快照等待，之后`j2_after_timeout04_gravity_01.jsonl`再次实证健康G。原始记录均保留，未把“客户端没有发下一条状态请求”等同于此前move没执行。
- 按ponytail最小修改只改主机：`five_control.Board.rpc`在未取得新鲜完整五帧快照时，按原I/O预算0.7秒有界读取，再要求时间戳仍在100ms内；不扩大新鲜度阈值、不重放/续期旧请求，真实超时和MCU重启仍拒绝，未恢复驱动或写固件。新增回归覆盖第一次只读到部分快照、第二次完整快照后仅发送一次RPC，以及持续无新遥测/MCU重启时不发送/不递增请求号。
- 换段rc2实证：`j2_sweep_03.jsonl`完成下降段133.316413→112.705293°，76个sample健康、1个trajectory_result；第二目标field6返回rc2=BSP_CAN_NOT_READY，未开始该段。末帧J4速度0.036631823rad/s，其余轴也在0.04内。源码`FiveControl_Move`仍要求五轴瞬时|v|≤0.04，状态在上位机检查后可变化；持续PD持姿波动是合理候选原因，但固件只有汇总rc2，未记录瞬间子分支，不得把具体哪轴越阈值说成事实。rc2也不同于规划/角度非法的rc1。此前进度直接称“静止准入拒绝”过满，已向用户澄清。
- 为避免段末主动持姿影响下一段启动，`five_gravity_sweep.py`每个trajectory_result后显式原rpc7→观察实际GRAVITY→记录`waypoint_handover`并验31/31/31/fault0/limited0，再规划下一段；失败取消也观察相位交接而非立即把旧POSITION状态当新G。只使用既有模式，不新增保护/使能/ACK/重试或修改板端准入。
- `j2_sweep_05.jsonl`以计划峰值0.08rad/s真实通过两次换段，完成leg1、leg2，leg3回扫中断；共11169反馈快照、454sample、2trajectory_result、2已确认waypoint_handover，原始角实测112.705293004～190.887558772°，独立J2源count去重后速度带0.02～0.20rad/s内正/负2350/2220条（不是已做动态/接触审核的合格拟合点）。第三段从190.887559扫至约121.644822°后fault32，无完整end，不声称四段完成或新模型验收。
- 最新fault32的有数据回放定位：用同步五轴实测角/速度、当前CAD/RNE、计划加速度和实际软件PD公式重建请求；在板时1508560ms的捕获点，J3 v=−1.706959605rad/s、持姿反馈项约1.809956N·m、总请求约2.279173N·m>1.8N·m预算；该回扫J2重建绝对峰值约3.334301N·m<5N·m。是离线回放需求，不是声称板上实际发送了超预算力矩；规划起时采样有数毫秒误差，但J3目标为常值且超额明显。支持“J3主动持姿反馈导致本次输出检查中断”，不是上次换段rc2的精确子原因。J3已捕获原始角26.021～28.097°内的快速反馈波动；高主动持姿增益15/1需处理，不能通过继续增加J2预算或反复ACK重扫掩盖。恢复原5/.5为候选，尚未改源或烧录；原纯G/手动阻抗不动。
- 当前实板读数：`after_j2_sweep05_status_01.jsonl`末板时1701356ms为DEGRADED/fresh31/enabled31/submitted31/fault32/limited0/Gage2ms，驱动state均1、温度正常，支撑G约[0,1.124134,0.479632,−0.020143,−0.060219]N·m。主动轨迹已取消，最后故障锁存保留，无再次ACK/运动。整轮镜像仍3C728955…F8418，没有固件修改、刷写、复位或失能。
- 原始日志SHA-256：01=`CF43842FC01E83662805AEAC826F26D85F3E95697DA8548E139792C338F76AF2`，03=`B2719508427833C5CC67166430944EB3389384B319719BD898AC0F6F462C3733`，05=`CED8FBB273C9DA8DC3FD05745880FF54B0793504E9D6766FD6838FCAD2F936BD`。本轮free_space仍按原header记录，不把“使能时托稳”冒充整个采样全程无接触的事后确认。既有J345确认旁注不自动覆盖本轮J2。
- 验证/收尾：两个离线Python套件12/12实际通过（合成/Mock/既有几何对照，不是实物模型验收）；采样脚本AST解析通过，05实板连续通信未再触发Stale board time，并实际验收两次中间G交接。README/AGENTS同步最新状态。本轮完整J2序列、持姿增益修正/部署、接触标注复核、分段有效数据审核及独立验证/拟合部署未完成；不因已出现正反向数据就冒称全程测试完成。下一步需解决J3主动持姿增益，涉及再次更新实板时应先满足相应现场承托条件。

## 2026-11-19（America/New_York）｜删除运动/换段瞬时静止门槛；开始既有数据重力残差分析

- 用户明确删除“五轴瞬时速度都不超过0.04rad/s”，说明实机加速度带来的其它轴影响尚未辨识到该步；随后要求完成后用现有运动数据计算重力补偿还需多少力并分析。按embedded-firmware-workflow/ponytail仅修改已授权运动/采样入口，不把此要求扩为清错、删除真实失联/温度/机械行程/力矩检查或修改零点、G、主动持姿参数。
- 已修改`Task/five_control.c`的`FiveControl_Move`，去掉`!still()`；删除`tools/five_gravity_sweep.py`换段前的0.04阈值与3秒静止等待。旧协议取数、冷启和独立故障确认等非轨迹入口未改；没有新增替代速度门槛。现有名义CAD RNE/速度阻尼和加速度项未被说成实机已辨识，也未擅自开关该算法。
- 快速验证已交付：重建主机目标成功，`five-motion-admission`实际通过；新增每个关节分别正/负方向超过0.04rad/s时接受有界新轨迹并完成一次真实控制Tick的10例断言，仍要求无fault且submitted31。后续原非法目标/行程/有限值用例保持。候选ARM/全回归及已有数据分析继续进行中，尚未烧录、清错、追加运动或部署拟合系数。
- 删除门槛收尾：全32/32主机CTest通过，76步完整ARM构建和`check_v12_image.py`结构门禁通过。候选`build/v12-no-motion-rest-gate-20261119/ylamp_v12_five_axis.bin`为155224字节，SHA-256 `D2A6AD87DC288DB8B91FDB25E7D1E74B190064BDCC79841D18954EA47ACDB7A8`；与原3C…ELF/BIN逐字节比较`g_joint_reference_config/g_five_mit_hw/g_five_mit_tracking/g_five_support_gains/g_five_j3_gravity_scale/g_five_j3_gravity_cosine_nm`均相同。没有为门槛删除顺带降低J3增益或改重力。候选未烧录、没有连接/操作实板，不能把实板原门槛称为已消失。
- 已有数据分析实现：新增纯离线`tools/five_gravity_residual_report.py`，从原记录审核身份/参考/时间/控制状态，按SHA关联J345原无接触追认；不把J2未追认或失败记录改标为成功。`five_identify.py`仅将原真实时间对齐体提为`align_feedback`供复用，原默认严格load_run审核不放宽（原多段425000ms限制仍未改）。探索性读取允许明确标注的健康前缀：段两端各排除1秒、首次控制异常前1秒；不是伪造end/handover。只用被扫关节的低动态方程，使用同步实测五轴姿态，不扣除持姿PD而抹去真实缺量。
- 模型：J2～J5共8个共享势能梯度组合＋各自Fc/B奇速度项，不拟合J1或基座倾斜，不把有效重力组合当唯一质量/COM。扣除的是小量名义CAD动态估计、上限0.03N·m，不是实机加速度/速度耦合辨识。当前补差定义相对原G（含J3的1.18/−0.01cos），候选全局形式为G_CAD+Yg*delta，若将来部署须吸收替换原J3经验项，不能叠算。
- 实际首次计算的固定“每五个2秒块留一块”因J5正向留出不足15点而拒绝（未生成半成品输出、未声称成功）。改为仅依关节/速度方向激励数量选择完整2秒留出块，不看残差，训练/留出每个方向至少15；不把一个时间块拆给两组、不降低质量阈值或伪造更多J5点。J2/J3/J4/J5入选100Hz点350/1249/2668/128，正负分别155/195、486/763、831/1837、35/93。时间点有相关性，不当作独立试验数量。
- 初算及扩展验证输出`gravity_residual_01.json/.csv`和最终`gravity_residual_02.json/.csv`；最终220个训练2秒块、400次重采样399满秩/1秩不足，归一化条件数6.0897，状态始终exploratory_not_deployable/deploy=false，399/400不是模型正确率。用户已进度获知数值和置信限制；报告`build/v12-no-motion-rest-gate-20261119/gravity_analysis.md`列出全部结论、条件切片、原角/姿态及部署限制。
- 数据内预测补差第5/50/95百分位（不是置信区间）：J2[−0.368183,+0.051500,+0.294252]、J3[−0.029082,+0.023876,+0.040620]、J4[−0.051015,−0.020050,+0.050226]、J5[+0.017430,+0.033895,+0.050230]N·m。当前G加训练摩擦与候选G加摩擦的内部留出RMSE依次0.144971→0.121631、0.097099→0.080192、0.110948→0.098240、0.132846→0.114226N·m；不能把包含摩擦的原始误差说成全是重力缺量。
- 跨记录仅J2可检查：整份旧J2排除训练后0.249588→0.194180N·m，整份新J2排除后0.329692→0.146524N·m（对比均用训练得到的摩擦项）。J3/J4/J5没有独立重复记录；J2全程无接触未独立追认，Fc/B中B≈−1.232952为模型警示，不是可部署负阻尼。去掉名义动态扣除的重力预测敏感度第95百分位J2～J5仅0.002707/0.000335/0.001122/0.001055N·m；这仍不等于动态实机辨识通过。
- 代表姿态点的95%条件区间：新姿态J2 raw113.295°补差+0.330098[+0.233353,+0.395071]、raw189.795°补差−0.447611[−0.516380,−0.377678]N·m；J3 raw17.291°为+0.035710[−0.009187,+0.099339]；J4 raw−182.464°为+0.023498[+0.001759,+0.045580]；J5 raw−57.025°为+0.037202[−0.128468,+0.135371]。每点固定其它轴的实际采样姿态，详见报告，不能推广成固定增量。J3/J5跨零不支持此时确定加力，也不因拟合重改零点/CAD。
- 验证：离线14/14通过，新增四轴合成势能/摩擦回收且G1恒不拟合，以及失败前缀/无接触未确认/原SHA和文件字节不变/故障前排除的格式回归；不是实机模型验收。README/AGENTS已同步候选未部署及报告入口。本轮无重刷、复位、失能、清错或新运动；实板仍3C…、最后实读fault32未由本分析清除。后续若用户要求更新实板，需明确满足该次现场交接条件；重力候选仍缺J2接触追认、独立验证/摩擦模型复核，不自动应用。
- 最后溯源复核通过：最终JSON内全部源码SHA和四份输入原文件SHA均与磁盘一致，deploy=false、8重力组合、399有效重采样、CSV有45个角度点；原数据未修改。JSON SHA-256 `775435E883954E79F132AC148D1391ABD510238C37B22F7A142B6B94B9081816`，CSV `D3368C11007BCBECE80A262B9927920D45D7F24174E90E602ED175C8F211F9AC`；候选BIN再次实算仍D2A6AD87…B7A8。文档diff --check退出0，仅Git换行提示，未发生新的实板动作。

## 2026-11-19（America/New_York）｜修改对比与最终结果交付

- 需求与范围：用户要求“对比修改了什么”，随后要求“给我一个最终结果”。本轮只核对并交付已有源码变更、离线分析和最后实板状态，不追加实现、运动、清错或烧录。
- 操作与发现：复核当前运动准入、主机采样/时间戳处理、补偿配置及最终`build/v12-no-motion-rest-gate-20261119/gravity_analysis.md`。改动包括候选固件删除Move的全轴瞬时静止门槛、主机删除换段静止等待、前轮已实测的逐段原G交接和新鲜完整遥测有界读取，以及既有数据的离线重力/摩擦分离分析；不是已把新拟合公式写入实板。
- 交付结论：J2在最新采样姿态raw113.295°/189.795°的额外有符号力矩约+0.330/−0.448N·m，不能统一提高倍率；J3/J5代表点95%条件区间跨零，不能确定继续加力；J4代表点小幅补差仍缺独立重复验证。零点/CAD、现有J3的1.18倍/−0.01cos和主动持姿参数本次未改。全部估计依赖其它轴姿态，非全工作区固定偏置。
- 验证与状态：本轮为源码/文档核对，没有重复运行构建或主机回归；此前32/32主机测试、14/14离线Python测试、完整ARM构建和镜像门禁通过，不能替代实板验证。最后已确认板上镜像仍3C728955…，候选D2A6AD87…未烧录，拟合deploy=false；最后实读fault32未清除。本轮唯一写入为本过程记录。
- 未完成事项与风险：新运动准入仍未在板上生效，新重力模型尚未独立验证/部署，J3主动持姿瞬态超预算未解决；不把“代码和离线分析已完成”称为“整机补偿更新已完成”。最终交付报告、条件补差与已生效/未生效状态，未发起新硬件操作。

## 2026-11-19（America/New_York）｜下一步建议：稳定主动采样，再补J2和独立验证

- 需求与范围：用户询问“那你觉得下一步是什么”。本轮为建议与状态核对，不将询问视为修改参数、烧录、清错或恢复运动授权。
- 实际核对：读取README顶部、过程日志最新记录和`config/five_mit.c`；当前J3主动采样增益仍15/1、预算1.8N·m，纯G经验项仍1.18/−0.01cos，删除运动静止门槛的候选仍未部署。依据既有离线回放的约2.279N·m请求超预算证据，不把该瞬态归为已证实的静态重力缺量。
- 已交付建议：先核对J3速度尖峰与编码器差分，并将主动采样增益回到此前5/.5作为待验证首选，纯G/手动阻抗/原预算不动；与已批准删除运动静止门槛的改动合并为一次候选更新。随后完成J2已批准行程内正反向、两档速度采样，同步五轴实际姿态；J1继续不扫，J345既有数据保留，只针对独立复测和J5正向薄弱数据补充。重力/摩擦重新分离后，用未参与拟合的记录验证并做新旧补偿对照，再决定部署；J3经验项若被新模型吸收须替换，不能重复叠加。
- 验证与状态：本轮仅完成源码/文档核对，没有运行新的数值拟合、主机测试、目标构建或硬件操作；不把建议增益说成已经有效，也不复用历史承托确认声称当前满足复位/烧录条件。唯一文件修改为本日志。
- 未完成与风险：上述实现、镜像验收/现场交接、完整J2采样及独立验证均属下一步建议而非完成事项。方案讨论已交付；实际板况仍以最后实读记录为准，未主动清除fault32或启动电机。


<!-- original-process:end -->
