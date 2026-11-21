# 过程历史｜2026-11-09

> Source: code/process.md; original dates, timezones, wording and within-day order are preserved.
> Paths inside original entries remain relative to code/. Historical states are not current instructions.
> Current entries: [process.md](../../process.md) / [README.md](../../README.md).
> Original SHA-256: 4C7CCD31CA17676A9FF4F418397FD5164063B5933E65F67640396A409DCB073B; preamble: # 过程日志 followed by two LF; original records: 40; payload bytes: 169827.

<!-- original-process:begin -->
## 2026-11-09（用户本地日期）｜“一动就报错”的只读诊断

- 需求与范围：用户询问新固件为何仍一动就报错。本轮仅定位原因，不实施保护/阈值/故障退出策略变更，不使能、清故障、停核、复位、启停IMU或再次烧录。采用embedded-firmware-workflow和aixprobe读取运行状态，按ponytail复用现有接口，不新增诊断框架；已交付锁存故障结论。
- 驻留身份与工具：OpenOCD普通连接，Cortex-M7 r1p2、CPUID0x411FC272、DBGMCU0x10016483，目标始终running；依据新ELF核对status地址0x20002730/长度184、uwTick地址0x20003B30及两轴配置布局。运行中分块读取185496字节Flash并计算SHA256，确认为本次坐标修复版 `F5B2FBE49C2D9D85FD6B35503BEB467D32C234949BEC6B142F20C0270BF2FFB2`，不是未烧入新代码。
- 实机结果：tick101622～101759的四组读取均为Task FAULT=5、reason SPEED=7、policy1、feedback_mask3、disable_submitted_mask3；两个失能提交状态均OK，且两轴实际新鲜反馈state0。数据年龄18～25ms、计数增加，CAN1 CCCR0x1000/ECR0/PSR0x708，非Bus-Off；温度MOS30°C/转子27°C。J1参考−37.82345°、J2参考−154.51762°，均位于各自软件区间，读数时速度约−0.140～−0.420°/s；该静止读数不是触发故障瞬间的速度。
- 直接配置证据：两轴RAM的measured_speed均0.17453292rad/s（约10°/s），start_speed均约1°/s，protection_flags均7。对照Task/motor_task.c::check_feedback和fault：活动阶段直接检查未作该处低通/持续时间确认的反馈速度，单次控制检查超10°/s可返回SPEED，继而双轴提交失能并锁定，撤去重力支撑。故障原因保持，但反馈会继续更新；当前实现没有锁存触发轴号、原始帧或速度峰值，不能从当前静止值断言是某一轴/噪声/用户具体速度导致。
- 控制目标冲突：Mode的Kd按30ms低通速度在1～8°/s区间平滑提升，再受10单位/秒斜率限制；Task却以原始反馈10°/s立即失能。快速手动拖动可能先触发失能，尚未建立期望阻尼；保留这种低速试验阈值与“高速拖动大阻尼”目标不匹配。本次坐标修复此前明确保留了该策略，没有修复这一冲突。最后保存的J2请求Kd约0.4055、FF约+0.05509N·m仅为故障前请求残留，不是失能后的实际输出或触发峰值。
- 验证与收尾：完成当前固件身份、四组运行快照、RAM阈值和源码判断链交叉核对；tick157417再次读取仍为同一FAULT/SPEED。首次尝试单次读取整个应用超出OpenOCD命令可输出长度，未取得哈希；改为4096字节分块后通过，未因此改写目标。正常shutdown退出调试器，目标继续运行。本轮无源码/配置修改，未运行无关构建或主机回归；仅追加本日志。
- 待决策与边界：应另行确认正常拖动速度区间、异常超速判据及退出时负载支撑行为后调整保护，不盲目删除所有保护或擅自放大阈值。运行中正负跳变根因、补偿方向和手感仍未据本次静止诊断验证；提醒负载需可靠支撑，故障后KEY不会绕过锁定。

## 2026-11-09（Asia/Shanghai）｜外部只读代码审查：仅记录发现的逻辑问题

- 审查时间：2026-11-09 11:29–11:56（Asia/Shanghai）。只读审查 `code/` 下 App/BSP/Device/Task/Mode/config/tests 及上位机 `tools/dm_serial.py`、`tools/dm_tuning.py`，并用厂商 SDK `C:\Users\rlyoulo\Desktop\model\H7_BSP\H7_BSP-HEAD` 与 Bosch BMI088 寄存器定义交叉核对。按要求只写问题，不写已核对无误的项，也不含修复实现；本轮未改源码、未连接硬件、未烧录。
- 快照说明：审查期间 `CMakeLists.txt`、`tests/test_bsp.c`、`tests/test_motor_serial.inc`、`Task/motor_serial.c/.h`、`App/main.c`、`App/serial_dma.c`、`BSP/UART/bsp_uart.c`、`README.md` 与本日志的时间戳连续变化；下列行号以 2026-11-09 11:56 的文件内容为准。
- 问题 P1｜IMU 运行期加速度计上电寄存器顺序与初始化路径相反：`update_power()` 的地址表固定为 `{0x11, 0x7C, 0x7D}`，因此“开”方向先写 ACC_PWR_CONF(0x7C)=0x00 再写 ACC_PWR_CTRL(0x7D)=0x04；同文件初始化路径与 Bosch 要求是先 CTRL=0x04、等 5ms、再 CONF=0x00。关断方向（CONF=0x03→CTRL=0x00）顺序是对的，单一固定地址顺序必然让其中一个方向错。
  - 影响：加速度计可能停留在 suspend，无 ACC DRDY；`BMI088_Run` 因 `ready[0]==used_ready[0]` 一直不启动采样，`BMI088_GetSample` 永不成功，`CAL_IMU_FRESH` 永不置位，`JointCalibration_Mark(2,…)` 的“精确 5 标志”判据使 J2 朝上零点标定恒返回 NOT_READY；因 `drag_j2_zero_rad` 可从配置恢复，拖动功能不受影响，现场不易发现。
  - 出处：`code/Device/BMI088/bmi088.c` 第 137–180 行（`update_power`，第 139–141 行固定地址/数值表、第 142–143 行按 `power_step` 选地址、第 174 行递增）、第 68–75 行（初始化先写 `{0x7D,0x04}` 后写 `{0x7C,0x00}`）、第 86–88 行（结束时先写 `{0x7C,0x03}` 后写 `{0x7D,0x00}`）；外部依据 Linux 内核 `bmi088-accel-core.c`（PWR_CTRL=0x7D、PWR_CONF=0x7C，`bmi088_accel_power_up()` 先 CTRL 后 CONF）。
  - 时间：2026-11-09 11:45–11:52（Asia/Shanghai）；外部依据访问时间 2026-11-09 11:45（Asia/Shanghai）。
- 问题 P2｜IMU 初始化失败被静默吞掉：`main()` 中 `if (BSP_IMU_Init() == HAL_OK) { (void)BMI088_Init(); }`，失败既不置 `g_fault_stage`，也不产生状态位或 LED。
  - 影响：P1 这类失败没有可观测入口，只能靠上位机 op10 才知道“IMU 不可用”；`BSP_IMU_Init` 失败时连 `BMI088_Init` 也不会被调用。
  - 出处：`code/App/main.c` 第 157 行；同函数第 155–158 行其它外设失败均走 `Error_Handler`。
  - 时间：2026-11-09 11:52（Asia/Shanghai）。
- 问题 P3｜`MotorTask_Run` 对 `RECOVERING` 的返回码自相矛盾：控制周期未到时的短路返回把 `RECOVERING` 当作 `BSP_CAN_OK`，正常出口却把它算作 `BSP_CAN_ERROR`。
  - 影响：现有调用者都用 `(void)` 忽略，暂无表现；任何按返回值判断“是否故障”的新调用者会读到抖动结果。
  - 出处：`code/Task/motor_task.c` 第 459–461 行与第 521–522 行。
  - 时间：2026-11-09 11:52（Asia/Shanghai）。
- 问题 P4｜`fault()` 路径下 `recovery_return` 恒等于 `FAULT`：函数先把 `state` 改成 `MOTOR_TASK_FAULT`，再调用 `begin_recovery()`，而 `begin_recovery()` 用 `task.status.state` 记录“故障前状态”。
  - 影响：“恢复后回到故障前状态”在该路径是死逻辑（当前被 `resume` 分支掩盖）；一旦有分支清掉 `recovery_resume`，状态会落到 FAULT 而不是故障前状态。
  - 出处：`code/Task/motor_task.c` 第 147 行（`recovery_return = task.status.state`）、第 158–171 行（`fault()` 先赋值 `state` 后调用 `begin_recovery`）。
  - 时间：2026-11-09 11:52（Asia/Shanghai）。
- 问题 P5｜循环 DMA 游标别名导致整缓冲静默丢弃：`HAL_UARTEx_RxEventCallback` 用 NDTR 求游标，`at == previous` 时视为“无新数据”。若两次服务之间 DMA 恰好推进整整 `BSP_UART_DMA_RX_CAPACITY`(512) 字节（长时间屏蔽中断或 HT+TC 合并），这 512 字节既不交付也不计数。
  - 影响：该丢包不可观测，遥测中的 `rx_overflows` 无法反映它。
  - 出处：`code/BSP/UART/bsp_uart.c` 第 167–184 行。
  - 时间：2026-11-09 11:56（Asia/Shanghai）。
- 问题 P6｜遥测序号与丢帧计数语义不一致：`due` 成立时就自增 `link.sequence`，随后该帧若被 RPC 抢占而丢弃，序号已经前进；`link.tx_drops` 同时统计“RPC 抢占遥测”与真实发送失败。
  - 影响：上位机看到的 `sequence` 跳号不等于发送失败，`tx_drops` 不能当误码率；协议文档未区分这两类。
  - 出处：`code/Task/motor_serial.c` 第 266–267 行、第 290 行、第 293 行、第 297 行；上位机对应解析见 `code/tools/dm_serial.py` 第 161 行（不区分这两类）。
  - 时间：2026-11-09 11:56（Asia/Shanghai）。

## 2026-11-09（America/New_York）｜接回目标后烧录当前最新版

- 需求与范围：用户说明已接回控制板、电机尚未上电，并要求“先烧录新版代码2”；按当前最新源码构建产物执行备份、烧录、完整镜像回读和启动通信检查，不发送使能、运动或写参请求。使用embedded-firmware-workflow、aixprobe及ponytail；aixprobe专用MCP未暴露，沿用本机OpenOCD/ST-Link原生工具链，不新增调试框架。
- 状态：本次新版部署和空闲启动/通信验收完成，已先通过进度消息交付烧录、完整回读及启动结果；原CAN/卡死定因尚未完成。PresentOnly设备包含STM32 STLink与CH340 COM34；连接前未发现已有OpenOCD/GDB/Python监控进程。本轮没有修改控制源码，仅同步README部署状态和本日志。
- 构建：build/arm-audit当前Debug完整固件构建成功，Ninja报告无需重编译；ylamp_firmware.bin为197244字节，SHA256 `330E9A2EF4BD6F5627ABEEB1C5AD7038E618EFD08DF49FF2AB79A4C0071AB678`。沿用已有匹配ELF，不覆盖build/arm历史部署镜像及旧故障证据。
- 连接与备份：使用本机OpenOCD 0.12.0+dev-snapshot（2026-09-24）普通SWD连接，目标电压3.301818V，CPUID `0x411FC272`、DBGMCU_IDCODE `0x10016483`，识别STM32H72x/73x Rev Z、单Bank 1MiB。停核后完整备份到build/arm-audit/before_latest_20261109_flash.bin，1048576字节、SHA256 `F6D17332C689367207714434B9194730FA47EA4873F0A9C0599CD790715DCDD5`；没有覆盖已有镜像或故障dump。
- 烧录与比对：flash write_image erase写入上述匹配ELF，Flash写入对齐197248字节；应用有效长度197244字节完整回读到build/arm-audit/readback_latest_20261109_flash.bin，逐字节比较全部一致，SHA256与构建bin均为 `330E9A2EF4BD6F5627ABEEB1C5AD7038E618EFD08DF49FF2AB79A4C0071AB678`。校验通过后执行一次reset run，没有反复重刷。
- 启动与运行取样：COM34打开前DTR/RTS=false，230400/8N1仅监听4秒，收到200帧、解析错误0；Task为IDLE/NONE、两轴反馈state0（失能），Kp/Kd/FF均0。随后在失能空闲下短暂停核，按本次匹配ELF读取：g_rtos_control_iterations为104087，栈历史最小余量1840字，g_fault_record.magic=0，g_fault_stage=12（WAIT），HAL Tick105339；读取后resume。首次安装捕获reset_flags `0x01FA0000`可能累积旧固件未清除标志，不能作为原故障定因。
- 重连与后验：正常关闭OpenOCD后重新普通SWD连接，无复位即可读到相同CPUID/芯片ID，电压3.295354V；随后关闭调试会话并释放探针。最后COM34只读6秒接收45300字节/300帧（50Hz）、解析错误0，MCU Tick150633→156613未回退；两轴反馈计数各增加598，末帧数据年龄各10ms，始终state0，Task始终IDLE/NONE。can_observations、bo_ms、rx_overflows、crc_errors、tx_drops和uart_error均0，RX始终运行。
- 证据路径：build/arm-audit/flash_latest_20261109.log、reconnect_latest_20261109.log、startup_latest_20261109_serial.log及上述Flash备份/回读；匹配ELF为build/arm-audit/ylamp_firmware.elf。本轮无源码变更，不重复此前已通过的49项双配置离线回归；本次实际执行ARM构建检查和实物部署/空闲验证。
- 边界与下一步：用户先声明电机未上电，但本次实测确有两轴持续新鲜反馈，只能确认节点在线且回报失能，不能由此断言主功率供电或机械状态；没有从PC发送任何串口字节、握手、使能、运动、失能或写参命令，固件自身保持既定只读CAN查询。未验证实际KEY使能、补偿极性/手感、带载拖动故障复现、DMA最坏服务时延或独立断电冷启动。原CAN与卡死仍需同次故障现场，不删除尚需的临时取证、不擅加看门狗/复位或改变自动恢复出力策略。

## 2026-11-09（America/New_York）｜新版部署后再次抓到CAN故障现场

- 需求与状态：用户在新版部署后报告依旧触发CAN错误，要求查看。已通过进度消息交付首次实际故障数据；本轮只读取证完成，原故障尚未定因，下一步需核对实际收发器供电/发送波形。未修改源码、烧录、复位、清错或发送控制请求。使用embedded-firmware-workflow与aixprobe，保留本次同运行镜像和原始现场。
- 只读串口：COM34/230400监听8秒，PC发送0字节，接收60401字节/400帧、解析错误0；Tick268693→276673，Task始终FAULT/TRANSPORT，两轴反馈始终state0且末帧数据年龄各5ms，计数各增加798。rx_overflows、crc_errors、tx_drops、uart_error均0。MCU和反馈链仍在运行，不把本次CAN故障称作CPU卡死。
- 首错与Bus-Off：首次异常Tick167793，LEC5、TEC16、REC0；同一毫秒首次Bus-Off快照LEC5、TEC248、REC0。首错TXBRP=0x40，Bus-Off时0xC0；error_observations=300仅是软件异常采样次数，不能当成300个物理错误帧。
- 同次调试现场：仅在持续新鲜失能反馈后普通连接并短暂停核，使用本次已全长回读一致的build/arm-audit/ylamp_firmware.elf读取。PC在FreeRTOS Idle的prvCheckTasksWaitingTermination，g_rtos_control_iterations=384162、栈历史最小余量1834字，故障记录magic=0、阶段WAIT、启动reset_flags仍0x01FA0000。Task recovery_attempts=3、recovery_exhausted=true、recovery_phase=FEEDBACK、recovery_resume=false；last_transport为旧NOT_READY。最新CAN诊断Tick169499已经BusOff0、TEC0、REC0，硬件CCCR=0x1000（INIT清除）、TXBRP=0、TXFQS空闲8，BSP恢复状态0。故“现在仍显示通信故障”不等于当前总线仍在Bus-Off。
- 已保存：build/arm-audit/fault_after_latest_20261109_serial.log、fault_after_latest_20261109_gdb.log、fault_after_latest_20261109_openocd.log、fault_after_latest_20261109_dtcm.bin（131072字节）和fault_after_latest_20261109_message_ram.bin（512字节）。GDB会话已执行resume并断开；调用返回非零，monitor寄存器输出被PowerShell包装为NativeCommandError，目标读数、两个完整dump和OpenOCD连接日志实际存在，不能把主机调用状态当作MCU异常。消息RAM为恢复后现场，队列槽已可能复用，不能据此还原首错帧载荷。
- 已知限制：恢复完成后还需两轴失能、新鲜、角度/温度正常、速度低于启动门限并连续稳定100ms；现有快照表明尝试预算耗尽，但未保存每次门控失败的时间序列，不能仅从最后静止样本反推当时必然静止。后续核对实际GPIO/时钟和发送物理链路，不擅自放宽恢复出力条件或用自动重启掩盖故障。
- 引脚/时钟实际核对：再次先监听1秒确认50帧新鲜失能FAULT反馈，才短暂停核读配置，并在finally恢复运行。未找到配套H723 SVD，按项目ST官方stm32h723xx.h及HAL定义核对而非套用其他系列。GPIOD_MODER=0xFFFFFFFA、OTYPER=0、OSPEEDR=0、PUPDR=0、AFRL=0x99，PD0/PD1均AF9推挽、无上下拉，未被其他业务占用；RCC_PLLCKSELR=0x0180C022、PLL1DIVR=0x01030027、PLL1FRACR=0，按板级HSE24MHz得到PLL1Q120MHz；D2CCIP1R=0x10002000选择PLL，APB1HENR=0x100使能FDCAN，NBTP=0x12021C09对应1Mbps、75%采样点。CCCR仍0x1000、IE=0x1B800001，配置与既定方案相符；事后配置正常不排除瞬态硬件故障。
- 错误含义边界：本地Bosch M_CAN User's Manual 3.3.1第18页定义LEC5为期望发送显性而监测到隐性，且Bus-Off恢复时也可用5表示空闲序列进度。本次first快照BusOff0、TEC16、REC0、error_passive0，因此最早异常并非仅因正常恢复状态而显示5；但不能据此单独断定具体线材、终端、电源或收发器已坏。在线官方资料查询未返回可用内容，实际依据为既有本地厂商PDF及本次现场。CAN1原理图U1的VDD接VCC_5VS、VIO接3.3V、STBY接地，5V收发器电源需板载USB或24V链路，不等同于SWD测得3.3V。
- 证据固化与收尾：另存fault_after_latest_20261109_firmware.elf/.bin防止后续构建覆盖匹配镜像；ELF SHA256 `EC43E32A4268F121F0FB5D07C5F961343B64BDCA6674ECBE5B85190A97A60003`，bin保持上述330E镜像身份。DTCM SHA256 `4A466E0111BBA5E3DE50E6F3E5611E265A02901C3E3A00F05EB13C3F495E15AF`，消息RAM SHA256 `819A20370B67A38B6B197FAB3EDF8AA3897C10338F0BAA465E69956D9D5484E2`。引脚/时钟读数存pin_clock.log，结束后正常shutdown释放OpenOCD，板卡继续运行；末次2秒监听99帧、解析错误0，Tick891233→893193，Task仍FAULT/TRANSPORT，两轴失能且数据年龄14ms，can_observations仍300、uart_error0，无新异常增长。原始167793首错与Bus-Off记录未清除；物理发射链瞬态、电源/共地/终端/连接需进一步实测，尚无可据此直接实施的确定性源码修复。

## 2026-11-09（America/New_York）｜评估是否硬件损坏

- 需求与范围：用户询问是否硬件已经损坏。本轮仅复核已保存的故障后串口、引脚/时钟读数及Task恢复代码，未重新连接目标、停核、复位、烧录或发送执行器命令；无源码修改。
- 结论及验证：已有现场确认曾发生发送位错误及Bus-Off，但随后CAN恢复且两轴持续新鲜回传，不能据此认定永久性硬件损坏，也不能排除间歇性硬件故障。Task在三次恢复尝试耗尽后锁存FAULT，解释持续报错状态，不等同于总线一直故障，也不能解释最初位错误的根因。
- 边界与后续：供电瞬态、共地/接触、终端和干扰只是待验证方向；事后配置正确不能排除软件时序或恢复逻辑问题。建议下一步受控比较失能、使能静止与拖动时的首错，并结合已知正常的供电/短线交叉验证，再决定是否更换器件；本轮未执行这些带输出或接线变化的测试。
- 状态：硬件损坏判断已交付并完成记录，原CAN故障根因仍未确定。仅追加本日志，无需重跑源码构建或回归；没有新增实物验证结果。

## 2026-11-09（America/New_York）｜拖动手感满意后的下一步建议

- 需求与范围：用户说明当前拖动效果暂时满意，询问后续方向和优化建议。使用embedded-firmware-workflow及ponytail核对现状，不把讨论当成实现授权；无源码、参数、镜像或恢复策略变更，无目标连接和执行器动作。
- 已核对：README当前部署与未决问题、Mode/drag_mode接口及主要控制计算、config/motor_config.c、上位机拖动状态展示和CSV字段。现有实现已有重力、25%实测加速度动态补偿、手端几何阻尼及转保持生命周期；界面已有目标/误差/前馈展示，现有CSV主要导出电机反馈；定向搜索未找到主动点到点轨迹或示教回放实现。
- 建议及交付：保留当前拖动作为基线；优先补齐控制指令与反馈的同步记录，并闭环CAN故障。下一项具体控制功能建议原位保持→单次小角度运动→到点保持，依次验收J2、J1和双轴，之后再扩展示教回放；建议复用现有输出提交入口，按独立Mode管理主动运动生命周期，不重建通用框架。
- 模型边界：主动运动的目标位置、速度、加速度应来自同一连续轨迹，动态前馈使用规划加速度，与拖动的实测加速度部分补偿明确分开，避免直接叠加；暂不建议增加摩擦补偿或提高拖动惯量抵消比例。以上是拟议方案，尚未实现、调参或实机验证。
- 验证与状态：本轮仅文档/源码定向检查和方案讨论，已通过进度消息交付结论；在线一手技术资料搜索和打开未返回可用内容，未据此声称查证成功。只追加本日志，无需重跑构建或控制回归。原CAN根因、模型实机精度和主动运动验收仍未完成。

## 2026-11-09（America/New_York）｜记录现状并收拢拖拽Mode

- 需求与状态：用户要求先记录当前问题和已有修改，再将拖拽整理成一份Mode代码，随后讨论单次运动的速度/加速度/路径规划。使用embedded-firmware-workflow及ponytail；成果已通过进度消息交付，源码整理与离线验证完成。整理阶段未获烧录或主动运动实现授权，故该阶段不连接目标、不改活动RAM、不发送控制命令；用户后续烧录授权及其预检另记下条。
- 先行记录：README顶部新增行为/修改汇总及P01..P15问题总表，区分已修复、实测未定因、模型假设及尚未验收项；记录用户“当前拖拽效果暂时满意”，不将手感反馈当作通信或安全验收。保留原历史与故障证据。当前目录无Git仓库，不声称有可用Git差异或提交。
- 范围与实现：原实现已有DragMode生命周期，本次将Mode/drag_geometry.c/.h的几何、雅可比、阻尼计算及公开检查接口原样移入Mode/drag_mode.c/.h，移除原两个辅助文件，更新CMake源清单、C/C++测试头引用和README。Task仍负责公共门控、使能/失能、恢复及唯一CAN提交；config原参数与模型系数未改。没有新增模式框架、RTOS任务、摩擦补偿、恢复策略或主动轨迹。
- 契约：保留Enter/Update/Stop/Resume/Reset、状态值及38项协议，保持公式、计算运算顺序、滤波、前馈预算、零点、参考圈、KEY和恢复条件。Reset仅清算法状态，不表示硬件已停止；几何检查接口同属DragMode而非另一模式。
- 快速验证：重构前build/host主机Debug原49/49通过。新增tests/test_drag_mode.c及drag_mode_standalone，仅链接Mode、config及数学库，不链接Task、Device/BSP实现或RTOS；重构前独立链接按预期因两个DragGeometry外部符号缺失失败，合并后构建与运行通过，覆盖预览、进入、重复Stop保持点不漂移、保持/继续、失败更新原子性及独立实例。首次构建未补MinGW运行时PATH而失败无编译诊断；按README补PATH后正常，不将主机工具环境问题记为固件缺陷。
- 镜像边界：部署匹配ELF/bin归档fault_after_latest_20261109_firmware.*已核对SHA256仍为EC43E32A…/330E9A2E…；README已改为引用固定归档，避免本轮重编译覆盖默认产物后误解旧现场。未改写归档、Flash备份和故障dump。
- 完整验证：主机build/host Debug与build/host-release Release各50/50通过（0.82/0.81秒），包含新增独立Mode与现有Task、故障、坐标、CAN恢复回归；ARM build/arm-audit与build/arm-audit-release完整应用及BSP链接检查均成功。Debug应用197244字节、Release134328字节，DTCM40624字节、DMA AXI768字节。源码扫描无废弃drag_geometry引用（仅README迁移说明）、Mode无HAL/BSP/电机提交/RTOS调用，ARM未解析符号仅数学、内存函数与只读模型配置；C++头及链接检查通过。未修改上位机/建模脚本，不重复其无关测试。
- 候选身份与边界：重构Debug bin SHA256 `66408758101B938B24FDACFC9474755AEC82FAB803C3D2154A584441C01BE190`，Release `C502B0F4EC433ED5E89761F9DE7B9F46876B21F709047A6F360D1CB5E3BDAC2E`；旧部署固定归档仍保留，不使用新ELF解释旧内存。原CAN根因、实际模型精度、手感实机回归和主动运动验收未完成，不从离线测试宣称解决。

## 2026-11-09（America/New_York）｜授权烧录预检与单次运动Mode讨论

- 需求与范围：用户“允许烧录，然后我们开始讨论单次运动Mode”；授权部署上条整理的拖拽版，不授权本轮实现主动运动。继续embedded-firmware-workflow、ponytail并按aixprobe调试流程核对目标；专用探测MCP未暴露，实际仍使用项目本机工具，不引入新框架。
- 实际预检：PresentOnly枚举见ST-Link和CH340 COM34，未发现既有OpenOCD/GDB/Python监控进程。候选Debug镜像长度与SHA256核对仍为197244字节/66408758…。COM34以230400/8N1打开，预设DTR/RTS=false，仅监听3秒，PC发送0字节，收到22651字节/150帧，解析错误0，finally关闭。
- 发现与暂停原因：Tick1417573→1420553，两轴计数各增加596、反馈年龄各5ms；两轴state均1（使能），Task为RUNNING/NONE，拖拽Kp0；J2 FF约−0.0354N·m，仍有实际支撑请求。can_observations、rx_overflows、crc_errors、tx_drops、uart_error均0仅代表该窗口，不能追溯关闭原故障。预检断言按预期拒绝带使能进入停核/擦写，不是遥测解析或固件崩溃。
- 当前动作边界：已先告知用户托住负载、通过原上位机F6双轴失能，待新鲜反馈确认后再烧录；没有停核、复位、擦写、修改目标RAM，也未发送协议握手、失能、使能、运动或写参请求。烧录授权已记录，不需重新询问授权；当前缺的是撤去支撑前的机械安全条件，不宣称已经烧录。
- 再次核对：随后同样只读监听2秒收到99帧、解析错误0，末帧两轴仍state1、反馈年龄各5ms，Task仍RUNNING；J2位置−2.638094rad、FF约−0.00113N·m。末帧Tick13973小于前一次1420553，说明两次取样间MCU运行时间已经重置或重启；本轮没有发复位/烧录/串口请求，缺少同次复位原因，不能推断是用户复位还是原复位问题。仍未进入停核/烧录。
- 讨论稿：README补充关节空间路径/公共时间缩放、静止到静止五次曲线、双轴共用s(t)，位置/速度/加速度同源，时长需兼顾速度/加速度/jerk与名义力矩余量。五次曲线不是jerk连续或七段S曲线，不承诺笛卡尔直线或时间最优。拟议主动MIT为规划位置/速度、独立Kp/Kd和M(q)*ddq_d+c(q,dq)+g(q)；不叠加拖拽25%加速度回灌/手端阻尼，无摩擦补偿。所有新限额、正常中止、模式切换和故障后动作尚待讨论，不改变现有策略。
- 验证与剩余：沿用上条已完成的50/50双配置主机回归和ARM双配置链接；本段无控制代码修改，不重复构建。候选实机烧录、完整回读、启动及普通重连尚未执行；原CAN根因、带载手感回归与主动运动验证仍未完成。

## 2026-11-09（America/New_York）｜再次授权后烧录单模块拖拽版

- 需求与状态：用户再次明确“烧录”；部署前述单模块拖拽Debug版，不修改控制参数、源码、KEY、恢复策略，不实现主动运动。使用embedded-firmware-workflow与aixprobe流程；成果已先通过进度消息交付，写入、完整应用回读、一次reset run、串口运行及普通调试重连验证完成。烧录任务完成，CAN/电机联机验收未通过，原根因未关闭。
- 实际预检：ST-Link与CH340 COM34在场，无既存调试进程；ARM Debug完整构建报告ninja无需重编译。串口3秒收到150帧/22651字节、解析错误0、PC发送0字节；Task已为FAULT/TRANSPORT，valid_mask0，两轴本次启动反馈计数均0、Kp/Kd/FF均0。首错及Bus-Off Tick1263、LEC5、TEC16/248、REC0，异常观测321；不同于上条仍在拖动的预检，当前无有效电机回传。新鲜失能断言未通过，不将默认state0、失能提交状态0或无回传当作实际失能。
- 操作边界：已明确告知无回传不能确认失能，并要求保持负载支撑、不要按KEY；按此次烧录授权继续，在停核前再次只读监听约1.2秒确认始终FAULT、计数0与零控制目标，未发送握手、使能、运动、失能或写参请求。没有通过修改固件门控绕过预检；实物电机电源/状态未测得，保留这一限制。
- 连接与证据：OpenOCD/ST-Link普通SWD连接，目标电压3.293737V、实际SWD950kHz，CPUID0x411FC272、DBGMCU_IDCODE0x10016483，STM32H72x/73x Rev Z、单Bank1MiB。停核后完整Flash备份为build/arm-audit/drag_mode_single_20261109_before_flash.bin（1048576字节，SHA256 `0F7225EC426FCFD1D77D8D1061D319C852EC70CAD6708B7FFD4E52A69CE49344`），应用前缀与旧归档330E镜像逐字节相同；同时保存131072字节DTCM、512字节CAN消息RAM和FDCAN寄存器，保留此次擦写前现场，未覆盖旧证据。
- 烧录与校验：flash write_image erase写入匹配ylamp_firmware.elf，有效应用197244字节、对齐写入197248字节；全长回读drag_mode_single_20261109_readback.bin与候选bin逐字节一致，SHA256 `66408758101B938B24FDACFC9474755AEC82FAB803C3D2154A584441C01BE190`。通过后才执行一次reset run。外层脚本在shutdown命令关闭Tcl套接字时因预期EOF处理不当返回非零，finally又遇到已断链；实际写入、完整回读、reset run均已有成功记录，OpenOCD进程已退出，不据此重复擦写或误报MCU故障。
- 首轮启动验证：COM34/230400只读4秒收到30201字节、199帧、解析错误0，Tick41573→45533连续前进，PC发送0字节。Task仍FAULT/TRANSPORT、两轴反馈计数0、valid_mask0；首错/Bus-Off仍在Tick1263、LEC5与TEC16/248，can_observations321，UART及软件溢出/CRC/TX丢弃计数0。只确认MCU程序和串口遥测持续运行，未确认CAN链可用、实际电机状态或主功率供电。
- 重连与归档：普通关闭后重新SWD连接，电压3.295354V、CPUID和DBGMCU_IDCODE与前次相同；短暂停核读FDCAN CCCR0x1001、ECR0xF8、PSR0x7E7，随后resume、shutdown，未再次复位。OpenOCD退出0；halt前的unknown state警告不替代后续成功读取/恢复结果，也不作为新的MCU故障结论。新镜像固定归档build/arm-audit/drag_mode_single_20261109_firmware.elf/.bin，ELF SHA256 `05CB23DA339E9FDEB4F9557F879E3746F7E7110D86936B739A199EF11F8DF65F`、bin仍66408758…；旧EC43…/330E…归档哈希未变。
- 最后后验与资源：再次COM34只读2秒收到99帧、解析错误0、PC发送0字节，Tick170573→172533持续前进；仍FAULT/TRANSPORT、valid_mask0、两轴计数0，异常观测仍321，UART错误/软件溢出/CRC/TX丢弃均0。串口正常关闭，无OpenOCD/GDB进程残留，MCU恢复运行；不能把无反馈的默认state0当实际失能。
- 涉及文件与剩余：已同步README当前部署身份并补齐本日志；保留单次运动讨论稿，不重复此前50/50双配置回归（本次控制源码无变化）。原CAN根因、实际电机供电/状态、独立断电启动、KEY带输出和手感回归未验收。本轮未新增诊断代码或控制策略，未删除历史证据，未宣称通信故障已修复。

## 2026-11-09（America/New_York）｜路径规划选型与多自由度扩展审查

- 需求与范围：用户要求联网查阅优质路径/时间规划方案，结合现有机械臂指出缺项、复杂处及可简化合并的接口，并给出可选择路线。本轮仅分析和审查；使用embedded-firmware-workflow与ponytail，未修改控制源码、参数、协议、README讨论稿或部署镜像，未连接目标、运行监控、烧录、复位或发送电机命令。仅追加本项目过程日志。
- 状态：已先通过进度消息交付实际结论，分析与记录核对完成。推荐讨论路线为关节空间路径、公共标量进度的限jerk时间律、统一物理模型及现有唯一提交入口；不把方案记录为实现或用户已选定的策略。
- 在线取证：web查询/打开接口本轮未返回可引用正文；随后经只读HTTP实际取得Ruckig官方文档、官方仓库README与input_parameter.hpp、TOPPRA首页及Python API、MoveIt时间参数化与URDF/SRDF文档、Pinocchio官方首页以及Ruckig的RSS 2021论文摘要（arXiv:2105.04830），返回HTTP 200。Ruckig README首次请求连接失败，第二次成功；不将失败请求列作验证通过。资料入口为https://docs.ruckig.com/、https://github.com/pantor/ruckig、https://hungpham2511.github.io/toppra/python_api.html、https://moveit.picknik.ai/main/doc/examples/time_parameterization/time_parameterization_tutorial.html、https://moveit.picknik.ai/main/doc/examples/urdf_srdf/urdf_srdf_tutorial.html及https://stack-of-tasks.github.io/pinocchio/。
- 资料结论：Ruckig提供速度/加速度/jerk约束及时间/相位同步；多轴同时到达不自动保证公共进度或笛卡尔直线。当前社区版中间点功能会转云端，不能据此设计MCU实时链路。TOPPRA的JointTorqueConstraint可接逆动力学和力矩界，但其二阶约束不等于自动满足jerk界；MoveIt官方也明确时间参数化、Ruckig平滑及碰撞复检的职责差异。时间最优不是力矩/能耗最优，后处理后仍须复验原约束。
- 代码核对：检查README当前状态和惯量假设、Mode/drag_mode.c、Mode/joint_coordinates.h、Task/motor_task.c/.h、Task/motor_serial.c/.h、config/motor_config.c/.h、App/rtos.c、tools/dm_serial.py、tools/dm_monitor.py及惯量估算脚本的相关部分。定向文件搜索未找到当前主动轨迹、URDF/Xacro/SRDF模型；现有主动运动内容仍是讨论稿。当前Task直接调用DragMode生命周期与配置检查；模型为双轴A/B/D解析式；144字节串口包固定反馈和目标分区，上位机按两轴解析，不能只把JOINT_COUNT改大；CSV未覆盖完整指令/模型/事件时间序列。没有把这些扩展限制直接判为现有双轴运行缺陷。
- 规划与简化建议：提供五次曲线、公共进度Ruckig/S曲线、TOPPRA力矩约束以及未来MoveIt上层的分层选择。保留拖动Mode单模块、Task/Device/BSP边界及单周期多轴计算；新增Motion时仅抽真正共用的FK/J/M/c/g纯物理模型，不复制拖动25%实测加速度回灌/手端阻尼，不增加摩擦补偿。以固定上限N轴数组、单一坐标映射、带版本/任务ID的运动契约替代继续叠双轴分支；暂不引入模式注册框架、每轴RTOS线程或MPC。
- 缺项及风险：需明确J3是否实际固定；若被动姿态变化，不能继续无条件当固定刚体负载。既有模型仍缺已验证的编码器/CAD方向和驱动等效惯量；协议量程不能作为持续力矩/安全运动限额。执行层仍需轨迹缓存/共同时间基、v/a/jerk与力矩余量、正常制动及到点收敛定义、同步记录与周期/CAN时差测量。CAN首错/Bus-Off根因及独立停机/负载支撑尚未闭环；可继续离线设计，不声称带载主动运动已经可验收。
- 验证与剩余：已完成官方页面/原始源码的只读交叉核对、本地文件定向审查及日志回读；再次搜索未发现上述模型/主动轨迹文件，核对唯一目标提交、固定串口分区和CSV入口实际存在。无源码变更，不运行无关主机/ARM构建或实机测试。尚未实现/测试任何规划器，未测Ruckig在H723上的WCET/内存，未验证模型精度、轨迹跟踪、碰撞或故障动作；新模式正常中止/恢复策略未获得本轮实施授权，不沿用拖动自动恢复来重放主动轨迹。原CAN根因和实机主动运动验收仍未完成。

## 2026-11-09（America/New_York）｜将Mode边界、扩轴契约与规划优先写入技能

- 需求与范围：用户要求将两个Mode独立、只共享公共物理模型、Task不堆拖动分支、扩轴不只改数量宏，以及重点“保护前移，不靠增加报错门槛”有机融入技能。按本轮最新要求仅更新embedded-firmware-workflow技能及本日志；使用skill-creator规范维护现有技能，沿用ponytail的最小改动原则。不改固件、不烧录。
- 状态：技能更新、验证与本轮日志收尾已完成，成果与首次快速校验结果已通过进度消息交付。没有把先前拟议的模型剥离、算法目录或协议改造记作已实现。
- 实际修改：修改技能SKILL.md及references/architecture-code.md、lifecycle-safety.md、pubsub-protocol.md、review.md五个现有文件；不修改技能发现信息或agents配置。主入口突出规划优先，具体边界分别融入架构、生命周期、通信与审查规范。
- 架构与扩展原则：拖动与主动运动同时存在时保持独立Mode，各自拥有控制策略和生命周期；共享坐标、标定与FK/J/M/c/g等公共物理事实，不共享私有控制状态或混叠两种前馈。Task负责公共输入、调度及唯一输出提交，不推进Mode私有阶段；不预建通用模式框架。扩轴按模型、标定、协议版本/数量/统一记录、缓冲带宽、上位机和记录端到端核对，不把协议容量等同于控制或动力学已支持。
- 保护取舍：优先合理目标、连续轨迹、能力匹配、时间缩放和制动余量；不添加无依据、重复或互相阻塞的报错、逐次确认及锁定，不默认恢复已删除的瞬时速度跳闸。新增保护需有具体风险与阈值依据，并同时验证正常可用性；保留与实际风险匹配的失联、设备故障、过温、越界等必要处理，不据此批量删除既有检查或擅改启停/恢复出力。
- 已执行验证：以python -X utf8运行skill-creator的quick_validate.py，退出0并返回“Skill is valid!”；定向回读五个文件确认主入口与引用内容一致，审查条目限定于具有独立生命周期的拖动/主动运动，不强迫所有简单行为增加Mode。最后修订后再次运行同一校验仍通过，本轮日志追加后已回读，历史记录保留。
- 未执行与边界：本轮只有技能文档和日志变更，不运行无关主机/ARM构建、串口或硬件测试；未建立Algorithm/、Model/目录，未修改现有控制公式、保护参数、协议、KEY或恢复策略。先前模型独立、空算法目录和扩轴实现仍未执行；用户暂不实施运动能力限额的决定不变。原CAN根因及主动运动实机验收仍未闭环，不以本轮技能更新宣称修复。

## 2026-11-09（Asia/Shanghai）｜轨迹规划方案选型：A/B/C/D 结论与本机落地方式

- 需求与范围：用户给出四方案比较表（A 五次多项式＋统一时长；B 公共进度＋Ruckig 限 jerk；C TOPPRA 力矩约束时间规划；D MoveIt），随后追问 B/C 区别、是否开源与是否联网、C 相对 B 的劣势、点到点与中间点的区别、能否部署在 MCU。本轮只记录**选型结论与依据**，不写实现、不改源码、不编译、不烧录、不连接硬件。
- 结论表（本机：J1/J2 双 DM4340P、5ms 控制周期、MIT 位置/速度/PD＋前馈）：

| 方案 | 能解决什么 | 本机主要限制 | 结论 |
|---|---|---|---|
| A 五次多项式＋统一时长 | 位置/速度/加速度同源，静止到静止 | 不限速度/加速度峭度；两端 jerk 不连续 | 可作最小验证基线，不单独采用 |
| B 限 jerk 的点到点（Ruckig 类） | v/a/j 构造性受限、起停平滑、可同时到位 | 不感知力矩与轴间耦合；不约束几何路径 | **第一版采用**，但用固件内自写闭式 S 曲线 |
| C TOPPRA（固定路径＋力矩约束） | 给定路径上按力矩约束取时间最优 | 不处理 jerk；需辨识过的动力学模型；本机力矩不 binding | 暂不采用，前置条件未满足前不启用 |
| D MoveIt | IK、避障、自碰撞、复杂任务路径 | 2 DOF 下明显过度（IK 有闭式解），需 ROS/URDF | 5+ DOF 或末端路径需求时再评估 |

- 结论一（选 B，但落地形态不是 Ruckig）：固件内实现 2-DOF 同步、jerk 限幅的闭式 S 曲线（上升/匀速/下降七段，各轴同时结束），约 150–300 行 C；计算量几十 µs（5ms 周期 <1%），状态只需几十字节，无动态分配、无新依赖。**不把 Ruckig 引入固件**：其当前版本要求 C++20（README 注明 C++17/11/03 版需另询）、公开 API 用 `std::vector`（有堆分配）、输入校验文档为 throw；与本工程的 C11（`CMakeLists.txt:4`）、全静态分配（`config/FreeRTOSConfig.h:19-20` 的 `configSUPPORT_DYNAMIC_ALLOCATION 0`）、`--specs=nano.specs --specs=nosys.specs` 无堆（`CMakeLists.txt:175-180`）、`-Wall -Wextra -Werror`（`CMakeLists.txt:22-33`）、逐文件代码生成约束（`CMakeLists.txt:173-174` 对 `App/fault.c` 加 `-mgeneral-regs-only`）四项纪律冲突。
- 结论二（三处定性纠正）：
  - “公共进度＋Ruckig”是两个机制：Ruckig 是各轴独立 profile 加 `Synchronization::Time` 统一总时长，不共享进度变量；共享进度 s(t) 属路径参数化（C 一族）。本机只有关节空间点到点，不需要共享进度。
  - **C 一般不会比 B 更快**：B 解“仅端点约束”的状态到状态问题（Ruckig 声明 time-optimal），C 解“给定路径”的约束最优；C 多一条路径约束、可行集更小，同一对起止状态下 C 的最短时间 ≥ B。C 的“快”只相对“同路径朴素定时”。
  - **本机 C 的力矩约束不会 binding**：软件 FF 预算 0.2/0.7 N·m、电机 `tmax` 28 N·m（`config/motor_config.c:15,27,33,45`），而实际需求为 J2 重力峰值 0.4922 N·m（`g_drag_model.gravity_sin_nm`，`config/motor_config.c:72`）加 10 rad/s² 下惯量项约 0.071 N·m（`d≈0.007086`，`config/motor_config.c:71`）；耦合项 `b_cos≈3.2e-5`，`B/√(A·D)≈0.45–0.61%`（`config/motor_config.c:66-70`）。力矩不成为瓶颈时，C 退化为“没有 jerk 限制的 B”。
- 结论三（点到点／中间点／固定路径是“约束类型”不同，不是分辨率不同）：点到点只约束起止状态，中间 p/v/a 由求解器全程逐点给出（限 jerk 时 a(t) 分段线性、jerk 分段恒定，5ms 采样下全程每点都有值）；中间点是“额外强制经过给定位姿”，代价是可行集变小、折线拐角需过渡、可能不可行，Ruckig 社区版该功能走其云 API 且不可实时（本地实时为 Pro）；固定路径 p(s) 才是 C 的输入。细腻程度由采样周期（本机 5ms）与 jerk 上限决定，与上述三类无关。若要末端走直线，正确路径是“笛卡尔直线 → IK → q(s) → 对 s 做 S 曲线”，不是堆关节空间中间点。
- 结论四（部署位置）：**规划在 MCU，目标/限值由 PC 低频下发**。PC 侧 `tools/` 已是 Python，可用 `pip install ruckig` 或 `toppra` 离线算，但“离线算＋200Hz 流式下发”对 2-DOF 点到点不划算：需新增实时设定值流（230400 8N1 下 200Hz×约 15B≈3KB/s，带宽可行，但现有 151B/20ms 遥测已占 7.55KB/s，预算 23.04KB/s，见 `config/serial_config.h:4-7`），且必须新增“设定值超时即停”看门狗；链路断在半路是最大新增风险，现有 stop-and-wait RPC 不适合流式。不做云端/服务器运算；`ruckig.com` 的 S 曲线页是浏览器内演示，不是服务器 API。
- 结论五（分阶段计划）：
  - 阶段 0：定运动接口（目标参考圈角度、v/a/j 上限、START/ABORT）与 `MotorTask` 新状态，复用现有安全门：`policy_ready`、KEY、反馈新鲜度、`MotorTask_ClampTarget` 区间钳位（`Task/motor_task.c:46-56`）、`max_step_ms` 中止（`Task/motor_task.c:465`）、跟踪误差中止（`Task/motor_task.c:228-234`）。
  - 阶段 1：固件内 2-DOF 同步闭式 S 曲线＋名义重力前馈；实机验证 1°/5°/20° 点到点，跟踪误差 <1°、无过冲。
  - 阶段 2：实测摩擦/惯量/跟随带宽后再定 v/a/j 默认值；README 明确 CAD 与编码器正方向未实机核验、驱动惯量未知（`README.md:139-149`），在此之前不采用依赖模型的规划。
  - 阶段 3：确有末端直线/工具路径需求时，加“路径层＋对 s 做 S 曲线”（不需要动力学模型）；仅当节拍成为硬指标且模型辨识到位时再评估 TOPPRA（须另做 jerk 后处理，最优性随之丢失）。
  - 阶段 4：≥5 DOF 或复杂避障再评估 MoveIt（MoveIt 2 内部使用 Ruckig，位于 ROS 侧而非 MCU）。
- 结论六（与现有代码集成的四个必做项）：
  - `v_des` 必须发轨迹速度：拖动模式刻意发 `v_des=0` 让 Kd 当阻尼（`Mode/drag_mode.c:296-303`），轨迹模式若沿用 0，`kd*(v_des - v)` 会变成对抗运动的阻力（kd 1.2/1.8，`config/motor_config.c:24,42`）。
  - 必须带重力前馈：`hold_kp=5`（`config/motor_config.c:18`）下 J2 纯 PD 稳态误差 = 0.4922/5 ≈ 0.098 rad ≈ **5.64°**，已超过 `tracking_error_rad=5°`（`config/motor_config.c:19`），新模式会自行中止；应复用 `drag.gravity_nm`。
  - 需“从当前速度生成限 jerk 停止段”的能力：现有 STOP 是抓当前位姿加 `hold_blend` 渐变（`Mode/drag_mode.c:268-276`），对运动中手臂等于让 PD 刹车。
  - 量化边界：位置 LSB=2×12.5/65535≈0.381 mrad；`v_des` LSB=2×10/4095≈0.0049 rad/s，在 5ms 下对应 jerk≈0.98 rad/s³，故 jerk 限幅低于约 1 rad/s³ 没有意义（`Device/DM_Motor/dm_motor.c:39-48,187-192`）。
- 数据出处（本仓库）：`config/motor_config.c/.h`（量程、保持增益、拖动 Kd/速度阈值、A/B/D 与重力系数）、`config/motor_tuning_fields.def`（38 项调参）、`Task/motor_task.c/.h`（状态机、区间钳位、跟踪误差、步长中止）、`Mode/drag_mode.c`（拖动/保持/渐变与前馈）、`Device/DM_Motor/dm_motor.c`（16/12/12 位编码）、`config/serial_config.h`（230400/20ms/151B）、`CMakeLists.txt`、`config/FreeRTOSConfig.h`、`README.md:74-112`（公式与参考力臂约定）。
- 外部资料（访问时间 2026-11-09 16:36，Asia/Shanghai）：Ruckig README https://github.com/pantor/ruckig （MIT；C++20，C++17/11/03 版另询；state-to-state 保证时间最优；online `update` 与 offline `calculate`；社区版中间点走 cloud API 且不可实时，Pro 全本地；MoveIt 2 采用；适配 500µs/2000Hz 周期）；https://docs.ruckig.com/example_01.html （在线轨迹生成用法）；https://ruckig.com/s-curve-trajectory.html （浏览器内 S 曲线演示，非服务器）；toppra README https://github.com/hungpham2511/toppra （给定 p(s) 与约束求 s_dot(s) 时间最优；`pip install toppra`）；Ruckig 论文 arXiv:2105.04830（RSS 2021）。
- 未完成与风险：上述数字均来自源码参数与离线计算，**未实机验证**跟踪误差、摩擦与跟随带宽；`A/B/D`、驱动惯量与编码器极性均未辨识（`README.md:139-149`），故“本机 C 收益小”依赖未验证的模型量级；若将来节拍成为硬指标或需要固定几何路径，须重新评估 C。本轮未改源码、未编译、未连接硬件、未烧录。
- 时间：2026-11-09 16:35–16:46（Asia/Shanghai）。

## 2026-11-09（America/New_York）｜核对前序命令实际完成范围

- 需求与范围：用户询问是否执行了前面所有命令，还是只执行最后的技能更新。本轮仅核对完成状态，使用embedded-firmware-workflow的事实核查与记录规则；不把询问状态扩展为固件实施或烧录授权。
- 已核对与结论：上一轮完成的是技能及参考规范更新，不是前序固件改造。当前Mode目录仍为drag_mode.c/.h和joint_coordinates.h，未发现Model/、Algorithm/；Task仍直接持有DragMode状态并调用其生命周期，串口TELEMETRY_SIZE仍为144。公共模型剥离、空算法目录、两个Mode的集成边界整理及扩轴协议仍未实施；运动能力限额按用户要求暂不执行。
- 责任与状态：已先向用户明确说明漏接了前序实施工作，写入技能不能代替代码完成，也不能把连续追加要求视为作废。状态核查与本轮记录完成，前序代码工作仍待落实。
- 变更与验证边界：本轮仅追加本日志；验证为目录、源码关键声明和既有日志的定向检查，无控制源码或技能再次修改，不运行无关构建，不连接硬件、不烧录。原CAN根因和主动运动实机验收状态未改变。

## 2026-11-09（America/New_York）｜逐项落实公共模型、模式边界与扩轴遥测

- 需求与范围：继续落实前序全部实施要求，并阅读用户最新规划、调研适合MCU的C语言Ruckig；不因新调研遗忘未完成代码。使用embedded-firmware-workflow、ponytail。运动能力限额暂不执行，Algorithm目录保持空白；主动运动算法与接口等待后续规划，不烧录、不连接硬件。
- 状态：本轮获准实施的清单已完成，结果已逐项通过进度消息交付；模型、模式边界、协议/PC改造、技能规则和文档均已落实。已将几何/平移雅可比、M/c/g和坐标映射拆到Model/，物理参数唯一来源为config/robot_model.c；拖动的滤波、增益与补偿预算仍在Mode。保留旧参数ABI，不改变已记录的J2竖直朝上参考零点。运动能力限额按要求暂不执行，Algorithm/确认为空目录；主动Motion算法/运动契约也按要求不提前填充，不将“预留身份”记为第二套可运行控制。
- Mode边界：Task不再读取拖动私有保持点/滤波/阶段；模式边界提供公共阶段与跟踪语义。DRAG可运行；MOTION身份明确但尚不可运行，选择返回NOT_READY，不伪造主动运动实现。不改变KEY、失能、恢复出力或故障门限。
- 协议与PC：新增Task/motor_telemetry.c/.h纯编码器；v2为u16长度、64字节公共头、数量及64字节统一关节记录，携带逻辑ID/CAN身份、原反馈时间/计数、MIT五输入、参考角/误差。2/3/6轴总帧长200/264/456字节，TX容量256→512；230400/8N1下遥测最多占60%预算，2/3轴20ms、6轴至少33ms。RPC仍是原v1可靠会话/事务封装，不改重试、去重、时效或启停。PC兼容旧v1遥测，新增按身份的动态卡片/曲线/CSV；非既有双轴布局仅监控，未假装38项参数和动作接口也已扩轴。当前实际Task与物理模型仍2轴，编译期明确防止只修改数量宏。
- 保护职责：Task发送层不再逐轴钳位重写模式目标；将同样的拖动目标限幅移入DragMode生成环节，保持原量化余量。后续Motion应在规划时满足整条轨迹约束、正常停止与制动余量，不能靠发送端裁剪后仍声称轨迹同步。本轮没有增加限速跳闸、确认次数、门控、RTOS线程或恢复出力策略；原公共故障检查仍在。技能的规划优先/不过度保护条款沿用上一轮已验证修改，本轮没有重复修改技能。
- 完整验证：主机Debug、Release各52/52通过（原50项加独立模型与遥测记录测试）。纯模型61姿态网格检查M正定、M/c/g、双列雅可比有限差分及非法输入原子性；Mode独立生命周期、保持点、模式未就绪拒绝通过。C编码覆盖2/3/6轴长度/CRC/容量哨兵/重复身份/无效数值；Python逐分包、坏帧重同步、混合v1/v2、真实工作线程、动态卡片/曲线及CSV离线自检通过。另实际运行C编码器--vectors，将三帧交给Python解析器逐字段核对身份、时戳、反馈和八项目标/参考值，全部通过；交叉核对调用最初因测试字段名及PowerShell引号失败，修正调用后通过，未将其误报为固件缺陷。Python两文件py_compile通过。
- ARM及镜像：Debug/Release完整ylamp_firmware及bsp_link_check均编译链接成功；Debug bin200112字节、SHA256 `EA11AB0A45EBCE1D26718BAD97E1E5FAAC89FEE492C4CAFC89B0452C6DBF8384`，Release135472字节、`35ABBEFDA5B008D708EC109CCB837B35CF12911B381904A974D5C8080406557E`。应用DTCM42424字节、AXI DMA1024字节（512 RX+512 TX，原MPU覆盖内）。增加固定RAM不等于最坏栈/实时性已验收，未对本候选测板上水位、WCET或串口满负载。已部署固定归档drag_mode_single_20261109_firmware.bin仍66408758…；旧fault_after_latest归档仍330E9A2E…，哈希核对未变，不以新ELF解释旧现场。
- 已读最新规划：MCU规划、限jerk点到点优先，暂不引入TOPPRA/MoveIt。本轮只调研Ruckig，不安装或接入外部库，不提前填充Algorithm/。
- 文档与边界：README顶部列出每项实施/延期状态，补全v2公共头/关节记录/带宽和v1 RPC兼容说明，将之前“已部署”实现标明历史基线。CSV已附MIT五输入及状态，但仍按新反馈采样，不是5ms全周期记录，反馈冻结时不追加独立故障行，模型分项与完整事件流仍待后续设计，P15仅部分改善。模型仍为2自由度近似，协议扩容不等于控制模型支持更多轴；原CAN物理链故障与卡死根因未闭环，本次重构不声称修复。未连接串口/目标、未停核/复位、未写活动RAM、未烧录；当前板上仍为上一轮镜像。

### 本轮Ruckig纯C候选调研与规划记录纠正

- 取证：只读访问GitHub仓库、README和固定版本原始源码；git ls-remote取得DiamondY/ruckig_c当前HEAD `e251e6e9ad6718dd51051f5b32014a28dae91bea`、mika4128/scatti HEAD `88c22cde32d448c467e686f433a664014e8dd501`，关键内存/计时源码按这些提交核对，避免仅凭变化中的README判断。未克隆、安装、引入或运行第三方库，未测任何候选在H723上的耗时/内存，也没有将作者测试报告记成本项目通过。

| 候选 | 已查到的内容 | 对本机的取舍 |
| --- | --- | --- |
| DiamondY/ruckig_c | 纯C99，运行库不链接C++；README以官方Community 0.17.3作冻结oracle，提供p/v/a、v/a/j界、同步、calculate/update/采样及差分/分配审计测试。创建句柄与向量仍经calloc/free；声明计算/采样阶段不再分配。 | 优先作为离线对照候选，而非直接进固件。需裁到固定双轴PTP、改为调用者静态工作区/有界内存，再实测时间/空间和异常输入；不接入其追踪、中间点等当前不需要功能。 |
| mika4128/scatti | 纯C99的Ruckig移植，内存/数学可替换钩子；源码create经SCATTI_CALLOC，update无条件调用clock_gettime(CLOCK_MONOTONIC)，中间点路径存在resize。README还声称较广Pro特性及桌面性能。 | 备选，不把“可裸机”宣传当作已移植。需处理默认分配、POSIX计时和扩容路径，并核对来源/许可与算法范围；README示例CRuckig旧类型与所读源码SCatti命名也需核对。作者桌面耗时不可当本板WCET。 |

- 建议：先固定候选提交，在PC用官方固定版对照，覆盖短行程、零位移、正反向、不同轴位移、非零起始速度/加速度、可行/不可行状态和从运动状态制动；再做固定内存及ARM交叉编译/板上预算评估。未来Motion只消费同一轨迹的p/v/a，经公共Model生成前馈，不继承拖动的实测加速度回灌。此处只是后续接入路线，不填充Algorithm、不新增运动参数、不提前实施Motion。
- 纠正用户最新“轨迹规划方案选型”条目中的技术断言（保留原文作为历史，不用未经验证的数值指导执行）：
  - 官方是C++，但不能据此断言“必须std::vector堆分配、必须throw、所以不能MCU”。已核对官方v0.17.3的utils.hpp：固定DOF使用std::array；ruckig.hpp支持throw_error=false及validate_input<false>。仍须处理具体版本工具链、时钟和内存；本项目偏好纯C，不等于官方算法不可嵌入。
  - 速度量化步长约0.004884 rad/s除以5ms得到约0.977 rad/s²，是加速度差分量级，不是jerk rad/s³；不能据此宣称jerk小于1无意义。jerk要按加速度变化/时间及实际执行器量化共同评估。
  - tmax=28是MIT编码量程，不是已验证可用持续力矩；拖动FF预算也不等于总MIT力矩或主动能力。尚不能认定力矩约束必不生效；不同约束集合下不能泛称TOPPRA一定慢于Ruckig。用户暂不制定能力限额的决定保持。
  - 重力应复用公共Model计算，不读DragMode诊断或私有状态。静止到静止七段S曲线只是Ruckig问题的一个子集，不能不经验证就承诺150–300行覆盖任意初始p/v/a、同步与限jerk制动；几十微秒也尚未实测。
- 一手资料入口：https://github.com/DiamondY/ruckig_c （README、src/ruckig_c/alloc.c）、https://github.com/mika4128/scatti （README、src/scatti.c）、https://github.com/pantor/ruckig/tree/v0.17.3 （include/ruckig/utils.hpp、ruckig.hpp）。本轮仅给出有依据的候选介绍与适配风险，不宣称已找到经过本机验收的即插即用纯C版本。

## 2026-11-09（America/New_York）｜评估官方Ruckig C++与现有C固件混合编译

- 需求与范围：用户更倾向保留官方C++文件，与当前C文件混编并进行简单适配。使用embedded-firmware-workflow与ponytail核对可行性，已先通过进度消息交付结论；本轮仅方案评估和日志，不提前实现Motion、填充Algorithm或改变电机输出。
- 当前工程事实：CMake已启用C/CXX，默认分别为C11/C++17；arm-gcc.cmake已有arm-none-eabi-g++、Cortex-M7硬浮点ABI及-fno-exceptions/-fno-rtti。现有ARM审计构建识别的编译器为GNU 10.3.1，不能据“已有g++”直接宣称支持上游当前全部用法。SDK原生startup_stm32h723xx.s已调用__libc_init_array；仍建议显式初始化纯算法实例，不在构造期间访问硬件。FreeRTOS静态分配、禁用动态分配的策略不变，但该设置本身不禁止C++/libc堆分配，需另审计实际链接和调用路径。
- 上游源码取证：只读核对官方仓库及main的CMakeLists.txt、ruckig.hpp、utils.hpp、calculator.hpp和calculator_target.hpp。git ls-remote观测HEAD为99751e6ebc7c963bb80e99fbbe66f2c279d1db57，v0.17.3为cb99a04ce488f83701aaee6efd9c9f0d36a3d43b；未把main版本号当成已发布标签。当前main构建目标要求C++20、默认启用云客户端；固定DOF的StandardVector使用std::array，Ruckig默认throw_error=false，calculate按此模板校验并返回错误码。未据此宣称所有可选功能都无堆分配。
- 建议路线：优先官方固定版本/提交，加一个模块私有C++适配实现和仅暴露C类型的薄C接口；原C驱动、Task和DragMode不改写。只为Ruckig相关目标设置所需C++标准，保留现有异常/RTTI策略；静态库关闭云客户端、示例、Python和MCU目标内的上游测试，不调用中间点/动态DOF路径。保留官方求解公式、double精度与数值容差，不做逐文件C改写或经验性数学裁剪。未来Motion持有轨迹生命周期，Task仍只调度/提交；公共Model仍是前馈物理事实唯一来源。
- 单次运动的最小接法：目标变化时用官方calculate生成双轴轨迹，周期按同一时间用trajectory.at_time采样p/v/a，接MIT位置/速度及Model动态前馈。官方update还调用std::chrono::steady_clock计时；优先利用已有calculate/采样API避免这一平台依赖，而非先改核心源码。同步到达不自动代表关节公共进度或笛卡尔直线，时间最优也不代表力矩最优；后续路径和动力学约束仍须单独设计，不能用新增报错门槛替代规划。
- 验证与状态：已完成本地配置/启动源码及上游源码定向核对，Algorithm目录仍为空；只修改本日志。没有下载接入第三方库、编译Ruckig、全固件重链接、板上耗时/栈/内存测量或实物测试，不为文档分析重跑无关回归。后续先固定版本并做主机轨迹边界测试、ARM完整链接及堆/异常/计时符号审计，再测板上最坏耗时和栈；尚未验证的兼容性与实时预算不写成通过。未连接目标、烧录、复位、修改参数或新增保护，原CAN根因状态未改变。分析结论及本轮记录已完成。
- 一手资料入口：https://github.com/pantor/ruckig 、https://docs.ruckig.com/ ，具体核对文件见上；本条更新后续选型优先级，不删除前述纯C候选和被纠正的历史判断。

## 2026-11-09（America/New_York）｜拉取官方Ruckig并融入Algorithm层

- 需求与范围：用户授权拉取官方C++并有机接入算法层，遵守技能；随后要求继续。使用embedded-firmware-workflow与ponytail，不将算法接入扩成主动Mode业务或烧录授权。成果已先通过进度消息交付；本轮算法接入、过程记录、主机全量双配置回归、ARM完整双配置链接及离线资源审计已完成，不代表主动运动或板上验收已完成。
- 拉取与来源：实际git clone官方https://github.com/pantor/ruckig，稀疏检出include/src/cmake及根部文件，固定detached HEAD 99751e6ebc7c963bb80e99fbbe66f2c279d1db57。保留.git来源、MIT许可证、README及上游构建；构建本身不联网拉依赖。上游CMake标识0.19.6，不冒称发布标签。唯一源补丁为四个头文件brake/profile/calculator_target/ruckig删除未使用的iostream包含，共4行；无核心公式、浮点精度或容差修改。该补丁避免桌面流全局初始化，git diff可核对。
- 接口与组织：新增Algorithm/CMakeLists.txt和joint_trajectory.h/.cpp；复用官方静态目标，局部关闭云、示例、上游测试、Python、benchmark，不覆盖全局CACHE选项。算法C++20，原C11保持；禁用异常/RTTI/fast-math，生成分段和栈用量信息。ylamp_task构建依赖算法库，但Task/Mode控制源码不改；BSP全量链接检查显式whole-archive算法与官方核心，防止静态抽取规则掩盖链接问题。
- 算法契约：固定双轴PTP，一个任务/Mode独占的私有静态实例，placement new显式初始化，无全局对象构造或硬件/RTOS依赖；当前ARM工作区5848字节，另4字节指针。Init/Reset仅使轨迹失效，不操作电机；Calculate所有尝试均使旧计划失效，错误保留时长输出并拒绝后续采样。输入为连续参考圈的SI起终p/v/a及有限正v/a/j约束；采用Time同步、周期整数倍时长，calculate只在新规划时求解，at_time按统一显式时间采样p/v/a/jerk，终点不外推。保留原生错误码，不把正常规划约束当新增运行期跳闸；超限初态可走官方预制动。尚不处理力矩/碰撞/工作区间约束或笛卡尔路径，非零终v/a不冒称保持。
- 快速验证：主机Debug新增joint_trajectory测试通过（0.32秒），64组行程覆盖零位移、微小/短/长行程、正反及单轴不动；另测非零起终p/v/a、采样连续重规划、超速初态制动、非法/非有限输入、失败输出原子性及失效、重复初始化/Reset和不同周期。与直接官方API核对时长及采样结果，差分检查p/v/a导数关系，轨迹加速度接公共Model生成有限前馈。C++ new/delete覆盖与C malloc/calloc/realloc/free链接包装审计期间堆调用计数0。ARM GCC10.3.1算法及上游静态库编译通过；GCC关于std::pair ABI仅输出工具链说明，未改ABI或绕过错误。
- 资源审查与必要适配：初始Debug未优化构建的TargetCalculator单层3920字节、Block单层3632字节，调用链对现有8192字节任务栈存在预算风险；std::optional防御分支还将abort及newlib的信号/堆实现带进强制链接镜像（主机测试未观察到分配）。仅对官方库及薄适配器在Debug/Release统一设置-O2，保留Debug符号和-fno-fast-math，不修改公式、任务栈或其他模块优化级别。重建后两种ARM配置最大算法单层栈帧3808字节、Block 656字节、Sample 208字节，全部.su条目均为static；这些数值不是完整调用链上界，板上栈峰值仍待测。BSP检查通过目标文件路径强制链接官方库，避免其C++20要求传播到SDK头检查；SDK/原C++检查仍为C++17。
- 完整验证：最终主机Debug/Release分别53/53通过，包含新增轨迹C→C++→官方核心测试；两个配置的ARM完整ylamp_firmware与whole-archive bsp_link_check均成功。强制链接ELF包含四个JointTrajectory入口且无未解析符号；nm审计未见malloc/calloc/realloc/free（含newlib _r变体）、_sbrk、分配new/delete、__cxa_throw、abort、std::chrono或桌面流初始化符号。适配器无全局构造入口，私有storage 5848字节、context指针4字节；上游git diff --check通过，补丁仍只有4行删除。
- 候选与部署区别：当前MOTION仍返回NOT_READY，应用无算法调用，所以实际固件暂不抽取算法代码/工作区，不能以其尺寸声称算法已运行。Debug应用200100字节，SHA256 8F527CE98A1EA1A2AE7642A92CE5C45B02806F9E24130B15023F3C2800DEC73F；Release135472字节，SHA256 6A58549C4EE1F982D8865799351603C3A68121889A9AC569222F51599242FE70；两者DTCM42424字节、AXI DMA1024字节。包含全算法的强制检查镜像Debug FLASH247436/DTCM37712字节、Release FLASH199692/DTCM37720字节，二者DMA1024字节；这是链接检查镜像而非运行应用，不能直接据此推算主动运动整机资源。
- 涉及文件：上述Algorithm新增/拉取文件、根CMakeLists.txt、tests/headers.cpp、tests/test_joint_trajectory.c、tests/trajectory_alloc.cpp、README.md及本日志。README同步原空目录要求已被本次授权接入取代，MOTION仍不可运行；没有新增业务状态机、线程、运动限额默认值、保护门槛、KEY、重试/恢复或调参变化。
- 部署与验证边界：本轮未连接目标/串口、未停核/复位、未写活动RAM或烧录。已部署固定归档bin仍66408758…、ELF05CB23DA…，旧故障bin330E9A2E…均核对未变。板上最坏求解耗时、调用链栈水位、实时预算及实际主动运动仍未测；原CAN根因未关闭，不能用算法单测宣称设备验收通过。下一步由Motion消费统一时间的规划p/v/a及公共Model前馈，具体生命周期、运动能力和停止/恢复策略仍按用户后续规划，本轮不提前填充或增加保护。

## 2026-11-09（America/New_York）｜算法接入后的下一步建议

- 需求与状态：用户询问下一步做什么；使用embedded-firmware-workflow与ponytail核对当前接口并给出建议，本轮仅分析和记录，未实施新Mode或修改控制源码。建议已先通过进度消息交付，日志补齐后本轮分析完成。
- 核对事实：阅读最新过程记录、Algorithm/joint_trajectory.h、Mode/motor_mode.c/.h及Model/robot_model.h。当前轨迹接口支持统一时间的双轴p/v/a/jerk，公共Model接收显式运动量；MotorMode_Available仍只允许DRAG，MOTION尚未运行。算法编译/离线测试通过的历史结果不替代板上求解耗时、完整调用链栈峰值或执行器跟踪验证。
- 建议顺序：先约定Motion最小契约，再离线打通轨迹→公共Model→MIT五输入。建议首版从原位保持进入单次点到点、到点保持，不自动往返；正常中止建议限jerk减速再保持，具体KEY映射、完成条件与恢复策略待用户后续规划，未擅自实施。位置/速度来自同一次轨迹采样，规划加速度进入模型前馈；Motion独立设置Kp/Kd，不继承拖动的25%实测加速度回灌或变阻尼策略。
- 验证路线与边界：建议先无电机出力做板上规划/采样时序和栈测量；具备可用通信与停机手段后，再原位保持、J1保持而J2小位移、J2保持而J1小位移，最后双轴同步。同步记录目标/实际位置速度、误差、前馈分项与周期。用户暂缓整机能力辨识的决定保持；单次实物试验仍需明确该次v/a/j及增益，不能使用协议最大值冒充能力。本轮未选数值，未增加重复确认、瞬时速度跳闸或锁定门槛。
- 验证及涉及文件：只做上述现有接口与日志的定向核对，不为分析重跑构建或将上一轮53/53当成本轮新测试；仅追加本process.md。未连接目标、复位、调参、烧录或出力，原CAN根因状态不变。下一项具体工作建议为Motion契约与离线MIT输出验证，尚未获得本轮实施新控制行为的指令。

## 2026-11-09（America/New_York）｜位置阻抗控制接入独立MotionMode

- 需求与范围：用户要求“完成阻抗的位置控制”。使用embedded-firmware-workflow与ponytail，在已有官方Ruckig和公共Model上实现最小单次双轴关节空间位置阻抗，不新增笛卡尔规划、摩擦补偿、自动往返或全套能力辨识。成果已先通过进度消息交付；本轮源码、上位机入口、完整离线验证和文档收尾完成，未烧录，不代表实物运动已验收。
- 当前已实施：新增Mode/motion_mode.c/.h；进入时捕获当前参考编码器角，1秒渐入保持；接受显式每轴目标/v/a/jerk后，同一时间采样双轴p/v/a，MIT接p/v及独立Kp/Kd，公共Model接实际q/dq与规划ddq生成完整名义M*ddq+c+g，不重复外环PD、不继承拖动25%实测加速度回灌。两轴Kp=5 N·m/rad、Kd=0.5 N·m·s/rad为未实机整定初值，独立于原38项拖动RAM参数；原Drag参数不变，默认仍DRAG。
- 生命周期与规划：单次移动后终速/终加速度为0并保持，不自动重复。正常STOP使用当前参考p/v/a生成最短逐轴限jerk制动，共用采样时钟、先停轴保持；不是参考速度骤清零。新增算法Brake及PositionBounds入口，使用官方极值提前拒绝越界轨迹，拒绝的新任务保留原保持点而不触发失能。Motion不套用拖动保持的5°误差跳闸，误差用于观测；原Task越界、设备故障、温度、时序、失联和Bus-Off恢复规则保留。恢复后重新捕获当前姿态保持，不恢复旧轨迹；不添加重复确认或瞬时速度保护。
- 分层与坐标：公共Mode入口分派两种实现，Task只做原调度/门控/统一输出，并增加MotorTask_Move单次任务接口。连续参考编码器rad及同一参考圈逆映射沿用；J2竖直朝上零点-2.640383005142212 rad不变。不将同一时间计算冒称两帧CAN同时到达或力矩最优。
- 首轮验证：主机构建通过，joint_trajectory、drag_mode_standalone、motion_mode_standalone三项通过；Motion覆盖进入保持、独立增益、全量模型前馈核对、目标/v/a/jerk连续、单次到点保持、中途制动和非法输入原子性。首次构建暴露独立标定测试缺Algorithm头路径、独立Motion测试缺假hfdcan1定义，已补齐后重建通过；不是硬件故障。
- 操作入口与接口收尾：USART1原可靠RPC增加可选op12能力0x594C5001，不改变v1基础版本、38项调参或v2遥测布局。支持只读Mode、失能态选Mode、BEGIN/8字段/COMMIT/ABORT、1秒冻结模型分项快照。目标/速度/加速度/jerk为deg制f32，Task接口为rad；目标是连续参考编码器角，非建模角。新任务仅在Motion已保持时接受；整包完整性、配置版本、3秒草稿过期和Mode代际检查阻止旧草稿在停止/故障/重入后生效。COMMIT不隐式使能、不自动重试或循环；UART原封套去重覆盖相同重传。原KEY启动也进入Motion原位保持；正常STOP不重放旧任务，F6仍释放支撑。
- 上位机：tools/dm_tuning.py新增“位置阻抗”页，空白目标/v/a/j显式填写，选择Drag/Motion、发送单次任务，显示目标/实际/误差、阶段/时长和g/c/M·ddq。capability为可选，旧固件缺Motion不会破坏原监控/拖动；无新启用来源或逐次确认。tools/dm_monitor.py转交扩展事件，已有v2/CSV继续保存通用MIT五输入与参考误差。模型分项只是RPC冻结快照，不冒称新增同步5ms全周期CSV或事件流。
- 完整离线验证：最终主机Debug/Release分别56/56通过；新增motion_mode_standalone、motor_task_motion、motor_task_recovery-motion，原拖动及故障/恢复回归保持。Motion测试包含上下界附近82个制动切入点、p/v/a连续和v/a/j约束、模型前馈逐项核对、原位渐入/单次到点/不重复、KEY启用、5°以上柔顺偏差不套Drag跳闸、参考圈逆映射、错误目标保留保持、部分/重复/超时/STOP/代际变更草稿拒绝，以及Bus-Off恢复后新鲜位置保持而不重放原计划。算法堆审计扩展到Brake/PositionBounds及16种正反速度/加速度制动，计数仍0。最初阶段55项全量回归已通过，新增恢复专项后最终为56项，不将旧计数冒作最终结果。
- Python验证：最终python -X utf8 tools/dm_monitor.py --self-test、tools/dm_serial.py和两工具py_compile均通过。新增真实Panel离线检查空值拒发、显式双轴八值仅排一个motion作业、选Mode不顺带启用、目标/实际/误差文本和过期禁用；Client检查动作排队过期不执行、完整分段提交、不重复运动以及旧固件缺capability仍可在线。原2/3/6轴遥测/卡片/CSV、会话与可靠重传测试仍通过；这些不访问真实串口/USB。
- ARM与资源：Debug/Release的ylamp_firmware和whole-archive bsp_link_check均编译链接成功；四ELF未解析符号为0，未见malloc/calloc/realloc/free/_sbrk、分配new/delete、__cxa_throw、abort、chrono、桌面流或全局C++构造入口。官方上游补丁仍只有之前四个iostream删除，git diff --check通过。本轮Motion实际引用算法，应用已抽取求解器和5848字节工作区，不再沿用上一阶段“应用未抽取算法”的资源结论。Debug bin308656字节、SHA256 5DCBC388C82C50E27612C528AF6C7D95EFE948BF1EEF8B8316125EB2E1501AB6；Release242640字节、2128B2CC9F48D4B9FD68928EF57E3EB510385925AF1DC76E9B6921AEAD2C4FC9；两者DTCM48824字节、AXI DMA1024字节。原部署归档bin哈希仍66408758101B938B24FDACFC9474755AEC82FAB803C3D2154A584441C01BE190，未覆盖。
- 栈与实时性边界：给本次规划调用链相关Task/Mode/App源增加-fstack-usage，仅生成构建证据，不改任务栈或调度。ARM求解器最大单层仍3808字节、Block656；Motion Move单层Debug192/Release200，周期Update344/312；RPC入口608/568（Release另有12字节外壳），Debug motion_rpc另360字节。已核对UART→RPC→Task→Mode→算法及KEY停止路径，不能将某一帧大小当完整栈上界；8192字节控制任务栈未做板上水位/IRQ叠加验收，求解也仍在原单一控制任务中，未测WCET/最坏抖动。没有据离线通过擅自改栈、加线程或放宽时序门槛。
- 涉及文件及未完成验证：Mode/motion_mode.c/.h、公共motor_mode、Task/motor_task及tuning扩展、Algorithm薄接口、config/motor_config、根CMake、相关C/Python测试、工具与README/process。README顶部为现行候选，旧NOT_READY/讨论稿已标历史；原拖动参数和物理标定值未变。未连接目标、写活动RAM、停核、复位或烧录，板上求解时间/栈峰值、真实跟踪、增益/极性和机械能力仍待验。名义前馈检查只防编码不可表示，不是已辨识力矩能力或PD总输出限制；到点保持不等于实测收敛。原CAN物理链问题未闭环，不因本轮离线通过宣称故障已修复。下一步先无出力测规划/采样预算，再托住负载验原位保持与单轴小行程，最后双轴。

## 2026-11-09（America/New_York）｜授权烧录位置阻抗版

- 需求与范围：用户明确“烧录”，部署前述MotionMode位置阻抗Debug版；不修改源码、控制参数、零点、默认Mode或启停/恢复策略，不主动使能或发送运动目标。使用embedded-firmware-workflow及aixprobe核对流程；专用aixprobe MCP未提供，沿用本机ST-Link/OpenOCD原生工具。成果及首次空闲检查已通过进度消息交付；烧录、回读、普通重连和记录收尾完成，只读初始化暴露的问题见下，不冒称上位机联机全通过。
- 构建与身份：重新构建build/arm-audit的ylamp_firmware，完整链接成功；有效bin308656字节，SHA256 `5DCBC388C82C50E27612C528AF6C7D95EFE948BF1EEF8B8316125EB2E1501AB6`，与上一轮验证候选相同。匹配ELF SHA256 `8931FC5A4136C28F630A6C5BEC1A9A1FACF0ED5585862D6FF4F2022E8573CD3B`；两者复制为固定motion_impedance_20261109_firmware.*归档，旧归档保留。无控制源码变化，不重复此前56/56全量主机回归。
- 预检与边界：无现存调试/上位机进程占用；COM34/230400只监听3秒收到22651字节、150帧、解析错误0、PC发送0字节。旧固件为遥测v1、Task FAULT/TRANSPORT；两轴反馈计数均215但年龄约14608秒，Kp/Kd/FF为0；首错/Bus-Off在tick3413，LEC5、TEC16/248、REC0、异常观测321。已告知用户过期state0不代表当前实际失能，并提示托稳负载、暂不按KEY，按明确烧录授权继续。
- 目标与备份：OpenOCD普通SWD连接，ST-Link V2J43S7，电压3.296970V，实际SWD950kHz，CPUID0x411FC272、DBGMCU_IDCODE0x10016483；Flash为STM32H72x/73x单Bank1MiB，与当前H723构建一致。停核后完整Flash备份1048576字节为build/arm-audit/motion_impedance_20261109_before_flash.bin，SHA256 `9BE05CE1DCDD3CD83C9AA54358780C49A3BF319EDC27A867F6E853ADCD1B2688`；同时保存131072字节before_dtcm.bin，未清除历史证据。
- 写入与校验：flash write_image erase匹配归档ELF，对齐写入308672字节；verify_image验证308656字节成功，独立dump_image完整应用回读与bin SHA256一致；校验后执行一次reset run并正常shutdown，OpenOCD退出0。日志motion_impedance_20261109_flash.log，回读motion_impedance_20261109_readback.bin；均位于build/arm-audit。
- 首次启动验证：COM34/230400只监听4秒收到40001字节、200帧、解析错误0、发送0字节；遥测已为v2，tick48893→52873持续前进，Task IDLE/NONE、Mode DRAG/OFF。两轴计数均5161、反馈年龄10ms、实际state0失能，目标Kp/Kd/FF为0；CAN异常观测0，UART错误/溢出/CRC/TX丢弃均0。只证明此观察窗口空闲通信正常，不据此关闭原CAN故障根因。
- 普通重连：烧录会话退出后重新SWD普通连接成功，CPUID及DBGMCU_IDCODE相同；本次重连未halt/reset、未写寄存器，正常shutdown退出0，日志motion_impedance_20261109_reconnect.log。最终无OpenOCD进程残留。
- 只读接口检查与限制：运行现有SerialReader能力发现，完成基础版本、38项配置及Motion能力识别（motion_supported=true）；随后初始化查询因“MCU时间戳不新鲜，未发送控制请求”退出，不能称上位机正常完整初始化通过。统计50次只读/会话发送、解析错误0；未提交mode/command/motion任务。源码中连续RPC优先应答会推迟遥测，可能与本次初始化时戳过期有关，尚未验证根因或修改。随后拟以25ms间隔做只读复核，但COM34打开被PermissionError拒绝，发送0字节；没有抢占端口、关闭用户窗口或再次刷写。
- 后验状态：在上述两次接口尝试之间，独立只监听2秒收到20001字节、99帧、解析错误0、发送0字节，tick191393→193353连续前进；此时为DRAG/RUNNING、两轴实际使能，反馈年龄5ms、计数38305。参考角J1约-67.1554°、J2约-132.8792°；CAN异常观测0、UART错误/溢出/CRC为0，TX丢弃累计13（RPC与遥测竞争时的计数，不冒称物理溢出）。本轮没有发送使能命令，不能将后续使能归因为自动启动，也不替用户停止当前运行。
- 用户追加目标设置问题：已核对位置阻抗页实际控件与坐标契约，说明在失能态选Motion、KEY进入原位保持后发送双轴绝对参考角；不将参考角与建模角/相对位移混淆，J2朝上参考仍约-151.2828°。位置保持、轨迹与实际收敛的边界不变。
- 涉及文件及限制：只更新README当前部署身份与本日志，生成build/arm-audit内归档/备份/回读/日志；没有固件源码修改或新诊断代码。未由本轮操作切换Motion、按KEY或进行保持/移动/制动/外力试验，未测板上求解耗时/栈水位或独立供电重启；实物增益、极性、跟踪、初始化时戳过期及原CAN根因仍待验。

## 2026-11-09（America/New_York）｜简化为一键原位位置阻抗

- 需求与用户决定：上位机点击阻抗按钮即自动捕获当前位置、进入位置阻抗；被手推开后仍回到这一次记录的原位，不能持续更新中心。用户明确速度/加速度/jerk上限暂不配置、后续考虑。本轮因此只接通固定中心阻抗，不填写默认轨迹能力、不新增移动任务；固定目标速度/加速度为0，回位来自MIT PD，不冒称Ruckig限速回位。
- 状态：成果及必要快速检查已先通过进度消息交付，随后完整验证、文档和日志收尾完成。用户后续明确“允许烧录”，已完成新镜像部署、回读、普通重连及空闲只监听验证；没有主动使能/移动或替用户点击阻抗按钮。
- 实际修改：Mode增加显式CaptureHold，单次捕获新鲜连续参考角并取消旧轨迹；保持中心不随外力反馈变化。Motion从零渐入新中心的Kp，活动交接保留旧Kd/FF后在原1秒内过渡到现有Motion增益和完整名义前馈。Task复用唯一输出与双轴使能握手；空闲显式启用、活动Mode不先失能/再使能。捕获发生在请求执行时，等待使能应答不重新捕获且不提前爬升增益。原KEY/F5、F6、通信恢复及真实故障处理保留；显式按钮可在新鲜失能反馈后解除人工失能锁定，不自动解除其他故障。
- 协议与上位机：可选Motion能力升为0x594C5002，op12/global/id8携带配置版本一次性捕获/启用；取消运动草稿，配置草稿或过期版本拒绝。原8字段显式移动API保留但页面移除八个输入与发送目标按钮，主入口变为“阻抗：锁定当前位置”，不要求原启动复选框或额外KEY。旧0x594C5001仍支持只读Motion状态但禁用新按钮；动作不自动新序号重试，原可靠封套去重不变。未改变遥测布局或38项Drag配置。
- 涉及文件：Mode/motion_mode.c/.h、Mode/motor_mode.c/.h、Task/motor_task.c/.h、Task/motor_tuning.c/.h、tools/dm_tuning.py、tools/dm_monitor.py、tests/test_motor_task.inc、根CMakeLists.txt、README.md及本日志。没有改算法核心、物理参数、零点、Drag手感、运动上限或瞬时速度保护。
- 快速验证：主机Debug构建通过，motion_mode_standalone与新增motor_task_impedance两项通过；Python两工具py_compile及dm_tuning.py离线自检通过。新增任务测试覆盖无KEY/无八字段启动、应答前不渐入、捕获点不随应答姿态/正反推开改变、回拉项符号、再次点击显式重记、拖动直接交接无使能空档、真实陈旧反馈故障不自动清除、F6及显式再启用。首次专项测试因复用“只启动过一次”的测试辅助函数断言失败，改为该用例自行发第二次启动后通过；不是控制实现或硬件故障。
- 完整离线验证：主机Debug/Release各57/57通过；两种ARM配置的ylamp_firmware与whole-archive bsp_link_check完整链接通过，Debug应用未解析符号为空。python -X utf8 tools/dm_monitor.py --self-test、tools/dm_serial.py及两工具py_compile均通过，包含新按钮不依赖启动复选框、只排一个捕获动作、旧固件禁用、过期请求不执行，以及原遥测/RPC/Drag/故障/恢复回归。两种应用DTCM48840字节、AXI DMA1024字节；CaptureHold单层Task栈Debug72/Release64字节、Mode176/168字节，只是静态单层结果，未做板上总栈/实时测量。
- 授权烧录：使用aixprobe流程；专用MCP不可用，沿用本机OpenOCD 0.12.0+dev/ST-Link。预检COM34被现有pythonw上位机占用、打开拒绝，发送0字节；未关闭用户窗口或抢占串口。检查目标CPUID0x411FC272、DBGMCU_IDCODE0x10016483，匹配STM32H72x/73x的1MiB单Bank。归档为build/arm-audit/impedance_hold_20261109_firmware.elf/.bin；bin309904字节、SHA256 D87DF8FA248E22A5F244F6B59D14C3C71A04103F12E8A79125CAEDD13B10DE23，ELF SHA256 92D47DDB383C1F562EF7678AA83D3C1E27188C1CC88ED37E9B2F935805438E3B，原归档未覆盖。
- 备份与校验：停核后保存完整Flash1048576字节为同前缀before_flash.bin（SHA256 05B1A6F7842DEC2F8A13F48E238860DA5E1CE1216E82484FE76223E156885D0F）及DTCM131072字节before_dtcm.bin。写入对齐309920字节、verify_image309904字节成功，独立readback.bin全长SHA256与候选一致；reset run一次后正常shutdown，退出0。再次普通SWD连接、不halt/reset读回相同CPUID/芯片ID并正常退出；flash.log及reconnect.log保存在同目录，无OpenOCD进程残留。
- 烧录后只读观察：COM34随后可打开，230400波特率只监听3秒收到30208字节、151帧、解析错误0，PC发送0字节。最新遥测v2、Task IDLE/NONE、DRAG/OFF，tick182353，双轴反馈计数18109、年龄10ms、实际state0，Kp/Kd/FF均0；CAN异常观测、UART错误/溢出/CRC/TX丢弃均0。只证明此窗口空闲通信正常；未运行完整上位机发现或实物阻抗回位，原初始化时戳问题和CAN根因不因此关闭。已告知须重开上位机加载新按钮。
- 未覆盖与剩余：上述只验证命令和软件状态，不证明电机实际零误差回位、无振荡或已完成增益整定；当前Kp=5、Kd=0.5沿用上一版未实机整定初值。旧版上位机初始化连续RPC与遥测竞争问题和CAN物理链根因未在本轮修复；没有修改调度、通信恢复或增加保护门槛。Ruckig后续移动能力/上限仍按用户决定暂缓。

## 2026-11-09（America/New_York）｜原位阻抗回正精度反馈与只读核对

- 需求与范围：用户反馈“感觉回正精度不够”。本轮按embedded-firmware-workflow与ponytail做定向诊断，不将反馈推定为调参、增加积分/摩擦补偿或重新烧录授权。初步结论已先交付，本轮只读核对与记录完成，实际误差主因尚未确认。
- 核对事实：读取config/motor_config.c、Mode/motion_mode.c及Device/DM_Motor/dm_motor.c相关路径。当前两轴Motion请求Kp=5 N·m/rad、Kd=0.5 N·m·s/rad，保持时固定目标p、期望v/a=0；没有积分或摩擦补偿，外力不持续更新目标中心。静止时阻尼项趋零，未补偿重力/其他持续扰动及静摩擦可能形成残余位置误差，不能仅凭“会回拉”保证零误差。
- 数值核对：由现有请求值直接计算，1°位置误差对应弹簧项0.0872665 N·m；举例未补偿力矩0.1 N·m对应约1.1459°平衡误差，只是量级示例，不是实测摩擦或重力残差。按当前PMAX±12.5 rad和16位位置编码计算协议一LSB约0.0218569°，不等于真实编码器绝对精度；没有据此断言本次精度问题一定不是量化。
- 实际尝试与验证边界：尝试COM34/230400纯监听，打开被PermissionError拒绝，PC发送0字节；未抢占串口、关闭上位机、停核、复位、修改参数或使能。没有本次静止误差、正反方向残差或时间序列，不能区分尚在缓慢回位与已停止的静差，也不能确定主要发生于J1还是J2。代码/数值核对不替代实机验证；没有源码变化，不重跑无关主机/ARM全量构建。
- 后续所需：用户提供松手停稳后的目标/实际/误差截图或各轴数值，再区分摩擦滞回、重力方向性残差与收敛过程，分别整定；增大Kp缩小静差会同时增加手推刚度，不承诺只调Kd即可消除静差。原固定中心、已部署镜像、暂缓运动上限/摩擦补偿的决定均保持不变。本轮仅追加本日志。

## 2026-11-09（America/New_York）｜回传力矩参与柔顺推移与增强回位的状态策略评估

- 需求与范围：用户希望提高回正精度，但不希望手推更硬，提出使用达妙回传力矩进行状态机处理。本轮使用embedded-firmware-workflow与ponytail定向核对现有实现并提出候选行为；已先交付分析结论，仅追加本日志，没有修改控制源码、参数、协议或已部署镜像，也没有烧录或改变电机状态。
- 当前链路：Device/DM_Motor/dm_motor.c按12位有符号映射解出torque_nm，接口注明其为驱动估算而非外置传感器实测；Task/motor_task.c复制完整反馈后仅调整位置参考圈，因此该字段已经可供Mode使用。Mode/motion_mode.c目前只用位置/速度计算固定中心PD及名义M*ddq_des+c+g，未使用回传力矩判定交互。config/motor_config.c的Motion两轴Kp=5、Kd=0.5保持未变。
- 判据边界：不能把回传力矩绝对值或“回传减命令”直接当作手推力矩；自己的回位驱动力与重力支撑同样进入反馈。拟用实际q/dq及回传力矩结合公共M/c/g估计外部扰动，优先评估广义动量残差，避免直接差分速度取得高噪声加速度。机器人动力学约定M*ddq+c+g=tau_motor+tau_external时，未建模摩擦也会混入残差，不能把残差直接命名为准确人手力矩，更不能只凭低速/静止认定已经松手。
- 资料核对：实际读取De Luca等《Robot Collisions: A Survey on Detection, Isolation, and Identification》（IEEE TRO 2017，作者主页PDF）的动量观测器、残差、滤波和模型不确定性部分，支持使用驱动力矩及关节运动估计外部扰动，也明确摩擦等不确定性影响判断；没有取得本轮达妙手册正文，故厂商接口语义仅按当前源码和既有已核实记录陈述，不伪称本轮在线手册复核通过。
- 候选策略（尚未获具体行为确认、未实现）：仅在Motion固定中心阻抗内设柔顺推移、松手判定、增强回位/到位保持。推移Kp先保留5，松手后回位Kp候选10，Kd先保持0.5，增益平滑变化；原始锁定中心不重记，M/c/g前馈不因软硬切换而关闭，不增加摩擦补偿或积分。推力判据应有迟滞及持续时间，并由无手接触与推/放手样本确定阈值，不能凭协议量程或空闲样本猜测。回位中重新检测到外部接触时退回柔顺；该判据不是故障跳闸，不改变公共失联、器件故障或KEY/F6及恢复策略。
- 风险与未完成：提高刚度会提高偏离中心时的恢复转矩；平滑切换减少突变但不等于已证明稳定或被动。高刚度保持下首次接触仍有检测延迟，不能保证手感完全不变；人手静止顶住与摩擦造成的静止残差在现有观测下可能难以区分，不能保证准确识别每次松手或消除静差。候选仅需Mode内少量静态估计/计时状态，不拟增加Task线程或在Task堆叠私有分支；具体资源与实时预算须实现后实测。
- 验证与下一步：完成当前Device→Task→Mode及公共模型/配置的只读核对；未采集本轮实机力矩序列、未调用串口/调试器、未执行增益切换或人机交互试验，不为方案评估重跑无关构建。下一步确认上述切换行为，先记录自由回位、静止无手接触、推移及顶住样本以评估残差可分性，再整定阈值并实现/测试，覆盖自身回位不误判、噪声不反复切换、固定中心不漂移及重新接触可柔顺。原CAN根因与模型未辨识问题未因此关闭。

### 用户追加｜联网查找相似开源方案

- 已完成定向联网检索并读取下列原始README、部分源文件及作者论文；已先通过进度消息交付结果。本轮为只读调研，没有克隆/接入依赖、修改控制或运行第三方测试，不把项目作者报告当成本机验收结果。

| 开源项目 | 实际核对内容 | 当前工程的取舍 |
| --- | --- | --- |
| mikhel1984/ext_observer | README与项目主页确认MIT许可、C++实现广义动量/扰动/滑模/Kalman等观测器，并关联Mamedov与Mikhel的2020年《Practical Aspects of Model-Based Collision Detection》。完整读取关联论文的模型辨识、扰动估计及检测部分；本轮未取得其lib目录源码，不将README核对冒充逐行源码审查。 | 作为观测器算法及理论对照，优先最简单动量残差，不搬入完整观测器集合、MATLAB或ROS包装。论文强调CAD不含摩擦和驱动惯量，不能仅换更复杂观测器就保证接触灵敏度。 |
| andrealo20/residual-collision-detection（qcollide） | 实际读取README、MIT LICENSE、CMakeLists.txt、include/qcollide/observer.h、detector.h及src/observer.c、detector.c。C99，固定容量状态；动量积分使用C转置乘速度及残差反馈，检测使用触发/释放双阈值和连续样本计数。README明确全部为均匀杆平面机械臂仿真，不含真机验证。 | 适合作为轻量C参考，不直接接入其平面动力学；继续用本机J1竖直/J2俯仰公共模型。其释放判定为低于释放阈值立即解除，并非完整松手持续判定或回原位策略；不能声称已有完全同需求状态机。未审计完所有非法数值、时序及模型边界，不能按“C99”直接认定可烧录。 |
| tud-amr/mobile-manipulator-compliance | 读取原始README和作者论文arXiv:2403.13079摘要。基于电流而非独立力矩传感器实现Kinova Gen3 Lite阻抗，论文报告实机引导/跟踪试验；README包含电流/力矩比例、延迟及摩擦标定说明。 | 输入条件最接近当前电流估算力矩，借鉴反馈标度/延迟标定和柔顺跟踪思路；不采用其ROS/Docker/移动底盘架构，也不据此恢复用户未批准的摩擦补偿。论文精度不代表本机可达精度。 |
| matthias-mayr/Cartesian-Impedance-Controller | 读取项目README及src/cartesian_impedance_controller.cpp；BSD-3-Clause，C++阻抗基库支持运行时刚度/阻尼及参数滤波。项目说明在Franka/KUKA上使用，且列出不补偿关节摩擦等限制。 | 借鉴刚度/阻尼平滑更新与模型/控制边界，不整套引入笛卡尔控制、Eigen/ROS，也不把其已有接触硬件条件套给达妙。它本身不是电流推力识别或自动松手回位状态机。 |

- 综合建议仍为“公共模型+实测位置/速度/驱动力矩→轻量动量残差→迟滞/持续时间交互判定→Motion内柔顺/增强回位状态→原MIT唯一输出”。控制前馈继续用期望加速度，观测器用实际运动，二者目的分离；不把观测残差再当助力前馈叠加，不影响Drag或增加触发失能门槛。本轮没有找到已经针对DM4340P并在MCU上验证完整“手推柔软、自动判松手、回固定中心”的即插即用工程。
- 资料入口：https://github.com/mikhel1984/ext_observer ，https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2020.571574/full ，https://github.com/andrealo20/residual-collision-detection ，https://github.com/tud-amr/mobile-manipulator-compliance ，https://arxiv.org/abs/2403.13079 ，https://github.com/matthias-mayr/Cartesian-Impedance-Controller 。本轮未固定上游提交，后续如实际引入应先锁定和审计版本。尝试GitHub树API读取时返回403，后改用公开原始文件核对；部分未取得目录/头文件已如实排除，不据失败响应声称检查通过。
- 收尾与验证边界：开源选型结论及本日志已完成。未改固件/参数，不运行无关全量构建；没有本机残差样本、检测延迟/误判率或增益切换实测，不承诺无感切换、完全准确松手识别、零误差或稳定性证明。候选Kp=5/10、Kd=0.5仍只是先前提出的待整定方案，未写入配置或烧录。

## 2026-11-09（America/New_York）｜复核原位阻抗结构与下一步验证顺序

- 需求与范围：用户询问下一步并怀疑当前控制结构不对。本轮使用embedded-firmware-workflow与ponytail做局部结构分析；不将讨论推定为重构、调参、增加状态机或烧录授权。已先交付结论，分析与日志收尾完成，仅更新本日志。
- 当前源码事实：重读README现行部署说明、Mode/motion_mode.c、Mode/motor_mode.c、config/motor_config.c及robot_model.c、Model/robot_model.c、Task/motor_task.c提交路径和Device/DM_Motor/dm_motor.c编码。原位模式保持首次捕获的连续参考位置，期望速度/加速度为0；MCU只提交p/v/Kp/Kd及模型前馈，PD由电机MIT执行一次，未发现MCU再叠同一PD的双重位置环。名义前馈使用实际q/dq及期望ddq；原位时惯性前馈为0，不等于真实惯性消失。当前Kp=5、Kd=0.5，固定目标回位不调用Ruckig；后者是用户暂缓运动能力上限后的既有行为，不据此推断静差根因。
- 结论边界：该实现属于基础关节弹簧阻尼阻抗，不能仅凭回正精度反馈判定基础方程或Task/Mode结构错误。它让同一组固定Kp/Kd同时决定手推刚度与回位响应，没有独立的无人接触精修策略；这与“手推不硬、松手高精度”需求尚未完全匹配，但缺少实际时序，仍不能区分缓慢收敛、静差、方向滞回、模型偏差或执行器问题。CAD参数明确为名义初值、驱动等效惯量未知，不能认定已经辨识或适合作精确接触检测。
- 在线核对：实际打开Franka官方libfranka 0.15.0的cartesian_impedance_control.cpp示例及官方overview；示例由位姿/速度计算弹簧阻尼并经雅可比转置提交，支持“基础阻抗外环不必以外力测量为输入”的判断，不把其驱动自带补偿/硬件条件等同于达妙。读取TU Delft项目原始README，确认其另做延迟、电流—力矩及摩擦标定；该事实不是用户已同意启用摩擦补偿。本轮Frontiers论文页面打开失败，不据此虚称新增论文正文核对。
- 修正建议顺序：先前“直接增加软硬切换”的建议作为后续候选保留，不应先当成已确认修复。下一步建议先冻结现有控制，用已有遥测/CSV记录J1、J2分别小幅正反推移和明确松手后的回位，提取中心是否固定、回位时间、停稳误差及方向重复性；再判断优先修正模型、整定响应，还是确有必要引入交互/精修策略。静止不等于松手，接触阶段需要人为标记，不能从低速度自动生成真实松手标签；观察到方向差异只作摩擦/滞回等线索，不单凭一次试验确诊。
- 数据与力矩验证：确认tools/dm_monitor.py的现有CSV已含MCU/反馈时间、目标/实际参考角、速度、误差、Kp/Kd、FF及估算力矩，可先复用，不新增大框架或协议。可以结合协议实际量化和反馈延迟比较MIT预期合力矩与电流估算反馈，不能将此一致性视为真实输出轴力矩已标定，也不把tau_fb减命令直接当人手外力。现有CSV不是5ms完整命令/反馈同步记录，不能据它直接识别全部高频执行动态；如证据显示时间分辨率不足，再局部补记录。
- 后续架构边界：保持Drag/Motion两个Mode及公共物理模型，不在Task堆叠交互私有阶段，不先换成另一种控制架构。若后续需要动量残差，先离线或只观测验证，不直接回灌FF；交互判定通过后再评估Motion内柔顺/回位策略。未批准的积分、摩擦补偿、运动能力默认值、增益切换或新增故障门槛均不实施。
- 验证与剩余：本轮完成上述源码、记录字段及在线一手示例核对；没有采集新的实机序列、操作串口/调试器、修改控制或运行带输出试验，也未重跑与分析无关的主机/ARM构建。未量化本机回正精度、真实力矩标度、摩擦/模型误差或接触可分性，原CAN根因未关闭；下一步是取得并分析基线回位记录，而不是宣称结构已修复或要求新增保护验收后才能进行现有范围调试。

## 2026-11-09（America/New_York）｜核对串口回传是否支持回位静差调试

- 需求与范围：用户询问当前串口是否支持前述回位/静差调试。按embedded-firmware-workflow与ponytail只读检查现有配置、遥测生产/解析及CSV导出，没有修改固件、上位机、协议、增益或部署镜像；本轮仅追加过程日志。
- 状态：结论已先通过进度消息交付，源码核对及日志写入回读已完成。现有链路足够做慢速正反向回位、停稳误差和重复性比较，不需要先改固件或提高波特率；不能据此直接完成精确摩擦辨识或可靠自动松手检测。
- 实际核对：config/serial_config.h配置230400波特率、20ms名义遥测周期。Task/motor_telemetry.h与motor_serial.c使用v2公共头及固定关节记录，当前双轴200字节/包；快照包含目标/实际连续参考角、误差、速度、电流估算力矩、Kp/Kd、合计前馈、模式/状态及MCU/反馈时间。目标为最近提交值，不是驱动已执行应答，也不是全部5ms控制周期的同步实测。
- 上位机与CSV：tools/dm_serial.py解析上述记录；tools/dm_monitor.py现有CSV_FIELDS已经保存这些核心字段。每轴仅在有效且CAN反馈计数变化时新增样本；记录有容量上限，导出当前保留窗口。RPC占用、UART忙等可能跳过遥测，序号与时间可辅助检查间断；不将名义50Hz宣称为无丢失实时日志，也不将异时到达的两轴反馈视为物理同时采样。
- 缺项与判断边界：现有CSV无人工松手事件标记，需测试者另记松手时刻；重力、耦合及惯性分项只可通过现有RPC状态快照读取，没有逐帧写入CSV。驱动回传力矩不是独立外力传感器，方向滞回只能作为摩擦/模型偏差线索。上述限制不阻碍先用同一个固定中心分别正向、反向推开并松手停稳，再导出CSV进行基线分析。
- 验证与剩余：完成上述源文件及实际CSV字段对照；未连接串口、发送命令、使能、复位或烧录，未取得本轮新实机数据。没有可执行源码改动，不重跑无关主机/ARM全量构建。待获得用户正反向回位CSV及松手时刻后分析；静差根因、真实力矩标度及原CAN首错根因仍未关闭。

## 2026-11-09（America/New_York）｜恢复USB-CAN主监控与MIT线值记录

- 需求与范围：用户要求改回CAN回传以提高采样频率。使用embedded-firmware-workflow及ponytail复用已有GS_USB Reader和MCU CAN RPC，不引入新依赖、通用传输框架或CAN自定义遥测分包。仅改变PC主链路和相关记录，不改变Drag/Motion、模型、零点、MIT控制周期/增益、KEY、故障或自动恢复策略。
- 状态：成果已先通过进度消息交付，源码、离线回归、完整编译/链接与日志收尾完成。没有连接设备、烧录、使能或进行实机频率/稳定性测试，当前已部署镜像仍为此前impedance_hold_20261109串口主链路镜像。
- 固件接线：App/main.c恢复MotorTuning_Init(&hfdcan1)，沿用0x7E0/0x7E1、8帧FIFO及同一控制任务每轮至多一个CAN RPC。UART DMA/可靠RPC仍保留作显式备用；两入口由同一任务串行派发，但协议仍要求单个上位机拥有配置事务，不能同时运行两个配置写入者。未新增UART拒绝、重连或恢复策略；没有改变CAN位时序、引脚、IRQ或RTOS调度。
- 上位机与数据：tools/dm_monitor.py默认GUI/探测使用USB-CAN，--serial为备用，原--probe --port兼容为显式串口选择。复用原USB资源释放、双向线程和队列；普通入口始终不发送0x7FF或裸MIT/使能，MCU仍是电机查询与控制唯一发送者。直接逐帧接收0x11/0x12，并只读解码总线上0x01/0x02的MIT量化五输入；不额外通过CAN重发完整UART数据包。排除21..23量程寄存器回复，特殊命令、非使能反馈或100ms内无新MIT时不沿用旧控制值。
- 记录契约：CSV保留反馈并增加数据/参考来源、MIT原始帧及PC指令时间/年龄；CAN连续参考是按回读区间和连续反馈进行的主机重建，不冒充MCU状态。CAN的MCU tick、反馈tick和遥测序号留空；最近MIT不是同帧执行确认，Task/Mode/模型分项仍为低频RPC快照。tools/dm_tuning.py只缓存既有回读供Reader标注，不增加RPC操作或采样轮次。串口v2多轴解析、显示与CSV仍保留；恢复的CAN器件契约明确仍为当前双轴，不能宣称CAN已任意扩轴。
- 涉及文件：App/main.c、Task/motor_tuning.h、Task/motor_serial.h、tools/dm_monitor.py、tools/dm_tuning.py、README.md及本日志。README顶部明确新源码未部署、操作入口、时间/量化/来源限制及单查询源规则，保留历史镜像与故障记录。
- 快速验证：两Python工具py_compile通过；dm_monitor.py --self-test通过，覆盖默认CAN GUI选择、真实Reader的CAN反馈/MIT与CSV、60帧突发不按50Hz抽样、单查询源及启动/离线不接管、特殊命令解析、过期/非使能控制值清除、参考圈跳变、USB资源收尾，并保留UART v1/v2及2/3/6轴、原38项调参、阻抗按钮与F5/F6回归。--help核对默认CAN及显式UART参数通过；全部为离线验证。
- 完整验证与产物：主机Debug/Release分别构建并运行57/57 CTest通过，包含CAN调参、查询调度和原位阻抗；ARM Debug/Release的ylamp_firmware和bsp_link_check完整链接均通过，Debug应用nm未解析符号为空，dm_serial.py独立离线自检通过。两应用DTCM48840字节、AXI DMA1024字节，未增加固件状态内存或任务栈；这不是板上最坏时序/栈水位验收。Debug bin309904字节，归档为build/arm-audit/can_primary_20261109_firmware.elf/.bin，SHA256为3DBEBA67D75F9B6DD1F82EE90C58536F1B996BF3D58334A9017A27E1636D080E；Release bin243504字节，SHA256为B68269695AFCC0D3B51AE501091F9AB7F824678D326821BF889B4B23B7FB270E。未覆盖任何历史部署归档。
- 剩余与风险：活动5ms MIT对应每轴名义200Hz反馈，空闲10ms查询对应名义100Hz，均未实测；回传频率不代表无丢包、MCU内部同步或绝对稳定。原LEC5/Bus-Off物理根因、参数应答载荷歧义、适配器/PC时间误差及回位静差仍待实机数据。需要新固件部署后CAN RPC才可用，不自动关闭用户窗口、抢占串口、复位或烧录。

## 2026-11-09（America/New_York）｜授权烧录USB-CAN主监控版

- 需求与范围：用户明确要求“烧录”。使用embedded-firmware-workflow及aixprobe核对归档、识别目标、备份、烧录、校验及重连；本轮没有修改控制源码、参数、KEY、恢复策略或进行电机使能/运动试验。当前无aixprobe调试MCP入口，沿用原生ST-Link/OpenOCD，不虚构工具会话。
- 状态：烧录与日志收尾已完成；刷写、主机逐字节比较、普通重连和串口被动检查通过相应检查，CAN联机因未枚举适配器且MCU记录Bus-Off未验收。README与本条目已回读，旧Flash应用前缀与原impedance_hold归档逐字节一致，当前读回再次匹配新镜像；无OpenOCD进程残留。
- 镜像：固定归档build/arm-audit/can_primary_20261109_firmware.elf/.bin；bin309904字节、SHA256为3DBEBA67D75F9B6DD1F82EE90C58536F1B996BF3D58334A9017A27E1636D080E，ELF为A85F70D203DEE7367349102577157B6485727B0613C5C3F2EA1EBF3728A9A0EF。没有烧录bsp_link_check或覆盖历史部署归档，本轮未重跑上一轮已经通过的57/57主机双配置及ARM完整链接。
- 烧录前观察：COM34/230400被动监听1.5秒收到76帧、解析错误0、PC发送0字节；Task FAULT/TRANSPORT，两轴最后反馈已过期约3452735ms，历史状态1不能当作当前使能或失能确认，CAN记录Bus-Off及LEC5。没有发送失能、清错或使能指令。
- 目标与工具：OpenOCD0.12.0+dev、ST-Link V2J43S7，普通连接实读CPUID=0x411FC272、DBGMCU_IDCODE=0x10016483，Flash探测STM32H72x/73x、单Bank1MiB；目标电压约3.295V。清空目标examine-end工具临时设置，运行态识别不halt/reset，刷写阶段才halt并开启work-area-backup。首个命令误选hla_swd，在目标连接前被原生st-link驱动拒绝；读取本机配置后改用swd成功，没有因此复位或重复擦写。
- 备份与部署：停核后保留完整Flash1048576字节can_primary_20261109_before_flash.bin，SHA256为8B489D6E069D2F6C609BCBF3091FDC5031264E1A48DA5DD4640FBD4D9487DCF0，另存DTCM131072字节before_dtcm.bin。写入匹配应用ELF，停核时verify_image通过，再完整dump应用309904字节至readback.bin，执行一次reset run并正常shutdown，OpenOCD退出0。随后主机逐字节及SHA256比较与候选一致；未恢复旧RAM或在运行态执行借用RAM的verify_image。上述文件及flash.log均在build/arm-audit同前缀。
- 重连与通信：普通SWD重新连接、不halt/reset，实读相同CPUID/芯片ID、CPU running，正常退出，见reconnect.log。COM34被动监听3秒收到151帧30208字节、解析错误0、PC发送0字节，MCU tick继续递增；Task FAULT/TRANSPORT、反馈valid_mask=0/两轴count=0，不能将初始化state0解释为电机确认失能。CAN first/Bus-Off tick1263ms、LEC5，TEC248/REC0；UART溢出、CRC、TX丢弃及UART错误计数均0。调用现有scan_adapters返回空列表，当前不能做USB-CAN实收频率或RPC联机验收，不自动切换链路、占用其他窗口或接管查询。
- 文档与边界：README顶部改为实际已部署镜像，保留原串口阻抗归档历史及已有故障限制；本日志记录本轮实际结果。未覆盖带电运动、回正精度、物理断电重启或CAN稳定性；原CAN首错/Bus-Off根因仍未关闭，刷写并不等于通信故障修复。

## 2026-11-09（America/New_York）｜上电后自动辨识前检查与通信恢复

- 需求与范围：用户要求上电后自动测试、辨识摩擦及重力偏差，并提供Stribeck摩擦、傅里叶激励及RMSE评估示意。该要求改变此前“不加摩擦补偿”的意向，但尚无新实测系数，不将授权视为可以虚构辨识或直接启用未经验证的补偿。使用embedded-firmware-workflow、ponytail及aixprobe核对现有模型、Motion/RPC与上电状态；暂未修改控制源码或新增Mode。
- 状态：通信恢复后已按用户后续授权开始自动试验；已完成±3°及±20°双轴逐轴往返和真实CSV采集，摩擦/重力参数拟合与独立验证仍进行中，尚未部署新补偿。
- 首次实测：USB-CAN枚举仍为空，COM34可用。被动监听3秒151帧、解析错误0、PC发送0字节，Task FAULT/TRANSPORT、反馈valid_mask0/双轴count0，Bus-Off tick1263、LEC5、TEC248；故障记录与上次电机未上电阶段一致，不能把“电机已上电”直接等同通信恢复或当前关节角已知。
- 已完成动作：读取App/main.c与Task恢复路径，确认默认启动只查询、不因反馈使能；现有Bus-Off恢复有次数限制，未改为无限重试或绕过故障。通过原生OpenOCD重新识别CPUID0x411FC272/芯片ID0x10016483后执行一次reset run，未擦写Flash、临时设置examine-end、修改RAM参数或发送电机使能。复位包含现有OUT1供电初始化，不将其描述成完全无硬件影响。
- 恢复后实测：COM34/230400被动监听5秒250帧、解析错误0、PC发送0字节；Task IDLE/NONE、双轴有效反馈mask3/state0/反馈年龄10ms，约100Hz电机反馈计数增长，CAN异常观测0，UART错误/溢出/丢弃0。J1连续参考约−55.7243°、J2约−108.7481°；仅证实该观察窗口，不能认定原CAN故障物理根因已经修复或USB-CAN已连接。
- 模型与边界：公共模型现有8项以内的名义动力学参数中，重力仅有J2的sin项，标记朝上零点仍为−151.2828024927°；模型约定J1竖直、J3固定负载、板/螺丝/线束忽略。实际配置关节区间J1 −210..0°、J2 −200..−60°是既有软件范围，并非新验收的无碰撞激励空间；Motion要求每次运动显式提供速度/加速度/jerk，用户先前暂缓了这些能力值。J3未使能不自动证明机械固定；不能据一次静止数据区分静摩擦、重力误差、力矩估算偏置，更不能辨识惯量。
- 用户后续决策（当前有效）：用户允许在现有关节区间内自主测试，明确放宽不必要限制、不反复确认；加减速度上限按SI为±2 rad/s²，最终速度上限160 rpm≈16.7552 rad/s。中途“60 rpm”已被用户纠正为笔误；该次±3°试验报告保留当时60 rpm记录，不改写历史。当前MIT配置速度编码上限±10 rad/s并非机械能力实测值；测试请求不得超过编码量程，不擅改驱动解码比例。只做规划约束，不恢复瞬时速度跳闸。J3固定负载仍属未独立测量的模型假设，不作为反复确认或禁止现有小范围测试的新门槛。
- 上位机发现与修复：首次原位捕获已执行并使能，但连续诊断RPC抢占串口遥测，40个遥测时隙被跳过，主机随后因时间戳不新鲜报错；CAN观测仍0，未启动位移作业。仅调整PC串口RPC节奏：遥测年龄到20ms时让出发送机会、最多等250ms新遥测，不延长固件动作寿命、不重发动作；采集器关闭不需要的周期大快照但保留Task状态及动作后的Motion快照。无新固件、保护、Mode、恢复或KEY逻辑。
- 试验与证据：新增tools/dm_identify.py，复用SerialReader、可靠RPC、现有Motion和CSV格式；只有--run才使能并执行，正常末尾回到捕获目标继续有力矩保持，退出PC不等于失能。异常仅请求一次正常STOP，不盲清错/重新使能。build/identification/20261109T151532Z保留失败记录；20261109T151844Z完成±3°、0.05 rad/s试验（2430条双轴样本），J1未克服静摩擦、J2仅小幅运动；20261109T152026Z完成±20°、0.2 rad/s、a≤2、jerk8的自动试验，两轴实际双向转动，稳态回正误差约J1 6.38°、J2 5.19°，CAN观测0。到达规划目标不冒充实际零误差到达。
- 已执行验证：新采集器无硬件计划自检、Python编译及dm_monitor.py --self-test通过；后者覆盖原CAN/UART、多轴解析/GUI/CSV、38项RPC与动作过期/未知结果，新增遥测让时与无遥测不发RPC测试。控制固件没有变更，不重复无关ARM全量编译。
- 下一步：继续多速度真实双向采样、分离重力与摩擦，并用独立轨迹验证；之后才考虑惯量/耦合更丰富激励。尚未拟合/部署新补偿，不宣称完成系统辨识，也不把无外力传感器的电流估算标为真实已标定力矩。

### 后续实际结果补记：有效摩擦/重力拟合，候选固件未部署

- 继续完成20261109T152326Z、152447Z、152650Z、152842Z、152958Z、153121Z、153311Z、153522Z真实双向记录，速度0.12～0.6 rad/s、a≤2；部分J2轨迹跨过人工朝上零点。训练与独立验证按整个记录分开，均未新增CAN异常观测；这不是历史Bus-Off物理根因已修复的证明。
- tools/dm_fit_identification.py用实测速度局部回归求加速度，扣除CAD名义惯量/耦合，在实际运动样本拟合库仑+黏性与Stribeck备选。最终identified_20261109.json：J1 Fc=0.60803 N·m、Fv=0.22539 N·m·s/rad；J2 Fc=0.48521、Fv=0.41516、重力sin系数−0.58510 N·m。独立验证力矩RMSE约J1 0.06319、J2 0.03989 N·m。Stribeck未稳定改善留出误差，未选用；拟合bias未当成恒定助推。
- 候选源码已在公共Model增加纯摩擦计算，Motion仅按轨迹期望速度叠加摩擦前馈，保留M·ddq+c+g，不以实际速度噪声做正反馈。Kp=5/Kd=0.5不变，固定保持的期望速度为0、摩擦前馈为0，不能声称已解决手推松手后的静差。惯量仍为CAD初值、非完整惯量辨识。配置加减速度上限2、用户速度160rpm，当前协议编码仍±10 rad/s；不新增瞬时速度跳闸或Mode。
- 已执行主机Debug/Release各57/57、ARM Debug/Release应用与BSP完整链接、Python采集器/拟合器自检及上位机离线回归；候选Debug bin311472字节，尚未烧录。相关源码为Model/robot_model.*、config/robot_model.c、config/motor_config.*、Mode/motion_mode.*及对应测试。
- 为准备部署执行park_before_flash_20261109T154110Z，向人工UP目标移动后显式F6，回读双轴新count/state0/age10ms。J2实际−145.6874°，距人工UP −151.2828°约+5.5954°，不是实际竖直到位；该判断未用IMU。用户随后询问依据，部署尚未停核/擦写，已说明目标到位不等于实际到位。

## 2026-11-09（America/New_York）｜开启IMU核对几何关系并继续辨识

- 需求与授权：用户明确允许开启IMU、取得实测信息并继续系统辨识。沿用既有自动试验/烧录授权及关节、速度、加速度范围，不新增重复确认、保护门槛或恢复策略，不自动覆盖人工朝上零点。
- 状态：新鲜IMU实测及补偿成果已先交付；先部署摩擦/重力v1，再部署附加惯量v2，真实闭环对照、慢速大角度最终复核及日志收尾已完成。精度范围与剩余问题见下，不宣称完成所有系统参数辨识。
- 实测与边界：当前J2编码器约−145.6874°、相对人工UP +5.5954°；IMU预热后数据年龄5～9ms，Roll约+5.92°、Pitch约−3.71°，加速度约(0.629,1.000,9.64)m/s²。axis仍0（未确认），不能直接把传感器Pitch或+Z倾角等同连杆竖直偏差，更不能声称已独立验证真实竖直。
- 工具改动：tools/dm_tuning.py新增由原TX线程执行的只读calibration_read排队动作；tools/dm_identify.py新增显式--imu、预热/新鲜回读与同一冻结快照里的IMU/编码器记录。连续完整诊断导致首次开启请求排队过期，未发送；改用既有精简查询后开启成功。未修改MCU IMU生命周期、协议、控制线程、零点或KEY。首次输出报告因未就绪姿态NaN被严格JSON拒绝，已将不可用派生角记录为null并重新采集，不丢弃已有CSV。
- 下一步：多姿态停稳数据核对IMU随J2运动的轴向、比例和重复性；独立检查重力相位/偏置与摩擦分离，再部署经验证候选并做同条件闭环对比。不以电流估算力矩拟合误差冒充外部测量绝对精度。
- 多姿态实测：新增20261109T155146Z（J2中心−130°、±50°、0.25rad/s、停稳5s）与155347Z（±42°、0.4rad/s独立验证），均完成且CAN异常0。停稳后同快照IMU/编码器拟合三维安装轨道a=c+u·cos(q2)+v·sin(q2)，训练18快照、独立20快照；独立相对角RMSE0.03318°、最大差0.05175°，不是绝对竖直标定。Roll对编码器斜率约1.001，Pitch也随J2改变，未将其简单当固定安装偏角；未覆盖原人工零点。
- 更新辨识：identified_imu_20261109_v1.json包含上述IMU与原多速度训练/验证；J2更新Gs=−0.57955726、Fc=0.48597053、Fv=0.41571019，独立力矩RMSE0.03709 N·m。J1不变、独立RMSE0.06319。离线试加cos重力项未改善独立误差且与bias相关，未增加模型项或使用拟合bias恒定助推。
- 已部署：identified_imu_20261109_v1_firmware.elf/.bin，bin311472字节、SHA256为6FB319DDCDEED1977BA420DC67C486A17B9BB4EFA21C38DB671064CD7B78280D。部署前disable_imu_before_flash_20261109T155646Z回读双轴state0、新反馈count/age5ms，约半秒无编码器漂移。原生OpenOCD重新确认CPUID0x411FC272/芯片ID0x10016483，停核备份完整1MiB Flash与128KiB DTCM，再写入/verify、dump应用及reset run；应用回读逐字节/SHA256一致。首条带内嵌引号的命令在解析阶段失败、未接触目标，保留原日志；修正后完成，没有重复刷写。普通不复位重连ID可读，初始curstate=unknown尚未poll，随后串口tick/IMU采样推进证实运行，不能把unknown当成已观察running。
- 部署验证：主机Debug/Release各57/57 CTest通过、ARM双配置应用及BSP完整链接、采集/拟合器自检与上位机自检通过。新增测试覆盖只读calibration_read不触发标定/使能，未就绪姿态null可严格JSON保存；采集器等待原位捕获时不将前一帧人工F6锁定误判为新运动故障。20261109T155110Z保留该主机时序失败，随后修正后的正式记录成功；没有修改MCU故障策略。
- 补偿后真实对照：20261109T155914Z重复J2中心−130°、±50°、0.25rad/s、a2/jerk8/停稳5s。正向位置RMSE6.46836→0.58079°，反向6.95898→0.14068°；两端停稳误差+5.4934/−5.4396→+0.1172/+0.5493°。Kp/Kd不变，CAN异常观测0。先后顺序测试存在温度/初态差异，不宣称随机对照或全工况精度保证。后续J1跨J2姿态及更丰富加减速验证仍进行中。
- J1跨姿态：20261109T160101Z在J2目标人工UP、160303Z在J2目标−100°，J1均±24°、0.25rad/s完成。前者J1两端停稳误差+1.1158/+0.6546°，后者+0.7442/+0.8076°；回中分别−0.8087/−0.7507°，并非零静差。前者J2实际−151.7855°，距人工UP约−0.5027°，不是已独立确认绝对竖直。跨J1姿态的IMU相对角验证18快照RMSE0.05055°、最大0.07777°。所有窗口CAN异常观测0。
- 动态记录与建模：160431Z双轴逐轴±30°、1rad/s、a2、jerk12；160557Z双轴逐轴±26°、J2中心−140°、0.75rad/s、a1.5、jerk10，均返回捕获目标并保持。tools/dm_fit_identification.py增加有效附加对角惯量候选、实测速度局部回归带宽敏感性，以及力矩/动量积分回归；不增加板上辨识状态机、算法线程或新Mode。因微分带宽改变会明显改变惯量，未直接采用微分结果；积分用CAD的p=M(q)·dq及拉格朗日位置偏导，剔除粘滞/换向/间断窗口，只拟合CAD以上的附加项。
- 惯量证据：inertia_crosscheck_20261109.json以160431Z/160101Z训练，160557Z/160303Z/155914Z独立验证。0.25s积分窗得到J1附加0.0270437158、J2附加0.0367185807 kg·m²；0.15/0.4s为J1 0.02314/0.02613、J2 0.03522/0.03704。独立窗口平均力矩RMSE分别0.03861→0.03322、0.04436→0.01704 N·m。重叠窗口不是独立统计样本；这些为电流估算标度下的输出侧有效值，未独立校准转子惯量/力矩标度，B仍CAD名义值，不称完整耦合辨识。
- v2源码：Model/robot_model.h/.c增加非负有限extra_inertia_kg_m2[2]，与CAD参数分列，A加J1额外惯量、D加J2额外惯量，B、c不变；config/robot_model.c采用上述0.25s结果，保留v1重力/摩擦值，未采用积分联合拟合的全部其余系数。Motion名义前馈编码检查使用更新后的A/D；Drag原25%动态补偿比例、力矩预算和限幅不变，但共享的M已更新，本轮未做人手拖动验收。tests/test_robot_model.c新增附加惯量逐项力矩验证、耦合不变及非法值不修改输出。Kp5/Kd0.5、KEY、通信/停止/恢复策略未改。
- v2离线验证与部署：主机Debug/Release各57/57通过，ARM两配置应用与BSP完整链接，Debug bin311624、Release244656字节，DTCM48864/AXI DMA1024字节。固定归档identified_imu_20261109_v2_firmware.*；bin SHA256为0CEC0ECA379B15DC487A708FD9A8D9517280DAD4F08D459A009819925323ED0D。F6后确认两轴新反馈state0、age5ms且约半秒无>0.2°漂移，再停核备份完整Flash/DTCM，刷写/verify及应用独立读回逐字节一致；刷前Flash应用前缀匹配v1。reset run后普通SWD重连经poll实读running及相同芯片ID；原生日志、备份、回读均保留在build/arm-audit同前缀。未在使能运动时halt/校验Flash。
- v2相同轨迹复核：20261109T161319Z重复160557Z的双轴逐轴±26°配置。J2正/反/回中移动RMSE为1.3095→0.4680、0.6759→0.3679、0.7886→0.3551°；J1为0.7860→1.0911、1.3255→0.9689、1.0040→0.6019°，首段变差，不能概括全面改善。两轴两端停稳误差分别J1 +0.1432/+0.9059°、J2 −0.3548/+0.6701°。CAN异常0、UART CRC/溢出/错误0；串口实收每轴约46Hz，266个遥测时隙因通信竞争跳过，不冒称MCU每个200Hz周期都已记录。结束Motion HOLD、两轴state1，新反馈age5ms，IMU仍开启。
- v2慢速最终复核：20261109T161854Z重复J2中心−130°、±50°、0.25rad/s、a2、jerk8、停稳5s。正/反向移动RMSE0.400005/0.135141°，两端停稳误差+0.357048/+0.177653°，回中−0.202583°；比较未补偿时6.468/6.959°及v1的0.581/0.141°，不是用提高Kp/Kd换取。基线IMU轨道用于此独立记录19快照，相对角RMSE0.048464°、最大0.086874°，覆盖相对人工UP −28.895..70.926°。这些是顺序、非随机试验，不能保证温度/负载/全速域绝对精度。
- 最终检查与状态：再次执行拟合器、采集器、dm_monitor.py、dm_serial.py离线自检及四工具py_compile，均通过；积分回归自检包含双轴同时运动的已知惯量/耦合合成数据，但实机本轮仍逐轴激励，不能混称真实同时运动验收。生成acceptance_imu_20261109_v2.json归集v0/v1/v2同条件对照、镜像哈希、IMU验证及限制。最后新鲜回读为Motion HOLD、Task HOLD/NONE、双轴state1，J1约−60.2924°、J2约−139.6768°，IMU flags19/age0ms、Roll约11.948°/Pitch−2.913°；未在末尾F6移除负载支撑，关闭PC采集不等于停机。CAN异常观测0、UART CRC/溢出/错误0，累计遥测时隙跳过497，未声称完整200Hz记录。
- 涉及文件与未覆盖边界：本轮涉及Model/robot_model.*、config/robot_model.c、config/motor_config.*、Mode/motion_mode.*、tests/test_robot_model.c及Motion测试、tools/dm_tuning.py、dm_identify.py、dm_fit_identification.py、README.md和本日志；工具复用既有TX所有者和Motion，不留下临时板上诊断分支。人工UP零点未改；加速度与速度编码限制、原故障/KEY/恢复未改。绝对安装轴/加速度计偏置和力矩标度未独立标定，J3固定假设、B耦合辨识及CAN旧Bus-Off根因仍未闭环；固定HOLD的摩擦前馈为0，未保证手推松手零静差，Drag新惯量下的手感待实测。板上规划WCET/栈水位、热稳定、不同负载与真实双轴同时运动仍未验收；不得将本次局部精度改善扩大为整机精度保证。

## 2026-11-09（America/New_York）｜确认部署及汇总辨识前后模型

- 需求与状态：用户询问是否已经烧录、模型具体修正；已核对源码、前述历史部署记录及验收产物并完成说明。本轮仅核对及补日志，无固件源码修改、连接目标、复位、重烧或运动。
- 核对结果：v2归档bin与上一轮实际Flash回读311624字节逐字节一致，SHA256匹配acceptance_imu_20261109_v2.json，慢速与动态两份验收均为complete_holding。这确认上一轮已部署并实机运行，不冒称本轮重新实时读取了目标Flash或状态。
- 前后总结：相较本轮辨识前CAD版，J2重力sin系数−0.49224575→−0.57955726 N·m，幅值约增17.74%；新增Motion按期望速度的平滑库仑+黏性摩擦，J1 Fc/Fv=0.608030/0.225389，J2=0.485971/0.415710；A在CAD基础上加0.02704372、D加0.03671858 kg·m²，D总值约0.04380450。B及c保留CAD，未重算或独立辨识质量/质心；附加值是输出侧有效参数，非独立标定转子惯量。IMU用于相对几何一致性核对，未改人工UP零点、未引入IMU姿态控制环。
- 行为及边界：Kp5/Kd0.5不变，Drag不新增摩擦助推、原25%动态补偿策略不变但共享更新M；固定Motion HOLD期望速度0，摩擦前馈0，手推松手静差未闭环。引用上一轮J2慢速RMSE及端点实测，未宣称J1全面改善、绝对竖直或完整动力学辨识。无可执行修改，不重复全量编译/测试；原未覆盖项和CAN根因保持。

## 2026-11-09（America/New_York）｜诊断位置阻抗手推回正静差

- 需求与状态：用户反馈阻抗回正精度仍不足，要求说明原因。按embedded-firmware-workflow及ponytail检查实际保持控制链，已先交付结构性结论；本轮只诊断，不调整参数、实现新策略或启动硬件。相关离线专项复核及日志收尾已完成。
- 已证实路径：Task/motor_task.c:686的CaptureHold经Mode/motor_mode.c:65捕获中心，MotionMode_Enter将参考速度/加速度初始化为0；Mode/motion_mode.c:191之后仅MOVING/STOPPING采样轨迹，HOLDING手推偏离不会自动重规划。摩擦在:217使用reference.velocity，因此即便实际手推动了，固定保持的摩擦FF仍0；惯量使用reference.acceleration同样为0。中心未随手推更新，不能将此归因为目标点被覆盖。
- 结构解释：稳态保持近似为MIT的Kp*e-Kd*dq+g_model，Kp5/Kd0.5；没有积分、静态残差估计或松手/回位控制状态。停住后Kd项为0，M*ddq_des为0、速度耦合趋零，小静摩擦/重力残差需由非零位置误差产生的弹簧力矩平衡。数学核算：1°仅对应0.08727 N·m，0.05 N·m未补偿残差可对应约0.57296°误差（示例，不是本次实测残差）。运动库仑参数不能直接当最大静摩擦，也不能在零速永远沿误差方向叠满Fc，否则改变交互力及可能过补偿。
- 验证范围：已读取tests/test_motion_mode.c:135的“手推保持时摩擦FF为0”断言及test_motor_task.inc:854的固定中心、目标速度/加速度0与回拉符号断言，现有测试本来只验证控制律，不模拟实机静摩擦收敛。上一轮0.400/0.135°属于主动规划轨迹RMSE，不是人工推开松手的回零验收；本轮未采新的松手数据，不能判断当前每轴摩擦/重力/驱动误差各自占比。
- 改进方向与边界：建议讨论保留低刚度的稳态残差补偿及交互期间的冻结/释放，或明确的柔顺/回位策略，不优先继续抬Kp或加惯量。速度接近0不代表人手已释放，电流估算不是独立外力测量；未据建议新增状态机或自动动作，也不以新保护阻止运行。公共模型、零点、Mode/Task、固件镜像均未修改。
- 专项复核：ctest --test-dir build/host -R '(motion_mode_standalone|motor_task_impedance|robot_model)'实际执行3/3通过；仅验证上述现有模型/保持代码路径，不等于手推回位实测通过。本轮无可执行源码改动，不重跑ARM全量构建或重烧。日志已回读，当前现场静差分项与交互补偿设计仍待后续。

## 2026-11-09（America/New_York）｜多阻抗中心双向重复测试及最终轮局部辨识

- 需求与范围：用户澄清每方向停点稳定但正反不一致，授权多点、多方向、多次实测及最后一轮系统参数辨识。针对同一目标的双向回位差，沿用已授权关节范围、速度160rpm/现有MIT编码上限及加减速度±2rad/s²，不新增保护或自动恢复策略。
- 状态：同中心多点/双向/重复/双轴测试、最后独立复核、参数对照分析、主机工具时序修复及日志收尾均已完成。本轮未改变控制固件或部署候选参数；不将局部辨识扩大为全参数精确标定。
- 初始实测：COM31 USB-CAN与COM34 CH340均枚举；COM34/230400只读回读双轴新反馈、Motion HOLD、CAN异常观测0。J1约−62.653°/目标−68.336°，J2约−108.006°/目标−106.914°。该单次静止不用于推断静摩擦上限或声称回位精度。
- 命令所有权：发现原dm_monitor.py --tune窗口仍运行，通过已核对源码的“断开”按钮关闭其监控连接，保留GUI及数据，不关闭进程、不失能、不清故障；实验继续由现有SerialReader TX线程及MCU Motion统一发命令。
- 采集器修改：tools/dm_identify.py增加--bidirectional/--repetitions/--center-j1；每轴从正负偏置回到同一中心，轮间交换顺序，记录中心、来向、重复编号。停稳统计增加误差标准差/跨度、目标均值和末段速度；默认旧计划保持。自检覆盖同中心、方向交替、双轴与返回捕获点，实际通过。
- IMU边界：首次final_bidirectional_train_p1_20261109在运动前开启IMU被现有固件以BUSY拒绝（当前使能保持），失败报告/CSV保留；该次未发送捕获/位移动作。不为开启IMU擅自移除负载支撑，本轮编码器/电流估算测试先不依赖新IMU数据，未修改既有人工UP零点。
- 正式测试：final_bidirectional_train_p1b_20261109已启动，J1/J2中心−65°/−145°，±10°、0.2rad/s、a1、jerk6、停稳4秒，每轴三轮正反返回。自动轨迹接近不等于人手推开松手，报告明确manual_push_release=false。更多姿态、留出点与参数分析待实际完成后补记；尚未改MCU、编译或烧录。
- 已完成的七组：p1b/p2/p3训练、p4/p5独立点位、p6同中心速度对照，以及coupled双轴同向/反向同时运动，六个不同中心、64次回中（含8次双轴回中），77344条双轴样本；七份报告均complete_holding，CAN异常观测/UART错误/CRC/接收溢出均0。使用COM34/230400的MCU时间戳采集，不宣称每个200Hz控制周期都被记录；USB-CAN已重新枚举COM31，但未改PC测试程序传输架构。
- 实测核心：−65°/−145°、0.2rad/s时，J1三次同向停点在编码器分辨率内完全重复，正侧/负侧平均误差−1.12309°/+1.21561°，双向差绝对值2.33870°；J2为0.40799°。同中心0.5rad/s时差值变为J1 0.85241°、J2 0.54642°。其他中心J1为0.53184～1.61744°、J2 0.23318～0.94093°；双轴同时回中J1约1.756～1.912°、J2约1.399～1.576°。这证明有方向/过程相关停点差，不是随机抖动，不能据此唯一断定非对称摩擦系数、重力偏置或回差。
- 离线分析：tools/dm_fit_identification.py增加同中心逐方向重复统计、按整段记录留出验证、留一训练记录稳定性、正反黏性/重力cos项备选及当前系数对照；--deployed-inertia显式扣除已有附加惯量，避免摩擦重拟合再次吸收惯量。无偏置输出也单独验证，不以未部署的bias让模型指标显得更好；分离的正负库仑常数与常值bias不可唯一辨识，不强行部署。首次七组总报告final_model_bidirectional_20261109.json已生成，新增系数尚未写入固件。
- 发现并修复采集器捕获完成时序：原来已处于HOLD时，排队的impedance动作尚未ACK就会被旧HOLD帧判成完成，导致captured_target_rad/末尾返回锚点使用旧目标。七组中绝对中心均显式指定且逐方向目标已校验，回中统计仍有效；基线目标另有原始遥测记录，不改写旧CSV/报告。现在须收到本次motion_ack并等其后的新遥测HOLD才完成；自检覆盖旧帧、未ACK、旧ACK、阶段未结束及序号回卷。正在补跑validation_capture_fix同中心正反两轮实机验证；只改主机工具，非新增MCU门控或保护。
- 最终独立复核完成：final_bidirectional_validation_capture_fix_20261109为complete_holding，捕获ACK序号108791；读到的新捕获目标−70.73997°/−106.49794°与后续基线目标完全一致。目标−65°/−145°、0.2rad/s正反两轮复核仍有J1 1.43161°、J2 0.31692°双向差，说明差异并非旧HOLD误读造成；跨轮次终点有变化，不能承诺所有方向的重复散布恒为0。
- 最终规模及报告：8组、6个中心、72次回中（8次双轴同时回中）、87920条双轴样本，全部complete_holding。final_model_bidirectional_20261109.json按3组训练、4组验证、1组最终测试划分；最后测试只评估、不参与系数拟合或模型选择。报告保存逐方向/逐次误差、重复标准差、路径族、模型候选、当前系数对照、积分惯量候选和捕获锚点审计。旧7组锚点字段被明确标注为pre-ACK，绝对中心目标仍逐次通过一致性检查，原始记录不覆盖。
- 参数结果及决策：当前J1有效模型验证RMSE0.097714N·m，新候选中较优Stribeck去除拟合bias后0.101095，四组验证均未优于现有系数；最终独立测试0.076652/0.076198仅微小差异，不能据单组推翻前述结论，保留旧系数。J2较小库仑+黏性候选Gs=−0.60292978、Fc=0.47184277、Fv=0.42957415，验证去bias RMSE0.055835→0.052496N·m，最终测试0.056513→0.054348；这是离线电流估算力矩预测改善，不是部署后位置误差改善。额外正反黏性/cos重力项仅微小改善且条件数更高，未采用；新积分惯量在不同窗口/激励下变化，未据停车专项短行程改写现有A/B/D。候选无自动部署，也没有以恒定bias或误差方向叠满Fc。
- 已证实的结构边界：Kp5/Kd0.5不变，固定HOLD的期望速度/加速度为0，移动摩擦FF为0，因此再拟合一组移动参数不能自动补掉静止方向记忆。实测支持存在方向/接近过程相关停点差；不唯一证明非对称摩擦、回差、重力偏差各自数值。真实外力、绝对竖直/末端位置、力矩标度、最大静摩擦、预滑动参数和完整耦合惯量仍未独立辨识；没有新增积分/松手状态机或运行保护。
- 最终验证与状态：采集器/拟合器离线自检及两脚本py_compile通过；总报告自动核对8/72/87920计数、训练/验证/最终测试分离、全部CAN/UART/CRC/接收溢出为0、最终捕获锚点一致及Kp/Kd未变，均通过。没有MCU可执行源码改动，不运行无关ARM构建或擦写。最后测试反馈年龄5ms、Task HOLD/NONE、双轴state1；实际/目标约J1 −70.95853°/−70.73997°，J2 −106.49794°/−106.49793°，仍有力矩保持。累计UART遥测时隙跳过1130，不是电机CAN丢帧，也不冒充完整200Hz记录；历史Bus-Off根因未因此闭环。
- 涉及文件与收尾：仅tools/dm_identify.py、tools/dm_fit_identification.py、README.md、process.md及build/identification下实际数据/JSON产物。文档已区分当前v2与未部署候选，记录新的COM31枚举及COM34快照采样方式。下一阶段若要降低双向静态回位差，需要另行设计并验证零速残差/预滑动补偿，不能仅提高刚度或把方向常数塞入公共物理模型。

## 2026-11-09（America/New_York）｜模型误差与手推后阻抗中心变化的只读诊断

- 需求与范围：用户追问回位差是否来自建模参数，并新增反馈“大力下压J2一定角度后，锁定阻抗点会变化”。优先检查中心所有权及恢复生命周期，不调参、不运动、不复位、不烧录；仅追加本日志，控制源码保持原样。
- 已交付发现：正常Motion HOLD更新不按实际位置重捕获，手推位移本身不应修改reference.position；但Task/motor_task.c:168-180的通信/陈旧反馈故障路径会重置Mode，begin_recovery也清空Mode，恢复结束在:457调用start，ARMING完成在:510调用MotorMode_Enter，最终Mode/motion_mode.c:63用当时实际位置初始化新的reference.position。原锁定中心未独立保存，因此成功自动恢复确有丢失原中心的可达路径。这是恢复语义与固定阻抗目标的冲突，不能用摩擦或惯量参数解释目标变量被重写。
- 事实边界与纠正：当前尚无用户这次强推前后同步的目标/状态/事件记录，不能断言这次一定触发Bus-Off或恢复，也不能把实际停点偏移当成目标已改写。此前日志“中心未随手推更新”只适用于未发生故障/重入/显式捕获的正常HOLD，不能泛化为恢复后仍保持原中心；本条予以澄清。上一轮无CAN异常的自动轨迹回中数据不等价于大力手推故障恢复验收。
- 参数解释：停稳后惯量及速度耦合项趋零，A/B/D偏差能影响接近过程但不直接提供稳态支撑。重力幅值/零点/轴倾斜、驱动力矩标度和外部负载可造成共同偏移；正反差还可能含静摩擦/预滑动、传动滞回或驱动零速行为，现有电流估算与编码器记录不能唯一分离。无论哪种物理残差，均不应擅自改写业务锁定目标。
- 测试与覆盖：先读取既有test_impedance、test_recovery及坐标分支用例；正常推开测试断言中心不变，但恢复测试强调新姿态重启/不重放旧轨迹，没有覆盖“固定阻抗中心跨恢复保留”的组合契约。现有Debug主机四项专项motion_mode_standalone、motor_task_impedance、motor_task_coordinate-hold、motor_task_recovery-motion通过4/4；这是既有测试复跑，不是新故障已修复或实物复现。首次调用不存在的CMake路径失败后，核实实际D:/CMAKE/bin/ctest.exe并成功执行，未改工具配置。
- 待办与建议：先同步比对连续坐标下的锁定中心、实际位置、Task状态/原因及Mode代次，区分真实重捕获、仅停点偏移及整圈显示变化；修复方向是将锁定中心作为独立业务目标，临时恢复仅重置运行态，不自动重捕获。若恢复后返回原中心，应走连续参考/受约束回位，不能直接恢复旧高刚度MIT帧引起突跳；这涉及恢复行为，尚未实施或宣称验收。当前无新的现场硬件状态读回。

## 2026-11-09（America/New_York）｜暂存阻抗中心恢复缺陷，继续稳态误差分项分析

- 用户决策：将“大力下压后锁定阻抗中心变化”单列待修复，当前不修改故障恢复；继续分析目标不变条件下的稳态误差。
- 待修复问题：通信故障自动恢复清空Motion状态并以新反馈位置重新进入，可能覆盖原中心。源码路径已证实，本次手推是否触发该路径尚无同步现场记录；未修复、未烧录、不能标为已解决。
- 本轮范围与状态：按embedded-firmware-workflow及ponytail做最小、只读诊断，使用已有八组CSV/报告复核真实目标、停稳时间和误差分项，不启动运动、不调参、不追加控制保护。问题已存为README P16；稳态数据分析与文档收尾已完成。
- 数据复核：用已有总报告的3组训练、4组验证、1组最终测试路径，逐份读取CSV及marker，在每个中心回中段的末半窗复核报告均值；共80个被测关节窗口（64个单轴、8个双轴段各计两轴），全部目标跨度0、Task HOLD/原因NONE、Motion HOLD、被测轴state1、参考速度0，重新计算的误差均值与原报告差小于1e−6°。该结论仅限这些固定目标窗口，不否认暂存的恢复重捕获缺陷。
- 确实停稳而非等待不够的证据：各窗约末2秒内J1/J2位置跨度中位数均0、最大均0.02185661°；将末半窗再分两半后，位置误差均值变化最大J1 0.00043713°、J2 0.00437132°，比相应双向差小得多。反馈速度仍存在±0.002442等量化噪声，峰值约0.01221001rad/s，不能将该速度噪声等同于真实持续位移或声称无限时长稳定。
- 误差分项：定义共同偏移m=(e_plus+e_minus)/2、方向半差h=(e_plus−e_minus)/2，均以目标减实际为正；这只是现象分解，不是唯一物理辨识。中心−45°/−175°、0.6rad/s的J1：e_plus=+0.77857668°、e_minus=−0.77247732°，m=+0.00304968°、双向差1.55105400°，方向项占主；中心−80°/−160°、0.3rad/s的J2：+0.61570057°/+0.27692319°，m=+0.44631188°、双向差0.33877738°，含明显共同偏移；中心−60°/−125°的J2：+0.47085958°/−0.47006728°，m=+0.00039615°、双向差0.94092686°。故不能把两轴所有稳态误差统一归为一个重力幅值或固定方向常数。
- 编码核算：按Device/DM_Motor/dm_motor.c的float32截断与解码重算，Kp请求5实际编码约4.88400507、Kd请求0.5约0.49938950；16位位置LSB=0.02185694°，12位力矩LSB=0.01367521N·m，按编码刚度折算0.16042818°。零参考速度编码为−0.00244200rad/s，零FF编码为−0.00683761N·m。这些是协议量化/请求等效值，不是实物绝对精度；不能以其单独解释所有1～2°方向差或擅改驱动量程。
- 指令/电流估算对照：稳态窗口按编码后的p/v/Kp/Kd/FF重建MIT合成力矩请求，与回传电流估算力矩比较，窗口均差范围J1 −0.0107601..−0.0068907N·m、J2 −0.0110956..−0.0051878N·m，平均绝对均差各约0.00895/0.00893N·m。两者不是独立力矩标定，反馈位置/速度自身也量化，不能据此证明真实输出准确、辨识电流偏置或把+0.009N·m常值直接写入控制。
- 重力小修的量级边界：当前Gs=−0.57955726，上一轮候选−0.60292978，差−0.02337252N·m；保持人工UP零点、按候选差值除编码刚度，仅作等效弹簧角估算，在J2−160°约+0.04156°，全角域上界约0.27419°，不是实际部署后误差改善。它不足以单独解释该点共同偏移0.44631°；不代表零点/轴倾斜/其他重力项已被排除。
- 控制解释及建议：当前HOLD近似tau=Kp*e−Kd*dq+g_hat，稳态满足Kp*e=未补偿重力及其他静态残差；惯量和速度耦合项不能直接消掉静差，低刚度使小残差可对应可见角误差。既有运动库仑/黏性模型不包含保持中的静态历史/预滑动。若要保留手推柔软又提高松手回位精度，应单独讨论Motion保持残差补偿/回位行为；不直接常开积分，因为持续手压也可能被积分当作应消除误差而增大对抗。没有独立外力/接触状态，不能仅凭零速宣称人已松手；本轮未实现新策略。
- 涉及文件及验证：仅README.md与process.md；原CSV/报告不改写、控制源码/固件/零点/Kp/Kd不变。Python离线分析实际执行并完成80窗状态/目标/均值断言及编码数值检查，无硬件访问；未运行无关ARM构建或重烧，也未采新手推/松手数据。README P16与本节已回读核对，待办为实际现场目标/状态同步取证、恢复目标保存修复，以及零速残差原因进一步分离。

## 2026-11-09 12:35（America/New_York）｜用户手推演示的实时只读采集

- 需求与范围：用户要求“从现在开始检测，我来演示”。启动现场记录，观察固定阻抗目标是否在手推、故障或恢复期间变化；不自动执行轨迹、不重新捕获中心、不修改参数、不使能/失能、不复位或烧录。
- 操作与涉及文件：复用tools/dm_monitor.py的open_serial及tools/dm_serial.py的Parser/telemetry，通过COM34、230400 8N1被动接收；不建立RPC会话、不发送字节，CAN上位机保持用户操作。没有修改控制源码或采集工具源码。运行期数据写入build/identification/manual_push_watch_20261109T173505Z/telemetry.jsonl及events.jsonl，每0.5秒刷新；原始记录包括UTC/相对时间、MCU序号、两轴原始/连续目标与实际位置、速度、力矩、Kp/Kd/FF、Task状态/原因、Mode/阶段及通信计数。只读进程会跨Task故障继续记录，不因FAULT/RECOVERING自行停采。
- 首次实测与快速验证：17:35:05 UTC启动，首帧Task HOLD(4)、原因NONE(0)、Motion(1)/HOLD(4)，两轴state1、反馈年龄5ms。锁定连续目标J1 −46.74107°、J2 −124.83593°，首帧实际位置与目标相同；约23秒时文件持续增长、记录约50帧/秒，回读数据有效，初始parser_errors、CAN观察值、UART错误、CRC错误、RX溢出均0。已明确提示用户“记录已开始，可以演示”。这些只是采集启动检查，不是手推缺陷或稳态精度验收。
- 当前状态及待办：采集继续运行，等待用户完成手推演示后按时间轴分析目标变化、实际偏差和故障/恢复事件；P16仍未修复，尚不能确认本次演示触发机制。无固件改动，不运行无关构建或主机回归；采集进程结束只关闭串口，不发送电机停止指令。数据可由当前执行会话58157读取；创建该输出目录中的stop.request可请求结束观察，不影响电机输出。

## 2026-11-09 12:39（America/New_York）｜人工手推演示收齐与固定目标停点误差分析

- 需求与范围：用户通知“演示完了”。仅结束被动观察、分析本次实测并记录，不实施新的控制/恢复策略，不改参数、不烧录。沿用embedded-firmware-workflow与ponytail，复用既有协议解析，无新增诊断固件或常驻工具源码。
- 采集收尾：通过输出目录stop.request通知会话58157正常退出，进程确认“STOPPED observer only; no motor command sent”，关闭COM34不停止电机、不改变上位机。记录UTC 17:35:05.837～17:36:46.461，5032帧/100.623322秒、49.99835Hz，MCU序号无缺口、相邻tick均20ms；原始JSONL共8179919字节，SHA256 fb01e68ef672b7a1661a03677b42915dd5d09b768fd46b67abb12b3d32d0e395。analysis.json另存统计，不覆盖原始数据或events.jsonl。
- 已交付事实：原始/连续目标两轴跨度均严格为0，锁定目标J1 −46.74107453°、J2 −124.83592754°，Task始终HOLD(4)/原因NONE(0)、Motion(1)/HOLD(4)、两轴state1；CAN观察、UART错误、CRC/解析错误、RX溢出、TX丢弃均0。反馈计数每轴增加20124，最大反馈年龄5ms。故本次未捕获目标重写、自动恢复或通信冻结，不能把正常HOLD结论扩展为已修复P16。
- 末尾停点：最后10秒各500帧，J1实际−40.59928646°、目标减实际−6.14178806°；J2实际−120.15851837°、误差−4.67740917°，各轴该窗位置跨度0。相对最终位置±0.05°范围内的连续尾段约25.40秒/32.52秒；这是编码器静止平台，不是独立松手/接触检测。演示全程两轴误差极值J1 −21.0264°..+22.9935°、J2 −20.2177°..+22.4908°，实际最大速度0.87668/1.08181rad/s，均无模式/增益切换。
- 源码与力矩核对：Mode/motion_mode.c:214-229的摩擦FF只用reference.velocity，本次两轴参考速度全程0、Kp/Kd请求5/0.5，因此移动摩擦FF为0；保持未含静态残差记忆。按Device/DM_Motor/dm_motor.c的float32编码截断重建，末10秒弹簧恢复力矩均值约−0.523538/−0.398712N·m；编码FF约−0.006838/−0.307692N·m，总MIT请求约−0.530395/−0.706472N·m，电流估算回传均值−0.539706/−0.717238N·m。J2请求FF约−0.299571N·m，包含模型重力，不是锁点移动。此对照不等于独立输出力矩标定，不能直接把差值作为偏置写入。
- 判断与边界：本次明确为“业务目标不变、实际停点偏离”。低刚度恢复力与静态摩擦/残差平衡的解释与数据一致，但没有外力/接触传感，不能唯一分离静摩擦、线束外力、传动滞回或重力误差，也不能仅凭平台判定已经松手。既有运动Fc不等于最大静摩擦，不能简单按误差符号在零速时全额注入；A/B/D在静止无直接消差作用。下一步应先讨论Motion保持残差与交互回位策略，保留固定业务目标；本轮未新增积分、助推、状态机或保护。
- 涉及文件及验证状态：README.md、process.md与build/identification/manual_push_watch_20261109T173505Z的采集/分析产物；无MCU/GUI源码改动，停止采集没有停止电机。全部5032帧的序号/tick单调、目标跨度、误差字段一致性断言通过；追加复核原始SHA256、帧数、状态/增益、有效掩码、反馈计数及各500帧尾窗均值/位置跨度，均通过。请求/电流估算力矩均值差小于0.02N·m的对照通过，但不是独立力矩精度验收。进程回读仅剩原上位机pythonw PID23904，被动采集进程已退出；README与本节已保存，诊断记录收尾完成。未运行无关ARM构建/主机控制测试。P16仍待修复，现场真实外力与绝对末端位置未测量。

## 2026-11-09（America/New_York）｜澄清实际回正速度与摩擦前馈参考速度

- 需求与范围：用户质疑“回正过程中的速度不是零，为何没有回到原点”。只读核对现有源码和刚才5032帧记录，解释控制适用范围；不启动硬件访问、不修改控制策略或参数、不烧录。使用embedded-firmware-workflow与ponytail复用已有证据。
- 源码结论：Mode/motion_mode.c:181读取实测速度用于动力学速度耦合及MIT速度误差；:193-196仅MOVING/STOPPING采样Ruckig，固定HOLD不因实际位置偏离而生成新轨迹；:217-224的摩擦FF与MIT目标速度均取reference.velocity。故实际手推/回正速度非零不意味着摩擦FF非零；本次整段目标速度均0，而非只有最后停稳时为0。此前“速度为零”的说法必须明确限定为目标/参考速度，不能指实际运动速度。
- 数据复核：逐帧断言两轴target_velocity均0通过；按|实际速度|>0.05rad/s计，J1/J2各有490/509帧运动。朝固定目标运动的示例（误差与速度同号，不等于已独立检测松手）：t=54.102s时J1实际速度+0.153847rad/s、误差+0.095750rad；t=26.362s时J2实际速度−0.305250rad/s、误差−0.346761rad；两者参考速度仍0、Motion HOLD、Kp5/Kd0.5。
- 解释与边界：回位由Kp*(q0−q)弹簧产生，不是参考轨迹驱动。接近目标时弹簧力减小，实际运动阻尼和摩擦耗散动能，可能在目标前停住；零速时阻尼消退，但剩余恢复力仍可被静态阻力平衡，故“曾经在动”不保证最终零误差。这是与记录相容的机制，不是对真实摩擦/外力的唯一辨识，也不能将稳态误差归为零速阻尼。现有规划速度摩擦模型没有覆盖固定阻抗中心下的被动回正；未据此直接改用实测速度做助推、添加积分或重新规划。
- 涉及文件与状态：仅追加process.md，原始记录/报告及固件不改写；源码与全记录速度区分检查完成，未运行无关构建或控制回归。保持残差/交互回位的具体实现和P16恢复丢中心仍待后续处理。

## 2026-11-09（America/New_York）｜编码器差分滤波驱动Motion摩擦补偿

- 需求与范围：用户要求根据实际编码器位置微分、重点滤波后补偿摩擦，并自行测试。本轮实现和离线验证，不烧录、不访问控制硬件或自动运动；采用embedded-firmware-workflow及ponytail，只改Motion内部估计，不向Task增添Drag/Motion分支。
- 状态：实现与离线验证已完成；未部署。独立Motion测试、最终57项主机回归、H723 ARM完整链接、既有人工手推记录的生产C代码回放及README/日志核对均已完成。
- 控制变化：Motion按新反馈计数与接收时间戳对连续参考圈编码器位置差分，再以50ms时间常数一阶低通；剔除一个编码步长所对应的速度噪声（连续软死区），再调用既有Fc*tanh(v/0.02)+Fv*v。取代原规划速度摩擦项、不双重叠加。固定目标下手推/回正均可参与，停住后归零。其余M*ddq_des、c(q,dq)、g(q)、目标p/v及Kp5/Kd0.5不变；Drag不变，未增加速度跳闸、积分、误差方向助推或静态记忆。
- 生命周期与契约：Enter只初始化位置/时间/计数，滤波速度从0开始；重复缓存帧不作为零速度样本，同毫秒新帧等待时间推进；采集间隔超过既有反馈年龄或时间倒退仅重置估计器，实际失联仍由Task原策略处理。Reset清估计，不重写阻抗目标。millis/计数自然回绕、非法更新不提交半状态均在主机检查中覆盖。CAN/串口协议、RTOS周期/栈配置、IRQ/DMA、接线、零点和极性未改。
- 文件：Mode/motion_mode.c/.h、config/motor_config.c/.h、tests/test_motion_mode.c、tests/test_motor_task.inc及README.md/process.md。估计与摩擦输入诊断保留在Motion状态，不改变现有线包字段布局。测试入口复用motion_mode_tests；新增--replay仅属主机测试程序，不进入固件。
- 已执行验证：独立Motion断言覆盖固定目标下正负编码器运动、与电机速度相反时仍使用位置差分、50ms滤波数值、新/重复/同毫秒帧、实际采样间隔、计数与时间回绕、一编码步长噪声、换向滤波、停住归零、重入和原动力学分项保留；连同原轨迹及边界停车检查通过。全套ctest 57/57通过，ARM ylamp_firmware链接成功（FLASH312376B、DTCMRAM48920B），并未执行烧录。
- 集成与资源复核：补充真实Device解码→Task连续坐标→Motion→MIT命令路径的双向编码器运动测试，电机回报速度刻意与运动相反；两个方向均产生正确符号的摩擦补偿、保持目标不变，FF仍恰为原惯量/耦合/重力加一份摩擦。补测后重新构建并最终ctest 57/57通过。ARM静态栈证据中MotionMode_Update帧408B、friction_velocity帧56B；原控制栈2048 words（8192B）和静态内存策略未变，不将单帧静态数字当作整条调用链或运行时栈余量验收。
- 旧数据生产C回放：先复核原JSONL SHA256 fb01e68ef672b7a1661a03677b42915dd5d09b768fd46b67abb12b3d32d0e395，5032帧约50Hz快照逐帧输入motion_mode_tests --replay。滤波速度峰值J1/J2为0.831719/0.950647rad/s，消噪后摩擦输入峰值0.824090/0.943017rad/s，摩擦力矩峰值0.793771/0.877993N·m；分别650/708帧非零，尾10秒各500帧摩擦均0。两轴目标跨度严格为0，参考速度仍0，不将50Hz快照冒充完整200Hz控制记录。派生统计保存在build/host/motion_friction_replay_20261109.json，未改写原始记录。该回放只验证同一段已测位置下新算法的输出，不是新控制器闭环运行，不能据此宣称回正精度改善。
- 限制与待办：50ms滤波有延迟；当前pmax12.5下编码步长约0.00038148rad，对应软死区约0.0076295rad/s（0.43714°/s），真实极慢运动也会减弱/没有补偿。此为运动摩擦补偿，不保证已停住的静差归零，也不等于独立外力检测或稳定性验收。P16恢复时丢阻抗中心的既有缺陷仍未修复；不能以本次目标不变测试宣称故障恢复已修好。离线回放不能证明实机回位精度改善，硬件手感/闭环振荡/静差仍待部署后测量。

## 2026-11-09（America/New_York）｜烧录并按滤波优先顺序实机验收

- 需求与授权：用户明确允许烧录、驱动实机；严格先验收编码器差分滤波，合格后才进行回正。沿用embedded-firmware-workflow、ponytail及aixprobe流程，不新增速度跳闸、保护锁定或新的启停/恢复策略；P16暂存不改。
- 首次交付与状态：测试固件已烧录并读回验证，收尾及实机验收进行中。原串口50Hz没有滤波速度字段，因此临时在Motion内部增加仅测试宏启用的256条RAM环形记录，每条96B，记录每个控制周期的编码器/时间/计数、原始差分、低通/消噪速度、摩擦/总FF及目标；通过SWD运行中读取，不暂停已使能MCU、不增加CAN/UART流量。验收后移除临时诊断，恢复普通构建并重新烧录。
- 烧录前事实：COM34读回两轴失能state0、反馈新鲜；连续1秒位置跨度J1 0、J2 0.00038147rad，才暂停CPU。原CAN上位机继续运行，未抢占其CAN接口。
- 已执行烧录：原生ST-Link/OpenOCD连接STM32H723，CPUID 0x411FC272、DBGMCU_IDCODE 0x10016483；备份整片1MiB Flash及128KiB DTCM，烧录312936B测试镜像，verify_image通过，应用区读回SHA256与bin相同，均3E4A1933620830026D2A593741AF333A958CB59A2021D2414C7F4FA0ED162C2D。文件build/arm-audit/filter_trace_20261109_*；复位运行后普通SWD重新连接再次读到相同ID，不依赖连接复位。
- 待执行：静止噪声、逐周期滤波递推/量化/换向延迟实测；仅通过后进行同中心双方向自动轨迹回中，不能冒充人工推开松手测试；最后移除诊断、普通固件烧录及验证，记录数值与最终状态。50ms滞后、极低速消噪损失和零速静摩擦仍需实测评估，不提前声称精度改善。
- 滤波阶段已通过（仅限本次测试范围）：build/filter_acceptance_20261109保存原始trace.jsonl、samples.csv、report.json及analysis.json；连续10678条约53.39秒，序号无缺口，除首次dt0外10677个控制步均5ms。反馈时间在进入时含8/2ms、9/1ms各一次，其后均5ms；低通递推最大绝对误差3.26e−8rad/s，消噪/摩擦重算最大误差2.80e−9rad/s、7.80e−8N·m。
- 静止与运动观测：基线尾5秒J1编码器恒定，J2跨度一个编码步0.0218566°；J2原始差分RMS0.0127664rad/s、低通后0.000834647rad/s，下降93.46%，消噪后双轴摩擦均0。分别进行J1/J2目标+8°、−8°、返回基线，目标速度0.1rad/s、加速度0.5rad/s²、jerk3rad/s³；六个停稳尾2秒各400条，位置跨度均不超过一个编码步、摩擦峰值均0。全程实际差分峰值约0.68665/0.68679rad/s，摩擦峰值0.73730/0.73387N·m；未出现持续自激或Task/CAN异常。
- 延迟与边界：与同一编码器离线50ms居中割线速度比较，相关性最高滞后两轴均45ms，相关系数0.99666/0.997997；这是编码器内部一致性、不是独立测速器精度标定。J2两个运动段各有1条与瞬时量化差分反号，不能声称低通无换向滞后。当前响应与50ms设计相符、稳定尾段不注入力矩，因此允许进入后续自动回中阶段；未验证所有高速/负载或人工松手工况。57项主机回归重跑通过。
- 首轮回中有效性复核：build/identification/filter_return_20261109_center1，±8°、0.3rad/s、1rad/s²、6rad/s³、每段停4秒，双轴每方向两轮。工具完成是“参考目标到位”，不是实际零误差。逐段CSV发现J1负偏移、J2正偏移的部分段实际跨度0，未克服静摩擦，不能把8个返回窗口全部视为有效双向回正。该组J1停点误差1.55182°，J2−0.08743..−2.27313°；这是混有无效激励的观测，不作为完整方向一致性验收。全组CAN观察0、解析invalid0；串口记录统计有112个遥测序号缺口，不能称该组50Hz全帧连续。
- 诊断已清理与普通版部署：移除Mode/motion_mode.c中临时环形记录及调用，恢复CMAKE_C_FLAGS，无测试宏、24KiB记录或新增串口字段残留；普通固件FLASH312376B、DTCM48920B，完整链接成功。确认两轴state0、位置跨度均0后烧录普通版，verify_image通过，读回SHA256一致6798C7A47249AC6F53DC534265A70CC9190AA5B53BAF812066D107EFEBC64420；build/arm-audit/filter_normal_20261109_*。正常断开重连可读相同芯片ID，无暂停/复位运行验证；测试版控制任务历史栈最小余量529words=2116B，仅为已运行路径的观测。
- 后续回中重测：只将实验目标偏移改为±15°以获得真实双向激励，控制Kp/Kd/摩擦系数/滤波/模型/速度及加速度均不改；普通版上测试两个姿态。未改零点，临时捕获/实验目标不应被当作用户新的建模零点。测试结束将恢复本次测试前已读到的锁定目标J1−0.8157856464385986rad、J2−2.178797960281372rad。
- 普通版两组已完成：build/identification/filter_return_20261109_normal_center1及normal_center2，中心约−40.57743°/−120.09295°及−60°/−145°；每轴两轮正反、±15°、0.3rad/s、1rad/s²、6rad/s³、停4秒。分析逐段检查回中起点确在对应侧超过2°、实际返回位移超过2°，16/16有效；不是仅凭Ruckig参考完成。actual_return_analysis.json逐窗保留方向/实际位移/误差。没有进行人工外力推开松手，不把自动目标往返当作固定中心外力释放。
- 回正结果未达到“静差已解决”：第一中心J1正侧返回误差1.11469°/1.18026°、负侧0.15300°/0.08743°，方向均值差1.02726°；J2正侧1.20211°/−0.24042°、负侧−0.43713°/−0.45899°，方向均值差0.92891°。第二中心J1正侧0.88252°/1.25408°、负侧−0.10108°/−0.84420°，方向均值差1.54094°；J2正侧0.31586°/3.24295°、负侧1.21897°/−0.23056°，方向均值差1.28520°。最大绝对误差J1 1.25408°、J2 3.24295°；两个中心不同、同方向重复也不完全一致，不能用一个固定偏置概括。
- 稳定性与采样边界：上述停稳末半窗约2秒，各J1位置跨度0，J2最大一个编码步0.0218566°；未观察持续自激。每组末帧CAN异常观测均0，invalid解析均0；50Hz串口报告仍分别有115/113个序号缺口（每轴各5051/5184条样本），不能把这些组冒充无丢帧的200Hz记录或由此证明所有短暂事件都被捕获。专门滤波阶段的SWD序号无缺口独立成立。
- 最终输出与恢复：普通版保持运行，经现有Motion/Ruckig接口恢复测试前业务目标−0.8157856464385986rad/−2.178797960281372rad，未重捕获中心或改建模零点；再记录5秒。最后读回Task HOLD4/原因0、Motion/HOLD、两轴state1且反馈新鲜、CAN异常观测0；实际J1−48.97049267°、J2−125.90690120°，目标减实际+2.22941814°/+1.07097366°。该最后位置不是16个测试窗口的一部分，亦说明目标恢复不保证物理精确回中。build/filter_acceptance_20261109/final_status.json与final_restored_samples.csv保留证据。关闭采集只关闭COM，不失能/不停止板上保持。
- 最终验证与收尾：移除临时诊断后重新构建主机并57/57通过；普通ARM ELF中无trace_filter/g_motion_filter_trace符号，CMAKE_C_FLAGS无测试宏，Flash312376B、DTCM48920B。README已更新当前部署/误差及历史离线条目，数据不覆盖。当前结论为“滤波在实测范围通过、普通固件部署完成，回正残差未解决”，不宣称独立力矩精度/绝对末端精度、全速度全负载稳定或已修复P16。后续应分开研究运动摩擦与零速静态/历史残差，并补固定中心人工推开释放验证；本轮未擅自加积分、误差助推、调高刚度或追加保护。

## 2026-11-09（America/New_York）｜回正静差的下一步方案评估

- 需求与状态：用户询问剩余回正静差怎么办。本轮为方案分析，已先交付建议及交互代价；分析与日志收尾完成，候选控制策略尚未获得实施确认，未改控制源码、连接设备、启动运动或烧录。
- 核对与发现：复核Mode/motion_mode.c、config/motor_config.c、config/robot_model.c及上一轮两个中心的有效回中报告。当前Motion按实测编码器差分滤波速度补运动摩擦，无静态积分；停稳后该摩擦项归零。16个有效自动回中窗口最大绝对误差J1约1.254°、J2约3.243°；这不是固定中心的人工推开释放试验，也没有同工况补偿开/关对照，不能声称已量化精度改善或确定全部残差来自摩擦。
- 候选方案：保留现有Kp/Kd、50ms速度滤波、物理模型和前馈，在Motion内叠加慢速有界残差力矩b，候选更新为db/dt=Ki*(q_lock-q)，总前馈为原前馈+b；积分力矩与增长速度限幅，并结合实际输出饱和处理anti-windup。目标点不随误差更新，不往Task新增Drag分支，不新增Mode或速度跳闸。Ki、限幅及启停/重置规则待实测验证，本轮未选值或实施。
- 物理与交互边界：积分调节的是摩擦、模型偏差和外力等混合残差，不是独立静摩擦辨识。仅有编码器及带噪电流估算时，不能可靠区分持续手压和静摩擦卡住；不能用零速自动宣布松手。持续按住会让积分逐渐增大回拉力，即使Kp不变也不能承诺长按手感完全不变。需验证超调、低速爬行及极限环，不承诺数学零误差。
- 数值解释：用Kp=5 N·m/rad乘上述最大位置误差，仅得约0.109/0.283 N·m的比例项；静止摩擦力处于可变平衡区间，该数值不是最大静摩擦或起动力矩实测值，不能直接拿来定积分限额。
- 下一步验收建议：先在相同中心、相同双向轨迹与重复次数下做补偿开/关对照，再做固定中心人工推开、持续按压、松手测试；同时记录锁定目标、实际误差、滤波速度、原前馈与新增积分，验证回正残差及交互代价。先独立验证残差环，再决定是否需要更复杂静摩擦模型。
- 验证与未覆盖：本轮仅源码与已有数据复核、日志回读；无可执行代码修改，不重复构建或冒充新的实机验收。在线检索调用未返回可用正文，未据此引用论文结论。仅修改本日志；已知P16恢复重捕中心问题及CAN历史问题未在本轮修复，当前硬件实时状态未重新读取。

## 2026-11-09（America/New_York）｜用户反馈回正超调，暂停实机调试

- 需求与状态：用户反馈“现在出现超调”，计划关闭电机休息，要求记录后留待后续优化。本次问题与暂停决定已记录；控制优化及实机调试暂停，不继续运行自动测试或采集。
- 现场事实与边界：超调为用户本次现场反馈，尚未测得涉及关节、幅度、持续时间、是否反复振荡及触发动作；本轮未连接设备或复现，不能以此前停稳尾段无持续自激的结果否定此次现象，也不能据此认定根因。
- 当前控制基线：沿用上一轮普通固件记录，保留现有动力学前馈、实测编码器差分及50ms低通的运动摩擦补偿。前述慢积分仅为讨论中的候选方案，AI未实现或烧录，不把此次超调误记为新增积分导致；既有回正静差及P16恢复重捕中心问题仍待处理。
- 后续优化入口：重新上电后先核对实际固件与锁定目标，再复现固定中心推开/松手回正，记录目标/实际位置、实际速度、滤波速度、前馈分项及时间戳，量化超调、收敛时间和最终静差。依据数据检查阻尼、滤波换向滞后、摩擦补偿及模型偏差，先处理超调与静差的取舍，再决定是否引入慢积分；这些是待执行计划，不是已完成诊断或修复。
- 实际操作与验证：仅读取技能规范及既有process末尾，并追加、回读本条日志；仅修改process.md，无源码/参数/零点修改，无构建、烧录、实机测试或电机控制命令。用户计划关机不等于已确认失能或断电，AI未读取或确认当前电机状态。


<!-- original-process:end -->
