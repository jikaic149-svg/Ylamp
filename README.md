# Ylamp V1.2 五轴机械臂（J1–J5）｜DM-MC-Board02

先读 [当前进度](process.md)，再按需查本页源码入口和某一天的历史。历史中的“当前”“进行中”和旧参数只适用于原记录，不能覆盖当前进度。

## 当前版本

- **重力与摩擦补偿本阶段结束，已上传Git**：2026-11-23用户明确工程接受，冻结当前9EC6固件及现有G/F、PI、名义惯量，不再自动调参、续扫或要求新验证。现有误差与覆盖限制如实保留，惯量探索候选不部署；“阶段结束”不等于全范围/全姿态物理标定通过。46份源码、工具、测试和日档的结项提交b5080ba已推送origin/main并远端SHA核验通过，后续回执文档沿同分支保存；build/原始数据/固件备份仍只在本地，详见[过程日志](history/2026-11-23/process.md)。
- 最新计算完成：inertia02的J1/J2三坡正反12腿原记录/603有效滑动样本不变，新增[位置—加速度分离计算](build/v12-friction-global-20261123/inertia_position_acceleration_01.json)。扣除线性位置偶项后，600ms整档留出追加惯量前→后RMSE J1 .053376→.054502、J2 .040237→.041186N·m；同姿态近速度差分仅10/11对，容差收紧后惯量估计明显变化，不支持部署。原[首轮分析](build/v12-friction-global-20261123/inertia_identification_02.json)及失败捕获保留，名义惯量/G/F/全部板端参数不改；本轮纯离线、无COM/SWD/新运动或健康查询，Python42/42及独立核解通过，详见[日档](history/2026-11-23/process.md)。12腿先前全部正常结束，20116反馈/347sample/12次健康G及DRAG交接，末1105797ms健康DRAG仅为上轮快照。
- 全局F候选02的[9EC6固件](build/v12-friction-global-20261123/ylamp_v12_five_axis.bin)**已实际部署**：169716字节/SHA `9EC65D69B5260BADF7FA526F367B3CA63E31646A17B8B87138BF5F27A0689514`，原UID/旧5D788 verify/1MiB备份/写后与不停核运行全回读通过，普通950kHz SWD/一次必要复位，无失败重刷。部署当时boot旧J3取数/ACK各一次后133484ms健康DRAG；后来已进入首批动态实试、失联退出和用户修复新启动，以上条为当前状态，不挪用旧boot恢复次数。原base/fast/high由新总F替换、不叠加，G/PI/手拖比例/预算不改；前置34/34主机、36/36Python/85000次C对照、ARM/门禁及15组保护有效，非新惯量或全域物理验收。
- **全局摩擦模型原始离线证据**：[friction_global_model_02.json](build/v12-default-drag-20261123/friction_global_model_02.json)，审计136原始文件/488单程、5655正反配对格，5149格进入滑动模型。逐轴全量回拟RMSE为`.03503/.02083/.00955/.01054/.01241`N·m，冻结第一轮模型的留出RMSE为`.03383/.02200/.00952/.00870/.01293`；不能混同全行程实机验收。J1部分工况仍>.05，J3新宽扫无合格配对、J4无第二轮新宽扫、J5本轮未扫；完整公式/源SHA/分工况残差及限制见模型和[当日日档](history/2026-11-23/process.md)。该离线阶段未操作硬件，现由上条9EC6部署承接；原JSON的deploy=false留作当时产物元数据，不伪改成物理全域通过。
- 本轮全角度扫描已结束而非仍在后台：J1/J2各两轮12主腿，J3/J4各首轮6主腿，共36主腿/274573反馈/4582sample；另10准备/回姿单腿。J5入场前原几何检查出现腕部part0/part1见证，原批次自动退出，未发J5辨识指令，不是电机故障。最后捕获3452152ms健康G/31/31/31/fault0/limited0，未新读板状态；后续本轮任务只有离线盘点/建模，不自动续扫、换圈或清错。见[当前进度](process.md)。
- 历史[正常默认G＋F拖拽5D788](build/v12-default-drag-20261123/ylamp_v12_five_axis.bin)曾完成部署：170556字节/SHA `5D788F4436092380F4B068C375C91C9CF37D4BC3772273E96017EF1C0DBD1B47`，UID/旧4F0E verify/1MiB备份/写后与运行全回读通过。首次擦除失败后保留现场、原停核状态降SWD至950kHz完成原应用范围写入，未整片擦除/额外复位，原失败日志保留。该boot旧J3/ACK各一次后87886ms健康DRAG，USB两次重连仍DRAG、cap0x594C4402；34/34原生、15/15Python、ARM/门禁和21组配置保护为当时结果。当前9EC6延续该生命周期、替换总F，不能拿旧状态或旧摩擦误差冒充新模型验收，见[日档](history/2026-11-23/process.md)。
- 历史[J1/J2高速摩擦4F0E](build/v12-friction-high-tail-20261122/ylamp_v12_five_axis.bin)170236字节/SHA `4F0E4CA8E54A452475188E4917D80EFAA16A3B410F4D481D3ED8785BB1699F54`，曾完成J2 G四段/J1J2 F四段健康交G验证。其G/F/PI和J4 N0/行程−180°～350°原样沿用5D788，仅新默认模式/会话交接改变；旧1587548ms健康G为旧boot记录，不覆盖当前DRAG验收。原镜像、备份及27225反馈保留，见[日档](history/2026-11-23/process.md)。
- 此前独立运动复测沿用4F0E：J1/J2 .8rad/s F的7/6格RMSE **.018328/.014299N·m，均达标**；J2 G宽扫50格RMSE **.071219N·m**，数值超.05但用户已工程接受/冻结。5D788沿用相同G/F/PI，已按用户新许可开始上条全角度摩擦矩阵；不将旧局部误差或本轮尚未拟合的采样冒充全局验证，详见[日档](history/2026-11-23/process.md)。
- 用户验收目标为误差<.05N·m，沿用此前RMSE口径但不等于用户已指定峰值口径；不新增电机门控。本轮只重测J2 G与J1/J2高速F，其余轴按用户工程接受暂不重扫，不篡改原统计。此前G的J3/J5及旧高速F超标记录保留在日档，不拿“工程接受”冒充全局物理标定通过；当前实测结论以前两条为准。
- 最新用户收敛为“小误差保持在较小范围即可”：J3先保留2771作为阶段性调试基线，不因局部略差反复调参或要求每侧都优于旧版。已测全幅/局部配对RMSE约.062/.067N·m、绝对残差P95约.092/.090、最大约.099/.111；不是全局/瞬时上限，也不是新增限幅/停机阈值。原坏结果和后续验证缺口保留，当前不追加运动/刷写，详见[当前进度](process.md)。
- 最新2771版[J3双向验证](build/v12-j3-gravity-ls-20261122/j3_validation_sweep_01.jsonl)四段已完成/全部健康交G，54860反馈/910状态，实测raw[−66.303037,128.813871]°；后查1508414ms仍健康G/31/31/31/fault0/limited0，COM36释放，无活动扫描。[单姿态冻结公式分析](build/v12-j3-gravity-ls-20261122/j3_validation_analysis_01.json)204合格正反格：旧→新RMSE .083956→.061947N·m（改善26.2%），但软件负角侧.048464→.060112及同记录局部复扫.054613→.067135均变差，不能宣称两侧/全局验收。不改当前参数或继续加力；下面“正在采样/J5仍越界/未扫”为此前阶段，完整证据/限制见[进度](process.md)。
- J3完整旧G×1.082553[试验版2771](build/v12-j3-gravity-ls-20261122/ylamp_v12_five_axis.bin)已部署/全回读，159812字节/SHA `2771F2359DBEAB662FB27E19FE36083FDAC9515BEA3B8EE19DCEA422DF3CBA4F`；32/32主机+25/25Python+ARM/门禁通过，只有两个系数字段6字节与旧46E不同。原cold已一次成功且不重复；用户新“松手了”后[3秒静态记录](build/v12-j3-gravity-ls-20261122/gravity_released_baseline_01.jsonl)末656262ms健康GRAVITY/31/31/31/fault0/limited0/Gage1，J3计数149295、五台state1/年龄2ms，1316反馈/23状态/end正常/解析错误0。J5实读仍−79.898°，低于记录下限−79.657593°，未回原起点范围；尚未主动扫动/没有运动RPC拒绝。需手动向行程内挪约2°再松手读起点，之后J3双向验证，不改端点/盲发运动，不把单姿态静态记录当完整辨识。完整状态与备份SHA见[当前进度](process.md)。
- 新离线[摩擦辅助重力重算](build/v12-friction-budget2x-20261122/gravity_from_friction_02.json)已完成：91完整记录、440摩擦段+24历史重力段、2517组正反配对；MATLAB/Python数值及空间块区间交叉核验通过，但不等于绝对重力已标定。J3简单候选约`1.082553*当前G3`可优先受检；J2/J5跨记录不足，J4当前行程宽验证缺口保留，全部未部署。重力校验后的[全局摩擦第一版方案](history/2026-11-22/process.md#重力前馈校验后的全局摩擦补偿方案第一版未执行)覆盖J1～J5，主矩阵240单程、已有数据优先复用，尚未执行；本轮无刷写/运动。
- 历史五轴主动总预算×2版[46E固件](build/v12-friction-budget2x-20261122/ylamp_v12_five_axis.bin)159812字节/SHA `46E8625BFB41EEEF9E4C97AC318371AB77B80A251F2B374FFAE0A804BB2165EB`，该轮烧录/测试结束：原32段与精确计时`.8`补样10段均完成，J5`.6/.8`各两轮无故障退出；与A688完整18段共60段。旧boot末1162016ms健康G/31/31/31/fault0/limited0，五台state1/年龄2ms，COM36/探针退出，无活动采样。主动上限[4,10,3.6,1.6,.8]，纯G/PI/摩擦不变；仅五个预算字段内8字节与A688不同。最终[数值整合](build/v12-friction-budget2x-20261122/friction_integrated_080_02.json)链接380段，其中130拟合/10独立计时验证/240冻结基底；摩擦新候选仍未部署，当前2771状态见[进度](process.md)，以下均为历史记录。
- 历史[快档摩擦固件](build/v12-friction-fast-20261122/ylamp_v12_five_axis.bin)159812字节/SHA `A688A949BAACB9965B2D02AFF3316BF69002D09D06083FC900EDE6B524B5A405`曾在J5`.6`回程fault32暂停；18段完整双向记录用于最终整合，另有一成功首段/一中断回段的无end文件单独保留，不补造完成。按真实板端start回放约−.413N·m需求超过旧.4预算，不是已发送超限命令；用户随后明确预算×2，新46E已完成上面的部署/全部测试。旧[80段分析](build/v12-friction-map-20261122/friction_fast_validation_01.json)、镜像和备份保留，不将旧DEGRADED/等待恢复状态当当前，详见[日档](history/2026-11-22/process.md)。
- 整机控制对象为V1.2五轴，不是旧双轴或单电机台架。机械依据为 `../Ylamp-V1.2/装配体2.step`。
- 历史源码基线a01b475的CAN3移除和统一五轴/双Pitch已由592541f承接，本轮新增补偿、手拖、历史角度、工具/测试与日档按Git现有main保存；origin/main已配置，不再沿用“没有远端”。有限速度PI/真恒速平台沿用500Hz及既有退出，当前配置以上述9EC6为准。首轮J1–J5双向10段已全部实动完成；J1/J4速度仍明显波动，同tick命令分项流尚缺，不把目标速度或回报估矩当完整辨识标签。
- 历史固定摩擦版`build/v12-speed-friction-20261121/ylamp_v12_five_axis.bin`157236字节/SHA-256 `3133AC5CC93F354ED3DA658896238B78D319DD77765DDACB31EB553E580ED9D7`，当时UID/完整写后及运行回读通过。用户接受首组效果后，第二负载构型五轴三档正反30段也已完成；3133两组累计60段/178,350反馈/3,020sample，末2981295ms健康GRAVITY/31/31/31/fault0/limited0为旧boot事实；当前已升级2A75，不能把旧状态当当前。[部署及两构型记录](history/2026-11-22/process.md)。
- 用户顺序“先叠加→先验证→新姿态”已推进第二组：速度试验运动轴仍为`[.64,.54,.12,.12,.13]*tanh(v_ref/.02)`N·m，32/32+18/18+ARM/门禁和原11组配置不变。首组性能混合的限制保留，但用户明确接受并要求下一阶段，不以J1/J4复核冻结采样。第二组高速方向阻力矩约`.654/.514/.117/.131/.129`N·m，尚属单次/局部估计，不自动改系数或宣称全部240组合完成。
- 两组六工况的[局部摩擦映射](build/v12-speed-friction-20261121/friction_map_2pose_01.json)已由[离线工具](tools/five_friction_map.py)算出：10个物理文件/60段/407个正反匹配格，按整档速度留出，J1～J5 RMSE约`.0494/.0418/.0134/.0087/.0143`N·m。仅J2保留受载代理项，J3常量、其余速度项；500次/轴条件块重采样全部成功，不是新姿态或实机部署验证。局部计算无需先补采；推广仍需独立复测及更多受载构型，原板端前馈不变、不能将总映射重复追加。
- 历史用户“测完”：当时[编排工具](tools/five_friction_campaign.py)新增180段，补齐4构型×2轮×三档×正反×五轴240段；714208反馈/12080sample，末7407188ms为旧3133健康G。原[四构型拟合](build/v12-speed-friction-20261121/friction_map_4pose_2round_01.json)1586配对格，独立第二轮RMSE J1～J5约`.0409/.0455/.0122/.0109/.0154`N·m；J2姿态4中速缺合格配对，不等同全部标签通过。该式后来随2A75部署；新快档候选仍仅离线，旧原始文件/镜像均保留，详情见[日档](history/2026-11-22/process.md)。
- 原3DE3和三档正反两轮60段基准保留：179,229反馈/3,032sample、六个正常end；初步方向阻力矩J1～J5约.64/.54/.12/.12/.13N·m，J2仅一轮姿态匹配，尚非通用Fc/B；低速爬行/观测间隔及全部限制详见 [当前进度](process.md) 和11-21日档。
- 分轴版仅速度P/I表相对9704变化11字节，其它控制/G/预算不变；32/32、18/18、ARM/门禁通过。对首轮速度RMSE本次J1/J3/J4约降26%/42%/38%，J5基本不变；J2回程仍不稳定且实际姿态未严格一致，不称全轴已平滑。三轮原数据/旧镜像保留，见 [当前进度](process.md)。
- 本版控制200Hz→500Hz，保留五次目标曲线，新增100ms五次PD平滑入场，主动J3降为5/0.5、J4降为3/0.3；G/零点/行程/预算/手动阻抗不变。32/32主机、16/16五轴Python、ARM/门禁通过，WAIT节拍及纯G五路反馈均实测500Hz；不代表主动路径稳定性验收。旧44AF05C0…J4范围版保留原位，原始J4范围仍−180°～350°。
- 通信实现此前通过观测/USB/Pitch零点自测、C/Python统一帧对照及隐藏GUI验证；当时平滑控制另通过32项主机、16项五轴Python、ARM/门禁，并完成部署和J4/J5实测。纯G/参考/手动支撑配置与旧44AF版逐字节一致，主动增益/周期/入场参数已变，不能再称全部控制配置不变；软件与镜像回读不替代精度/消振验收。当前阶段收尾另实跑34/34原生、42/42五轴Python、ARM增量无待编译及9EC6结构门禁，Git远端与同步以上方最新状态为准。
- 上轮J2四段及J4振动退出为旧44AF版记录；新073F版J4/J5八段174679份反馈均健康，J4实际raw约−172.20°～343.34°、J5约−77.14°～−36.32°。无ACK/失败重试；J4端点仍差5.6～6.7°，共同区间高频角波动未下降，不能宣称柔滑/精准验收。那轮未调J1；手扶800帧仅为静态通信基线，Yummy暂缺测，历史数据不删除。
- 最新纯离线分析参考 [正反扫辨识文档](../arm_learning/projects/reBotArm_control_py/docs/gravity_calibration_rs_2026-07-17.md)，联合5记录、44227个有效点分离共享重力/方向速度项/记录偏置/名义动态与残差，完成块重采样和跨记录检验。J4方向项约0.15～0.19N·m而本次条件G仅毫牛·米量级；新G在最新J45整份留出未改善，J5明显变坏，不部署新系数。完整公式/区间/限制见 [当日日档](history/2026-11-21/process.md)；实板参数未改。

详细逻辑见 [完整源码说明HTML](../output/ylamp-code-logic-20261120.html)：21节，含启动、控制周期、坐标/公式、旧J3、通信、故障与采样辨识的源码定位。

## 文件职责

| 位置 | 内容及维护边界 |
|---|---|
| [App/](App/) | 应用入口、RTOS、串口DMA及异常处理 |
| [BSP/](BSP/) | CAN、USB、UART、KEY、电源及OSPI2等板级接口 |
| [Device/](Device/) | 电机协议、BMI088、YummyIMU及遥测设备层 |
| [Model/](Model/) | 五轴几何、工作区及名义动力学 |
| [Mode/](Mode/) | 五轴轨迹、重力/阻抗支撑及独立手动拖动模式 |
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

权威参数为 [joint_references.def](config/joint_references.def)、[five_coordinates.h](config/five_coordinates.h)、[five_mit.c](config/five_mit.c)。原始反馈不归零；当前5D788沿用 [five_history.c](Task/five_history.c)连续参考：`q_reference = phase_[0,2π) + 2π*N`，raw保留用于诊断，零点/CAD不变。当前J4按用户最新“不减1圈”取N0且已存档，不是永久锁圈；raw约−213.5095°到连续+146.4905°需要加360°，这个raw映射整数+1不能混作历史N。启动历史恢复后按合法范围重选的通用优先级问题尚未重构，不能宣称任何存档分支都会无条件保留；运行中不把真实越界折回窗口。

```text
theta_i = raw_reference_rad[i] - zero_rad[i]
qCAD_i  = theta_i + CAD_offset_deg[i] * pi / 180
G2      = G_CAD2(qCAD) + [Yg(qCAD) * delta]_J2
G3      = 1.2774127431831357 * G_CAD3(qCAD) - 0.010825531721891 * cos(theta_3)
```

公式入口是 [FiveSupport_GravityModel](Mode/five_support.c)，静态残差计算在 [five_dynamics.c](Model/five_dynamics.c)。每次使用同一份五轴实际姿态，只采用新残差的J2输出；不按零点两侧、速度方向或采样段切换系数，不把固定姿态的正弦切片硬编码为通用公式。

J3此式为原完整`1.18*G_CAD3−.01*cos(theta3)`乘一次1.0825531721890982的已部署试验版，不是重复叠加摩擦，尚待独立实机效果验证。有效势能组合不是重新测得的物理质量/质心，名义M/c未更新；选定输出的支撑修正也不是一套重新辨识的完整保守刚体模型。预算在公式之后独立处理，纯G硬件/软件Kp/Kd仍为0。

手动拖动由[FiveDrag](Mode/five_drag.c)独立处理：只在J1/J2叠加部分同方向摩擦助力及正阻尼，使用实际速度，不使用位置锚点或速度PI。J3～J5逐帧保持输入G；零速不猜施力方向、不做启动冲量。拖动专用总预算J1/J2为1/4N·m，助力各≤.55N·m，原纯G/阻抗及主动预算不变。已部署5D788正常cold握手完成或显式ACK后进入DRAG，KEY在DRAG/原入场点阻抗间切换；`gravity`显式选择纯G且跨USB关闭保留。USB会话结束保留DRAG滤波器，仅运动/持姿任务取消交回正常DRAG；故障仍纯G且不自动清错/恢复，未新增自动使能。初始化只接收已使能反馈而未显式启控时仍沿用纯G，不凭在线自行助力。USB重连持续DRAG已实测，手力/全局稳定性未验收。原行程不变；端点只取消向外助力，J3～J5静态位置不成为手拖入口门槛，主动轨迹仍检查原行程。

## 上位机与控制边界

五轴历史：配置见 [five_history_config.h](config/five_history_config.h)。首次获得某轴有效位置即触发，之后任一轴相对最近成功记录变化默认150°时保存五轴快照；30°默认已取消，150°是当前可调实现值而非用户指定。参考板外部W25Q64仅使用`0x7fc000～0x7fdfff`，不动原IMU的`0x7fe000/0x7ff000`或MCU程序Flash。顺序追加64B记录、CRC与末尾提交标记、回读确认后才更新“已保存”；写中断电保留旧有效记录，后台旧扇区擦除可挂起以优先写新记录，不在控制环等待擦写完成。首次发现陌生非空分区不自动格式化。无芯片/存储故障有独立诊断，不自动使能或锁死电机；首条实机提交回读已过，但不承诺最大写入延迟或所有掉电场景。

当前版可用`python code/tools/five_control.py --port COM36 history`读取连续角、单圈相位、圈号、已保存位置与存储标志（使用原控制SESSION，不在已有轨迹期间另开）。普通USB raw遥测/CSV含义不变；它们不是连续角GUI，主动采样需用`--continuous-reference`。内部Flash链接区仍1MiB，历史使用板外芯片。最新ready/sequence5/errors0/saved31/restored31/writes_this_boot1，J4 N0、连续及存档+146.490510°；本轮未额外重启做断电保持实验，不把写后读回当所有掉电场景通过。

| 入口 | 用途 |
|---|---|
| [five_control.py](tools/five_control.py) | 五轴控制会话、状态、显式旧协议J3入口和采样协议 |
| [j3_pitch_monitor.py](tools/j3_pitch_monitor.py) / [.cmd](tools/j3_pitch_monitor.cmd) | 统一五轴反馈＋BMI088/Yummy双Pitch，两个对照图、一个CSV；开始/暂停只采集，不启停电机，不再支持--joint切换 |
| [usb_monitor.py](tools/usb_monitor.py) | USB普通五轴快照和公共解码/组包；原专用FD适配器工具已移除 |
| [five_gravity_sweep.py](tools/five_gravity_sweep.py) | 显式主动采样，同步记录全部五轴实际姿态 |
| [five_global_friction_scan.py](tools/five_global_friction_scan.py) | 冻结5D788的全角度单受载批次；连续参考、原速度PI、逐腿实际计时，默认不启控/清错 |
| [fit_pitch_zero.py](tools/fit_pitch_zero.py) | 统一CSV默认分别拟合J2/BMI与J3/Yummy，按源计数去重；兼容历史单轴CSV |
| [five_gravity_residual_report.py](tools/five_gravity_residual_report.py) | 既有数据的离线重力/摩擦分析，不授权部署 |
| [urdf_collision/](tools/urdf_collision/) | CAD/仿真/工作区验证工具，不能代替实物或碰撞验收 |

J3是旧协议，失能静默不等于通信坏；仅走唯一所有者的显式旧协议流程，不使用新版0xCC/0x33读取，不自动唤醒、ACK或复位。每boot的单次限制、独立冷启动和故障处理保持。

冷启动没有五轴角度区间准入：ColdStart、KEY、旧J3取数及ACK使用`observe(false)`，纯G也不检查记录行程。`joint_references.def`的上下限同时用于坐标分支与主动轨迹检查，不是冷启动窗口；J4的−180°～350°为用户给定原始编码器运动行程。主动Move/Speed的`observe(true)`及目标1°余量与冷启动是不同条件，不能以“删除冷启动区间”名义一并删除零点、坐标换算或主动行程。新请求尚未明确取消主动运动行程。

轻拖动入口：`five_control.py --port COM36 --seconds 120 --log <新JSONL路径> drag`，120秒内保留该控制会话，正常结束显式交回G；不要另开控制会话。op16的scope0/field251查询能力`0x594C4401`，field15/value`0x44524147`显式进入，phase新增DRAG=7，原phase取值及field7退出保持。客户端先核能力，不对旧固件盲发入口；不会自行使能或清故障。C1D0及BC90两轮300秒试用均已正常结束，参数仍保存在固件中，退G不删除拖动模型。J1手感比例不是新摩擦辨识系数，更不是J1/J2重力修正；无测力的人手拖动数据不能直接当无接触G标签。

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
| 2026-11-22 | 摩擦候选部署/五轴支撑验收 | [process.md](history/2026-11-22/process.md) |

整理前3298行README全文也保存在 [2026-11-20日档](history/2026-11-20/process.md) 的原文快照区；这是本次归档时间，不是把旧章节重新标成今天。原文SHA-256及保全核对记录一并保留，旧流程只用于历史查证。

原 [mit_20261107/](history/mit_20261107/) 保留原样。不创建第二套 `archive/`、报告或临时目录；后续详细记录只追加至 `history/YYYY-MM-DD/process.md`，根 `process.md` 保持60行以内。
