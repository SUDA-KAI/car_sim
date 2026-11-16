# 传感器自主定位接入（最新）

已导入 `stereo_20261102_135012` 双目内外参，运行独立 Domain 43 / 分区 patrol_sensors 的激光—IMU / 双目速度融合、SLAM 在线建图和 Nav2。导航未桥接真值位姿、车轮里程计或仿真 TF；理想 IMU 世界姿态也被移除。相机—雷达—外置 IMU 使用用户确认的安装测量。

39 项相关测试通过，浏览器验证通过。一次冷启动短距离目标完成，实际前进 1.014 m，融合估计位移误差约 1.15 cm；LIO 与视觉均记录到运动。未手工清图，命令经过观测有效性门控。Domain 74 的独立故障注入确认：融合预测仍新鲜但定位观测消失时，门控输出零速度。

这是短距离接入验证，不代表传感器模式整圈、打滑或实车精度通过。新地图不自动复用旧世界巡逻坐标。完整配置和限制见 `SENSOR_LOCALIZATION.md`；摘要在 `artifacts/sensor_localization_validation_summary.json`，冻结配置在 `calibration/sensor_navigation_bundle/`。下面各节保留为历史基准记录。

# 5 m 窄弯导航优化（2026-11-02）

本次完成一次完整闭环路线验证：中心线半径 5 m、净宽 1.7 m；100 kg 已装配车模、L2、双 IMU、四路超声波和概率式融合启用。前进指令上限 0.3 m/s，角速度上限 0.4 rad/s，最小转弯半径 2 m。定位仍为理想真值，Point-LIO 关闭，无界面运行；验证进程结束后关闭。

- 巡逻两段均 SUCCEEDED，整圈 178.408 仿真秒；物理轨迹长 46.888 m，覆盖左右两弯和上下直线，累计航向约 354.3°（最终朝向在 0.12 rad 容差内）。没有将动作成功直接当成实测通过。
- 巡航弯段速度中位数 0.274 m/s，所用弯段采样区域中速度低于 0.03 m/s 的比例为 0；上方直线交接仍有明显减速，最终到达停车。弯段统计使用 x > 6.5 或 x < -2.5 的区域，不能理解成无条件连续不停。
- 几何最小名义车身余量 0.246 m，保守轮胎扫掠包络余量 0.123 m；均无重叠。此为几何检验，不是接触传感器或实车防撞认证。
- MPPI 求解失败 0、倒车恢复 0、等待恢复 0、全局规划失败 0。仍记录 35 次控制周期超时警告，实时调度尚有优化空间。
- 29 项针对性测试通过；四路原始 Range 与导航适配数据各核对 64 对同时间戳消息。该次实测没有 +inf，正负无穷及 NaN 语义另由单元测试覆盖。

主要修改：物理墙体/地面/占据图/路线同步扩为 5 m；弯内 10° 切向约束点（约 0.87 m 间隔）；直线交接的两段 NavigateThroughPoses；前进 Dubins 全局搜索；4 s MPPI 预测与速度相关的局部地图/路径范围；使用 Nav2 RangeSensorLayer 而非硬圆弧障碍层。旧 `ultrasonic_fusion:=arcs` 保留为对照，原始超声波话题和 10 cm 近障提示保留。

排查中保留了失败记录：首尾重合的单次闭环请求会原地误报成功，物理行驶距离检查已将其排除；旧圆弧融合在弯中失败，临时关闭该层并保留静态墙体碰撞后，同一段成功通过；仅改概率融合而保持稀疏点仍失败，说明问题不能全部归因于 MPPI 或某一个模块。未做控制变量完整对照，不把本次综合改善归因于单一参数，也不宣称原 3 m 场景通过。

证据：`artifacts/navigation_5m_validation.json`、`artifacts/navigation_5m_dense_validation.log`、`artifacts/5m_range_adapter_live.json`、`artifacts/5m_sonar_layer_diagnostic.log`、`test_scenes/results/narrow_turns_mppi_20261102_153824.*`（含完整配置、场景元数据快照、巡逻报告、CSV、轨迹/速度/余量图）。原 3 m 场景资源备份于 `artifacts/narrow_turns_3m_before_20261102/`。

# 用户确认的第 3 版整车装配应用（2026-10-30）

已读取保存于 14:18:30 的第 3 版装配，浏览器与文件一致，12 个支架/探头本体全部确认。生成 models/assembled 下的冻结快照、9 份英文名网格、固定 TF YAML、URDF 和带雷达/双 IMU/超声波的模型 SDF。正常启动默认选择已应用版本，安装预览显示完整装配；网页草稿不会直接修改运行版本。

- 2 项新增几何/快照测试与 10 项原相关测试通过；逐项比对源网格哈希、单位、视觉偏移与角度，原车辆惯量/碰撞/关节/控制插件保持不变。URDF、SDF 及启动参数检查通过。
- 实际隔离启动：ROS_DOMAIN_ID=64、IGN_PARTITION=assembly_verify。L2 云/扫描、两路 IMU 和四路超声波均有数据；双 IMU 静止 Z 比力约 +9.8 m/s²。
- 全部 24 个静态 TF 与装配快照的完整三维变换逐项一致，包括各支架和四个本体。修正了超声波 TF 的前缀过滤，确保只跳过已有四个测距坐标，而不误跳过支架/本体帧。
- 安装预览曾成功启动；未保存界面截图（窗口在截图前已关闭）。数据验证进程已停止。没有启动真机节点或发布硬件域运动指令，也未以新版装配运行自主巡逻。
- 新增外形不增加支架质量、惯量或碰撞，原车 100 kg 动力学不变；声学 TF 沿用用户原测距定义，没有从 STL 猜测声学中心。相机仍为示意外形。URDF 提供结构/动力学，实际传感器由 SDF 与仿真启动流程提供。

证据：artifacts/applied_assembly_build.json、artifacts/applied_assembly_urdf.log、artifacts/assembled_runtime_validation.json、artifacts/assembled_runtime_verify.log、artifacts/assembled_gazebo_preview.log。

# 前超声波纵向位置确认（2026-10-30）

用户确认前探头原先用 1.5−0.11 计算，应改用 1.2−0.11，因此前探头距车尾为 **1.09 m**，保持距车头 0.11 m。前右/前左的 base_link X=0.49 m，相对第三横杠 X=0.4 m；后探头距车尾 0.14 m 不变。

同步更新超声波 JSON、测量表、实测 TF、装配参考帧及生成的 URDF/预览场景；所有其他部件和超声波外形子部件的局部调整保留。历史 1.39 m“待核对”记录在本节得到解决。文件与浏览器草稿已备份到 artifacts/front_ultrasonic_correction.json 所记目录；真机节点未重启。

# 车长 1.2 m 与横杠参考原点修正（2026-10-30）

用户确认车长以实际车身/STL 的 1.2 m 为准。第三横杠距车尾 0.69 m、距左侧 0.45 m、离地 0.495 m，故 base_link 下应为 (0.09, 0, 0.495) m；旧 -0.06 m 来自 1.5 m 车长。仅修正参考关系，未缩放车身。

- 更新安装 YAML、测量表/超声波配置的车长、装配保存记录、安装原点标示；重建传感器 URDF 和安装预览 SDF。L2 和外置 IMU 在 base_link 中分别为 (0.09,0,0.929) 与 (0.09,0,0.645) m。
- 传感器相对横杠坐标逐项保留；既有装配部件的父子关系、局部位姿、说明、确认状态和标记逐项保留。浏览器在新 revision 中恢复未保存草稿，只有横杠参考位置改变；直接以 base_link 为父的部件未挪动。
- 6 项装配后端、8 项安装表、10 项三维雷达/外形/超声波单元测试通过；组合 SDF 有效，传感器装配 URDF 通过 check_urdf。确认超声波 JSON 与安装 TF 在修正后给出同一组 base_link 坐标。浏览器显示 (90,0,495) mm，L2 世界安装位置验证一致。
- 前超声波原记录距车尾 1.39 m，按新车长会超出车头 0.19 m。未推测新探头位置或按比例缩放，需要用户在装配页面核对；保留此限制。历史探头墙距、导航和传感器联调数据使用旧位置，不能据此认定新位置再次通过动态导航测试。
- 备份包括修正前 YAML/JSON 与打开中的浏览器草稿，位置记录在 artifacts/body_origin_correction.json。本次没有改动或重启真机 ROS 节点；当时没有运行 Gazebo，因此未自行启动或重启仿真。

证据：artifacts/body_origin_correction.json、artifacts/origin_browser_validation.json、artifacts/origin_corrected_editor.png、artifacts/origin_corrected_sensors.sdf、artifacts/origin_corrected_urdf.log。下文历史验证记录保留原始坐标与结果。

# L2 三维雷达与双 IMU（2026-10-30）

正常启动默认 `use_l2:=true`，在测量外参处注入 Gazebo 三维 GPU 雷达、L2 内置 IMU 和外置 IMU。输出 `/unilidar/cloud`、`/unilidar/imu`、`/imu/data_raw`；Nav2 `/scan` 从三维点云的近水平高度窗口生成，原独立二维雷达被移除。安装 YAML、真机驱动和真机建图配置未改动。

- 3 项传感器测试及 3 项已有安装外形测试通过；覆盖带行填充/大端点云、无效距离、投影高度窗/接缝、瞬时点时间、测量外参及车辆惯量/碰撞/关节保留。启动参数解析和生成 SDF 检查通过。
- ROS 域 42、IGN_PARTITION=patrol_sim，narrow_17 的有界 Gazebo 物理测试通过。静止时两路 IMU Z 比力约 +9.8 m/s²，角速度约零，姿态含场景初始 3° 航向；内置、外置 frame 分别为 unilidar_imu、imu_link。
- 前进/转向后真值位移约 0.800 m，内置 IMU 峰值 Z 角速度约 0.0390 rad/s；停止指令已发送。实际云/投影扫描约 10 Hz（按仿真时间），双 IMU 实收约 166～168 Hz，目标设置为 200 Hz，受 2 ms 物理步长量化及消息调度影响，不把配置目标写成实测速率。
- 排除了朝上雷达看见的 Gazebo 灯光辅助图形：禁用灯光图示，并将不影响方向光照明的光源位置放到量程外。侧墙在雷达坐标中的最高返回 z≈0.585 m，与 1.5 m 墙高和 0.929 m 雷达安装高相符（GPU 栅格近似）；未用硬编码高度截断三维点云掩盖异常。
- 可选 `pointlio:=true` 已在同一域收到模拟点云及 L2 内置 IMU，发布了建图点云/里程计。修正可靠性 QoS 兼容问题。测试中存在数秒仿真时间的处理滞后，不宣称 LIO 精度、实时性或闭环定位通过；Nav2 的定位来源仍沿用场景设置。

最终已重新启动带三维雷达投影的 Nav2 MPPI 自动窄道巡逻（不启用可选 Point-LIO）。导航激活后，观测位置从 x≈-2.296 m 到 -1.993 m，实测速率峰值约 0.196 m/s，雷达/双 IMU 同时持续发布。证据 `artifacts/l2_nav2_motion.json`、`artifacts/l2_nav2_patrol.log` 和 `test_scenes/results/narrow_17_mppi_20261030_111203.run.json`。这是启动与运动联调，不代表完整往返、窄弯或坡道已经通过；Gazebo 保持运行。

证据：`artifacts/l2_imu_live_validation.json`、`artifacts/l2_imu_live_validation.log`、`artifacts/l2_imu_verified.log`、`artifacts/l2_sensor_check.sdf`。第一次动态检查因灯光图示产生假回波而失败，原记录保留在 `artifacts/l2_imu_before_light_fix.json`；修正后通过。

模型局限：规则上半球 360×19 射线、零噪声 IMU；非真实 L2 扫描轨迹或硬件噪声复刻。所有点 time=0，表示同一时刻的快照，无真实逐点运动畸变。坡面感知、低矮障碍盲区、真实时间同步/IMU 漂移及完整导航性能另需验证。相机不参与本次流程。

# 四路超声波接入（2026-10-29）

默认读取 `/home/a/ros2_ws/test/ultrasonic_settings.json`，按左后角坐标转换为居中的 base_link，用户确认安装高度 0.49 m。顺序为后右、前右、前左、后左；x=-0.61/0.64 m，y=±0.475 m。按 JSON 的 1.5 m 长度进行坐标换算，现有 1.2 m STL 不缩放，车长差异仍需确认。

Gazebo 每路以 61 条水平射线、10 Hz 生成理想扇形测距，发布标准 Range 和四路安装 TF。独立圆弧投影节点保留超声波方位不确定性，通过局部 ObstacleLayer 标记/清除；每个观测源显式配置高度上下限，避免 Humble 的单源默认高度过滤。无回波只清空，异常扫描不作清空证据。未模拟真实声学、多径、材质、串扰、延迟、噪声或三维锥体。

隔离 ROS 域 64、IGN_PARTITION=ultrasonic_verify 的实际集成测试通过：

- 四路墙距与几何预测误差小于 0.1 mm（本次理想仿真），TF 高度均为 0.49 m。
- 前右临时障碍使读数从 0.50916 m 变为 0.19501 m；局部地图对应圆弧区域出现 254（障碍），移走后恢复 253（该区域已有静态墙膨胀代价，不能误判成动态障碍）。
- 移除右墙后通道 1、2 返回正无穷无回波，恢复右墙后恢复原测距；测试障碍与墙均已还原。
- 4 项新增单元测试、2 项控制器配置测试、7 项场景测试通过；Python 语法检查与带探头 SDF 校验通过。

证据：`artifacts/ultrasonic_live_validation.json`、`artifacts/ultrasonic_live_test.log`、`artifacts/ultrasonic_unit_tests.log`、`artifacts/ultrasonic_integration.log`、`test_scenes/results/narrow_17_mppi_20261029_155003.run.json` / `.nav.yaml`。此次没有运行自主巡逻，不宣称窄弯问题已解决。新增障碍层后的整圈、坡道、低摩擦仍需另测；历史无超声波基线可用 `use_ultrasonic:=false` 对照。没有开启/重启真实串口驱动，没有更改实车 TF。

# MPC / MPPI 接入（2026-10-29）

新增 `controller:=mppi`（Nav2 MPPI，Ackermann 模型），`controller:=teb` 可对照，默认 scene 保留已有参数。速度限制和曲率限制继承所选场景；MPPI 使用轮胎扫掠包络规划、完整轮廓碰撞评分，移除低角速度死区。每次自动测试额外记录解析后的导航参数和插件名。

2 项配置切换/约束测试和 7 项原场景测试通过，Python/Bash 语法检查通过。无界面窄弯实测：前 3 个航点成功，第 3 点耗时 108.874 仿真秒；第 4 点期间主动结束。存在低速反复修正，不宣称整圈、坡道或低摩擦测试已通过；多处配置同时改变，不是算法单因素比较。阶段性实测包络余量见 `test_scenes/mppi_validation.json` 与对应 CSV/clearance 图；不是实际接触检测。原始巡逻快照在关闭时仍为 running，单独验证报告记录 partial_validation_stopped。测试进程已停止。

# 全套窄道地形（2026-10-27）

新增 narrow_turns、narrow_ramp_6deg、narrow_ramp_12deg、narrow_slip_normal、narrow_slip_low、narrow_slip_ice。常用启动名 turns/ramp_*/slip_* 默认映射到窄版，旧宽版通过 wide_* 显式选择。

- 通道净宽均为1.7米；坡面和平台宽1.7米，连续侧墙禁止宽路绕行。弯道中心线半径3米，内外边界半径2.15/3.85米；曲面网格宽度误差小于1毫米。车身不缩放。
- 7项场景/几何测试通过，含全套导航图/航点一致性、坡面宽度、弯道径向净宽、三维姿态外包络测距和摩擦区无底板重叠；8个窄道SDF检查有效。
- 12°窄坡实际巡逻4/4航点成功，真值高度约0.85米、俯仰约12°。
- 窄弯实际巡逻第3个航点失败：偏向内侧，中心线偏差最大约0.378米，名义包络最小余量约0.022米，规划器报告 Starting point in lethal space。保留失败；不宣称窄弯导航已通过。
- 6°窄坡及窄摩擦三组本次仅完成生成/几何和参数检查，不以旧宽版结果冒充新窄版通过。

证据：`test_scenes/narrow_suite_validation.json`、`artifacts/narrow_suite_tests.log`、`artifacts/narrow_turns_verify.log`、`artifacts/narrow_ramp12_verify.log`、对应CSV/JSON和.clearance.png。新增测距使用完整三维车体姿态投影，弯道使用曲线内外边界，不能沿用直线y=0距离。结果仍为理想真值定位基准，不是实车通过保证或接触传感器碰撞检测。

# 1.7 米窄道测试（2026-10-27）

新增 narrow_17_baseline 原参数对照和 narrow_17 低速窄道版。墙内表面 y=±0.85，净宽 1.70 m，地图 2 cm 分辨率对应 85 个空闲栅格；车宽保留 0.9 m。初始中心偏移 10 cm、车头偏角 3°，先前进 10 m 再倒车返回。

- 两个 SDF 验证有效；5 项场景测试通过，新增物理/栅格净宽与旋转车身包络余量检查。
- 两版本均完成全部 2 个往返航点。
- 稳态前进段最大横向偏差：原参数 0.1147 m、低速版 0.0509 m；全程最大偏差分别为 0.1147 m 和 0.1002 m（低速版全程值由初始偏移主导）。
- 保守计入最大轮角扫掠的估计最小侧墙余量：原参数约 0.1137 m、低速版约 0.1464 m。此为采样几何估算，不是接触传感器数据。
- 低速版指令正负翻转更多（152 对 19），虽然横向摆幅减小，仍不能宣称消除纠偏振荡或达到实车安全要求。
- 修正了新窄道版 Gazebo 公共限幅的左右不对称，并修改 TEB/平滑器参数。未分离单项影响，不把该配置问题认定为全部摆动的唯一原因。

详见 `test_scenes/NARROW_CORRIDOR.md`、`test_scenes/narrow_comparison.json`、两组 `.clearance.json/.png` 及 `artifacts/narrow_baseline.log`、`artifacts/narrow_conservative.log`。Python/Bash 语法检查通过。

# 简单场地与地形测试（2026-10-27）

新增 turns、ramp_6deg、ramp_12deg、slip_normal、slip_low、slip_ice 六个三维场景及二维导航图、航点、自动/手柄入口。默认理想三维真值定位和静态已知地形图；轮里程计单独保留，避免空转伪造通过。

- 6 个场景 SDF 均经 ign sdf -k 检查有效；4 项场景测试通过（地图/出生点/航点一致性、坡度与网格法线、摩擦区域无重叠底板、规划路径距离计算）。
- 无界面导航实测：turns 6/6、6°坡道 4/4、12°坡道 4/4 航点完成。
- 三维真值验证：6°坡道最高约 0.420 m、俯仰约 6°；12°坡道最高约 0.852 m、俯仰约 12°。
- 普通与极低摩擦路面用相同加速/刹车/转向序列测试；3 秒刹车窗口实际位移约 0.367 m 对 1.641 m，极低摩擦组轮速差显著更大。物理表面设置生效。
- 逐航点 JSON、CSV 和指标汇总/绘图入口已验证。Python/Bash 语法检查通过。

证据和边界：`test_scenes/validation_results.json`、`test_scenes/results/`、`artifacts/test_turns.log`、`artifacts/test_ramp6.log`、`artifacts/test_ramp12.log`、`artifacts/normal_physics_profile.json`、`artifacts/ice_physics_profile.json`。尚未验证全部摩擦场景的自主巡逻完成率、AMCL 抗打滑能力、实际三维定位、悬挂/轮胎/电机极限或实车坡地能力。

# 真实双摇杆遥控确认（2026-10-27）

用户实际操作飞智冰原狼 4 后，已确认 LB=4、左摇杆纵轴=1、右摇杆横轴=3。读取到实际前进指令 0.5 m/s 和非零转向指令（例如 0.3458 rad/s），Gazebo 里程计测到前进 0.5 m/s、横摆约 0.1738 rad/s，航向持续变化。整个检查未注入模拟 Joy 消息。GUI 与遥控节点现已重新启动并保持运行。证据：`artifacts/real_dualstick_drive.log`。

# 双摇杆映射更新（2026-10-27）

真实遥控时发现用户前后操作左摇杆、左右操作右摇杆。旧配置只读取左摇杆横轴，因此右摇杆横轴变化时 angular.z 保持零。现将转向改为右摇杆横轴（axis_steering=3），速度继续读取左摇杆纵轴（axis_speed=1），LB=4 不变。补充左右摇杆分工测试后，6 项遥控测试全部通过。已重启 GUI 仿真应用新配置。

# 真实手柄接入（2026-10-27）

用户执行驱动安装后，已确认 xpad 与 joydev 加载，接收器 USB 045e:028e，输入设备 event1/js0，当前普通用户可访问。ROS_DOMAIN_ID=42 的 joy_node 已收到真实 8 轴、11 按键 Joy 消息，初始回中无按键，遥控节点进入 ready_hold_enable 状态。GUI 手柄仿真已启动，Nav2 未启动。随后观察到真实左右摇杆 axes 0/1/3/4 均覆盖 [-1,1]；尚未按下任何按钮时 cmd_vel 始终为零，车辆保持静止。实际 LB 操作和断线恢复仍待验证。记录见 `artifacts/real_joystick_observe.log`。

证据：`artifacts/real_joy_initial.log`。下方保留之前尚未安装驱动时的排查记录。

# 手柄遥控更新（2026-10-27）

- 新增手柄专用启动入口 `run_teleop_sim.sh`，手柄模式不启动 Nav2。
- 5 项遥控逻辑测试通过，覆盖启动回中/松键、阿克曼前进倒车、无原地旋转、松键与超时停止、超时重连、非法消息及配置。
- 使用隔离 ROS_DOMAIN_ID=50、IGN_PARTITION=patrol_joy_test 无界面验证。模拟 Joy 消息经实际遥控节点和桥接器驱动 Gazebo，车辆移动 0.6626 米、航向改变 0.0539 rad。松键和输入停止后 cmd_vel 归零，超时后不能持键直接重启。
- 硬件未连通：当前 USB/Linux/SDL 未枚举出手柄，通信域 0、42 初始没有 Joy 数据。真实手柄的型号、按键映射及掉线行为等待设备识别后验证。
- 证据：`artifacts/joystick_tests.log`、`artifacts/joystick_sim_check.log`。Python/Bash 语法检查通过。

# STL 车身与滤波地图更新（2026-10-26）

默认配置已切换到滤波地图及 100 kg STL 模型。本节为当前版本验证，后面的记录保留为第一版历史。

- 6 项测试通过，包括均质长方体解析惯量核对、非封闭网格拒绝、URDF 各 link 合计质量/质心/惯量与 STL 实体积分一致、孤立噪点清理、薄墙与边界墙保留、小缺口闭合和未知区域保护。
- `car_body.urdf`、`patrol_car.urdf` 经 check_urdf 解析通过；可行驶 SDF 和滤波世界经 ign sdf -k 检查通过。
- Gazebo Fortress 无界面实测：100 kg 车模在滤波地图中完成全部 4 个航点，前进约 2 米并倒车返回，进程退出码 0。记录在 `artifacts/stl_filtered_patrol.log`。
- 激光、时钟、里程计、TF 接通；新巡逻验证期间 TEB 短暂启用摆动恢复后成功完成，未发生导航失败。
- Python 与 Bash 语法检查通过。

边界：STL 中的轮子仍是刚性视觉的一部分；物理轮采用独立圆柱碰撞。车体质量约 100 kg 由用户提供；均匀密度为用户指定假设，转向机构与轮胎参数未做实车标定。滤波可能去掉真实小障碍，原始地图与差异预览保留。地图是固定高度挤出，不是坡地重建。

# 验证记录（2026-10-26）

环境：ARM64，ROS 2 Humble，Gazebo Fortress 6.18.0；无界面运行。

- 地图转换单元测试 3 项通过：随机占据区域无损合并；大型地图网格顶点和三角面保持障碍物形状；地图原点、旋转、上下翻转和出生点验证。
- Python 语法与 Bash 语法检查通过。
- 演示场景：收到 `/clock`、`/chassis/odom`、`/tf`、`/scan`；激光包含 720 条有效射线，frame 为 `laser_frame`；里程计为 `odom → base_link`。
- 演示场景：Nav2 所有受管节点激活；连续到达两个直线目标点和两个转向目标点，详见 `artifacts/demo_straight.log`、`artifacts/demo_turn.log`。
- 实际地图：Gazebo、激光、里程计、TF、AMCL 和 Nav2 启动通过；完成全部 4 个航点的 2 米前进及倒车返回，巡逻进程退出码 0，见 `artifacts/actual_patrol.log`。
- 实际地图：已按 `map_20260817_154310.yaml` 的 0.02 米分辨率生成 4049 个障碍块并合并为带法线的单一静态碰撞网格；出生位姿 `(0.4164, 12.0958, -0.52)`。

未验证：实车动力学一致性、完整场地绕行、坡地、机械臂作业、实车 Point-LIO/GICP 定位性能。当前实景是二维固定墙高挤出，并非点云地形曲面。
