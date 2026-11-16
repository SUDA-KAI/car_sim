## 导航服务器仿真包

运行 `./pack_server.sh` 可生成 `dist/gz_patrol_nav.tar.gz` 及 SHA256 校验文件，包含车辆模型、地图、测试场景、L2/D6/A22 仿真、AMCL 多雷达融合、Nav2 MPPI，以及指定历史会话的聊天快照。重新打包前，可运行 `python3 scripts/export_navigation_chat.py` 刷新聊天快照。服务器上解压后运行 `./run_server_nav.sh narrow_turns`，默认无界面且不自动发送导航目标。部署依赖与仿真边界见 [SERVER_NAV_README.md](SERVER_NAV_README.md)。原有强化学习封装使用 `./pack_rl_server.sh`，见 [rl/README.md](rl/README.md)。

2026-11-06 的实车状态已整理为[仿真模拟方案报告](../docs/reports/2026-11-06_simulation_plan/仿真模拟方案报告.md)。新增 `./run_server_nav.sh recording_20261106_163030` 可加载最新录制地图的二维诊断场景；默认服务器入口启用独立的仿真 Ackermann 适配器和四路 A22 数据有效性门槛。新图包含历史轨迹强制清空区域，场景并非实测碰撞真值。

要在传感器定位下自动绕当前窄通道一圈，无需 RViz：

```bash
cd /home/a/ros2_ws/gz_sim
./run_sensor_lap.sh
```

已有传感器仿真则用 `./run_sensor_lap.sh --attach`。监控页 http://127.0.0.1:8769 显示巡逻状态；详细行为与限制见 [传感器导航说明](SENSOR_LOCALIZATION.md)。

仅启动传感器定位和导航服务、不自动发送目标：`./run_sensor_navigation.sh`；手工选点调试时可加 `rviz:=true`。当前采用 L2 平面扫描/IMU 里程计、双目速度融合与在线建图；查看页为 http://127.0.0.1:8769。启动后在新地图中设置导航目标，不自动使用原世界坐标的巡逻路线。完整操作、标定来源和验证范围见 [SENSOR_LOCALIZATION.md](SENSOR_LOCALIZATION.md)。原测试入口保留为独立真值基准。

# 小车 Gazebo 巡逻仿真

## 前轮先转、后轮再驱动

仿真车使用独立的前轮转向关节和后轮驱动关节。导航和手柄向 `/cmd_vel` 发速度指令；传感器定位模式先经过 `/localization/safe_cmd_vel`。底盘按实车 STM32 报告的 **0.90 m 轴距**计算左右前轮目标角；0.80 m 轮距与 0.20 m 轮半径仍是 CAD 仿真值。读取转向关节反馈，误差低于 0.035 rad 后才向后轮发速度；较大的转向变化会先停后轮，细小行驶纠偏保持连续。命令超时 0.30 s 后轮停车、前轮保持角度。`/sim/ackermann_cmd` 接受独立的速度和转角，支持停车打角；传感器定位/安全门控模式禁用该直接入口。

运行限速：前进 **0.5 m/s**、后退 **0.2 m/s**；虚拟中心轮转角 **±40°**，对应阿克曼内侧前轮最大约 **53.23°**。STM32 报告的转向硬限位为 ±45°，仿真仍采用更保守的中心轮运行限位。Nav2 默认前进上限 0.5 m/s，可通过 `max_speed:=` 降低。参数见 [`config/chassis_sim.yaml`](config/chassis_sim.yaml)。

两台低位 D6 默认启用：前部 `d6_b_link` 位于 `(0.410, 0, 0.320)`，后部 `d6_a_link` 位于 `(-0.600, 0, 0.320)`；各发布 10 Hz、720 格的 `/d6_b/scan`、`/d6_a/scan` 和对应 `/points`。车体自遮挡后的点云进入 Nav2 局部代价地图，使用 `marking: true`、`clearing: false`。可用 `use_d6:=false` 关闭。角度是现有纸板方向校准值；仿真为理想反射模型，不能验证玻璃误反射。参数见 [`config/d6_sim.yaml`](config/d6_sim.yaml)。

入口脚本不变。实现位于 `scripts/split_drive.py` 和 `scripts/split_drive_world.py`。`/wheel/odom` 由后轮关节转动积分得到，传感器定位模式不发布轮式里程计和轮式 TF。2026-11-03 已实测静止打角后前进、反向打角后倒车；整圈导航仍需另行验证。

> 服务器 PPO/Gymnasium 封装、Docker 和多实例运行方法见 [`rl/README.md`](rl/README.md)。地图、车模和测试场景均已改为相对资源路径，可以整体复制到其他目录或服务器。

ROS 2 Humble + Gazebo Fortress（`ign gazebo`）。默认加载 **map_20260817_154310 的滤波副本**，使用 `车身.STL` 构建的 100 kg 均质模型。ROS_DOMAIN_ID 默认 42，IGN_PARTITION 默认 patrol_sim；不会启动实车硬件。

## 启动

### 网页可视化工作台

```bash
./run_web_console.sh --open
```

打开 **http://127.0.0.1:8770**，在网页中选择地图、启动 Gazebo / Nav2、点击设置导航目标，并查看小车位姿、规划路径、激光与运行日志。支持导入 YAML + PGM / PNG 自动生成仿真场景，适配桌面和手机。控制台默认使用独立的 ROS 域 **73**，与下面的终端入口分开。说明见 [web_console/README.md](web_console/README.md)。

### 原有终端入口

终端一：

```bash
cd /home/a/ros2_ws/gz_sim
./run_sim.sh
# 无界面：./run_sim.sh gui:=false
```

等待约半分钟完成加载和 Nav2 激活，在终端二执行：

```bash
cd /home/a/ros2_ws/gz_sim
./run_patrol.sh --loops 1
# 循环巡逻：./run_patrol.sh --loops 0
```

Ctrl+C 停止。巡逻失败或每点超过 180 秒仿真时间会停止。默认路线在上方通道内前进约 2 米、倒车返回，共 4 个航点；路线在 `config/patrol_map_20260817_154310_filtered.yaml`。完整场地路线仍需继续选点验证。

## 简单地形与自动巡逻测试

转弯、坡道和摩擦测试入口已统一默认选择1.7米净宽场景：`narrow_turns`、`narrow_ramp_6deg`、`narrow_ramp_12deg`、`narrow_slip_normal`、`narrow_slip_low`、`narrow_slip_ice`。原简写 `turns`、`ramp_6deg` 等也会自动映射到对应窄道版。

```bash
./run_test_scene.sh narrow_turns auto
# 或 ./run_test_scene.sh narrow_ramp_6deg auto
# 手柄试坡：./run_test_scene.sh narrow_ramp_6deg manual
```

`auto` 一次启动场景、导航、巡逻路线和结果记录。默认采用独立三维真值定位作为车辆/控制基准，不代表实车定位或抗打滑控制已经通过验证。场景总览、参数、AMCL 对照和结果说明见 `test_scenes/README.md`。

### 现场宽度 1.7 米的直通道

```bash
./run_test_scene.sh narrow_17 auto
# 原参数对照：./run_test_scene.sh narrow_17_baseline auto
```

按真实 0.9 米车宽构建，不缩小模型；低速前进后倒车返回。会统计横向偏差和含车头摆角、轮胎扫掠的侧墙余量。说明见 `test_scenes/NARROW_CORRIDOR.md`。

## 四路超声波仿真

启动仿真时默认加载 `config/ultrasonic_settings.json` 的四路安装位置，高度默认 **0.49 m**。该文件从实车设置同步，不含串口地址与波特率。用户现已确认实际车长 **1.2 m**、宽 0.9 m，配置与 STL 已统一；只修正坐标换算，不缩放模型。前探头按用户确认的距车头 0.11 m 换算为距车尾 **1.09 m**（1.2−0.11），后探头距车尾 0.14 m。

| 通道 | 位置 | base_link x/y/z（m） | 朝向 |
|---|---|---|---|
| ultrasonic_1 | 后右 | -0.46 / -0.475 / 0.49 | 右 |
| ultrasonic_2 | 前右 | 0.49 / -0.475 / 0.49 | 右 |
| ultrasonic_3 | 前左 | 0.49 / 0.475 / 0.49 | 左 |
| ultrasonic_4 | 后左 | -0.46 / 0.475 / 0.49 | 左 |

发布 `/ultrasonic_1`～`/ultrasonic_4`（`sensor_msgs/Range`）和 `base_link → ultrasonic_1..4` 静态 TF。独立的 `ultrasonic_projection.py` 将有限回波投影为 31 个点组成的不确定方位圆弧；无回波只发布清空射线，不标记障碍，异常数据不作为清空证据。Nav2 局部地图在膨胀层前加载 `ultrasonic_layer`（ObstacleLayer），分别消费每路 `/ultrasonic_N/obstacles` 与 `/ultrasonic_N/clearing` 点云。四个探头均朝侧面，不能据此认为车头、车尾中央被覆盖。

没有直接沿用 Humble 的 RangeSensorLayer：在这组近距离回波/2 cm 网格下，测试未稳定得到障碍标记；其 VARIABLE 分支还会过滤 `+inf`，不能直接依赖它清除无回波区域。圆弧适配保留距离测量的角度歧义，避免把回波误当成正前方的精确单点；代价是可能比真实障碍更宽，会影响窄道通过，需要继续做导航验证。它不是接触传感器或停车保证。

```bash
./run_test_scene.sh turns auto controller:=mppi
# 手柄也会带四路测距
./run_test_scene.sh narrow_17 manual
# 与历史无超声波基线比较
./run_test_scene.sh turns auto controller:=mppi use_ultrasonic:=false
# 安装参数或高度覆盖
./run_sim.sh ultrasonic_settings:=/absolute/settings.json ultrasonic_height:=0.49
```

仿真使用 10 Hz、61 条水平射线的理想扇形测距，取最近回波生成 Range；按 JSON 使用 60° 扇角、0.55 m 最大范围，最小范围暂沿用驱动默认 0.03 m。JSON 中角度/量程是原工具的显示配置，必须再核对物理探头设置，不能据此认定硬件探测能力。该模型未模拟三维声束、材质反射、多径、串扰、硬件延迟或噪声。无回波为 `+inf`，含 NaN 的扫描不作为清空证据；没有新扫描时不重复发布旧测距。

仿真保持 ROS 域隔离，不打开真实串口，也不把实车周围的距离注入虚拟地图。实际驱动的话题名与此一致，但实车旧安装 TF 配置仍需替换，不能与本仿真 TF 混用。超声波接入不代表此前窄弯控制问题已解决，历史巡逻结果也不自动适用于增加 Range 层后的配置。

## 手柄遥控仿真

```bash
cd /home/a/ros2_ws/gz_sim
./run_teleop_sim.sh
# 无界面：./run_teleop_sim.sh gui:=false
```

此入口启动当前地图、车模、ROS/Gazebo 桥、`joy_node` 和阿克曼遥控节点。也可使用 `./run_sim.sh teleop:=true`。手柄模式自动关闭 Nav2，不运行巡逻脚本；从自主巡逻切换时先在原仿真终端 Ctrl+C，再启动此入口。

默认操作（飞智冰原狼 4，Xbox 兼容模式）：

1. 先松开使能键，把左摇杆回中。
2. 持续按住按钮索引 4（常见 Xbox 布局为 LB/左肩键）。
3. 左摇杆前后控制前进/倒退，右摇杆左右控制前轮转向。最大前进 0.5 m/s、倒车 0.2 m/s，最大方向盘等效前轮角 0.5 rad。
4. 松开使能键立即发送零速度；没有新 `/joy` 超过 0.35 秒时发送零速度。断线重连后需再次松键、回中。停车指令发出后，物理车模可能有短暂减速过程。

遥控按阿克曼关系 `angular.z = linear.x * tan(steering_angle) / 0.90` 发出 yaw rate，倒车时横摆方向自动反向；不能原地旋转。看门狗使用真实单调时钟，不依赖仿真时钟。

轴/按键配置在 `config/joystick.yaml`：`axis_speed: 1`、`axis_steering: 3`、`enable_button: 4`。实际手柄如有不同可调整索引；方向相反时将对应的 `speed_sign` 或 `steering_sign` 改为 `-1.0`。SDL 的 device_id 是枚举序号，不等同于 `/dev/input/jsN`；有多个设备时可设置 `device_name` 精确选择。

查看手柄设备和实时 ROS 信号（先启动遥控仿真）：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
ros2 run joy joy_enumerate_devices
ros2 topic echo /joy
# 另一个终端可观察：ros2 topic echo /sim/teleop_status
# 或：ros2 topic echo /cmd_vel
```

2026-10-27 已完成驱动安装：USB `045e:028e`（Xbox 360 兼容模式），`xpad` 已加载，输入设备为 `/dev/input/event1` 和 `/dev/input/js0`，普通用户有读取权限；已通过 ROS_DOMAIN_ID=42 收到真实 `/joy` 数据（8 轴、11 按键）。当前机器已装好驱动。以下命令仅供以后同内核重新安装时使用：

```bash
sudo bash /home/a/ros2_ws/gz_sim/drivers/xpad/install_driver.sh
```

详细来源与系统修改范围见 `drivers/xpad/README.md`。安装已验证成功；正常使用直接运行 `./run_teleop_sim.sh` 即可。

如果使用自定义 `SIM_ROS_DOMAIN_ID`，这里的 ROS_DOMAIN_ID 也要改成同一值。`/joy` 话题被创建不代表已有手柄数据，必须看到持续到达的 axes/buttons 才能确认接收成功。

手柄型号：飞智冰原狼 4。接收器档位下按 LOGO 键开机；无法回连时按官方说明长按 FN+LOGO，再拔插接收器。真实 ROS 输入及实际驾驶已验证：LB=4、左摇杆纵轴=1、右摇杆横轴=3，车辆能够前进并转向；无线掉线恢复尚未做物理断线测试。

软件验证已通过：6 项遥控测试，以及模拟 Joy → cmd_vel → Gazebo 的运动/转向、使能松开、超时停车和重连回中验证，记录在 `artifacts/joystick_tests.log`、`artifacts/joystick_sim_check.log`。真实手柄初始数据另存于 `artifacts/real_joy_initial.log`。驱动参数及轴符号参考 [ROS joy 官方文档](https://github.com/ros-drivers/joystick_drivers/blob/ros2/joy/README.md)。

## 地图滤波

原始地图保留在 `src/robot_core/robot_bringup/maps/`，滤波结果在：

- `maps/map_20260817_154310_filtered.yaml` 和 `.pgm`：给 Nav2 使用。
- 同名 `.png`：原始、滤波后及变更区域对比；蓝色是删除、红色是补入的占据像素。
- 同名 `.json`：滤波参数和统计。
- `worlds/map_20260817_154310_filtered.sdf` / `.obj` / `.json`：对应碰撞场景及出生位姿。

处理按地图物理尺寸进行：删除面积小于 0.004 m²（10 像素）的孤立占据区域，用半径 0.04 米的椭圆结构元做闭运算，填补厘米级墙面缺口；不会做整幅高斯模糊或开运算来侵蚀细墙。此版本去掉 231 个小区域（467 个占据像素），补入 1486 个像素。小于阈值的实际小障碍也可能被删除，保留源图便于比对和调整。

源图用灰度 205 表示未扫描背景，但源 YAML 的 `free_thresh: 0.25` 会把这些像素判为空闲。滤波副本明确保留灰度 205 为未知，并将 `free_thresh` 设为 0.196；未知区域不会被涂成道路，Nav2 不在未知区域规划。原始 YAML 未修改。

生成命令：

```bash
/usr/bin/python3 scripts/filter_map.py --radius 0.04 --min-area 0.004
/usr/bin/python3 scripts/build_world.py \
  --map maps/map_20260817_154310_filtered.yaml \
  --output worlds/map_20260817_154310_filtered.sdf \
  --spawn 0.4164 12.0758 -0.52 --height 1.5
```

地图保留原点、旋转和 0.02 米分辨率。滤波后障碍由 3043 个合并矩形挤出并烘焙为单一带法线 OBJ，供显示及碰撞共用。墙高仍是人为设定的 1.5 米；没有重建真实坡度和地面起伏。SDF、OBJ 和 JSON 都需保留。

## STL、URDF 和质量参数

原始文件 `车身.STL` 保留。它是封闭、面方向一致的整体网格，包含车架和四个轮子，尺寸为 **1200 × 495 × 900**（原始 X/Y/Z）。按毫米转米并采用原始 Y 轴向上，转换为 ROS 的 X 前、Y 左、Z 上：

```
x = X / 1000 - 0.6
y = -Z / 1000 + 0.45
z = Y / 1000
```

模型长宽高为 1.2 × 0.9 × 0.495 米。假定 X+ 为前进方向；前后朝向尚未由用户确认。

- `models/meshes/car_body_m.stl`：已变换到米制 ROS 坐标系的网格，无需再次缩放。
- `models/car_body.urdf`：独立的均质车身 URDF，质量 **100 kg**；视觉使用 STL，碰撞采用整体包围盒，适合模型查看或作为车体资源。
- `models/patrol_car.urdf`：可行驶版本，含阿克曼转向关节、滚动轮关节、雷达和 Gazebo 插件。它使用简化车架碰撞盒和四个圆柱轮碰撞体，避免把整份动态 STL 当成凹网格碰撞。
- `models/patrol_car.sdf`：从上述 URDF 通过 `ign sdf -p` 生成，世界加载此文件的模型内容。
- `models/car_properties.json`：质量、体积、质心、惯量、轴距及坐标变换的完整记录。

按封闭三角网格的实体体积积分计算均匀密度的质量属性（保留车架孔洞，并非包围盒估算）：体积约 0.085485 m³，100 kg 对应均匀密度约 1169.8 kg/m³；质心约 `[-0.017115, 0, 0.258350]` 米。质心坐标系与 base_link 平行，惯量单位 kg·m²：

```
[13.082271,  0,        -0.052108]
[ 0,       15.681771,  0       ]
[-0.052108, 0,        25.633599]
```

可行驶版本将少量质量分配给辅助轮子和转向节，并从 base_link 的空间惯量中扣除，所以全部物理 link 的合计质量、质心和惯量仍与 100 kg 均质 STL 一致。该假设用于仿真，不是实车称重或材料分布测量。

STL 是一个整体实体，没有独立可动轮网格，因此视觉中的原始轮子仍与车架刚性连接；物理滚动和转向由不重复显示的圆柱轮完成。要让外观轮子也随关节转动，需要从 CAD 单独导出车架和四个轮子。

底盘轴距已按实车报告值更新为 0.90 米；轮距 0.8 米、轮半径 0.2 米仍取 CAD 仿真值。雷达仍为离地 0.85 米的简化传感器。

修改质量并重建：

```bash
/usr/bin/python3 scripts/build_car.py --mass 100
# 随后重新运行上面的 build_world.py，将新模型写入场景。
```

## 导航与调试

Gazebo → `/chassis/odom`、`odom → base_link` 和 `/scan`；AMCL → `map → odom`。所有节点使用 `/clock`。复用原车 Smac Hybrid、TEB 和禁止原地旋转的行为树；配置副本为 `config/nav2_sim.yaml`，原有实车导航配置不修改。AMCL 替代实车 Point-LIO/GICP，二维扫描替代实车点云障碍层。

原始未滤波场景仍可单独加载，配合原始路线：

```bash
./run_sim.sh world:="$PWD/worlds/map_20260817_154310.sdf"
./run_patrol.sh --route config/patrol_map_20260817_154310.yaml
```

该原始场景保留第一版简化车模。人工演示地图为 `worlds/demo.sdf`，搭配 `config/patrol_demo.yaml`。`make_demo.py` 只重建演示车模，不覆盖 STL 生成的巡逻车模。

验证：`/usr/bin/python3 -m unittest discover -s tests -v`。运行记录见 `VALIDATION.md` 和 `artifacts/`。
# 可选 MPC 控制

自动测试可以加 `controller:=mppi` 切换到 Nav2 MPPI（采样式 MPC），例如 `./run_test_scene.sh turns auto controller:=mppi`。不指定时保留原 TEB。原理和参数见 [MPC.md](MPC.md)。

## 车身长度与新测量原点

2026-10-30 已按用户确认统一车长为 **1.2 m**，保持车宽 0.9 m。第三根横杠距车尾 0.69 m、左侧 0.45 m、离地 0.495 m，因此相对车身地面中心 `base_link` 为 **(0.09, 0, 0.495) m**。此前按 1.5 m 计算得到 -0.06 m，现已修正。传感器相对横杠的测量和直接相对车身摆放的支架不变；原点坐标轴及网页页脚从安装表动态读取。修正前 JSON/YAML 与浏览器草稿均已备份，见 `artifacts/body_origin_correction.json`。真机运行节点未改动，历史导航结果不自动代表新安装坐标下的验证结果。

## 已确认整车装配（第 3 版）

已将页面保存并确认的支架和四个超声波本体装入 Gazebo，正常启动默认加载 `models/assembled/assembly_snapshot.json`。完整外形检查用 `./run_sensor_preview.sh`，正常仿真仍用 `./run_sim.sh` 或测试入口。后续修改页面草稿后，保存、确认，再运行 `./build_assembly.sh` 并重启仿真。旧版可用 `use_assembly:=false` 对照。

生成的 URDF、SDF、固定坐标和网格在 `models/assembled/`。只新增视觉装配，原车 100 kg 质量/惯量/碰撞未改变；新增支架质量与声学原点精确标定需另行确认。说明与验证见 [assembly/README.md](assembly/README.md)。

## 支架三维装配页面

新增本地交互装配台：[http://127.0.0.1:8767](http://127.0.0.1:8767)。用 `./run_assembly_editor.sh` 启动，可拖动/旋转四份真实支架 STL 和四路超声波支架/探头本体、指定父部件、标记表面和填写安装说明，保存到 `assembly/assembly_settings.json`。装配草稿独立保存，确认安装关系后再接入运行中的仿真。操作与导出说明见 [assembly/README.md](assembly/README.md)。

## L2 三维雷达与双 IMU 仿真

正常启动和测试入口默认启用 `use_l2:=true`，不依赖摄像头。直接在 Gazebo 的 GPU 射线和 IMU 传感器上生成观测，不从真值里程计伪造测量。安装位置读取 `sensor_mounting_tf.yaml`：L2 底部/点云原点位于 base_link 的 `(0.09, 0, 0.929)` m，外置 IMU 位于 `(0.09, 0, 0.645)` m。内置 IMU 与外置 `imu_link` 分开，内置 IMU 相对雷达的偏移沿用 [Unitree 官方坐标定义](https://github.com/unitreerobotics/unilidar_sdk2#2-coordinate-system-definition)。

| ROS 话题 | 类型 | 内容 |
|---|---|---|
| `/unilidar/cloud` | PointCloud2 | L2 坐标的三维测距，字段 x/y/z/intensity/ring/time |
| `/unilidar/imu` | Imu | L2 内置 IMU，frame 为 unilidar_imu |
| `/imu/data_raw` | Imu | 独立安装 IMU，frame 为 imu_link |
| `/scan` | LaserScan | 从上述三维点云的近水平高度窗口投影，供 Nav2 使用 |

点云和投影扫描同一仿真时间戳；IMU 使用各自 Gazebo 采样时间。对应安装 TF 自动发布，`use_sim_time` 打开。IMU 提供传感器坐标下角速度、含重力的比力和理想世界参考姿态；静止水平时 Z 加速度约 +9.8 m/s²。输出使用可靠 QoS，兼容现有 Point-LIO 的 IMU 订阅。模拟域仍默认 42，不会订阅或修改真机域 0。

```bash
# 自动巡逻，使用三维雷达投影的 /scan
./run_test_scene.sh narrow_17 auto controller:=mppi
# 同时运行独立 Point-LIO 建图（额外计算负荷）
./run_test_scene.sh narrow_17 auto controller:=mppi pointlio:=true
# 仅传感器与建图环境，无自主行驶
./run_test_scene.sh narrow_17 world pointlio:=true
# 对照旧二维雷达；关闭 L2 也会关闭本次新增的双 IMU
./run_sim.sh use_l2:=false
```

`pointlio:=true` 使用 `config/pointlio_sim.yaml`，消费模拟 L2 及其内置 IMU，输出 `/pointlio/cloud_registered`、`/pointlio/odometry` 和 `/pointlio/path`。它目前是独立传感器建图结果，**没有替换 Nav2 的定位来源**；测试场景继续使用原有真值定位，普通地图继续 AMCL。不能把 Nav2 跑通等同于 LIO 定位精度通过。

采样参数在 `config/l2_sim.yaml`：默认 10 Hz、360×19 条规则射线，上半球仰角 0～89°、量程 0.1～30 m；IMU 目标 200 Hz（当前 2 ms 物理步长下实测约 166～168 Hz），噪声默认为零，可配置。这是用于接口与几何测试的近似值，非真实 L2 性能参数复刻；未模拟旋转棱镜的非重复轨迹、回波强度、材质反射、运动畸变、IMU 零偏漂移或同步误差。`ring` 是模拟仰角行号，`intensity=0`。每帧为瞬时快照，所有点的 `time=0`，不伪造逐点时间来诱发错误去畸变；因此不能用它验证真实扫描去畸变效果。

`/scan` 默认取 L2 坐标下 z∈[-0.03, 0.12] m 的点，并按水平方向取最近回波；其 frame 是 `unilidar_lidar`，原先位于 0.85 m 的独立二维传感器会移除。它不是地面分割，坡面/侧倾下的感知仍需进一步验证。朝上的雷达不能保证看到车身下方低矮障碍，四路侧向超声波也不能补全所有盲区。Gazebo 辅助坐标轴、L2 自身外壳和灯光图标排除出该 GPU 观测；其他场景物体仍参与测距。

检查实现与证据：`scripts/l2_sensor_config.py`、`scripts/l2_sim.py`、`tests/test_l2_sim.py`、`tests/live_l2_check.py` 和 `artifacts/l2_imu_live_validation.json`。安装预览入口仍只展示外形，不启动传感器/建图。

## 宇树 L2 外形与实测安装 URDF

### 在 Gazebo 检查安装方向

```bash
cd /home/a/ros2_ws/gz_sim
./run_sensor_preview.sh
```

此入口打开无墙遮挡的安装检查场，使用独立 `IGN_PARTITION=patrol_sensor_preview`。L2 显示真实网格；左右相机、独立 IMU 使用示意外形，尚未包含打印支架。红/绿/蓝轴分别表示设备 +X/+Y/+Z，L2 的蓝轴应竖直向上。按已填写 TF，L2 的 roll=pitch=yaw=0 表示底座水平、顶部朝上，不能因为“朝上”再额外绕 Y 转 90°。Yaw 仅决定水平转向，仍须以线缆方向核对。

标准 `run_sim.sh` 与测试入口现在也默认显示这份安装表中的 L2、相机、IMU 外形，并发布安装 TF；可用 `sensor_visuals:=false` 关闭，用 `show_sensor_axes:=true` 显示各设备轴。正常仿真现默认使用上文的 L2 三维近似测距和双 IMU；本安装检查入口仅展示外形。检查场不启动导航，车保持静止供观察；无需手柄。

已核对 Gazebo 实际画面，见 `artifacts/sensor_mounting_gazebo.png`。2 项外形/方向和姿态转换测试通过，场景 SDF 校验通过；追加外形不改变原车质量、碰撞、关节、控制插件或原传感器定义。

资源位于 `models/unitree_l2/`。宇树官方 SDK2 仓库此次核查不含 URDF/STL；官方坐标说明来自 [unitreerobotics/unilidar_sdk2](https://github.com/unitreerobotics/unilidar_sdk2)，外形网格和顶装/前装示例来自社区项目 [markgol/l2lidar_node](https://github.com/markgol/l2lidar_node)。不是把社区模型称作宇树官方模型。下载内容保留在 `upstream/` 与 `official/`，相关许可证一并保存；`sources.json` 记录固定提交、下载路径与 SHA256。

官方定义：点云原点位于雷达底部安装面的中心，+X 与底部线缆出线方向相反，+Z 垂直底面向上。因此，用户在 `sensor_mounting_tf.yaml` 中量到的底部中心可以直接作为 `unilidar_lidar`，无需增加设备高度。当前该点相对横杠为 `(0, 0, 0.434)` m，相对旧 base_link 为 `(0.09, 0, 0.929)` m。Yaw=0 沿用填写值，它要求雷达的 +X 朝车头；仅凭顶部朝上不能确认此条件。安装减震垫厚度若已包含在实测底部位置中，也不能重复相加。

下载的 STL 有 9734 个三角面，尺寸约 74.885 × 74.913 × 63.524 mm，单位已为 m。网格最小 Z 约 0.33 mm，保留上游网格及零 visual 偏移；不能把这点几何差异或壳体顶面当成点云原点偏移。

- `models/unitree_l2/l2.urdf`：独立 L2 外形，link 为 `unilidar_lidar`。
- `models/unitree_l2/l2_mounted.urdf`：按实测文件生成 `base_link → third_crossbar → unilidar_lidar`。
- `models/patrol_car_with_sensors.urdf`：保留原车模型，加入本次填写的雷达、摄像头、独立 IMU、超声波固定坐标；L2 使用真实外形网格。

这些是外形/安装描述，没有虚构雷达质量或惯量，也没有新增真实 L2 扫描插件。URDF 文件仅描述安装；运行时三维测距和双 IMU 由 SDF 注入及 ROS 适配器实现。没有修改硬件驱动或原始安装 YAML，新增 TF 仅在仿真域发布。L2 内置 IMU 与单独安装的 `imu_link` 是不同设备，不能混用；官方给出的内置 IMU 相对点云原点平移是 `(-0.007698, -0.014655, 0.00667)` m。本工作区驱动具有 `publish_imu_to_lidar_tf` 和 `publish_imu_orientation_tf` 开关，实际接入 robot_state_publisher 前应核对现有 TF 所有者，避免同一 child 出现两个 parent；本次未更改这些开关。

重新导出安装表后可重建描述：

```bash
cd /home/a/ros2_ws/gz_sim
/usr/bin/python3 scripts/build_sensor_description.py
```

三个 URDF 均通过 `check_urdf`，网格文件/下载哈希、安装位置和原车惯量保留检查通过。详情：`artifacts/l2_model_validation.json`、`models/unitree_l2/mounting_summary.json`。尚未验证新增装配在 Gazebo 的完整导航效果。


### 仿真传感器查看与 10 cm 近障警报

正常启动仿真后，用本机浏览器打开 **http://127.0.0.1:8768**。监测进程随仿真启动/退出，只连接仿真 ROS Domain 42，不连接真机。若不需要查看页和警报，可传 `sensor_monitor:=false`。

例如启动窄弯自主巡逻，同时启用 Point-LIO：

```bash
cd /home/a/ros2_ws/gz_sim
./run_test_scene.sh narrow_turns auto controller:=mppi max_speed:=0.3 pointlio:=true
```

网页提供以下视图：

- `/scan`：三维雷达截取的导航用二维扫描。
- `/unilidar/cloud`：L2 当前帧点云的俯视投影，不是累计地图。
- “配准点云累计图”：查看页将 `/pointlio/cloud_registered` 在同一消息坐标系下做 0.1 m 体素累计，保留最近最多 12,000 个体素，绘制最多 4,000 点；仅用于显示，不参与导航。需 `pointlio:=true` 并等待初始化。
- `/pointlio/local_map`：本工作区 Point-LIO 仅在初始化时发布一次，不能作为持续更新的地图。查看页后加入时可能收不到该消息。
- `/pointlio/cloud_registered`：Point-LIO 当前配准点云，不是整张地图。

Point-LIO 默认关闭，会增加计算负载；仅查看测距无需开启。当前测试导航仍使用场景配置的定位，Point-LIO 输出尚未替换 Nav2 定位。不同视图各自使用消息中的坐标系，页面标出 frame，不直接叠加。

四路超声波 `/ultrasonic_1` 到 `/ultrasonic_4`，依次为后右、前右、前左、后左。任一路有效距离 **≤0.10 m** 触发红色警报；超过 **0.12 m** 才解除以减少抖动。网页点击“启用警报声音”后可以蜂鸣（浏览器要求用户先交互），终端也会输出警告。启动参数 `proximity_threshold:=0.1` 可调整阈值，解除阈值为触发值加 0.02 m。

警报还发布 `/sim/proximity_alarm`（`std_msgs/Bool`）和 `/sim/proximity_status`（`std_msgs/String` JSON），以 2 Hz 墙钟更新。无回波、低于最小量程、无效值、缺失数据和超过 2 秒未更新分别处理；过近后的无效/过期数据不会解除原警报，只有有效的远距离或无回波消息会解除。已过期或未来的消息不用于解除警报。此警报**仅提示，不自动停车**。

距离从探头原点算起，四路探头主要覆盖两侧，车头/车尾中央有盲区；仿真测距也未模拟声学材质、多径和串扰。提示不等于整车防碰撞保证。

如果当前运行的是修改前启动的仿真，可另开终端执行 `./run_sensor_monitor.sh`，再打开同一网页。正常的新仿真已自动启动监测，不要重复运行此脚本，避免端口占用。端口默认仅监听本机回环地址；查看页文件为 `sensor_viewer/index.html`，后端为 `scripts/sensor_monitor.py`。

验证：`tests/test_proximity_alarm.py` 检查边界、滞回和断流；`tests/live_sensor_monitor_check.py` 在没有其他仿真节点时发送合成 ROS 消息进行端到端检查，拒绝与导航同时运行。验证不代表 Gazebo 场景中真实运动的防撞测试。

仿真导航角速度上限默认 **0.4 rad/s**，控制器（MPPI/TEB）与速度平滑器同步设置，左右对称。启动时可用 `max_yaw_rate:=0.4` 覆盖，例如：

```bash
./run_test_scene.sh narrow_turns auto controller:=mppi max_speed:=0.3 max_yaw_rate:=0.4
```

该参数是车体偏航角速度上限，不是前轮转角；最小转弯半径、角加速度和现有避障约束仍然生效。手柄控制保留自己的角速度及转角限制。单独提高角速度上限不能解决逐点停车、控制周期超时等问题。

### 2026-11-02：5 m 窄弯与连续巡逻

`narrow_turns` 现为中心线半径 **5 m** 的跑道形闭环，内侧墙半径 **4.15 m**，外侧墙内表面半径 **5.85 m**，净宽仍为 **1.7 m**，直线段仍长 8 m。物理墙体、地面、占据地图、出生点和巡逻点同步生成；出生点为 `(-2, -5, 0)`。原 3 m 资源备份在 `artifacts/narrow_turns_3m_before_20261102/`；历史结果不能直接当成新场景的对照试验。

```bash
cd /home/a/ros2_ws/gz_sim
./run_test_scene.sh narrow_turns auto controller:=mppi max_speed:=0.3 max_yaw_rate:=0.4
```

弯内每 10° 设置一个切向约束点，间隔约 0.87 m，减少稀疏航点连接时向内墙抄近。该路线标记 `navigation_mode: through_poses`，使用两次 `NavigateThroughPoses` 请求完成一圈，分段交接点在直线上，两侧各自的 180° 弯内连续通过；交接处仍会执行到达/朝向检查，并可能减速或短停。不能把闭环首尾重合的一整圈直接作为一次请求，否则当前 Nav2 会在起点误判到达。途中点用于约束每段路线，不逐点触发最终停车/朝向收敛。行为树将距离 0.7 m 内的下一个途经点移出待规划列表，每段终点使用最终到达检查。其他未标记的路线仍采用原逐点方式；需要逐点对照时可用 `scripts/patrol.py --mode waypoints`（不要与自动巡逻同时运行）。默认连续路线超时为每段 450 仿真秒，报告使用 `segments` 和 `laps` 记录分段及整圈结果，不将中间点伪记成精确停车成功。

窄弯的全局搜索采用前进 Dubins 模型，避免正常闭环路径中插入不必要的倒车段；保留原有倒车恢复能力。MPPI 保留 Ackermann 模型、车体/轮胎包络碰撞检查和 2 m 最小转弯半径；预测时域为 `40 × 0.1 = 4 s`，每次仍采样 512 条轨迹，角速度采样标准差由 0.10 调为 0.06 rad/s。局部地图尺寸和路径截取距离随前进上限匹配预测距离，避免高限速下预测轨迹超出局部地图。默认 0.3 m/s 时局部地图仍为 6 m × 6 m。

验证导航跟踪时默认不启动 Point-LIO（`pointlio:=false`），测试仍使用理想真值定位；不能把此结果当成真实传感器定位或真实车辆防撞验证。

超声波障碍融合默认使用 Nav2 `RangeSensorLayer`（`ultrasonic_fusion:=range`），不再把所有可能回波方向都直接标成致命圆弧。输入为 `/ultrasonic_N/navigation_range`，由适配器将原始 `+inf` 转为最大量程用于清空，将 `-inf` 转为最小量程用于近障标记；NaN 和非法有限量程不作为清空依据。原始四路话题和 10 cm 警报保持原样。旧圆弧融合可用 `ultrasonic_fusion:=arcs` 对照。概率式融合仍不能保证没有虚警或漏检，需继续验证移动障碍与实车声学特性。

整圈结果可用 `scripts/analyze_turns_run.py <结果前缀> --plot` 检查：除动作成功外，还要求覆盖左右两弯、上下直线、足够行驶距离与约一圈航向变化，并检查记录中的几何间隙。图表及 `.turns_analysis.json` 不将“起点误判到达”算作通过。
