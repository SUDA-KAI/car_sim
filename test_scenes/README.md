# 1.7 米窄道巡逻测试场

转弯、坡道、低摩擦路面的常用入口现已全部切换为 **1.7 m 净宽**。车仍是原来的 100 kg、长 1.2 m、宽 0.9 m 模型。预览：`narrow_scene_overview.png`。

| 场景 | 内容 | 兼容的简写 |
|---|---|---|
| narrow_turns | 窄道闭环，直线段8 m，弯道中心线半径3 m，12个航点 | turns |
| narrow_ramp_6deg | 窄道6°上下坡，4 m上坡+3 m平台+4 m下坡 | ramp_6deg |
| narrow_ramp_12deg | 同样宽度的12°上下坡 | ramp_12deg |
| narrow_slip_normal | 窄道普通摩擦对照，mu=1.0 | slip_normal |
| narrow_slip_low | 窄道低摩擦区，mu=0.05 | slip_low |
| narrow_slip_ice | 窄道极低摩擦区，mu=0.005 | slip_ice |
| narrow_17 | 直通道，低速前进约10 m再倒车返回 | — |
| narrow_17_baseline | 相同直通道，旧测试参数对照 | — |

## 启动

四路超声波已默认接入：位置来自 `/home/a/ros2_ws/test/ultrasonic_settings.json`，高度 0.49 m；仿真扇形射线生成 `/ultrasonic_1..4` 的 Range 消息，经圆弧投影加入局部障碍地图。与历史结果对照时添加 `use_ultrasonic:=false`。安装坐标、车长差异与理想模型限制见根目录 README 的“四路超声波仿真”。

新增 MPC 类控制器 MPPI，可在同一张地图上与 TEB 切换比较：

```bash
./run_test_scene.sh turns auto controller:=mppi
./run_test_scene.sh turns auto controller:=teb
```

详细原理、参数与局限见 [MPC.md](../MPC.md)。默认仍为场景原有的 TEB。

先关闭已有仿真，在 `/home/a/ros2_ws/gz_sim` 下执行：

```bash
./run_test_scene.sh narrow_turns auto
./run_test_scene.sh narrow_ramp_6deg auto
./run_test_scene.sh narrow_ramp_12deg auto
./run_test_scene.sh narrow_slip_normal auto
./run_test_scene.sh narrow_slip_low auto
./run_test_scene.sh narrow_slip_ice auto
```

一次只运行一个。`auto` 会启动场景、导航、巡逻和记录器，不用再启动巡逻脚本或按手柄；等待导航初始化后自动行驶。整圈可能需要几分钟。终端显示 `reached waypoint`；完成后车辆停止，窗口保留，Ctrl+C 退出。

原来的 `./run_test_scene.sh turns auto` 等命令也会自动选择窄道版。无参数执行默认选 `narrow_turns`。若要查看保留的旧宽版，显式使用 `wide_turns`、`wide_ramp_6deg`、`wide_ramp_12deg`、`wide_slip_normal`、`wide_slip_low`、`wide_slip_ice`。

```bash
# 手柄探索：LB使能，左摇杆速度，右摇杆转向
./run_test_scene.sh narrow_ramp_6deg manual
# 无界面
./run_test_scene.sh narrow_turns auto gui:=false
# 仅场景，不启动导航或手柄
./run_test_scene.sh narrow_slip_normal world
```

## 净宽、弯道与坡道

直通段墙内表面 y=±0.85米，厚0.2米。坡面、坡顶平台、下坡和低摩擦区均在连续侧墙内，通行净宽都是1.7米，不保留可绕开的宽平台。

弯道采用**等宽圆弧连接的闭环**：中心线半径3米，内边界半径2.15米，外边界半径3.85米。直线和弯道间相切连接。碰撞网格使用细分圆弧，宽度近似误差小于1毫米。该场景不是把旧地图直角弯硬夹成1.7米；它也不代表实车能通过现场所有窄直角弯。

窄坡道沿用6°/12°坡度与真实碰撞曲面。车模为刚性轮、没有悬挂/轮胎变形或真实电机力矩限制；坡顶仍是折线过渡。这些是仿真挑战，不是实车爬坡能力认证。

普通/低/极低摩擦三组的地面几何和路线相同，x=0..7米为摩擦对照区，区下面没有重叠的高摩擦地板。摩擦数值是测试参数，并非特定材料实测。

## 控制与定位

窄道默认采用独立低速TEB/平滑器参数：一般场景前进上限0.2 m/s，弯道0.18 m/s，倒车上限0.12 m/s。Gazebo公共正负限幅保持对称，具体前后速度由上层分别约束。旧的直道参数对照保留在 `narrow_17_baseline`。

**左右纠偏尚未完全消除。**此前直道对照里低速版横向摆幅减小，但角速度指令翻转次数增加。不能把换成窄地图或完成航点直接等同于控制器已调稳。

默认采用Gazebo独立三维真值定位：`map → odom` 恒等变换，`odom → base_link` 来自真实模型位姿。`/wheel/odom` 单独保留轮子推算位置，`/ground_truth/odom` 为真值，导航 `/chassis/odom` 在此模式使用真值。这用于分离车辆/控制问题与定位问题，不等于实车Point-LIO/GICP或抗打滑能力已通过验证。

局部代价地图使用静态已知障碍，避免俯仰的二维激光把坡地标成障碍。这不是三维地形感知测试。坡面在导航地图中标为可通行，Nav2不会自行判定车辆是否可爬。

平地可选择AMCL定位对照：

```bash
./run_test_scene.sh narrow_slip_ice auto localization:=amcl
```

此时轮里程计用于AMCL，真值仅用于独立评估；坡地仍建议先使用默认三维真值基准。

## 结果与余量

`results/<场景>_<北京时间戳>` 下保存：

- `.csv`：真实xyz和姿态、轮速/真值速度、控制指令、规划路径距离、中心线横向偏差与几何余量。
- `.json`：汇总指标，含最大横向偏差、估计最小车身和轮胎扫掠余量。
- `.patrol.json`：逐航点结果与整轮 success/failed/canceled，auto模式生成。

新窄道版记录器把完整三维姿态投影到地面平面，计算1.2×0.9×0.495米车身外包络以及约1.146米宽的最大轮角扫掠保守外包络。弯道对内外曲线边界测距，不用错误的“距y=0直线”指标。坡地会考虑roll/pitch对外包络的影响。余量为几何估算，不是物理接触传感器，不代表实车安全保证。

画通用结果图：

```bash
/usr/bin/python3 scripts/plot_test_results.py test_scenes/results/某次测试.csv
```

画适用于直道、弯道和坡道的窄道余量图：

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 scripts/analyze_narrow_test.py \
  test_scenes/results/某次测试.csv \
  --scene test_scenes/worlds/narrow_turns.json
```

`--scene` 要与记录的场景匹配，输出 `.clearance.json/.png`。轮速差是滑移线索，不是标定轮胎滑移率；到最新规划路径的距离包含重规划变化，不能直接当作统一合格标准。

## 本次窄道验证（2026-10-27）

- 7项结构/几何测试通过；8个窄道SDF（含两个直道对照）检查有效。
- `narrow_ramp_12deg`：4/4航点完成，真实高度约0.85米，俯仰约12°；车身投影与最大轮角扫掠的估计余量保持为正。
- `narrow_turns`：前2个航点完成，第3个航点失败。车辆在第一个弯道偏向内侧，中心线偏差最大约0.378米，名义车身包络余量降至约0.022米，随后规划器报告起点进入高代价/致命空间并停止。最大轮角扫掠的保守余量出现负值，但这不等同于接触传感器实测碰撞。
- 因此窄弯**尚未通过当前TEB导航测试**。保留失败地图和报告供复现，没有扩大通道、缩小车身或把失败写成成功。
- 6°窄坡和三种窄摩擦场景已生成并检查几何/物理参数，尚未完成各自完整自动巡逻实测。先前宽地图通过不作为窄地图通过证据。

完整汇总：`narrow_suite_validation.json`。曲线余量图在 `results/narrow_turns_20261027_115206.clearance.png`，坡道余量图在 `results/narrow_ramp_12deg_20261027_115304.clearance.png`。

## 验证与历史

本次新窄道套件的验证见 `narrow_suite_validation.json` 和根目录 `VALIDATION.md`。原直道的对照结果与边界见 `NARROW_CORRIDOR.md`、`narrow_comparison.json`。

保留的宽版历史测试：转弯6/6航点、6°和12°坡道各4/4航点完成；普通与极低摩擦3秒刹车窗口位移约0.367 m与1.641 m。它们在 `validation_results.json`，**不能冒充新窄道版已验证结果**。

完整极低摩擦窄道巡逻可能失败，失败会写入报告；此处不保证成功，也没有实现专门抗滑控制。

```bash
/usr/bin/python3 scripts/build_test_scenes.py
source /opt/ros/humble/setup.bash
/usr/bin/python3 -m unittest discover -s tests -p test_test_scenes.py -v
```

生成脚本会重建测试套件，不删除结果。实现依据：[Gazebo三维真值里程计](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/odometry_publisher/OdometryPublisher.cc)。
