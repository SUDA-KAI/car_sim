# 传感器定位与一圈导航

## 启动与查看

无需打开 RViz，自动启动仿真、定位、建图与一圈任务：

```bash
cd /home/a/ros2_ws/gz_sim
./run_sensor_lap.sh
```

打开 **http://127.0.0.1:8769**。巡逻启动后页面默认切到在线地图：青色是障碍，黄色是已行驶轨迹及当前目标连线，蓝点和箭头是车辆估计位置与朝向。页面还显示任务状态、路程、完成圈数、双目图像及四路超声波。黄色连线不是 Nav2 的完整规划轨迹。

一圈结束后停车，保留仿真供查看；Ctrl+C 结束本入口启动的进程组，包括 Gazebo 后台子进程。仅保留网页、关闭 Gazebo 窗口渲染：

```bash
./run_sensor_lap.sh gui:=false
```

如果已有当前版本的传感器仿真，使用 `./run_sensor_lap.sh --attach` 只连接它；连接模式退出只取消本任务，保留原仿真。旧入口若开启二维扫描匹配/回环，会提示重新启动，避免混用配置。

只启动感知与导航服务、暂不自动发目标：

```bash
./run_sensor_navigation.sh
```

RViz 只是可选调试工具；要手工选目标可加 `rviz:=true`，在已观测的新地图内使用 2D Goal Pose。没有发目标时，车辆保持不动。每次启动默认新建地图，不复用旧世界坐标的巡逻点。

## 当前实际数据链路

```text
L2 三维点云 → 墙面 XY 投影 /scan → 二维点到线匹配 + 实测 IMU 转角 → /localization/lidar_odom ┐
双目校正图 → SGBM 深度 + KLT 跟踪 + PnP → 视觉速度                                  ├→ EKF → /chassis/odom、odom→base_link
独立 IMU 角速度                                                                   ┘
/scan + 融合位姿 → SLAM Toolbox 按里程计累积在线地图 /map
观测的左右墙面 → 通道中线与切线目标 → Nav2 Hybrid A* + MPPI
Nav2 /cmd_vel → 观测有效性检查 → /localization/safe_cmd_vel → Gazebo
```

当前一圈入口用于**平面闭合通道**。激光里程计固定本帧间的旋转为 IMU 实测积分，用点到线扫描匹配估计平面位移；没有读取 IMU 的理想世界姿态。匹配不足、残差过大或运动不合理时不发布有效位姿。

EKF 融合激光平面位置、双目速度和独立 IMU 角速度，不重复融合激光的累计 IMU 航向。`odom→base_link` 只有 EKF 发布。二维建图按融合里程计累积，配置 `config/sensor_lap_slam.yaml` 暂时关闭二维扫描匹配和回环，尚未解决长期全局漂移。视觉前端也不提供视觉回环。

**Point-LIO 不再作为本入口的定位来源。** 原三维 Point-LIO 在转弯测试中出现反向位移；双目也出现过跟踪丢失。相关失败记录保存在 `artifacts/sensor_lap_*failure*`、`sensor_lap_lio_divergence*`。可选 `pointlio:=true` 仅启动三维算法作独立调试；它的 `/pointlio/odometry` 不进入当前 EKF。旧 `lio_odometry_adapter.py` 的一致性保护保留供调试，不属于当前主链路。

L2 的三维扫描仍是规则上半球、瞬时扫描近似。二维墙面投影采用雷达坐标中 `[-0.03,1.2] m` 高度范围，适用于当前竖直墙面；**本次不覆盖上下坡、悬空物体、真实点云噪声或实车定位**。

## 一圈任务如何生成目标

`scripts/sensor_corridor_lap.py` 分别拟合实时雷达观测到的左右墙面，逐步沿中线推进，取前方约 1.8 m 的位置和该处墙面切线朝向。每前进约 0.65–0.85 m 更新导航目标，Nav2 自己规划与控制。累计转过一圈并靠近记录的启动估计位置后，发送最终返回目标。

任务不读取场景半径、出生坐标、预制地图或旧世界坐标航点。它适用于当前单一闭合通道，不是带岔路环境的通用探索器。返回的是传感器估计起点，实际返回误差需由独立评估报告核实。

稀疏激光射线会在在线地图的轮廓边缘留下未知格子，传感器模式的 Smac 使用 `allow_unknown: true`。这允许规划越过未知格子，**不表示这些格子已经确认安全**。目标生成检查雷达可见范围和净空，Nav2 保留整车轮廓与已测障碍碰撞检查。[Smac 碰撞检查实现](https://github.com/ros-navigation/navigation2/blob/humble/nav2_smac_planner/src/collision_checker.cpp)

超声波负责概率避障和 10 cm 警报，不提供全局位置。警报本身仅提示；当激光/视觉定位观测均过期，或融合里程计、四路测距缺失时，命令检查节点输出零速度。该功能不是硬件急停。

## 标定、负载和隔离

双目标定来自：

`/home/a/ros2_ws/tools/kalibr_web_ui/data/sessions/stereo_20261102_135012/kalibr_stereo-camchain.yaml`

快照：`calibration/stereo_20261102_135012.yaml`。

- cam0 左、cam1 右；原图 1920×1080，pinhole-radtan，基线 0.247102920565 m。
- Kalibr `T_1_0` 将左光学点变到右光学坐标；TF 使用其逆描述右光学帧相对左光学帧的位姿。
- 相机—L2—独立 IMU 沿用用户确认的安装 TF，属于安装测量，未标成联合标定；外壳和支架记录不覆盖。
- 当前相机 384×216、4 Hz 仿真时间；K 同比例缩放，畸变和外参保持。软件施加 Brown 畸变一次，再发布原图、校正图及两套 CameraInfo。
- L2 3 Hz。默认仿真倍率目标 0.3；底盘前进/后退运行限速为 0.5/0.2 m/s，虚拟中心轮转角限位 ±40°，内侧前轮角度会更大。双 D6 按现有安装参数以 10 Hz 输出水平扫描并进入 Nav2 局部障碍层。墙钟运动慢于实时。
- ROS Domain 43、Gazebo 分区 `patrol_sensors`；原测试 Domain 42、真机 Domain 0 保持分离。不要把旧世界坐标巡逻入口当作传感器定位验证。

导航域不桥接 Gazebo 真值、车轮里程计或仿真 TF。桥接类型白名单在 `scripts/sensor_localization_config.py`；实际启动审计在 `artifacts/sensor_localization_current.json`。独立评估读取 Gazebo 原生真值，仅作结果对比，不发入 ROS，也不用于目标生成或估计。

主要配置：`config/sensor_ekf.yaml`、`config/sensor_lap_slam.yaml`。原 `config/sensor_slam.yaml` 保留二维匹配/回环实验设置，需显式指定使用。

## 记录与验证

- 任务报告：`artifacts/sensor_corridor_lap.json`。`state: succeeded`、`completed_laps: 1` 表示 Nav2 一圈任务完成。
- 2026-11-03 最新实测：滚动雷达目标和局部/全局滚动代价地图已接入，直线及第一个弯道前半段能行驶；第 4 次 Nav2 目标失败后任务停下。报告记录累计 23.52 m、转角约 1.56 rad、`completed_laps: 0`。累计路程含倒车恢复，不能视为一圈。当前入口仍属实验性，整圈尚未验证通过。
- 独立实际运动评估：`artifacts/sensor_lap_evaluation.json`；它与任务完成状态分开，不能把估计位置当真实返回精度。
- 仿真日志：`artifacts/sensor_lap_full.log`。
- 测试记录：`artifacts/sensor_lap_unit_tests.log`。
- 任务话题：`/localization/patrol_status`、`/localization/patrol_path`；当前激光话题 `/localization/lidar_odom`。
- 旧 `sensor_navigation_live_validation.json` / `sensor_localization_validation_summary.json` 为原三维链路的短距离历史验证，不能用于证明当前整圈性能。

保存当前地图（另开终端）：

```bash
source /opt/ros/humble/setup.bash
source /home/a/ros2_ws/install/local_setup.bash
ROS_DOMAIN_ID=43 ros2 run nav2_map_server map_saver_cli \
  -f /home/a/ros2_ws/gz_sim/maps/sensor_mapping \
  --ros-args -p use_sim_time:=true -p map_subscribe_transient_local:=true
```
