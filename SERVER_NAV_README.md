# 导航仿真服务器包

本包包括 Gazebo Fortress 车辆模型、两台 D6、L2、四路 A22、地图、测试世界、Nav2 MPPI 配置和 AMCL 多雷达定位模拟器。默认无界面启动 `narrow_turns`，不自动发送运动目标。实际机器人不会被连接或控制。

`conversations/01a1a001-0f1e-7333-a446-8b1a19cd30b3.md` 保存此前指定会话的用户与助手可见消息，带时间戳和导出时间。原会话仍可能更新；包内是打包时的快照。

`reports/仿真模拟方案报告.md` 对照最新实车状态列出同步参数、验证步骤和仿真限制；`reports/实车阶段问题清单.md` 与 `status_snapshot.json` 是报告写作时的只读依据。

## 服务器要求

- Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress、`ros_gz_bridge`、Navigation2（含 MPPI）、Python 3 的 NumPy、SciPy、OpenCV、PyYAML 和 Pillow。
- `gpu_lidar` 即使无窗口也需要 EGL/OpenGL；服务器可用显卡渲染，或设置 `LIBGL_ALWAYS_SOFTWARE=1` 用 Mesa 软件渲染。
- 两台 D6 是理想平面扫描，L2 也没有玻璃材料反射模型。定位与过弯的仿真结果不能代替实车验收。

## 解压与启动

```bash
sha256sum -c gz_patrol_nav.tar.gz.sha256
mkdir -p patrol_nav
tar -xzf gz_patrol_nav.tar.gz -C patrol_nav
cd patrol_nav
./run_server_nav.sh narrow_turns
```

已有 ROS 2 Humble / Gazebo Fortress 时用原生入口。默认 `ROS_DOMAIN_ID=73`、`IGN_PARTITION=patrol_nav_server`，可用 `SIM_ROS_DOMAIN_ID`、`SIM_IGN_PARTITION` 覆盖。其他场景例如 `./run_server_nav.sh narrow_17`；额外 ROS launch 参数可直接追加，例如 `max_speed:=0.2`。
新地图诊断入口为 `./run_server_nav.sh recording_20261106_163030`。该世界由同一张导航栅格拉伸生成，无法检出原地图被强制清空的真实墙点。服务器默认启用仿真运控适配器，四路 A22 或 AMCL/轮速数据失效时会停驱动；查看 `/sim/nav_drive_gate_status`。仿真目标仍需手工发送。

所有运行用地图 YAML 均附对应 PGM；URDF 与 SDF 引用的网格也在包内。`reference/latest_recording_20261106_161101/` 另附最新回环 PCD/NPZ 和高度过滤资料，供离线分析；834 MB 原始 rosbag 未打入服务器包。

如使用容器，在解压目录执行：

```bash
docker build -f Dockerfile.nav -t patrol-nav:humble .
docker run --rm --network host --ipc host patrol-nav:humble
```

在另一终端设置相同 `ROS_DOMAIN_ID=73` 后，可查看 `/d6_a/scan`、`/d6_b/scan`、`/localization/multi_lidar_scan`、`/amcl_pose` 和 `/local_costmap/costmap`。用 Nav2 发送目标或连接 RViz；默认不会自主发车。`/localization/scan_fusion_status` 的 `sources` 字段显示 L2、前后 D6 对 AMCL 的参与情况。

## 与实车方案的对应范围

轴距 0.90 m、前后速度限值 0.50/0.20 m/s、中心轮转角限值 ±40° 对齐现有底盘运行配置；0.80 m 轮距和 0.20 m 轮半径仍是 CAD 仿真值。前轮支持停车打角，后轮只在转向稳定后驱动。D6 使用当前实车安装坐标和话题，A22 位于侧面。AMCL 用轮里程计和采集时刻对齐的 L2/D6 端点修正位置；L2 的理想仿真点不直接标记局部障碍，局部地图由 D6、A22 和已知静态地图规划约束提供。

初始位姿由测试场景的出生位姿自动设置，仅用于可重复仿真。实车仍需人工校准初始位姿。仿真尚无真实玻璃反射、D6 纸板校准误差、轮胎打滑或人工后台纠错模型；请以实车日志检验。打包清单与各文件哈希见 `BUNDLE_MANIFEST.json`。
