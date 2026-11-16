# Gazebo Patrol RL 服务器数据包

本目录是可移动的 ROS 2 Humble + Gazebo Fortress 无界面仿真运行包。资源路径均为相对路径，不依赖打包机器的工作区。

## 内容

- `worlds/`：滤波地图生成的 SDF 世界和 OBJ 碰撞/显示网格
- `maps/`：对应 ROS 占据栅格
- `models/`：100 kg 阿克曼小车、车身 STL、720 线二维激光
- `config/bridge.yaml`：`cmd_vel`、里程计、激光、时钟和 TF 的 ROS/Gazebo 桥
- `rl/`：无界面/多实例启动器、接口清单和可选 Gymnasium/PPO 示例
- `Dockerfile.rl`：服务器容器构建文件

精确话题、量程、速度和出生位姿见 `rl/BUNDLE_MANIFEST.json`。

## 原生启动

```bash
./rl/run_rl_sim.sh
```

要求系统已安装 ROS 2 Humble、Gazebo Fortress 和 `ros-humble-ros-gz-bridge`。默认使用 `ROS_DOMAIN_ID=42`、`IGN_PARTITION=patrol_rl_0`。

## Docker 启动

```bash
docker build -f Dockerfile.rl -t patrol-rl:humble .
docker run --rm --network host --ipc host patrol-rl:humble
```

## 训练器接入

你可以忽略示例算法，直接使用以下接口：

- 发布 `/cmd_vel` (`geometry_msgs/msg/Twist`)
- 订阅 `/scan` (`sensor_msgs/msg/LaserScan`)
- 订阅 `/chassis/odom` (`nav_msgs/msg/Odometry`)
- 调用 Gazebo `/world/patrol/set_pose` 复位

Gymnasium 和 PPO 示例见 `rl/README.md`。本包没有绑定 PPO、GRPO、RLlib 或其他具体训练框架。
