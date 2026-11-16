# car_sim

ROS 2 Humble 与 Gazebo Fortress 仿真项目，用于小车底盘、传感器、定位、Nav2 导航和服务器端批量验证。项目默认只启动仿真，不连接真实底盘或执行器。

## 功能范围

- Gazebo Fortress 场景、车辆模型、地图和传感器模型
- Ackermann 转向/驱动仿真，以及 L2、D6、双目相机和超声波传感器
- Nav2、AMCL 多雷达融合、传感器定位与巡逻入口
- 浏览器仿真工作台、装配编辑器和强化学习环境
- 服务器无界面运行脚本、Dockerfile、打包脚本及验证记录

## 环境要求

- Ubuntu 22.04、ROS 2 Humble
- Gazebo Fortress（`ign gazebo`）
- 项目脚本使用的 ROS 包需在系统中安装或已构建
- 常用 Python 工具依赖见各子项目说明

本仓库可独立克隆，但部分 ROS 入口还会读取 ROS 工作空间中的包（例如 `pointlio`、驱动或自定义消息）。这些依赖不会由本项目自动安装。请先 source 对应工作空间，再运行入口脚本。

## 快速开始

克隆仓库后，从项目目录运行：

```bash
git clone git@github.com:SUDA-KAI/car_sim.git
cd car_sim
./run_sim.sh
```

无图形界面模式：

```bash
./run_sim.sh gui:=false
```

等待 Gazebo 和 Nav2 激活后，在另一个终端启动一次巡逻：

```bash
./run_patrol.sh --loops 1
```

停止进程使用 `Ctrl+C`。脚本的 ROS 域、IGN 分区和地图默认值请以脚本及 `config/` 中的配置为准。

## 常用入口

| 目的 | 命令 | 说明 |
|---|---|---|
| 浏览器工作台 | `./run_web_console.sh --open` | 默认访问 `http://127.0.0.1:8770`，详见 [web_console/README.md](web_console/README.md) |
| 基础仿真 | `./run_sim.sh` | Gazebo + Nav2 |
| 巡逻 | `./run_patrol.sh --loops 1` | 依赖仿真已启动 |
| 测试场景 | `./run_test_scene.sh narrow_turns auto` | 场景和结果说明见 [test_scenes/README.md](test_scenes/README.md) |
| 传感器定位导航 | `./run_sensor_navigation.sh` | 详见 [SENSOR_LOCALIZATION.md](SENSOR_LOCALIZATION.md) |
| 传感器定位巡航 | `./run_sensor_lap.sh` | 可使用 `--attach` 连接现有仿真 |
| 服务器导航包 | `./pack_navigation_server.sh` | 详见 [SERVER_NAV_README.md](SERVER_NAV_README.md) |
| 强化学习服务器包 | `./pack_rl_server.sh` | 详见 [rl/README.md](rl/README.md) |

## 文档索引

- [服务器导航部署](SERVER_NAV_README.md)：依赖、打包、服务器启动和参数
- [服务器部署概览](SERVER_README.md)：原有服务器运行方式
- [传感器定位说明](SENSOR_LOCALIZATION.md)：数据流、标定、启动和限制
- [验证记录](VALIDATION.md)：场景测试、验证结果和已知问题
- [MPC 说明](MPC.md)：相关控制算法记录
- [装配编辑器](assembly/README.md)
- [Web 工作台](web_console/README.md)
- [测试场景](test_scenes/README.md)
- [强化学习环境](rl/README.md)
- [验证产物索引](artifacts/README.md)

## 目录结构

```text
assembly/       车辆装配编辑器与导出模型
calibration/    仿真使用的传感器标定配置
config/         Gazebo、Nav2、传感器和巡逻参数
drivers/        仿真辅助驱动
maps/           栅格地图及元数据
models/         Gazebo 模型资源
rl/             强化学习环境及服务器部署
scripts/        仿真、地图、分析和打包工具
test_scenes/    测试场景、路线与结果
web_console/    浏览器工作台
worlds/         SDF 世界文件
```

## 部署边界

默认入口针对仿真环境。实车参数、传感器配置或测试记录仅供仿真和开发参考；不得直接视为实车安全参数。传感器模型和地图也不能替代真实设备标定或现场验证。

## 打包与结果

服务器包由 `pack_navigation_server.sh`、`pack_rl_server.sh` 等脚本生成到 `dist/`。`dist/` 属于构建输出，已由 `.gitignore` 排除。验证和调试产物集中在 `artifacts/` 与 `test_scenes/results/`，必要时可按索引查看。
