# Web3D · 环卫小车仿真导航台

纯 Web 的三维仿真界面：**Gazebo Fortress 在后端无界面跑物理**，浏览器用 three.js
渲染三维仿真过程，不弹出任何原生 Gazebo 窗口。

## 启动

```bash
# 首次运行: 下载前端第三方库 (three.js / roslib / STLLoader, 仓库不携带)
./fetch_vendor.sh

# 启动控制台
./run.sh
```

浏览器（VNC 内或本机）访问：**http://127.0.0.1:8780**

说明：`server.py` 自动定位 car_sim 仓库根目录（按 `web3d_sim/` 的上级目录、
`CAR_SIM_ROOT` 环境变量、`/home/a/car_sim` 的顺序查找），也可直接部署在
car_sim 仓库内的 `web3d_sim/` 目录下运行。

## 功能

- **三维仿真显示**：地图墙体立体化、小车模型、实时激光点云、规划路径、行驶轨迹
- **UI 加载地图**：切换内置/测试场景；上传 ROS 地图 YAML + PGM/PNG 自动挤出墙体生成三维场景
- **网页导航**：单击地面发送 Nav2 目标（NavigateToPose），实时显示剩余距离和结果
- **键盘遥控**：W/S/A/D（先点击三维区获得焦点），空格停车
- **视角**：左键拖动旋转，滚轮缩放，右键平移

## 架构

```
浏览器 (three.js + roslib)
  ├── REST  http://127.0.0.1:8780   ← 本服务 (server.py): 场景/地图/仿真启停/Nav2 目标
  └── WebSocket ws://127.0.0.1:9090 ← rosbridge: /amcl_pose /scan /plan /cmd_vel 实时数据
后台:
  Gazebo Fortress (无界面) + Nav2 + AMCL   ← /home/a/car_sim/run_sim.sh (ROS_DOMAIN_ID=74)
```

## 接口

| 方法 | 路径 | 说明 |
| --- | --- | --- |
| GET | `/api/scenes` | 场景列表（内置/测试/自定义） |
| GET | `/api/map` | 当前场景地图元数据 + 占据栅格（墙体） |
| GET | `/api/map/image.png` | 地图灰度图（地面纹理） |
| POST | `/api/sim/start` | `{world, controller:'mppi', max_speed}` 启动仿真 |
| POST | `/api/sim/stop` | 停止仿真 |
| GET | `/api/sim/status` | 仿真/时钟/导航状态 |
| POST | `/api/nav/goal` | `{x, y, yaw}` 发送导航目标 |
| POST | `/api/nav/cancel` | 取消导航 |
| POST | `/api/maps/upload` | `{name, yaml, image_b64}` 上传自定义地图 |

## 目录

```
web3d_sim/
├── server.py        # 后端: REST + rclpy(NavigateToPose) + 仿真进程管理
├── run.sh           # 启动脚本
├── static/          # 前端: index.html + app.js + vendor(three.js, roslib)
├── data/
│   ├── maps/        # 上传的自定义地图
│   └── worlds/      # 生成的三维场景 (sdf + json)
└── logs/sim.log     # 仿真日志
```

## 备注

- 仿真使用独立的 `ROS_DOMAIN_ID=74` 与 IGN 分区 `web3d_sim_74`，与 `run_sim.sh`
  默认的 42 域互不干扰
- 控制器固定使用 MPPI（TEB 未安装）
- 小车激光挂载位置取自项目 `sensor_mounting_tf.yaml`（L2 雷达 [0.6, 0, 0.5]）
