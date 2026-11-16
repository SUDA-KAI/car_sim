# MPC / MPPI 仿真控制

之前沿用了现有车辆的 TEB 配置。现在可以在相同 Nav2 框架下切换到官方 MPPI 插件；MPPI 是采样式模型预测控制的一种，不是新增一个独立的全局路径规划器。

先关闭旧仿真，一次只启动一个：

```bash
cd /home/a/ros2_ws/gz_sim
./run_test_scene.sh turns auto controller:=mppi
# 同一地图使用原 TEB 配置作对照
./run_test_scene.sh turns auto controller:=teb
# 其他地图也能指定 MPPI
./run_test_scene.sh narrow_17 auto controller:=mppi
./run_test_scene.sh ramp_12deg auto controller:=mppi
```

默认 `controller:=scene` 保持各场景已有的 TEB 配置。`manual` 模式仍用手柄；控制器选择只影响自动导航。初始化后自动发送巡逻点，完成一圈或失败后停止。关闭窗口或 Ctrl+C 退出。

## 原理

MPC 每个周期都做四步：根据当前位置和运动模型预测未来轨迹；综合跟踪偏差、障碍物风险及控制代价求出未来一段控制量；只执行第一步；收到新位置后重新计算。这叫滚动优化。

阿克曼单轨模型可近似写成：

```text
x_next   = x + v*cos(yaw)*dt
y_next   = y + v*sin(yaw)*dt
yaw_next = yaw + v*tan(delta)/L*dt
```

这里 L 为轴距，delta 为前轮等效转角。该简化模型的位置参考通常在后轴中心；当前 Gazebo 的 base_link 位于车身中部，存在参考点差异及转向执行延迟，因此预测不等于实际运动。

常见 MPC 用数值优化器求控制序列。MPPI 则对上一周期的控制序列加入随机扰动，模拟许多候选轨迹，按代价加权更新控制序列。它仍采用“预测、执行一步、反馈重算”的结构。

当前链路为：巡逻目标 → Smac Hybrid-A* 全局路径 → MPPI 局部控制 → 速度平滑 → Gazebo 阿克曼转向。MPC 与 A* 分别负责控制和全局规划，可以组合使用。

## 当前参数和适用范围

- 配置：`config/controller_mppi.yaml`，运行时覆盖逻辑：`scripts/navigation_config.py`。
- 控制频率 10 Hz，预测步长 0.1 s，共 40 步，即预测 4 秒；每周期 512 条采样轨迹，关闭轨迹可视化以减少计算。
- 采用 Ackermann 模型；前进上限由 `max_speed` 设置，默认 0.3 m/s，角速度上限由 `max_yaw_rate` 设置，默认 0.4 rad/s；最小转弯半径随场景继承，窄弯为 2 m。
- 用车体轮廓检查碰撞。带轮胎扫掠尺寸的测试地图按最大轮胎转角的保守外廓规划，仍保留原车模型、墙壁和地图净宽。
- 移除速度平滑器的 0.01 rad/s 角速度死区，避免低速转弯的小角速度被截断。其他加减速限制保留。
- MPPI 自身不包含这台车的转向执行器延迟、轮胎侧滑和摩擦辨识模型；不能据此宣称已解决打滑。当前 Humble 插件的运动约束也没有显式转角变化率约束。
- 测试地图仍使用理想位置反馈和静态地图，不能替代真实雷达定位与动态障碍物验证。

记录文件以 `<场景>_mppi_<时间>` 命名，额外保存 `.nav.yaml` 和 `.run.json`，便于确认每次使用的插件与实际参数。TEB 对照也会记录相同信息。

参考：[Nav2 Humble MPPI 官方说明](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/README.md)。

## 5 m 窄弯调整（2026-11-02）

中心线半径改为 5 m，净宽保留 1.7 m。窄弯全局搜索用前进 Dubins 连接；巡逻分成两段 `NavigateThroughPoses`，在直线上交接，每个 180° 弯内连续通过，交接点仍可能减速或短停。避免将闭环起终点重合的一整圈直接作为一次请求，否则 Nav2 会立即判断到达。局部地图与路径截取范围按最大速度和预测时域匹配；角速度采样标准差为 0.06 rad/s。

这些是巡逻、规划与控制的配合调整，不代表 MPPI 已包含转向延迟或轮胎滑移模型。5 m 场景的结果不能直接证明相同控制器在原 3 m 场景也通过。 弯内使用约 0.87 m 间距的切向约束点，并改用 RangeSensorLayer 概率融合超声波。一次完整路线验证成功：约 46.89 m、178.41 仿真秒，巡航弯段速度中位数 0.274 m/s，最小保守轮胎包络余量 0.123 m；仍有控制周期超时警告。详见 `VALIDATION.md`。

## 历史实际验证（2026-10-29，3 m 场景）

同一张 1.7 m 窄弯地图，无界面、独立 ROS 域 61：MPPI 加载成功，前 3 个航点均返回 SUCCEEDED，其中第 3 个是此前 TEB 失败的位置。第 3 点耗时 108.874 秒仿真时间，明显存在低速反复修正。验证在第 4 点期间主动结束，不能据此前 3 点宣称整圈通过。

本次同时扩大了代价地图轮廓并移除角速度死区，不把差异全部归因于 MPPI 算法。2 项控制器切换/约束测试与 7 项原场景测试通过；语法检查通过。完整证据与局限见 `test_scenes/mppi_validation.json`、`artifacts/mppi_narrow_turns.log` 和 `test_scenes/results/narrow_turns_mppi_20261029_141926.*`。测试已停止，原始巡逻快照在关闭时仍为 running；单独验证报告明确标记为 partial_validation_stopped。
