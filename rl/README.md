# 服务器强化学习封装

这套封装把现有地图、100 kg 阿克曼小车、360° 激光雷达和 ROS 2 桥保留为独立的 Gazebo Fortress 环境。它不启动 Nav2、RViz、手柄或实车驱动，适合服务器无界面运行。

## 接口

- 动作 `Box(2)`：`[速度, 转向角]`，均归一化到 `[-1, 1]`。环境内部转换成阿克曼 `Twist`；车辆不能原地旋转。
- 观测 `Box(96)`：90 个激光扇区的最小距离、车体系下目标 x/y、目标方位的 sin/cos、车速和横摆角速度。
- 默认出生点是世界坐标 `(0.4164, 12.0758, -0.52)`；Gazebo 阿克曼插件以出生位姿为里程计原点，因此环境目标使用局部 `odom` 坐标 `(2.0, 0.0)`，对应地图点约 `(2.152, 11.082)`。
- 结束：距目标小于 0.45 m，或激光最小距离小于 0.48 m；600 步截断。
- 奖励：目标距离进展为主，附加时间和转向惩罚；到达 `+30`、碰撞 `-20`。

奖励、目标点和阈值只是可运行基线，正式训练前应按任务重新标定。激光碰撞判定不是 Gazebo 接触传感器，若训练内容强调擦碰，建议后续增加 contact sensor。

## 本机快速验证

Gazebo 终端：

```bash
cd /path/to/gz_patrol_rl
./rl/run_rl_sim.sh
```

算法终端（首次需要安装 Python 依赖）：

```bash
cd /path/to/gz_patrol_rl
python3 -m pip install -r rl/requirements.txt
export PYTHONPATH="$PWD/rl"
export ROS_DOMAIN_ID=42
export IGN_PARTITION=patrol_rl_0
python3 rl/train_ppo.py --steps 100000
```

多实例训练：

```bash
# 终端 1；实例 i 使用 ROS_DOMAIN_ID=42+i 和 patrol_rl_i
./rl/launch_instances.sh 4

# 终端 2
export PYTHONPATH="$PWD/rl"
python3 rl/train_ppo.py --instances 4 --steps 1000000
```

每个实例是完整 Gazebo 进程。先根据服务器 CPU、内存和显存逐步增加数量；`gpu_lidar` 即使无窗口也需要渲染上下文。

## Docker / 上传服务器

生成精简归档：

```bash
./pack_rl_server.sh
# 输出 dist/gz_patrol_rl.tar.gz
```

服务器解压后，在解压目录构建：

```bash
docker build -f Dockerfile.rl -t patrol-rl:humble .
docker run --rm --network host --ipc host patrol-rl:humble
```

容器默认设置 Mesa 软件渲染。NVIDIA 服务器可用 NVIDIA Container Toolkit 启动并覆盖 `LIBGL_ALWAYS_SOFTWARE=0`。训练器放在同一容器最省事；若拆成两个容器，需要共享 host 网络、相同 `ROS_DOMAIN_ID` 和 `IGN_PARTITION`。

## GRPO 说明

Hugging Face TRL 中的 GRPO 面向“提示词 → 多个文本回答 → 组内相对奖励”的语言模型训练，不会直接消费本环境的连续二维动作。若你的 GRPO 是自研的连续控制版本，可以直接把 `PatrolCarEnv.reset()` / `step()` 当 Gymnasium 接口，并对同一状态采样一组轨迹后做组内优势归一化。

如果你指的是用视觉语言模型输出离散驾驶指令，建议另加一层动作离散化（例如 3 档速度 × 5 档转向），再把每组 rollout 的到达率、碰撞和进展转换成 GRPO reward。不要直接把 TRL 的 `GRPOTrainer` 接到当前连续动作空间。

## 已知边界

- 当前 `step()` 按墙钟等待传感器，不是锁步物理仿真；这是稳定的 PPO 基线，但吞吐量不是最优。需要更快训练时，可将世界设为暂停，并桥接 `/world/patrol/control` 做固定 `multi_step`。
- `set_pose` 会传送车辆回出生点并发送零指令；若后续发现轮速残留，可改成删除/重生模型或增加速度重置系统。
- 地图由二维占据栅格挤出，未表达真实坡度、材质和动态障碍。测试场景目录另含坡道、窄道和低摩擦场景，可通过 `world:=绝对路径` 切换。
