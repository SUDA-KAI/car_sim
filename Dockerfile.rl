FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    LIBGL_ALWAYS_SOFTWARE=1 \
    PYTHONUNBUFFERED=1
RUN apt-get update && apt-get install -y --no-install-recommends \
      ignition-fortress ros-humble-ros-gz-bridge python3-pip mesa-utils \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/patrol_sim
COPY . /opt/patrol_sim
RUN pip3 install --no-cache-dir -r rl/requirements.txt \
    && chmod +x rl/run_rl_sim.sh rl/launch_instances.sh rl/train_ppo.py
ENV PYTHONPATH=/opt/patrol_sim/rl
CMD ["./rl/run_rl_sim.sh"]
