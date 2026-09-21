FROM ros:jazzy

RUN apt-get update && apt-get install -y \
python3-colcon-common-extensions \
python3-pip \
git \
nano \
ros-jazzy-demo-nodes-cpp \
ros-jazzy-demo-nodes-py \
ros-jazzy-ros-gz \
ros-jazzy-robot-state-publisher \
ros-jazzy-xacro \
ros-jazzy-geometry-msgs \
ros-jazzy-navigation2 \
ros-jazzy-nav2-bringup \
&& rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

CMD ["bash"]