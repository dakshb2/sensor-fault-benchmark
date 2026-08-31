# sensor-fault-benchmark, containerised.
#
#   docker build -t sensor-fault-benchmark .
#   docker run --rm sensor-fault-benchmark \
#       ./experiments/scripts/run_trial.sh experiments/trajectories/box.yaml trial01
#
# Gazebo runs headless (server only, software rendering). There is no GUI in the
# container; trials are scored from topic data, which needs no display.

FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# --- add the OSRF repository (Gazebo Fortress lives here) ------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      curl gnupg lsb-release ca-certificates \
    && curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
         -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
         > /etc/apt/sources.list.d/gazebo-stable.list \
    && rm -rf /var/lib/apt/lists/*

# --- system + ROS packages -------------------------------------------------
# ros-gz-sim is deliberately absent: no arm64 binary is currently published
# (apt-cache madison shows a Sources entry but no arm64 Packages entry), so it
# is built from source in the next step.
RUN apt-get update && apt-get install -y --no-install-recommends \
      git \
      libgflags-dev \
      python3-pip \
      python3-colcon-common-extensions \
      ros-humble-robot-localization \
      ros-humble-ros-gz-bridge \
      ros-humble-ros-gz-interfaces \
      ros-humble-topic-tools \
      ros-humble-xacro \
      ros-humble-robot-state-publisher \
      ros-humble-rmw-cyclonedds-cpp \
      ignition-fortress \
    && rm -rf /var/lib/apt/lists/*

# --- build ros_gz_sim from source ------------------------------------------
# The launch file uses this package's `create` executable to spawn the robot.
RUN mkdir -p /deps/src \
    && cd /deps/src \
    && git clone -b humble --depth 1 https://github.com/gazebosim/ros_gz.git \
    && cd /deps \
    && source /opt/ros/humble/setup.bash \
    && colcon build --packages-select ros_gz_sim

# --- python tooling --------------------------------------------------------
# numpy is pinned below 1.25: evo will otherwise pull numpy 2.x, which is
# binary-incompatible with the system SciPy and with Humble's rclpy.
RUN pip3 install --no-cache-dir \
      "numpy==1.24.4" \
      "evo==1.37.0"

# --- middleware ------------------------------------------------------------
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# --- headless rendering ----------------------------------------------------
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV IGN_GAZEBO_HEADLESS=1
ENV QT_QPA_PLATFORM=offscreen
ENV SFB_WS=/workspace

# --- workspace -------------------------------------------------------------
WORKDIR /workspace
COPY . /workspace

RUN source /opt/ros/humble/setup.bash \
    && source /deps/install/setup.bash \
    && colcon build --symlink-install

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "source /deps/install/setup.bash" >> /root/.bashrc \
    && echo "source /workspace/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/workspace/docker/entrypoint.sh"]
CMD ["./experiments/scripts/run_trial.sh", \
     "experiments/trajectories/box.yaml", "docker_trial"]
