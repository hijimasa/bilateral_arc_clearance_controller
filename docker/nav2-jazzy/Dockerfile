FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Resolve dependencies from the manifest in a separate layer. Source changes do
# not invalidate the ROS/Nav2 dependency installation.
COPY package.xml /tmp/bac-src/bilateral_arc_clearance_controller/package.xml
RUN rosdep update --rosdistro jazzy \
    && apt-get update \
    && rosdep install \
      --from-paths /tmp/bac-src \
      --ignore-src \
      --rosdistro jazzy \
      --as-root apt:false \
      -r -y \
    && rm -rf /tmp/bac-src /var/lib/apt/lists/*

CMD ["bash"]
