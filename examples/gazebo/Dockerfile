FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-opencv \
      ros-humble-cv-bridge \
      ros-humble-gazebo-ros-pkgs \
      ffmpeg \
      xauth \
      xvfb \
    && rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/bin/bash", "-lc"]
