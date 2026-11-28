# A ROS 2 environment for testing the node without hardware (and without installing anything on the laptop).
# Build from the project root:  docker build -f docker/Dockerfile -t pid-ros .
# Use the same distro as the robot's Pi if you know it:  --build-arg ROS_DISTRO=jazzy
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-numpy python3-yaml \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY . /app
ENV PYTHONUNBUFFERED=1
# the base image's entrypoint sources /opt/ros/<distro>/setup.bash, then runs the command
CMD ["bash"]
