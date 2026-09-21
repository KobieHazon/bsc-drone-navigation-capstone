FROM ros:melodic-ros-base-bionic@sha256:b4366ca0536e2ad3654ed66fb4475bc43b7a0c15bd8a5be91fd5ac9dfaf935d0

# ROS Melodic supplies the Python 2 runtime required by these ROS nodes.
RUN apt-get update && apt-get install -y --no-install-recommends \
    git python-numpy python-opencv python-tk python-pil.imagetk python-enum34 \
    ros-melodic-tf ros-melodic-cv-bridge xvfb xauth \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/clydemcqueen/flock.git /tmp/flock \
    && cd /tmp/flock && git checkout 23d96cdbeb051f6690a1cd54ab260a0c455153db \
    && mkdir -p /opt/catkin_ws/src && cp -a flock_msgs /opt/catkin_ws/src/ \
    && cp LICENSE /opt/catkin_ws/src/flock_msgs/LICENSE \
    && /bin/bash -c 'source /opt/ros/melodic/setup.bash && cd /opt/catkin_ws && catkin_make' \
    && rm -rf /tmp/flock

WORKDIR /project
ENV PYTHONDONTWRITEBYTECODE=1
CMD ["bash", "scripts/test_simulation.sh"]
