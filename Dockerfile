# Use the artefacts ROS2 galactic base image
FROM public.ecr.aws/artefacts/ros2:humble-fortress-gpu

# Set the working directory and copy our project
WORKDIR /ws/src

RUN apt update && apt install -y python3-opencv ffmpeg && rm -rf /var/lib/apt/lists/*
RUN git clone -b ros2 https://github.com/rt-net/crane_x7_description.git

RUN apt update && rosdep install --from-paths . --ignore-src -r -y

RUN echo "deb [trusted=yes] https://raw.githubusercontent.com/moveit/moveit2_packages/jammy-humble/ ./" | sudo tee /etc/apt/sources.list.d/moveit_moveit2_packages.list
RUN echo "yaml https://raw.githubusercontent.com/moveit/moveit2_packages/jammy-humble/local.yaml humble" | sudo tee /etc/ros/rosdep/sources.list.d/1-moveit_moveit2_packages.list

# ROS dependencies
RUN apt update && apt install -y ros-humble-moveit-py ros-humble-image-geometry ros-humble-pcl-conversions ros-humble-pcl-ros
#  && rm -rf /var/lib/apt/lists/*

COPY . crane_x7_ros

RUN rosdep install --from-paths crane_x7_ros/crane_x7_gazebo --ignore-src -r -y
RUN rosdep install --from-paths crane_x7_ros/crane_x7_moveit_config --ignore-src -r -y
RUN rosdep install --from-paths crane_x7_ros/crane_x7_control --ignore-src -r -y

# Source ROS version and build
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install --executor parallel


WORKDIR /ws/src/crane_x7_ros

# Source colcon workspace and run the artefacts client
CMD . /ws/src/install/setup.sh && ls && artefacts run $ARTEFACTS_JOB_NAME
