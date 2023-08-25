# Use the artefacts ROS2 galactic base image
FROM public.ecr.aws/artefacts/moveit2:humble-fortress-gpu

# Set the working directory and copy our project
WORKDIR /ws/src

RUN git clone -b ros2 https://github.com/rt-net/crane_x7_description.git
RUN git clone -b humble https://github.com/ros-controls/gz_ros2_control.git

#RUN echo "deb [trusted=yes] https://raw.githubusercontent.com/moveit/moveit2_packages/jammy-humble/ ./" | sudo tee /etc/apt/sources.list.d/moveit_moveit2_packages.list
#RUN echo "yaml https://raw.githubusercontent.com/moveit/moveit2_packages/jammy-humble/local.yaml humble" | sudo tee /etc/ros/rosdep/sources.list.d/1-moveit_moveit2_packages.list
# ROS dependencies
#RUN apt update

#RUN apt install ros-humble-moveit-py

COPY . /crane_x7_ros/.

RUN ls

RUN rosdep install --from-paths . --ignore-src -r -y
# Source ROS version and build
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install


WORKDIR /src/crane_x7_ros

# Source colcon workspace and run the artefacts client
CMD . /ws/src/install/setup.sh && artefacts run $ARTEFACTS_JOB_NAME
