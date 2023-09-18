# franka_panda_gazebo_D435
Simulation in ros using franka panda robot and D435 camera.

# How to run
tips: I created these files on the desktop, if you created these files in other locations, you need to modify the path below accordingly.

test environment: Ubuntu 20.04, ROS Noetic
## 1. Install ROS Noetic
Follow the instructions in the link below to install ROS Noetic.
Link: https://wiki.ros.org/noetic/Installation
## 2.dowload libfranka repository and compile
```
sudo apt -y install build-essential cmake git libpoco-dev libeigen3-dev

git clone --recursive https://github.com/frankaemika/libfranka --branch 0.9.1 && cd libfranka

git checkout 0.9.1&&git submodule update

mkdir build && cd build&&cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF .. &&cmake --build .
```
## 3. create catkin workspace
```
mkdir -p catkin_ws/src&&cd catkin_ws&&source /opt/ros/noetic/setup.sh&&catkin_init_workspace src
```
## 4. Clone this repository
```
cd src && git clone https://github.com/Yang-GG/franka_panda_gazebo_D435.git
```
## 5.compile this package
```
rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka&&catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/Desktop/libfranka/build && echo "source ~/Desktop/catkin_ws/devel/setup.sh">>~/.bashrc&&source ~/.bashrc
```
## 6. run this package
```
sudo lsof -t -i:8080 | xargs -r -I{} sudo kill -9 {}&&roslaunch franka_gazebo panda.launch z:=0.25  world:=$(rospack find franka_gazebo)/world/stone.sdf controller:=cartesian_impedance_example_controller  rviz:=true use_gripper:=true 
```
## 7. run the python file
Run the controller to operate the robotic arm
You can use this repository if you have a gamepad.
Link:https://github.com/Yang-GG/Panda_Robot_PS4_controller.git

or You can use any network debugging helper to send control commands.
recommend tool:

command List:
- RotY-
- RotX+
- RotX-
- RotY+
- MoveToStart
- RotZ-
- RotZ+
- TraZ+
- TraZ-
- TraY+
- TraY-
- TraX-
- TraX+

# Reference
This repository is modified based on the following repository
- [franka_ros 0.10.1](https://github.com/frankaemika/franka_ros) --frankaemika
- [realsense-ros-gazebo](https://github.com/rickstaa/realsense-ros-gazebo.git) --rickstaa

# What changed
- Deleted some codes that are not used in this repository
- Fixed the problem that the color of the point cloud was incorrect.[issue](https://github.com/pal-robotics/realsense_gazebo_plugin/issues/31)

# License
- To use this repository, please refer to the license of the reference repository

# More
- If you have usage problems, please open an issue.
- If you benefit from this repository, please star us. Thank you for your support.
