# Intelligent Robotics Final Project - Warehouse Automation
### Group 5

### Project environment setup instructions

Assuming the initial setup of Ubuntu 20.04 LTS and ROS Noetic is done, for this warehouse automation project we are setting up the following packages:

##### 1. Installing dependencies:
- Run the following commands in terminal
```
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
```
```
sudo apt install ros-noetic-hls-lfcd-lds-driver
sudo apt install ros-noetic-dynamixel-sdk
sudo apt install ros-noetic-turtlebot3-msgs
sudo apt-get install ros-noetic-navigation
```
**Note** : Run the following commands in case on an error while running catkin_make in the subsequent steps:
```
sudo apt-get update
sudo apt-get install lsb-release
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install libignition-physics3-dev
```

##### 2. Setting up package:
- Run the following commands in terminal
```
git clone https://github.com/raghuveerbhat/WarehouseAutomation_ROS.git
cd WarehouseAutomation_ROS
catkin_make
```

##### 3. Verifying package install:
- Source the workspace by typing following commands and load the small warehouse world in gazebo simulation with following commands
```
source devel/setup.bash
roslaunch aws_robomaker_small_warehouse_world view_small_warehouse.launch
```

##### 4. Launching turtlebot3 in the warehouse environment:
```
cd ~/catkin_ws/
source devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
**Note**: Now you should be able to see the robot in the gazebo which has aws small warehouse world loaded in it.

##### 5. Moving the robot in gazebo
- Run the following commands in terminal
```
cd ~/catkin_ws/
source devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
**Note**: Press keys W,A,X,D robot will do the movements and S key to stop the action.

##### 6. Building the map on RVIZ and saving it for future use
The map for the warehouse can be built by moving the robot around the environment. Run the following commands:
```
cd ~/catkin_ws/
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_slam turtlebot3_slam.launch
```
The map can be saved by executing the following commands in terminal
```
cd ~/catkin_ws/
rosrun map_server map_saver -f ~/map
```

##### 7. Further resources to set up packages individually
- Setting up AWS Robomaker Small warehouse world (Guide [link](https://www.youtube.com/watch?v=o5Nu2VuYZqA)) and [link](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world)
- Setting up Turtlebot3 (Guide [link](https://www.youtube.com/watch?v=ji2kQXgCjeM&list=PLRG6WP3c31_XI3wlvHlx2Mp8BYqgqDURU&index=2) and [link](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/))
- Turtlebot3 repositories [link](https://github.com/ROBOTIS-GIT/turtlebot3) and [link](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)
