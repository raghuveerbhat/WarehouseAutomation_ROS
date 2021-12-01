# Intelligent Robotics Final Project - Warehouse Automation
### Group 5

### IR project environment setup instructions 

Assuming the initial setup of Ubuntu 20.04 LTS and ROS is done, for this warehouse automation project we are setting up the following packages:

##### 1.Setting up AWS Robomaker Small warehouse world (Guide [link](https://www.youtube.com/watch?v=o5Nu2VuYZqA)) :
- Either download and extract the repo from this [link](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world) in catkin/src/ directory or type the following command by opening terminal.
  ```
  cd catkin/src
  git clone https://github.com/aws-robotics/aws-robomaker-small-warehouse-world.git 
  ```
- Build the package by opening terminal and type commands: 
```
cd catkin
catkin_make
```
- Source the workspace by typing following commands:
```
source devel/setup.bash
rospack profile
```
- Load the small warehouse world in gazebo simulation with following commands
```
roslaunch aws_robomaker_small_warehouse_world view_small_warehouse.launch
```
	
**Note** : Now you should be able to see the gazebo simulation platform displaying the small warehouse world. Also we can load other worlds which are available in the launch directory in aws package.
Close the gazebo window and follow the further steps….

##### 2. Setting up turtlebot3 (Guide [link](https://www.youtube.com/watch?v=ji2kQXgCjeM&list=PLRG6WP3c31_XI3wlvHlx2Mp8BYqgqDURU&index=2) and [link](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/))
- Install dependency packages by typing opening a new terminal window and type command
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
```
- Either download and extract the turtlebot3 repo from the this [link](https://github.com/ROBOTIS-GIT/turtlebot3) and [link](https://github.com/ROBOTIS-GIT/turtlebot3_simulations), in catkin/src/ directory or clone it using following terminal commands
```
cd catkin/src 
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations
```
- Now building the package with following command
```
cd catkin
catkin_make
```

**Note** : If the catkin_make is giving error because of unmet dependency of ignition-common3-graphics, then ONLY run following commands (Please ignore if no error found at previous catkin_make):
```
sudo apt-get update
sudo apt-get install lsb-release
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install libignition-physics3-dev
```

##### 3. Spawning the robot in the gazebo 
- To spawn the robot into the gazebo we need to modify the “turtlebot3_world.launch” launch file in  “~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/” directory.

Change line, from ```<arg name="world_name" value="$(find turtlebot3_gazebo)/worlds/turtlebot3_world.world"/>``` to ```<arg name="world_name" value="$(find aws_robomaker_small_warehouse_world)/worlds/no_roof_small_warehouse.world"/>```  and save the changes.
- Execute the following command to spawn the robot in the gazebo
```
cd catkin/
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

**Note**: Now you should be able to see the robot in the gazebo which has aws small warehouse world loaded in it. 

##### 4. Moving the robot in gazebo
With the robot spawned in the previous step, run the following command to move the robot around the world.
- Open the terminal
```
cd catkin/
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
Press keys W,A,X,D robot will do the movements and S key to stop the action.


##### 5. Building the map on RVIZ and saving.
By moving the robot in the gazebo which has aws small warehouse world, the observation of the robot can be projected on to RVIZ. This projection is the map built by the robot which can be saved for further use.  To do this, execute the following commands 
```
cd catkin/
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_slam turtlebot3_slam.launch
```
By executing above commands, RVIZ loads automatically and with movement of robot the map is built on RVIZ which we can save by executing the following commands
```
cd catkin/
rosrun map_server map_saver -f ~/map
```


	









