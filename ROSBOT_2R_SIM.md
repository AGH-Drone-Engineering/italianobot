# How to prepare workspace for Intalianobot
## Prerequisites

### Specify the path where you want to have directory with the project
If you change the path check if it is free to use (doesn't exist). If it does, then change 
```bash
export MYPATH=~ # use the default or change to your own

# Check if the path exists if you changed it to your own 
if [ -e "$MYPATH" ]; then echo -e "\e[32mPath exists, go ahead with the rest of the instructions.\e[0m"; else echo -e "\e[31mPath does not exist, provide an existing path. \e[0m"; fi

```

### Install all necessary tools
```bash
sudo apt-get update
sudo apt-get install -y python3-pip ros-dev-tools stm32flash
```

 ## Build and run Gazebo simulation 
 (from https://github.com/husarion/rosbot_ros)

 ### Specify HUSARION_ROS_BUILD environmental variable and source the setup.bash file

```bash
export HUSARION_ROS_BUILD=simulation
source /opt/ros/$ROS_DISTRO/setup.bash
```


### Clone repository rosbot_ros
```bash
mkdir -p $MYPATH/ros2_ws/src
cd $MYPATH/ros2_ws/src
git clone https://github.com/husarion/rosbot_ros.git
```

### Building:
```bash
cd $MYPATH/ros2_ws
vcs import src < src/rosbot_ros/rosbot/rosbot_hardware.repos 
vcs import src < src/rosbot_ros/rosbot/rosbot_simulation.repos

# Build only diff_drive_controller and imu_sensor_broadcaster from ros2_controllers
cp -r src/ros2_controllers/diff_drive_controller src && cp -r src/ros2_controllers/imu_sensor_broadcaster src && rm -rf src/ros2_controllers

sudo rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Install italianobot:
```bash
 cd $MYPATH/ros2_ws/src
 git clone https://github.com/AGH-Drone-Engineering/italianobot.git
 cd $MYPATH/ros2_ws
 rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
```

### Install m-explore-ros2:
```bash
 cd $MYPATH/ros2_ws/src
 git clone https://github.com/robo-friends/m-explore-ros2.git
 cd $MYPATH/ros2_ws
 rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
```

### Compile:
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Running:
```bash
cd $MYPATH/ros2_ws
source install/setup.bash 
ros2 launch itabot sim.launch
```
 
 * Tutorial based on:
https://github.com/husarion/rosbot_ros
