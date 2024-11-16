# Simulated Robots Package 

This package will provide all of the necesarry files to get a simulated robot up and running. This includes the urdf, configuartion and launch files for a robot capable of mapping and localization as well as autonamous and tele-operated navigation. Currently only a differential drive robot is impremented, a four wheel robot is planned. The package is being written for humble, but once its finished it will be ported to jazzy and foxy (maybe also iron?). 

### Note: work in progress, not ready yet 

## Supported on
Currently supported:
 - [Ubuntu 22.04 LTS](https://releases.ubuntu.com/jammy/)
 - [ROS2 Humble Hawksbill](https://docs.ros.org/en/rolling/Releases/Release-Humble-Hawksbill.html)

 support for foxy and jazzy is planned.

## Dependencies
Get the package dependencies
```bash
sudo apt install -y                         \
    ros-humble-xacro                        \
    ros-humble-navigation2                  \
    ros-humble-slam-toolbox                 \
    ros-humble-gazebo-ros-pkgs              \
```

## Usage 
### Differential Drive Robot
The differential drive robot simulation can be run with the following command:
```bash
ros2 launch sim_bot diff_bot_sim.launch.py 
```
By default a tele-op joy node will be launched that will listen for a controller(xbox, ps4, etc) input. If that is not a suitable option, the robot can also be controlled with this command:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```

The simulation launch file offers serveral launch configurations to modify its behaviour. All of the launch configurations available are listed below:
```bash
Config      Options         Default     Description
world:=     <path_to_world> test.world  Relative path to test world                       
headless:=  True/False      False       Gazebo is visualized if False
rviz:=      True/False      True        Rviz is opened if True
slam:=      True/False      True        Localization and mapping is run if True
nav:=       True/False      True        Navigation stack is launched if True
```

## Install
To use this package please download all of the necesary dependencies first and then follow these steps
```bash
mkdir -p sim_ws/src
cd sim_ws/src
git clone https://github.com/Alexander-Levy/sim_bot.git 
cd ..
colcon build --symlink-install
```
Launch the simulation to test the package
```bash
source ./install/setup.bash
ros2 launch sim_bot diff_bot_sim.launch.py 
```

## TODO 
Package is still being worked on and not yet ready, will be adding these feature over the next couple of days.
### Done
 - [x] add diff drive robot 
 - [x] get simulation running
 - [x] get tele-op joy working
 - [x] add lidar to sim
 - [x] add camera to sim
 - [x] add depth camera
 - [x] add slam to sim
 - [x] add navigation to sim
### Pending
 - [ ] add four wheel robot
 - [ ] port to foxy
 - [ ] port to jazzy
 - [ ] update readme 
