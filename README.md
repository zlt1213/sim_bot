# Simulated Robots Package 

This package will provide all of the necesarry files to get a simulated robot up and running. This includes the urdf, configuartion and launch files for a robot capable of mapping and localization as well as autonamous and tele-operated navigation. Currently only a differential drive robot is impremented, a four wheel robot is planned. The package is being written for humble, but once its finished it will be ported to jazzy and foxy (maybe also iron?). 

### Note: work in progress, not ready yet 

## Supported on
Currently supported:
 - [Ubuntu 22.04 LTS](https://releases.ubuntu.com/jammy/)
 - [ROS2 Humble Hawksbill](https://docs.ros.org/en/rolling/Releases/Release-Humble-Hawksbill.html)

 support for foxy and jazzy is planned.

## Dependencies
Currently the package depends on:
 - xacro
 - gazebo
 - slam-toolbox
 - navigation2
 - ros2 control                 
 - ros2 controllers            
 - ros2 control-gazebo plugin  

```bash
sudo apt install -y                         \
    ros-humble-xacro                        \
    ros-humble-navigation2                  \
    ros-humble-slam-toolbox                 \
    ros-humble-ros2-control                 \
    ros-humble-ros2-controllers             \
    ros-humble-gazebo-ros-pkgs              \
    ros-humble-gazebo-ros2-control          \
```

## Usage 
### Differential Drive Robot
The differential drive robot simulation can be run with the following command:
```bash
ros2 launch sim_bot diff_bot_sim.launch.py 
```

The simulation launch file offers serveral launch configurations to modify its behaviour. All of the launch configurations available are listed below:
```bash
Config      Options      Default  Description                          
headless:=  True/False   False    Gazebo is visualized if False
rviz:=      True/False   True     Rviz is opened if True
```

## Install
To use this package please download all of the necesary dependencies first and then follow these steps
```bash
cd ~/<path_to_your_workspace>/src
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
 - [x] add diff drive robot 
 - [ ] add four wheel robot
 - [x] get simulation running
 - [x] get tele-op joy working
 - [x] add lidar to sim
 - [ ] add camera
 - [ ] add depth camera
 - [ ] add slam to sim
 - [ ] add navigation to sim
 - [ ] add ros2 control to sim
 - [ ] port to foxy
 - [ ] port to jazzy
 - [ ] update readme 
