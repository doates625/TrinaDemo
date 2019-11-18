# TRINA Baxter Embedded Sensor Control Demo
Written by Dan Oates (WPI Class of 2020)

## Install Instructions
These instructions are exclusively for ROS Kinetic on Ubuntu 16.04

### Update Git Submodules
- Run terminal in the repo root
- Update git submodules:
```
git submodule update --init
```

### Add Teensy UDEV Rules
- Copy text from: https://www.pjrc.com/teensy/49-teensy.rules
- Place file in: /etc/udev/rules.d/49-teensy.rules:
```
cd /etc/udev/rules.d
sudo touch 49-teensy.rules
sudo gedit 49-teensy.rules
```
- Reboot your PC for the change to take effect

### Install Baxter Simulator
- Install package dependencies:
```
sudo apt-get install gazebo7 ros-kinetic-qt-build ros-kinetic-gazebo-ros-control ros-kinetic-gazebo-ros-pkgs ros-kinetic-ros-control ros-kinetic-control-toolbox ros-kinetic-realtime-tools ros-kinetic-ros-controllers ros-kinetic-xacro python-wstool ros-kinetic-tf-conversions ros-kinetic-kdl-parser
```
- Run terminal in repo root
- Install Baxter simulator packages:
```
cd Catkin/src
wstool init .
wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter_simulator/kinetic-devel/baxter_simulator.rosinstall
wstool update
```
- Build source:
```
cd ..
source /opt/ros/kinetic/setup.bash
catkin_make
```
- Open Baxter startup script:
```
gedit src/baxter/baxter.sh
```
- Change 'your_ip' on line 26 to "127.0.0.1"
- Change 'ros_version' on line 30 to "kinetic"

### Upload Microcontroller Code
- Open the 'Firmware' project in PlatformIO (Visual Studio Code extension)
- Plug in the Teensy 4.0 with a micro USB cable
- Upload the code by pressing CTRL+ALT+U

### Run ROS Demo
- Run in terminal in the Catkin directory:
```
source devel/setup.bash
./src/baxter/baxter.sh sim
roslaunch trina_demo demo.xml
```
- Twist the encoder and see the left wrist rotate!
