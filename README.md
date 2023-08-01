# imu_turtle

This is a simple demo project for robotics courses at UEF that utilize ROS 2 (Humble Hawksbill). In this demo, we obtain quaternion data from an inertial measurement unit (IMU) connected to USB port to control an object in the ROS 2 `turtlesim` tutorial package.

## Prerequisites/Assumptions

- You have installed Ubuntu (v.22.04.1) on Raspberry Pi 4 Model B with at least 4 GB of RAM
- You have installed ROS 2 and configured your environment (sourcing ROS setup files and `colcon` -related functions in `.bashrc` shell startup script)
- You have installed `turtlesim` package
- You have installed Python (v. 3.10.6)
- You have installed NumPy (v. 1.23.5)
- You have installed `numpy-quaternion` (v. 2022.4.2)
- You have installed `pyserial` (v. 3.4) and added yourself to `dialout` group
- You have installed `pynput` (v. 1.7.6)
- You have the IMU connected to Raspberry Pi

## Installation

Clone the repository: 
```
cd <your_path>
git clone https://github.com/Sandmaenchen/imu_turtle.git
```

Edit the code in file `node1.py` on line 15 at `<your_path>/imu_turtle/ros2_ws/src/imu_turtle/imu_turtle` to match the USB port where you connected the IMU.

Build the `quaternion_interface` package and the `imu_turtle` package:
```
cd <your_path>/imu_turtle/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
MAKEFLAGS="-j1 -l1" colcon build --executor sequential --packages-select quaternion_interface
MAKEFLAGS="-j1 -l1" colcon build --executor sequential --packages-select imu_turtle
```

## Running the demo

Open a new terminal, navigate to `<your_path>/imu_turtle/ros2_ws`, and source setup files:
```
. install/local_setup.bash
```

Launch the project:
```
ros2 launch imu_turtle imu_turtle_launch.py
```

The above command should open a `turtlesim` window and spawn one turtle-resembling object in it. The object can be controlled with IMU as follows:
- Rotating the IMU around the z-axis changes the angular velocity of the object (the object rotates clockwise/anti-clockwise)
- Rotating the IMU around x-axis changes the linear velocity along x-axis of the object (the object moves forward/backward)
- Rotating the IMU around y-axis changes the linear velocity along y-axis of the object (the object moves left/right)
- Pressing space on keyboard resets the reference IMU orientation to the current orientation

[![Video:]]
(https://uef.cloud.panopto.eu/Panopto/Pages/Embed.aspx?id=3ba3a263-0cb5-4c8e-9624-afb9008ae759&autoplay=false&offerviewer=true&showtitle=false&showbrand=true&captions=true&interactivity=all)    // Video Link

## Author(s)

Matti Kortelainen, PhD 

forename.surname@uef.fi

## License and copyright
```
Copyright 2022 University of Eastern Finland.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```

## Acknowledgments

This demo project was created as a part of the Experts in Medical Computing project, funded by European Social Fund (ESF; S21823) and the Centre for Economic Development, Transport, and the Environment North Savo.

Idea for implementation of `KeyboardPublisher` class came from the [ros2-keyboard-driver](https://github.com/RoverRobotics-archive/ros2-keyboard-driver) repository's [KeystrokeListen](https://github.com/RoverRobotics-archive/ros2-keyboard-driver/blob/master/keystroke/listen.py) class. Otherwise, the codes were mainly written based on the [ROS 2 Humble tutorials](https://docs.ros.org/en/humble/Tutorials.html). 
