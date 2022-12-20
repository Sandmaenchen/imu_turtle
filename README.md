# imu_turtle

This is a simple demo project for robotics courses at UEF that utilize ROS 2 (Humble Hawksbill). In this demo, we obtain quaternion data from an inertial measurement unit (IMU) connected to USB port to control an object in the ROS 2 `turtlesim` tutorial package.

## Prerequisites/Assumptions

- You have installed Ubuntu (v.22.04.1) on Raspberry Pi 4 Model B with at least 4 GB of RAM
- You have installed ROS 2 and configured your environment (sourcing ROS setup files and `colcon` -related functions in `.bashrc` shell startup script)
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

## Author(s)

Matti Kortelainen, PhD 
forename.surname@uef.fi

## License and copyright

Copyright 2022 University of Eastern Finland

This demo project utilizes the [pySerial](https://pyserial.readthedocs.io/en/latest/pyserial.html) module for Python, subject to BSD 3-Clause License.