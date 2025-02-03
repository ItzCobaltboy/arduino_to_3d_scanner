# Custom 3d Scanner using Arduino and ROS2

This is a repository for a system of 3D object scanner made using Lidar sensors and stepper motors, the scan is created by ROS2

**Samples are provided**<br>
Multiple different types of objects, we have a slight error due to scanning at low resolution due to time constraints

-------------------------

# Prerequisites and Setup
### Software
- ROS2 Humble
- PySerial
- Numpy (version 1.XX **NOT** 2.xx) , Pandas
- Trimesh
- MatPlotLib
   
### Hardware
- Arduino UNO
- 1x Stepper Motor and 1x Servo Motor.
- 1x Stepper Driver (ULN2003) and 1x Servo Driver (PCA9685).
- 1x ToF LiDAR Sensor (VL53L0X).

--------------------------
# First time Setup
1. Install ROS2 Humble (Official Source)[https://docs.ros.org/en/humble/Installation.html]
2. Install Libraries using pip
```
pip install pyserial numpy pandas trimesh matplotlib
```
3. Install Arduino IDE (Official Source)[https://docs.arduino.cc/software/ide/#ide-v1]

--------------------------
# Launching

1. Update the Code on Arduino in the provided `.ino` file, and reset it.
2. Source the ROS2 Package.
```
source ROS2_code/install/setup.bash
```
3. Launch the ROS Nodes.
```
ros2 launch arduino_handler launch.py
```
4. Collect the data, and when u hit `^C` then the data is automatically saved as `.csv.` format.
5. Place the data in same folder as `point_cloud_to_3d.py` and run the script to get the Scatter Plot and `.stl` object.

