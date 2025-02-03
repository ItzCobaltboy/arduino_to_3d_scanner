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
1. Install ROS2 Humble [Official Source](https://docs.ros.org/en/humble/Installation.html)
2. Install Libraries using pip
```
pip install pyserial numpy pandas trimesh matplotlib
```
3. Install Arduino IDE [Official Source](https://docs.arduino.cc/software/ide/#ide-v1)

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


------------------------

# Methodology

We approach the 3D scanning problem using a one dimensional ToF sensor, hence we made a turn table design with the sensor fixed on a vertical axis on a pully or rail, the center of turntable is considered origin and we scan the object in horizontal slices and change heights with each slice, effectively giving us Cylindrical Coordinate system point clouds.

This Data is collected into the system  running ROS2 by Pyserial in the node `serial_handler`, which parses it into JSON and forwards it to `data_handler` via `/sensor_data`. The `data_handler` node processes the Cylindrical Coordinate system to cartesian and stores it in a `point_cloud.csv` file. 

We use Matplotlib to generate a scatter plot for showcase. A ConvexHull is generated from the points, and TriMesh constructs a 3D triangular mesh using the vertices and faces extracted from the ConvexHull.
