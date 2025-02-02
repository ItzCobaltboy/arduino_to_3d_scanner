# Custom 3d Scanner using Arduino and ROS2

This is a repository for a system of 3D object scanner made using Lidar sensors and stepper motors, the scan is created by ROS2

# SORRY
## No documentation as of now

but  we have two ROS nodes for data handling, the csv_to_3d_plot.py is our script to make a 3D model out of it (intended to be integrated into a node)

launch using 
```
ros2 launch arduino_handler launch.py
```

# Prerequisites
### Software
- ROS2 Humble
- Arduino IDE
- PySerial

### Hardware
- Two Stepper motors with motor drivers
- Two LIDAR sensors /// TODO

# LAUNCH IN ENv with numpy < 2.0
currently 
``` source myenv/bin/activate ```

