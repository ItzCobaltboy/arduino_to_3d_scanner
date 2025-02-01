import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/cobaltboy/Desktop/BuildABot/arduino_to_3d_scanner/ROS2_Code/install/arduino_handler'
