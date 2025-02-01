#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import numpy as np
import pandas as pd
import math
from typing import Dict

class PointCloudConverter(Node):
    def __init__(self):
        super().__init__('data_processor')
        
        # Subscribe to the scanner data topic
        self.subscription = self.create_subscription(
            String,
            '/scanner_data',
            self.scan_callback,
            10
        )
        
        # Initialize data storage
        self.points_df = pd.DataFrame(columns=['x', 'y', 'z', 'sensor_number'])
        
    def calculate_xyz_sensor1(self, reading: float, angle: float, 
                            sensor_offset: float, height: float) -> tuple:
        """
        Calculate XYZ coordinates for sensor 1 (side scanner)
        Args:
            reading: Distance reading from sensor
            angle: Turntable angle in degrees
            sensor_offset: X-axis offset of sensor from origin
            height: Z-axis height of measurement
        Returns:
            tuple: (x, y, z) coordinates
        """
        # Convert angle to radians
        angle_rad = math.radians(angle)
        
        # Calculate coordinates
        x = sensor_offset - reading * math.cos(angle_rad)
        y = reading * math.sin(angle_rad)
        z = height
        
        return (x, y, z)
    
    def calculate_xyz_sensor2(self, reading: float, angle: float, 
                            height_offset: float) -> tuple:
        """
        Calculate XYZ coordinates for sensor 2 (top scanner)
        Args:
            reading: Distance reading from sensor
            angle: Turntable angle in degrees
            height_offset: Radius offset for the circular scan
        Returns:
            tuple: (x, y, z) coordinates
        """
        # Convert angle to radians
        angle_rad = math.radians(angle)
        
        # Calculate coordinates
        x = height_offset * math.cos(angle_rad)
        y = height_offset * math.sin(angle_rad)
        z = reading
        
        return (x, y, z)
    
    def process_scan_data(self, scan_data: Dict) -> None:
        """
        Process incoming scan data and add to points dataframe
        """
        sensor_num = scan_data['sensor_number']
        reading = scan_data['sensor_reading']
        offset = scan_data['sensor_offset']
        angle = scan_data['turntable_angle']
        height = scan_data['height']
        
        if None in [sensor_num, reading, offset, angle, height]:
            self.get_logger().warn('Received incomplete scan data')
            return
            
        # Calculate XYZ based on sensor number
        if sensor_num == 1:
            x, y, z = self.calculate_xyz_sensor1(reading, angle, offset, height)
        elif sensor_num == 2:
            x, y, z = self.calculate_xyz_sensor2(reading, angle, height)
        else:
            self.get_logger().error(f'Invalid sensor number: {sensor_num}')
            return
            
        # Add point to dataframe
        new_point = pd.DataFrame({
            'x': [x],
            'y': [y],
            'z': [z],
            'sensor_number': [sensor_num]
        })
        self.points_df = pd.concat([self.points_df, new_point], ignore_index=True)
        
    def scan_callback(self, msg: String) -> None:
        """
        Callback function for processing incoming scan data
        """
        try:
            scan_data = json.loads(msg.data)
            self.process_scan_data(scan_data)
            self.get_logger().info(f'Processed point from sensor {scan_data["sensor_number"]}')
        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse JSON data')
        except Exception as e:
            self.get_logger().error(f'Error processing scan data: {str(e)}')
            
    def save_points_to_csv(self, filename: str) -> None:
        """
        Save the accumulated points to a CSV file
        """
        self.points_df.to_csv(filename, index=False)
        self.get_logger().info(f'Saved point cloud to {filename}')

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudConverter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Save the point cloud before shutting down
        node.save_points_to_csv('point_cloud.csv')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()