import rclpy
from rclpy.node import Node
import serial
import json
from std_msgs.msg import String

# Config
ARDUINO_ADDRESS = '/dev/ttyACM0'
BAUD_RATE = 9600
TIMER_FREQUENCY = 25.0  # Hz

class Scanner3DNode(Node):
    def __init__(self):
        super().__init__('scanner_3d')
        
        # Setup serial connection
        self.ser = serial.Serial(ARDUINO_ADDRESS, BAUD_RATE, timeout=1)
        
        # Create a publisher for JSON data
        self.publisher_ = self.create_publisher(String, 'scanner_data', 10)
        
        # Timer to read serial data at fixed rate
        self.timer = self.create_timer(1.0/TIMER_FREQUENCY, self.read_serial_data)
        
        # Initialize scan data structure
        self.scan_data = {
            "sensor_number": None,
            "sensor_reading": None,
            "sensor_offset": None,
            "turntable_angle": None,
            "height": None
        }

    def parse_serial_data(self, data_str):
        """Parse comma-separated serial data into JSON format"""
        try:
            # Split the incoming data by commas
            values = data_str.split(',')
            if len(values) == 5:  # Ensure we have all expected values
                self.scan_data["sensor_number"] = int(values[0])
                self.scan_data["sensor_reading"] = float(values[1])
                self.scan_data["sensor_offset"] = float(values[2])
                self.scan_data["turntable_angle"] = float(values[3])
                self.scan_data["height"] = float(values[4])
                return True
            return False
        except (ValueError, IndexError) as e:
            self.get_logger().error(f'Error parsing serial data: {e}')
            return False

    def read_serial_data(self):
        """Read and process serial data"""
        if self.ser.in_waiting > 0:
            try:
                # Read and decode serial data
                data = self.ser.readline().decode('utf-8').strip()
                
                if data and self.parse_serial_data(data):
                    # Convert to JSON string
                    json_str = json.dumps(self.scan_data)
                    
                    # Create and publish ROS message
                    msg = String()
                    msg.data = json_str
                    self.publisher_.publish(msg)
                    
                    # Log the published data
                    self.get_logger().info(f"Published data: {json_str}")
            
            except serial.SerialException as e:
                self.get_logger().error(f'Serial communication error: {e}')
            except Exception as e:
                self.get_logger().error(f'Unexpected error: {e}')

def main(args=None):
    rclpy.init(args=args)
    scanner_node = Scanner3DNode()
    
    try:
        rclpy.spin(scanner_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        scanner_node.ser.close()
        scanner_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()