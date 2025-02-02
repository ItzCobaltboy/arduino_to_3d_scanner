import rclpy
from rclpy.node import Node
import serial
import json
from std_msgs.msg import String

# Config
ARDUINO_ADDRESS = '/dev/ttyACM0'
BAUD_RATE = 9600
TIMER_FREQUENCY = 10.0  # Hz
LATERAL_SENSOR_X_OFFSET = 140.0
VERTICAL_SENSOR_Z_OFFSET = 10.0

class Scanner3DNode(Node):
    def __init__(self):
        super().__init__('scanner_3d')
        
        # Setup serial connection
        self.ser = serial.Serial(ARDUINO_ADDRESS, BAUD_RATE, timeout=1)
        
        # Create a publisher for JSON data
        self.publisher_ = self.create_publisher(String, 'scanner_data', 10)
        
        # Timer to read serial data at fixed rate
        self.timer = self.create_timer(1.0/TIMER_FREQUENCY, self.read_serial_data)
        self.ser.write(b'S')
        # Buffer for incoming data
        self.buffer = ""
        
        # Initialize scan data structure
        self.scan_data = {
            "sensor_number": None,
            "height": None,
            "distance": None,
            "turntable_angle": None,
            "sensor_offset": None
        }

    def parse_serial_data(self, data_str):
        """Parse comma-separated serial data into JSON format"""
        try:
            # Split the incoming data by commas
            values = data_str.split(',')
            if len(values) == 4:  # Ensure we have all expected values
                sensor_id = int(values[0])
                self.ser.flush()
                # Set appropriate offset based on sensor ID
                if sensor_id == 1:
                    offset = LATERAL_SENSOR_X_OFFSET
                elif sensor_id == 2:
                    offset = VERTICAL_SENSOR_Z_OFFSET
                else:
                    self.get_logger().warn(f'Invalid sensor ID: {sensor_id}')
                    return False
                
                self.scan_data["sensor_number"] = sensor_id
                self.scan_data["height"] = float(values[1])
                self.scan_data["distance"] = float(values[2])
                self.scan_data["turntable_angle"] = float(values[3])
                self.scan_data["sensor_offset"] = offset
                return True
            return False
        except (ValueError, IndexError) as e:
            self.get_logger().error(f'Error parsing serial data: {e}')
            return False

    def read_serial_data(self):
        """Read and process serial data between A and B markers"""
        if self.ser.in_waiting > 0:
            try:
                # Read one byte at a time
                char = self.ser.read().decode('utf-8')
                
                if char == '<':
                    # Start of new data, clear buffer
                    self.buffer = ""
                elif char == '>':
                    # End of data, process buffer
                    if self.buffer:
                        if self.parse_serial_data(self.buffer):
                            # Convert to JSON string
                            json_str = json.dumps(self.scan_data)
                            
                            # Create and publish ROS message
                            msg = String()
                            msg.data = json_str
                            self.publisher_.publish(msg)
                            
                            # Log the published data
                            self.get_logger().info(f"Published data: {json_str}")
                        self.buffer = ""
                else:
                    # Add character to buffer
                    self.buffer += char
                    
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