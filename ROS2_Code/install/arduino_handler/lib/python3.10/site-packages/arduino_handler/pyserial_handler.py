import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String

###################### Config #########################

arduino_Address = '/dev/ttyACM0'
baud_Rate = 9600
timer_frequency = 25.0



class SerialReader(Node):
    def __init__(self):
        super().__init__('serial_reader')
        
        # Setup serial connection
        self.ser = serial.Serial(arduino_Address, baud_Rate, timeout=1)
        
        # Create a publisher for serial data
        self.publisher_ = self.create_publisher(String, 'serial_data', 10)
        
        # Timer to read serial data at a fixed rate (1Hz)
        self.timer = self.create_timer(timer_frequency, self.read_serial_data)

    def read_serial_data(self):
        if self.ser.in_waiting > 0:
            data = self.ser.readline().decode('utf-8').strip()
            if data:
                msg = String()
                msg.data = data
                self.publisher_.publish(msg)
                self.get_logger().info(f"Published data: {data}")

def main(args=None):
    rclpy.init(args=args)
    
    serial_reader = SerialReader()

    rclpy.spin(serial_reader)

    serial_reader.ser.close()  # Close serial port when shutting down
    rclpy.shutdown()

if __name__ == '__main__':
    main()
