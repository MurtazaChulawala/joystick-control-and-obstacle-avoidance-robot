import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
import serial 
import time

class SensorPublisher(Node):
    def __init__(self):
        super().__init__("sensor_publisher")
        self.pub_ = self.create_publisher(Int64, "sensor_data", 10)
        self.declare_parameter("serial_port", "/dev/ttyACM1")
        self.declare_parameter("baud_rate",115200)
        try: 
            self.ser = serial.Serial(self.get_parameter("serial_port").value, self.get_parameter("baud_rate").value, timeout=1)
            time.sleep(3)
            self.ser.reset_input_buffer()
            time.sleep(0.1)
        except serial.SerialException as e:
            self.get_logger().error(f"failed to open serial port: {e}")
            self.ser = None
        
        self.timer = self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        if self.ser and self.ser.is_open:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode("utf-8").strip()
                msg = Int64()
                msg.data = int(line)
                self.pub_.publish(msg)
        else:
            self.get_logger().warn("serial port is not open")

def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = SensorPublisher()
    rclpy.spin(sensor_publisher)
    sensor_publisher.ser.close()
    rclpy.shutdown()

if __name__ == "__main__":
    main()