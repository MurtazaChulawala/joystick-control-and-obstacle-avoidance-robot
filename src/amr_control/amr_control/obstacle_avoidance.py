import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64

class obstacleAvoidance(Node):
    def __init__(self):
        super().__init__("obstacle_avoidance")
        self.joy_ = self.create_subscription(Twist, "/cmd_vel", self.joy_callback, 10)
        self.sensor_ = self.create_subscription(Int64, "/sensor_data", self.sensor_callback, 10)
        self.pub_ = self.create_publisher(Twist, "/cmd_vel_safe", 10)
        self.linear_ = 0.0
        self.angular_ = 0.0

    def joy_callback(self, msg:Twist):
        self.linear_ = msg.linear.x
        self.angular_ = msg.angular.z   

    def sensor_callback(self, sensor):
        result = sensor.data
        msg = Twist()
        if result < 10:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.pub_.publish(msg)
        else:
            msg.linear.x = self.linear_
            msg.angular.z = self.angular_
            self.pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance = obstacleAvoidance()
    rclpy.spin(obstacle_avoidance)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
