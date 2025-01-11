from ros_tools import ensure_ros_environment
ensure_ros_environment()

# Now we can safely import ROS packages
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BasicTestNode(Node):
    def __init__(self):
        super().__init__('basic_test_node')
        self.publisher = self.create_publisher(String, 'test_topic', 10)
        self.subscriber = self.create_subscription(
            String,
            'test_topic',
            self.listener_callback,
            10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS! Count: {self.counter}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.counter += 1

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

if __name__ == "__main__":
    rclpy.init()

    basic_test_node = BasicTestNode()

    try:
        rclpy.spin(basic_test_node)
    except KeyboardInterrupt:
        pass
    finally:
        basic_test_node.destroy_node()
        rclpy.shutdown()