import rclpy
from rclpy.node import Node
from std_msgs.msg import String
 
class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.publisher_ = self.create_publisher(String, 'test_topic', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
 
    def timer_callback(self):
        msg = String()
        msg.data = 'hello!'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
 
def main(args=None):
    rclpy.init(args=args)
    test_publisher = TestPublisher()
    rclpy.spin(test_publisher)
 
    test_publisher.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
