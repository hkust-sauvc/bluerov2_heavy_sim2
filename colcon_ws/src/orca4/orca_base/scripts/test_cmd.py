import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn

class RCController(Node):
    def __init__(self):
        super().__init__('rc_controller')
        self.publisher_ = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = OverrideRCIn()
        msg.channels = [1500, 1500, 1500, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing RC Override: "%s"' % msg.channels)

def main(args=None):
    rclpy.init(args=args)
    rc_controller = RCController()
    rclpy.spin(rc_controller)
    rc_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()