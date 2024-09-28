import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import OverrideRCIn

# enum buttons
# {
#   A,
#   B,
#   X,
#   Y,
#   LB,
#   RB,
#   BACK,
#   START,
#   BRAND,
#   LEFT_MUSHROOM,
#   RIGHT_MUSHROOM
# };

# enum axes
# {
#   LEFT_JOY_HORIZONTAL,
#   LEFT_JOY_VERTICAL,
#   LT,
#   RIGHT_JOY_HORIZONTAL,
#   RIGHT_JOY_VERTICAL,
#   RT,
#   CROSS_HORIZONTAL,
#   CROSS_VERTICAL
# };
# * RC 3 -- vertical
# * RC 4 -- yaw
# * RC 5 -- forward
# * RC 6 -- strafe
class JoystickController(Node):
    def __init__(self):
        super().__init__('joystick_controller')
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.rc_publisher = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.get_logger().info('Joystick Controller Initialized')

    def joy_callback(self, msg):
        # Example: Button 0 arms the drone
        if msg.buttons[6] == 1:  # Check the correct index for your joystick
            arming_req = CommandBool.Request()
            arming_req.value = True
            self.arming_client.call_async(arming_req)

        # Example: Button 1 sets mode to MANUAL
        if msg.buttons[7] == 1:  # Adjust button index as needed
            mode_req = SetMode.Request()
            mode_req.base_mode = 0
            mode_req.custom_mode = 'MANUAL'
            self.set_mode_client.call_async(mode_req)
        
        if msg.buttons[8] == 1:  # Check the correct index for your joystick
            arming_req = CommandBool.Request()
            arming_req.value = False
            self.arming_client.call_async(arming_req)
        
        if msg.buttons[0] == 1:  # Adjust button index as needed
            mode_req = SetMode.Request()
            mode_req.base_mode = 0
            mode_req.custom_mode = 'ALT_HOLD'
            self.set_mode_client.call_async(mode_req)

        # Send RC Overrides based on joystick axes

        rc_message = OverrideRCIn()
        rc_message.channels = [
            1500, 
            1500,  # 
            1500 + int(500 * msg.axes[4]*0.3),  # vertical
            1500 + int(-500 * msg.axes[3]*0.05),  # Yaw
            1500 + int(500 * msg.axes[1]*0.3),  # Forward
            1500 + int(-500 * msg.axes[0]*0.3),  # strafe
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        ]
        self.rc_publisher.publish(rc_message)

def main(args=None):
    rclpy.init(args=args)
    joystick_controller = JoystickController()
    rclpy.spin(joystick_controller)
    joystick_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()