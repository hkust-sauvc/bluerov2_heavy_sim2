import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn
from pynput import keyboard

class RCKeyboardController(Node):
    def __init__(self):
        super().__init__('rc_keyboard_controller')

        # Publisher for RC override
        self.publisher_ = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

        # Default RC values (1500 is neutral for most channels)
        self.rc_values = [1500, 1500, 1500, 1500, 1500, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        # Start listening to the keyboard
        self.get_logger().info("Starting keyboard listener...")
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        # Publish RC values at a fixed rate
        self.timer = self.create_timer(0.1, self.publish_rc_override)  # 10 Hz

    def on_press(self, key):
        try:
            # Adjust RC values based on key presses
            if key.char == 'w':  # Move forward (Pitch, Channel 2)
                self.rc_values[4] = max(self.rc_values[1] - 10, 1000)
            elif key.char == 's':  # Move backward (Pitch, Channel 2)
                self.rc_values[4] = min(self.rc_values[1] + 10, 2000)
            elif key.char == 'a':  # Move left (Roll, Channel 1)
                self.rc_values[5] = max(self.rc_values[0] - 10, 1000)
            elif key.char == 'd':  # Move right (Roll, Channel 1)
                self.rc_values[5] = min(self.rc_values[0] + 10, 2000)
            elif key.char == 'u':  # Move up (Increase throttle, Channel 3)
                self.rc_values[2] = min(self.rc_values[2] + 10, 2000)
            elif key.char == 'j':  # Move down (Decrease throttle, Channel 3)
                self.rc_values[2] = max(self.rc_values[2] - 10, 1000)
            elif key.char == 'q':  # Turn left (Yaw, Channel 4)
                self.rc_values[3] = max(self.rc_values[3] - 10, 1000)
            elif key.char == 'e':  # Turn right (Yaw, Channel 4)
                self.rc_values[3] = min(self.rc_values[3] + 10, 2000)

            # Log updated RC values
            self.get_logger().info(f"RC Values Updated: {self.rc_values}")
        except AttributeError:
            pass

    def on_release(self, key):
        # Stop the listener with ESC
        if key == keyboard.Key.esc:
            self.get_logger().info("Exiting keyboard control...")
            self.listener.stop()
            rclpy.shutdown()

    def publish_rc_override(self):
        # Create and publish the RC override message
        msg = OverrideRCIn()
        msg.channels = self.rc_values
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing RC Override: {msg.channels}")

def main(args=None):
    rclpy.init(args=args)
    node = RCKeyboardController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()