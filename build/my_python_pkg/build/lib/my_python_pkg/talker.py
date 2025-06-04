import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard
import threading

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        self.pressed_keys = set()
        self.running = True

        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.listener_thread = threading.Thread(target=self.listener.start)
        self.listener_thread.start()

        self.timer = self.create_timer(0.1, self.publish_state)

    def on_press(self, key):
        try:
            self.pressed_keys.add(key.char.lower())
        except AttributeError:
            self.pressed_keys.add(str(key))

    def on_release(self, key):
        try:
            self.pressed_keys.discard(key.char.lower())
        except AttributeError:
            self.pressed_keys.discard(str(key))

        if key == keyboard.Key.esc:
            self.get_logger().info('ESC pressed, shutting down...')
            rclpy.shutdown()

    def publish_state(self):
        if self.pressed_keys:
            msg = String()
            msg.data = '+'.join(sorted(self.pressed_keys))
            self.publisher.publish(msg)
            self.get_logger().info(f'Published: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.running = False
    node.destroy_node()
    rclpy.shutdown()