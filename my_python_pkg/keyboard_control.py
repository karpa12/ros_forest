import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard


class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher_ = self.create_publisher(String, 'keyboard_input', 10)
        self.pressed_keys = set()

        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()
        self.create_timer(0.1, self.publish_keys)
    def on_press(self, key):
        try:
            if key.char in ['w', 'a', 's', 'd', 'x']:
                self.pressed_keys.add(key.char)
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            if key.char in ['w', 'a', 's', 'd', 'x']:
                self.pressed_keys.remove(key.char)
        except AttributeError:
            pass

    def publish_keys(self):
        if not self.pressed_keys:
            return
        msg = String()
        msg.data = ','.join(sorted(self.pressed_keys))
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published keys: {msg.data}')


def main(args=None):
    rcplpy.init(args=args)
    node = KeyboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rcplpy.shutdown()
