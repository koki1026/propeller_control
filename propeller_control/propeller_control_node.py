import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from pynput import keyboard

class PropellerControlNode(Node):
    def __init__(self):
        super().__init__('propeller_control')
        self.left_prop_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_prop_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)

        self.left_prop_speed = 0.0
        self.right_prop_speed = 0.0

        self.timer = self.create_timer(0.1, self.publish_speeds)
        self.listener = keyboard.Listener(on_press=self.on_key_press, on_release=self.on_key_release)
        self.listener.start()

    def publish_speeds(self):
        # パブリッシュ
        left_msg = Float64()
        right_msg = Float64()
        left_msg.data = self.left_prop_speed
        right_msg.data = self.right_prop_speed

        self.left_prop_pub.publish(left_msg)
        self.right_prop_pub.publish(right_msg)

        self.get_logger().info(f'Published speeds: Left={self.left_prop_speed}, Right={self.right_prop_speed}')

    def on_key_press(self, key):
        try:
            # キー入力に応じた動作
            if key.char == 'w':  # 前進
                self.left_prop_speed = 2000.0
                self.right_prop_speed = 2000.0
            elif key.char == 'a':  # 左回転
                self.left_prop_speed = 1000.0
                self.right_prop_speed = 2000.0
            elif key.char == 'd':  # 右回転
                self.left_prop_speed = 2000.0
                self.right_prop_speed = 1000.0
            elif key.char == 's':  # 停止
                self.left_prop_speed = 0.0
                self.right_prop_speed = 0.0
        except AttributeError:
            pass  # 特殊キーは無視

    def on_key_release(self, key):
        # キーを離したら0に戻す
        self.left_prop_speed = 0.0
        self.right_prop_speed = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = PropellerControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down PropellerControlNode...')
    finally:
        node.listener.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()