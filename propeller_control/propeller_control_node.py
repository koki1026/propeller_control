import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from pynput import keyboard

class PropellerControlNode(Node):
    def __init__(self):
        super().__init__('propeller_control')
        self.left_prop_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_prop_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)

        ## pidパラメータの設定
        self.declare_parameter("linear_pid_gain_kp", 0.0)
        self.declare_parameter("linear_pid_gain_ki", 0.0)
        self.declare_parameter("linear_pid_gain_kd", 0.0)
        self.declare_parameter("linear_pid_gain_i_min", 0.0)
        self.declare_parameter("linear_pid_gain_i_max", 0.0)
        self.declare_parameter("linear_pid_gain_antiwindup", False)
        self.declare_parameter("anguler_pid_gain_kp", 0.0)
        self.declare_parameter("anguler_pid_gain_ki", 0.0)
        self.declare_parameter("anguler_pid_gain_kd", 0.0)
        self.declare_parameter("anguler_pid_gain_i_min", 0.0)
        self.declare_parameter("anguler_pid_gain_i_max", 0.0)
        self.declare_parameter("anguler_pid_gain_antiwindup", False)
        self.declare_parameter("hull_width", 1.0)
        self.linear_pid_gain_kp_:float = self.get_parameter("linear_pid_gain_kp").value
        self.linear_pid_gain_ki_:float = self.get_parameter("linear_pid_gain_ki").value
        self.linear_pid_gain_kd_:float = self.get_parameter("linear_pid_gain_kd").value
        self.linear_pid_gain_i_min_:float = self.get_parameter("linear_pid_gain_i_min").value
        self.linear_pid_gain_i_max_:float = self.get_parameter("linear_pid_gain_i_max").value
        self.linear_pid_gain_antiwindup_:bool = self.get_parameter("linear_pid_gain_antiwindup").value
        self.anguler_pid_gain_kp_:float = self.get_parameter("anguler_pid_gain_kp").value
        self.anguler_pid_gain_ki_:float = self.get_parameter("anguler_pid_gain_ki").value
        self.anguler_pid_gain_kd_:float = self.get_parameter("anguler_pid_gain_kd").value
        self.anguler_pid_gain_i_min_:float = self.get_parameter("anguler_pid_gain_i_min").value
        self.anguler_pid_gain_i_max_:float = self.get_parameter("anguler_pid_gain_i_max").value
        self.anguler_pid_gain_antiwindup_:bool = self.get_parameter("anguler_pid_gain_antiwindup").value
        self.hull_width_:float = self.get_parameter("hull_width").value


        self.left_prop_speed = 0.0
        self.right_prop_speed = 0.0

        self.timer = self.create_timer(0.1, self.publish_speeds)
        self.param_timer = self.create_timer(1.0, self.onTick)
        self.listener = keyboard.Listener(on_press=self.on_key_press, on_release=self.on_key_release)
        self.listener.start()

    def onTick(self):
        self.linear_pid_gain_kp_ = self.get_parameter("linear_pid_gain_kp").value
        self.linear_pid_gain_ki_ = self.get_parameter("linear_pid_gain_ki").value
        self.linear_pid_gain_kd_ = self.get_parameter("linear_pid_gain_kd").value
        self.linear_pid_gain_i_min_ = self.get_parameter("linear_pid_gain_i_min").value
        self.linear_pid_gain_i_max_ = self.get_parameter("linear_pid_gain_i_max").value
        self.linear_pid_gain_antiwindup_ = self.get_parameter("linear_pid_gain_antiwindup").value
        self.anguler_pid_gain_kp_ = self.get_parameter("anguler_pid_gain_kp").value
        self.anguler_pid_gain_ki_ = self.get_parameter("anguler_pid_gain_ki").value
        self.anguler_pid_gain_kd_ = self.get_parameter("anguler_pid_gain_kd").value
        self.anguler_pid_gain_i_min_ = self.get_parameter("anguler_pid_gain_i_min").value
        self.anguler_pid_gain_i_max_ = self.get_parameter("anguler_pid_gain_i_max").value
        self.anguler_pid_gain_antiwindup_ = self.get_parameter("anguler_pid_gain_antiwindup").value
        self.hull_width_ = self.get_parameter("hull_width").value

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