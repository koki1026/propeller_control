import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from pynput import keyboard
from geometry_msgs.msg import Twist
from simple_pid import PID

class PropellerControlNode(Node):
    def __init__(self):
        super().__init__('propeller_control')

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


        self.linear_pid_ = PID(self.linear_pid_gain_kp_, self.linear_pid_gain_ki_, self.linear_pid_gain_kd_, 0.0)
        self.anguler_pid_ = PID(self.anguler_pid_gain_kp_, self.anguler_pid_gain_ki_, self.anguler_pid_gain_kd_, 0.0)

        ## terget_twistのsubscribe
        self.terget_twist_sub = self.create_subscription(Twist, '/wamv/cmd_vel', self.terget_twist_callback, 10)
        ## current_twistのsubscribe
        self.current_twist_sub = self.create_subscription(Twist, '/wamv/sensors/imu', self.current_twist_callback, 10)
        ## pidによる推進力の計算
        self.force_cal = self.create_timer(0.01, self.force_cal)

        #publisher
        self.left_prop_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_prop_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)

        # Twist型の変数を定義
        self.target_twist = Twist()
        self.current_twist = Twist()


        self.left_prop_speed = 0.0
        self.right_prop_speed = 0.0

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

    # 現在の速度を取得
    def terget_twist_callback(self, msg):
        self.target_twist = msg

    # 目標の速度を取得
    def current_twist_callback(self, msg):
        self.current_twist = msg

    # 推進力の計算
    def force_cal(self):
        #目標値のセット
        self.linear_pid_.setpoint = self.target_twist.linear.x
        self.anguler_pid_.setpoint = self.target_twist.angular.z

        #現在値をもとに推進力を計算
        linear_force = self.linear_pid_(self.current_twist.linear.x)
        anguler_force = self.anguler_pid_(self.current_twist.angular.z)

        # 推進力を左右の推進力に分割
        left_force = linear_force + 0.5 * anguler_force * self.hull_width_
        right_force = linear_force - 0.5 * anguler_force * self.hull_width_

        # log
        self.get_logger().info(f'linear_force: {linear_force}, anguler_force: {anguler_force}, left_force: {left_force}, right_force: {right_force}')

        # 推進力をプロペラの回転数に変換
        self.left_prop_speed = Float64()
        self.right_prop_speed = Float64()
        self.left_prop_speed.data = left_force * 100000
        self.right_prop_speed.data = right_force * 100000

        # 値をマスク
        if self.left_prop_speed.data < 500.0 and 10.0 < self.right_prop_speed.data:
            self.left_prop_speed.data = 500.0
        elif -500.0 < self.left_prop_speed.data and self.right_prop_speed.data < 10.0:
            self.right_prop_speed.data = -500.0
        elif self.left_prop_speed.data < -2000.0:
            self.left_prop_speed.data = -2000.0
        elif 2000.0 < self.left_prop_speed.data:
            self.left_prop_speed.data = 2000.0

        if self.right_prop_speed.data < -2000.0:
            self.right_prop_speed.data = -2000.0
        elif 2000.0 < self.right_prop_speed.data:
            self.right_prop_speed.data = 2000.0
        elif self.right_prop_speed.data < 500.0 and 10.0 < self.left_prop_speed.data:
            self.right_prop_speed.data = 500.0
        elif -500.0 < self.right_prop_speed.data and self.left_prop_speed.data < 10.0:
            self.left_prop_speed.data = -500.0

        # パブリッシュ
        self.left_prop_pub.publish(self.left_prop_speed)
        self.right_prop_pub.publish(self.right_prop_speed)
    


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