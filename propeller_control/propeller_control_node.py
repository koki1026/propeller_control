import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from pynput import keyboard
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from simple_pid import PID
from sensor_msgs.msg import NavSatFix
from math import sqrt, cos, sin, pow

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

        # Twist型の変数を定義
        self.target_twist = Twist()
        self.current_twist = Twist()
        ## terget_twistのsubscribe
        self.terget_twist_sub = self.create_subscription(Twist, '/wamv/cmd_vel', self.terget_twist_callback, 10)
        ## current_twistのsubscribe
        self.current_twist_sub = self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.current_twist_callback, 10)
        ## current_positionのsubscribe
        self.current_position_sub = self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.current_twist_callback, 10)
        self.pre_position = NavSatFix()
        self.pre_position.latitude = 0.0
        self.pre_position.longitude = 0.0
        self.pre_position.altitude = 0.0
        self.position = NavSatFix()
        self.position.latitude = 0.0
        self.position.longitude = 0.0
        self.position.altitude = 0.0

        ##sim時間の取得
        self.sim_time_sub = self.create_subscription(Float64, '/simtime', self.sim_time_callback, 10)
        #sim時間が一定時間経過したときにPIDの計算と推進力の計算を行う
        self.sim_time = Float64()
        self.sim_time_pre = Float64()
        self.calc_flg = False #sim時間が一定時間経過したかどうかのフラグ
        self.dt = 0.0

        ## pidによる推進力の計算
        self.force_cal = self.create_timer(0.01, self.force_cal)

        #publisher
        self.left_prop_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_prop_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)

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

    # 目標の速度及び角速度を取得
    def terget_twist_callback(self, msg):
        self.target_twist = msg

    # 現在の角速度を取得
    def current_twist_callback(self, msg):
        self.current_twist.angular.z = msg.angular_velocity.z

    def current_position_callback(self, msg):
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude
        if self.pre_position.latitude == 0.0:
            self.pre_position.latitude = lat
            self.pre_position.longitude = lon
            self.pre_position.altitude = alt
        self.position.latitude = lat
        self.position.longitude = lon
        self.position.altitude = alt

    def sim_time_callback(self, msg):
        self.sim_time = msg
        if self.sim_time.data - self.sim_time_pre.data > 0.1:
            self.calc_flg = True
            self.dt = self.sim_time.data - self.sim_time_pre.data
            self.sim_time_pre.data = self.sim_time.data

    def deg2rad(self, deg)
        return deg * M_PI / 180.0
        
    #速度の計算
    def velocity_cal(self, lat1, lon1, lat2, lon2):
        lat1 = self.deg2rad(lat1)
        lon1 = self.deg2rad(lon1)
        lat2 = self.deg2rad(lat2)
        lon2 = self.deg2rad(lon2)
        RX = 6378137.0 #赤道半径 (m)
        RY = 6356752.314245 #極半径(m)
        dx = lat2 - lat1
        dy = lon2 - lon1
        mu = (lat1 + lat2) / 2.0
        E = sqrt(1 - pow(RY / RX, 2.0)) #離心率
        W = sqrt(1 - pow(E * sin(mu), 2.0))
        M = RX * (1 - pow(E, 2.0)) / pow(W, 3.0) #子午線曲率半径
        N = RX / W #卯酉線曲率半径
        return sqrt(pow(M * dy, 2.0) + pow(N * dx * cos(mu), 2.0)); // 距離[km]

    # 推進力の計算
    def force_cal(self):
        if self.calc_flg == False:
            return
        self.calc_flg = False

        #速度を計算関数
        self.velocity_cal()

        #目標値のセット
        self.linear_pid_.setpoint = self.target_twist.linear.x
        self.anguler_pid_.setpoint = self.target_twist.angular.z

        #現在値をもとに推進力を計算
        linear_force = self.linear_pid_(self.current_twist.linear_acceleration.x, dt=self.dt)
        anguler_force = self.anguler_pid_(self.current_twist.angular_velocity.z, dt=self.dt)

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