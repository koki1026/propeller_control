import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from pynput import keyboard
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from simple_pid import PID
from sensor_msgs.msg import NavSatFix
from rosgraph_msgs.msg import Clock
from math import sqrt, cos, sin, pow, atan2

M_PI = 3.14159265358979323846

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
        self.current_position_sub = self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.current_position_callback, 10)
        self.pre_position = NavSatFix()
        self.pre_position.latitude = 0.0
        self.pre_position.longitude = 0.0
        self.pre_position.altitude = 0.0
        self.position = NavSatFix()
        self.position.latitude = 0.0
        self.position.longitude = 0.0
        self.position.altitude = 0.0

        ##sim時間の取得
        self.sim_time_sub = self.create_subscription(Clock, '/clock', self.sim_time_callback, 10)
        #sim時間が一定時間経過したときにPIDの計算と推進力の計算を行う
        self.sim_time = float()
        self.sim_time_pre = float()
        self.calc_flg = False #sim時間が一定時間経過したかどうかのフラグ
        self.dt = 0.0

        ## pidによる推進力の計算
        self.force_cal = self.create_timer(0.01, self.force_cal)

        ##旋回方向を4タイプに分ける
        ## 1.左前旋回
        ## 2.右前旋回
        ## 3.左後ろ旋回
        ## 4.右後ろ旋回
        self.turning_direction = 1

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
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.sim_time - self.sim_time_pre > 0.1:
            self.calc_flg = True
            self.dt = self.sim_time - self.sim_time_pre
            self.sim_time_pre = self.sim_time

    def deg2rad(self, deg):
        return deg * M_PI / 180.0
        
    #速度の計算
    def velocity_cal(self, lat1, lon1, lat2, lon2):
        lat1 = self.deg2rad(lat1)
        lon1 = self.deg2rad(lon1)
        lat2 = self.deg2rad(lat2)
        lon2 = self.deg2rad(lon2)
        RX = 6378137.0 #赤道半径 (m)
        RY = 6356752.314245 #極半径(m)
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        mu = (lat1 + lat2) / 2.0 #μ
        E = sqrt(1 - pow(RY / RX, 2.0)) #離心率
        W = sqrt(1 - pow(E * sin(mu), 2.0))
        M = RX * (1 - pow(E, 2.0)) / pow(W, 3.0) #子午線曲率半径
        N = RX / W #卯酉線曲率半径
        return sqrt(pow(M * dlat, 2.0) + pow(N * dlon * cos(mu), 2.0)) #距離(m)
    
    #方位角の計算
    def bearing_cal(self, lat1, lon1, lat2, lon2):
        lat1 = self.deg2rad(lat1)
        lon1 = self.deg2rad(lon1)
        lat2 = self.deg2rad(lat2)
        lon2 = self.deg2rad(lon2)
        dlon = lon2 - lon1
        y = sin(dlon) * cos(lat2)
        x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
        return atan2(y, x)

    # 推進力の計算
    def force_cal(self):
        if self.calc_flg == False:
            return
        self.calc_flg = False

        #速度を計算関数
        distance = self.velocity_cal(self.pre_position.latitude, self.pre_position.longitude, self.position.latitude, self.position.longitude) / self.dt
        bearing = self.bearing_cal(self.pre_position.latitude, self.pre_position.longitude, self.position.latitude, self.position.longitude)

        # log
        self.get_logger().info(f'pre_position: {self.pre_position.latitude}, {self.pre_position.longitude}, position: {self.position.latitude}, {self.position.longitude}')

        #前回の位置を更新
        self.pre_position.latitude = self.position.latitude
        self.pre_position.longitude = self.position.longitude

        #目標値のセット
        self.linear_pid_.setpoint = self.target_twist.linear.x
        self.anguler_pid_.setpoint = self.target_twist.angular.z

        #現在値をもとに推進力を計算
        linear_force = self.linear_pid_(self.current_twist.linear.x, dt=self.dt)
        anguler_force = self.anguler_pid_(self.current_twist.angular.z, dt=self.dt)

        # 推進力を左右の推進力に分割
        left_force = linear_force + 0.5 * anguler_force * self.hull_width_
        right_force = linear_force - 0.5 * anguler_force * self.hull_width_

        # log
        self.get_logger().info(f'current_twist: {self.current_twist.linear.x}, target_twist: {self.target_twist.linear.x}')
        self.get_logger().info(f'current_twist_anguler: {self.current_twist.angular.z}, target_twist_anguler: {self.target_twist.angular.z}')
        self.get_logger().info(f'linear_force: {linear_force}, anguler_force: {anguler_force}, left_force: {left_force}, right_force: {right_force}')

        # 推進力をプロペラの回転数に変換
        self.left_prop_speed = Float64()
        self.right_prop_speed = Float64()
        self.left_prop_speed.data = left_force
        self.right_prop_speed.data = right_force

        # 値をマスク
        # if self.left_prop_speed.data < 500.0 and 10.0 < self.right_prop_speed.data:
        #     self.left_prop_speed.data = 500.0
        # elif -500.0 < self.left_prop_speed.data and self.right_prop_speed.data < 10.0:
        #     self.right_prop_speed.data = -500.0
        # elif self.right_prop_speed.data < 500.0 and 10.0 < self.left_prop_speed.data:
        #     self.right_prop_speed.data = 500.0
        # elif -500.0 < self.right_prop_speed.data and self.left_prop_speed.data < 10.0:
        #     self.left_prop_speed.data = -500.0
        DEAD_BAND = 10.0
        if abs(self.left_prop_speed.data) < DEAD_BAND:
            self.left_prop_speed.data = 0.0
        if abs(self.right_prop_speed.data) < DEAD_BAND:
            self.right_prop_speed.data = 0.0

        if self.left_prop_speed.data < -2000.0:
            self.left_prop_speed.data = -2000.0
        elif 2000.0 < self.left_prop_speed.data:
            self.left_prop_speed.data = 2000.0

        if self.right_prop_speed.data < -2000.0:
            self.right_prop_speed.data = -2000.0
        elif 2000.0 < self.right_prop_speed.data:
            self.right_prop_speed.data = 2000.0

        ## terning_directionの決定
        elif self.target_twist.angular.z >= 0.0 and self.target_twist.linear.x >= 0.0:
            self.turning_direction = 1
        elif self.target_twist.angular.z < 0.0 and self.target_twist.linear.x >= 0.0:
            self.turning_direction = 2
        elif self.target_twist.angular.z >= 0.0 and self.target_twist.linear.x < 0.0:
            self.turning_direction = 3
        elif self.target_twist.angular.z < 0.0 and self.target_twist.linear.x < 0.0:
            self.turning_direction = 4

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
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()