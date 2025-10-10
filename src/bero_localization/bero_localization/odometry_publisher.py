import math
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw: float) -> Quaternion:
    """yaw로부터 Quaternion을 계산."""
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


class OdometryPublisherNode(Node):
    """
    Wheel Odometry Publisher.

    /joint_states 토픽을 구독하여
    omni wheel 3개의 각속도로부터 odometry를 계산하고
    /wheel/odom 토픽으로 발행
    publish_tf 파라미터가 True일 경우 odom->base_link TF도 발행
    """

    def __init__(self):
        super().__init__("odometry_publisher")

        # ---------------- Declare Parameter ----------------
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_topic", "/wheel/odom")

        # EKF 등 TF를 외부에서 발행할 경우 False로 설정
        self.declare_parameter("publish_tf", False)

        # 로봇 기구학 파라미터
        self.declare_parameter("wheel_radius_m", 0.041)
        self.declare_parameter("wheel_to_center_m", 0.1465)
        self.declare_parameter("wheel_angles_deg", [60.0, 300.0, 180.0])
        self.declare_parameter("joint_names", ["wheel_joint_left", "wheel_joint_right", "wheel_joint_back"])  # noqa

        # pose, twist Covariance
        self.declare_parameter("pose_cov_diag", [1e-1, 1e-1, 1e6, 1e6, 1e6, 2e-3])
        self.declare_parameter("twist_cov_diag", [1e-2, 1e-2, 1e6, 1e6, 1e6, 1e-3])

        # ---------------- Get Parameter ----------------
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.publish_tf = self.get_parameter("publish_tf").value

        self.r = self.get_parameter("wheel_radius_m").value
        self.R = self.get_parameter("wheel_to_center_m").value
        self.joint_names = self.get_parameter("joint_names").value
        self.pose_cov_diag = self.get_parameter("pose_cov_diag").value
        self.twist_cov_diag = self.get_parameter("twist_cov_diag").value

        angles_rad = [math.radians(a) for a in self.get_parameter("wheel_angles_deg").value]

        # ---------------- Runtime Variables ----------------
        self.sin = [math.sin(a) for a in angles_rad]
        self.cos = [math.cos(a) for a in angles_rad]

        # 로봇의 현재 상태 (x, y, yaw)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # dt 계산용 Steady Clock과 이전 콜백 시각
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.last_callback_time = None

        # ---------------- Sub/Pub Initialization ----------------
        self.joint_sub = self.create_subscription(JointState, "/joint_states", self.joint_state_cb, 10)  # noqa
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        # TF Broadcaster는 publish_tf가 True일 때만 초기화
        self.tf_broadcaster = None
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
            self.get_logger().warning("odom-base_link TF publishing is enabled.")

        self.get_logger().info("Wheel odometry publisher started")

    def joint_state_cb(self, msg: JointState):
        """/joint_states 메시지를 수신하여 오도메트리를 계산하고 발행."""
        # 각 바퀴의 선형 속도 (v = ω * r)
        velocities = [msg.velocity[msg.name.index(name)] for name in self.joint_names]
        v1, v2, v3 = [vel * self.r for vel in velocities]

        # dt 계산용 Steady Clock과 메시지 발행용 joint_state stamp 시각
        now_steady = self.steady_clock.now()
        time_stamp = msg.header.stamp  # /joint_states 발행 시점으로

        # 첫 콜백 초기화
        if self.last_callback_time is None:
            self.last_callback_time = now_steady
            return

        # 이전 콜백과의 시간 간격(s) 계산
        dt_duration = now_steady - self.last_callback_time
        dt_sec = dt_duration.nanoseconds / 1e9

        # 비정상적으로 짧은 주기 방지
        if dt_sec <= 1e-6:
            return

        # 현재 시각 저장 (다음 loop에서 참조)
        self.last_callback_time = now_steady

        # 정기구학 기반 body 기준 속도 계산(120° 배치 기준)
        vx = (-self.sin[0] * v1 - self.sin[1] * v2 - self.sin[2] * v3) / 1.5
        vy = (self.cos[0] * v1 + self.cos[1] * v2 + self.cos[2] * v3) / 1.5
        wz = (v1 + v2 + v3) / (3.0 * self.R)

        # dt_sec 동안 base_link 기준 이동 거리 및 회전량
        dx_b = vx * dt_sec
        dy_b = vy * dt_sec
        dyaw = wz * dt_sec

        # odom 좌표계 기준으로 변환 후 누적
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        self.x += cos_y * dx_b - sin_y * dy_b
        self.y += sin_y * dx_b + cos_y * dy_b
        self.yaw += dyaw
        self.yaw = (self.yaw + math.pi) % (2 * math.pi) - math.pi  # normalization

        # 디버그 로그 출력
        self.get_logger().debug(
            "\n"
            f"vx: {vx:.4f}, vy: {vy:.4f}, wz: {wz:.4f}, dt: {dt_sec:.4f}\n"
            f"dx_b: {dx_b:.4f}, dy_b: {dy_b:.4f}, dyaw: {dyaw:.4f}\n"
            f"x: {self.x:.4f}, y: {self.y:.4f}, yaw: {self.yaw:.4f}"
        )

        # 계산된 odometry 정보 발행
        self.publish_odometry(vx, vy, wz, time_stamp)

    def publish_odometry(self, vx: float, vy: float, wz: float, time_stamp) -> None:
        """계산된 odometry를 /wheel/odom 토픽과 TF로 발행."""
        # Odometry 메시지 생성
        odom = Odometry()
        odom.header.stamp = time_stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # ---------------- Pose ----------------
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quat(self.yaw)

        # Pose covariance (diagonal only)
        odom.pose.covariance = [0.0] * 36
        for i, v in enumerate(self.pose_cov_diag):
            odom.pose.covariance[i*7] = v

        # ---------------- Twist ----------------
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        # Twist covariance (diagonal only)
        odom.twist.covariance = [0.0] * 36
        for i, v in enumerate(self.twist_cov_diag):
            odom.twist.covariance[i*7] = v

        # Wheel_odom 메시지 발행
        self.odom_pub.publish(odom)

        # publish_tf 파라미터가 True일 때만 TF 발행
        if self.tf_broadcaster:
            tf = TransformStamped()
            tf.header.stamp = time_stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
