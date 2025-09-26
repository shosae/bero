import math
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import JointState
from pop.Encoder import Encoder

INT16_MAX = 32767
INT16_MIN = -32768
INT16_RANGE = 65536

def wrap_around(curr: int, prev: int) -> int:
    """
    Encoder가 int16 (-32768~32767) 범위를 순환
    wrap around 보정 함수
    """
    delta = curr - prev
    if delta > INT16_MAX:
        delta -= INT16_RANGE
    elif delta < INT16_MIN:
        delta += INT16_RANGE
    return delta

class JointStatePublisherNode(Node):
    """
    10Hz로 발행되는 Encoder에서 값을 읽어
    omni wheel 3개의 각도(position)와 각속도(velocity)를
    /joint_states 토픽으로 발행
    """
    def __init__(self):
        super().__init__("joint_state_publisher")

        # ---------------- 파라미터 ----------------
        # CPR(바퀴축 기준, 모든 바퀴 동일하게 1875.0)
        self.declare_parameter("encoder_cpr_wheel", 1875.0)

        # Joint 이름
        self.declare_parameter("joint_names", ["wheel_joint_left", "wheel_joint_right", "wheel_joint_back"])

        # ---------------- 파라미터 로드 ----------------
        self.cpr = float(self.get_parameter("encoder_cpr_wheel").value)
        self.joint_names = self.get_parameter("joint_names").value

        # ---------------- 상태 변수 ----------------
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME) # Steady time, 시간을 정확하게 측정하기 위함
        
        self.last_callback_time = None   # dt 계산용
        self.last_ticks = (0, 0, 0) # (L, R, B)
        self.wheel_positions = [0.0, 0.0, 0.0]  # 누적 각도 (rad)

        # ---------------- 퍼블리셔 및 엔코더 ----------------
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.encoder = Encoder("can0", "500000")
        self.encoder.callback(self.encoder_cb, repeat=1)

        self.get_logger().info("Joint state publisher started.")


    # encoder callback
    def encoder_cb(self, data: tuple[int, int, int, int]) -> None:
        staus, e1, e2, e3 = data
        now_ros = self.get_clock().now()
        now_steady = self.steady_clock.now()

        # 첫 콜백 초기화
        if self.last_callback_time is None:
            self.last_ticks = (e1, e2, e3)
            self.last_callback_time = now_steady
            return

        dt_duration = now_steady - self.last_callback_time
        dt_sec = dt_duration.nanoseconds / 1e9

        # 속도 값이 비정상적으로 튀는 현상 방지
        if dt_sec < 1e-6:
            return

        # Δtick (wrap around 보정)
        ticks = (e1, e2, e3)
        delta_ticks = [
            wrap_around(ticks[i], self.last_ticks[i])
            for i in range(len(ticks))
        ]

        # ticks, 시간 저장
        self.last_ticks = ticks
        self.last_callback_time= now_steady

        # 각도 및 각속도 계산
        # delta_angle (rad) = (delta_tick / CPR) * 2 * PI
        delta_angles = [
            (d_tick / self.cpr) * 2.0 * math.pi
            for d_tick in delta_ticks
        ]

        # 각속도 (rad/s)
        velocities = [
            da / dt_sec
            for da in delta_angles
        ]

        # 누적 각도
        for i in range(3):
            self.wheel_positions[i] += delta_angles[i]

        # JointState 메시지 발행
        msg = JointState()
        msg.header.stamp = now_ros.to_msg()
        msg.name = self.joint_names
        msg.position = self.wheel_positions
        msg.velocity = velocities
        self.joint_state_pub.publish(msg)

    # encoder thread 확실하게 종료 
    def destroy_node(self):
        try:
            self.encoder.stop()
        except Exception as e:
            self.get_logger().error(f"Failed to stop the encoder during shutdown: {e}")
        super().destroy_node()

def main():
    rclpy.init()
    node = JointStatePublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
