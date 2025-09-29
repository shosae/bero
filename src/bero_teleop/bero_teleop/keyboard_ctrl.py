import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios

# 키보드 조작 설명 메시지
MSG = """
Keyboard Controller
---------------------------
Moving around:
   w
a  s  d
   x

q/e : rotate left/right

s : stop

u/j : increase/decrease linear speed by 10%
i/k : increase/decrease angular speed by 10%

CTRL-C to quit
"""

# 키 입력에 따른 이동 방향 정의 (vx, vy, wz)
MOVE_BINDINGS = {
    "w": (1.0, 0.0, 0.0),
    "a": (0.0, 1.0, 0.0),
    "d": (0.0, -1.0, 0.0),
    "x": (-1.0, 0.0, 0.0),
    "q": (0.0, 0.0, 1.0),
    "e": (0.0, 0.0, -1.0),
}

# 키 입력에 따른 속력 조절 정의 (10%씩 증감)
SPEED_BINDINGS = {
    "u": (1.1, 1.0),
    "j": (0.9, 1.0),
    "i": (1.0, 1.1),
    "k": (1.0, 0.9),
}


class KeyboardControllerNode(Node):
    """
    Keyboard Controller Node.

    주기적으로 Twist message를 발행
    한 번에 하나의 키 입력만 처리하여 로봇의 방향 및 속도를 제어
    """

    def __init__(self):
        super().__init__("keyboard_controller_node")

        # ---------------- Configuration ----------------
        self.settings = termios.tcgetattr(sys.stdin)  # 현재 터미널의 설정을 저장
        self.timer_period = 0.1  # 키보드 입력 읽는 주기 (s)

        # ---------------- Declare Parameter ----------------
        # 초기 선, 각속도 지정
        self.declare_parameter("lin_vel", 0.35)
        self.declare_parameter("ang_vel", 2.5)

        # ---------------- Get Parameter ----------------
        self.lin_vel = self.get_parameter("lin_vel").value
        self.ang_vel = self.get_parameter("ang_vel").value

        # ---------------- Runtime Variables ----------------
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0

        # ---------------- Timer & Publisher Initialization ----------------
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.print_status()

    def get_key(self, timeout: float) -> str:
        """
        터미널로부터 키 입력을 non-blocking 방식으로 읽음.

        지정된 timeout 동안 키 입력이 없으면 빈 문자열을 반환
        """
        fd = sys.stdin.fileno()  # file descriptor 받아오기
        tty.setraw(fd)  # 터미널을 raw 모드로 전환(엔터 없이 키 입력 즉시 읽기)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:  # 입력이 있으면
            key = sys.stdin.read(1)  # 한 글자 읽기
        else:  # 입력이 없다면
            key = ""  # 빈 문자열 반환
        termios.tcsetattr(fd, termios.TCSADRAIN, self.settings)
        return key.lower()  # 소문자로 변환해서 반환

    def timer_callback(self):
        """
        타이머에 의해 주기적으로 호출되는 callback 함수.

        키 입력을 확인하고, 그에 따른 로봇의 속도를 계산하여 Twist 메시지를 발행
        """
        key = self.get_key(timeout=self.timer_period)

        if key in MOVE_BINDINGS:  # 방향
            self.target_vx = MOVE_BINDINGS[key][0]
            self.target_vy = MOVE_BINDINGS[key][1]
            self.target_wz = MOVE_BINDINGS[key][2]
        elif key in SPEED_BINDINGS:  # 속도, 최대속도 클램핑
            self.lin_vel *= SPEED_BINDINGS[key][0]
            self.ang_vel *= SPEED_BINDINGS[key][1]
            self.lin_vel = max(0.13, min(self.lin_vel, 0.7))
            self.ang_vel = max(0.4, min(self.ang_vel, 4.15))
            self.print_status()
        elif key == "s":  # 정지
            self.target_vx = 0.0
            self.target_vy = 0.0
            self.target_wz = 0.0
        elif key == "\x03":  # Ctrl+C
            twist = Twist()  # 속도가 0인 Twist 메시지 생성
            self.cmd_vel_pub.publish(twist)
            raise KeyboardInterrupt
        else:
            pass

        # Twist 메시지 생성 및 발행
        twist = Twist()
        twist.linear.x = self.target_vx * self.lin_vel
        twist.linear.y = self.target_vy * self.lin_vel
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.target_wz * self.ang_vel
        self.cmd_vel_pub.publish(twist)

    def print_status(self):
        """terminal에 조작법과 현재 설정 속도를 출력."""
        print(MSG)
        print(f"currently:\tlinear speed {self.lin_vel:.2f}\tangular speed {self.ang_vel:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Stopping node!!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
