import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pop.driving_base import DrivingBase


class OmniDriveController(Node):
    """
    Omni Drive Controller.

    /cmd_vel 토픽을 구독하여 로봇 속도 -> 각 바퀴의 각속도로 변환하고 제어
    DrivingBase 라이브러리를 사용하며, 각 바퀴의 속도 명령은 기준값 WHEEL_CENTER(100)을 중심으로
    100보다 작으면 역방향, 100보다 크면 정방향으로 회전

    모터 속도가 증가함에 따라 속도 명령 대비 실제 속도 증가폭이 감소하는 비선형적인 동작을 보여,
    실제 측정한 명령-속도 매핑 테이블을 사용하여 비율을 보정
    """

    def __init__(self):
        super().__init__("omni_drive_controller")

        # ---------------- Configuration ----------------
        self.r = 0.041  # 바퀴 반지름 (m)
        self.R = 0.1465  # 로봇 중심에서 바퀴까지의 거리 (m)
        self.angles_deg = [60.0, 300.0, 180.0]  # 각 바퀴의 각도 (deg)
        self.vel_mapping()  # 명령-속도 매핑 테이블

        # 각도(rad) 및 sin/cos 값 미리 계산
        angles_rad = [math.radians(deg) for deg in self.angles_deg]
        self.sin = [math.sin(rad) for rad in angles_rad]
        self.cos = [math.cos(rad) for rad in angles_rad]

        # ---------------- DrivingBase & Subscriber Initialization ----------------
        self.driver = DrivingBase("can0", 500000)
        self.subscription = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )

    def vel_mapping(self):
        """실제 측정값 기반 명령-속도 매핑 테이블."""
        self.cmd_values = [i for i in range(0, 81)]
        self.actual_velocities = [
            0.0, 2.827898809, 2.97348254, 3.120183281,
            3.270235055, 3.416563459, 3.567359907, 3.713315974,
            3.855548673, 3.996292024, 4.156024557, 4.30495932,
            4.45054305, 4.592403412, 4.758093336, 4.901443045,
            5.047026776, 5.190376485, 5.341545269, 5.487129,
            5.650957239, 5.794306948, 5.957018177, 6.098133864,
            6.250791996, 6.399726759, 6.552757228, 6.697596285,
            6.853977786, 7.004774233, 7.154826007, 7.308601149,
            7.46014227, 7.616523771, 7.765830871, 7.91886134,
            8.054019637, 8.224177604, 8.379814431, 8.532472563,
            8.678056294, 8.844118554, 8.999010708, 9.151296503,
            9.304326971, 9.455868093, 9.612249593, 9.77160979,
            9.924640259, 10.09033018, 10.24373299, 10.39750813,
            10.55091094, 10.70543075, 10.86404628, 11.02154479,
            11.18202199, 11.33803116, 11.49068929, 11.64632612,
            11.80159061, 11.96951455, 12.17057648, 12.2312674,
            12.4118508, 12.57158333, 12.7246138, 12.87429324,
            13.02881305, 13.18444988, 13.32146986, 13.47412799,
            13.62082874, 13.77311453, 13.91236853, 14.07359041,
            14.23034425, 14.38449173, 14.53975622, 14.68869098, 14.82645564
        ]

        self.max_vel = max(self.actual_velocities)  # 최대 바퀴 각속도
        self.get_logger().info(f"Maximum achievable velocity: {self.max_vel:.2f} rad/s")

    def vel_to_cmd(self, target_velocity):
        """목표 각속도(rad/s)를 실제 명령값으로 변환."""
        abs_velocity = abs(target_velocity)
        sign = 1 if target_velocity >= 0 else -1  # 회전 방향 저장

        # 속도가 최대값을 초과하는 경우
        if abs_velocity > self.max_vel:
            return sign * 80  # 속도 한계치가 80

        # 선형 보간을 사용하여 매핑
        cmd = np.interp(abs_velocity, self.actual_velocities, self.cmd_values)

        return sign * int(round(cmd))

    def cmd_vel_callback(self, msg):
        # 수신한 Twist 메시지 (목표 로봇 속도)
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        # 역기구학 (Inverse Kinematics) 계산
        v1 = -self.sin[0] * vx + self.cos[0] * vy + self.R * wz
        v2 = -self.sin[1] * vx + self.cos[1] * vy + self.R * wz
        v3 = -self.sin[2] * vx + self.cos[2] * vy + self.R * wz

        # 각 바퀴의 선속도 -> 각 바퀴의 각속도(rad/s)
        w1 = v1 / self.r
        w2 = v2 / self.r
        w3 = v3 / self.r

        # 바퀴 각속도 한계 초과 시 가장 빠른 바퀴 기준 전체 scaling
        max_wheel = max(abs(w1), abs(w2), abs(w3))
        if max_wheel > self.max_vel:
            scale = self.max_vel / max_wheel
            w1 *= scale
            w2 *= scale
            w3 *= scale
            self.get_logger().info(
                f"Wheel angular velocities scaled by {scale:.2f} to fit hardware limits."
            )

        # 각 바퀴의 목표 속도 logging
        self.get_logger().info(
            f"Wheel Angular Velocities: w1={w1:.2f}, w2={w2:.2f}, w3={w3:.2f}"
        )

        # 목표 속도 - 하드웨어 명령 매핑
        cmd1 = self.vel_to_cmd(w1)
        cmd2 = self.vel_to_cmd(w2)
        cmd3 = self.vel_to_cmd(w3)

        self.driver.wheel_vec[0] = DrivingBase.WHEEL_CENTER + cmd1
        self.driver.wheel_vec[1] = DrivingBase.WHEEL_CENTER + cmd2
        self.driver.wheel_vec[2] = DrivingBase.WHEEL_CENTER + cmd3

        # cmd_vel -> HW 바퀴 명령값 logging
        self.get_logger().info(
            f"CmdVel: [vx:{vx:.2f}, vy:{vy:.2f}, wz:{wz:.2f}] -> Wheels: {self.driver.wheel_vec}"
        )

        # 하드웨어에 명령 전송
        self.driver.transfer()

    # 예상치 못한 종료에 로봇을 멈추도록
    def destroy_node(self):
        self.get_logger().info("Stopping the robot...")
        self.driver.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OmniDriveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():  # ctrl+c shutdown 충돌 방지
            rclpy.shutdown()


if __name__ == "__main__":
    main()
