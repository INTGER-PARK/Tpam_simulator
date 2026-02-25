#!/usr/bin/env python3
import sys, time, tty, termios, select, math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from palletrone_interfaces.msg import Cmd

def get_key_nonblock():
    """Non-blocking single key read (no arrows needed here)."""
    if select.select([sys.stdin], [], [], 0.0)[0]:
        return sys.stdin.read(1)
    return None

def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))

class KeyboardTeleopCmd(Node):
    def __init__(self):
        super().__init__("keyboard_teleop_cmd")

        # publishers
        self.pub_cmd = self.create_publisher(Cmd, "/cmd", 10)
        self.pub_dob = self.create_publisher(Bool, "/dob_enable", 10)

        # params (ros2 param set enable )
        self.declare_parameter("rate_hz", 50.0)

        self.declare_parameter("step_xy", 0.05)      # [m] per key
        self.declare_parameter("step_z",  0.05)      # [m] per key
        self.declare_parameter("z_min",   0.1)
        self.declare_parameter("z_max",   3.0)

        self.declare_parameter("step_yaw_deg", 1.0)  # [deg] per key
        self.declare_parameter("step_rp_deg",  1.0)  # [deg] per key
        self.declare_parameter("roll_lim_deg", 25.0)
        self.declare_parameter("pitch_lim_deg",25.0)

        self.rate_hz = float(self.get_parameter("rate_hz").value)

        self.step_xy = float(self.get_parameter("step_xy").value)
        self.step_z  = float(self.get_parameter("step_z").value)
        self.z_min   = float(self.get_parameter("z_min").value)
        self.z_max   = float(self.get_parameter("z_max").value)

        self.step_yaw = math.radians(float(self.get_parameter("step_yaw_deg").value))
        self.step_rp  = math.radians(float(self.get_parameter("step_rp_deg").value))
        self.roll_lim = math.radians(float(self.get_parameter("roll_lim_deg").value))
        self.pitch_lim= math.radians(float(self.get_parameter("pitch_lim_deg").value))

        # commanded setpoints
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.roll  = 0.0
        self.pitch = 0.0
        self.yaw   = 0.0

        # DOB enable state (node가 살아있는 동안 상태 유지)
        self.dob_enabled = False
        self._publish_dob(self.dob_enabled)

        period = 1.0 / max(1.0, self.rate_hz)
        self.timer = self.create_timer(period, self.on_tick)

        self.get_logger().info(
            "\n[KeyboardTeleopCmd]\n"
            "  Move: w/s (x+/-), a/d (y+/-), r/f (z+/-)\n"
            "  Yaw : q/e (yaw+/-)\n"
            "  Roll: t (+roll)\n"
            "  Pitch: g (+pitch)\n"
            "  DOB toggle: o\n"
            "  Reset attitude: x   Reset pos(x,y): space   Quit: Ctrl+C\n"
        )

    def _publish_dob(self, en: bool):
        m = Bool()
        m.data = bool(en)
        self.pub_dob.publish(m)

    def _publish_cmd(self):
        msg = Cmd()
        msg.pos_cmd[0] = float(self.x)
        msg.pos_cmd[1] = float(self.y)
        msg.pos_cmd[2] = float(self.z)

        msg.rpy_cmd[0] = float(self.roll)
        msg.rpy_cmd[1] = float(self.pitch)
        msg.rpy_cmd[2] = float(self.yaw)

        self.pub_cmd.publish(msg)

    def on_tick(self):
        key = get_key_nonblock()

        if key is None:
            # cmd는 주기적으로 계속 쏴주는 게 안정적
            self._publish_cmd()
            return

        changed = False

        # --- position commands ---
        if key == 'w':        # forward (+x)
            self.x += self.step_xy; changed = True
        elif key == 's':      # backward (-x)
            self.x -= self.step_xy; changed = True
        elif key == 'a':      # left (+y)  (좌표계 반대면 부호만 바꿔)
            self.y += self.step_xy; changed = True
        elif key == 'd':      # right (-y)
            self.y -= self.step_xy; changed = True
        elif key == 'r':      # up
            self.z += self.step_z; changed = True
        elif key == 'f':      # down
            self.z -= self.step_z; changed = True

        # --- yaw commands ---
        elif key == 'q':      # CCW yaw
            self.yaw += self.step_yaw; changed = True
        elif key == 'e':      # CW yaw
            self.yaw -= self.step_yaw; changed = True

        # --- roll/pitch commands (요청대로 한 방향 +step) ---
        elif key == 't':
            self.roll += self.step_rp; changed = True
        elif key == 'g':
            self.pitch += self.step_rp; changed = True

        # --- dob toggle ---
        elif key == 'o':
            self.dob_enabled = not self.dob_enabled
            self._publish_dob(self.dob_enabled)
            self.get_logger().warn(f"DOB {'ENABLED' if self.dob_enabled else 'DISABLED'}")

        # --- resets ---
        elif key == 'x':
            self.roll = 0.0
            self.pitch = 0.0
            self.yaw = 0.0
            changed = True
        elif key == ' ':
            self.x = 0.0
            self.y = 0.0
            changed = True

        # clamp / wrap
        self.z = max(self.z_min, min(self.z_max, self.z))
        self.roll  = max(-self.roll_lim, min(self.roll_lim, self.roll))
        self.pitch = max(-self.pitch_lim, min(self.pitch_lim, self.pitch))
        self.yaw = wrap_pi(self.yaw)

        # publish cmd
        self._publish_cmd()

        if changed:
            self.get_logger().info(
                f"pos=[{self.x:.2f},{self.y:.2f},{self.z:.2f}] "
                f"rpy=[{math.degrees(self.roll):.1f},{math.degrees(self.pitch):.1f},{math.degrees(self.yaw):.1f}] "
                f"DOB={'ON' if self.dob_enabled else 'OFF'}"
            )

def main():
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    rclpy.init()
    node = KeyboardTeleopCmd()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
