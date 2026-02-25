import os, time, math, threading
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

import mujoco
import mujoco.viewer

from palletrone_interfaces.msg import Input, PalletroneState, ArmCmd

from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

PHYSICS_HZ = 400.0
ZETA = 0.02
DELAY_TIME = 0.01

SIG_POS   = 1e-3
SIG_VEL   = 1e-3
SIG_GYRO  = 1e-3
SIG_SERVO = 1e-4

COM_PRINT_PERIOD_S = 1.0


def quat_to_rpy(q_wxyz: np.ndarray) -> np.ndarray:
    w, x, y, z = [float(v) for v in q_wxyz]
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    s = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(s)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    return np.array([roll, pitch, yaw], dtype=float)

def rpy_to_R_WB(rpy: np.ndarray) -> np.ndarray:
    r, p, y = float(rpy[0]), float(rpy[1]), float(rpy[2])
    sr, cr = math.sin(r), math.cos(r)
    sp, cp = math.sin(p), math.cos(p)
    sy, cy = math.sin(y), math.cos(y)

    return np.array([
        [ cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [ sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [   -sp,             cp*sr,             cp*cr]
    ], dtype=float)

class PlantRosNode(Node):
    def __init__(self):
        super().__init__("palletrone_plant")

        # -------- Load MuJoCo model --------
        pkg_share = get_package_share_directory("plant")
        xml_path = os.path.join(pkg_share, "xml", "scene.xml")

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = 1.0 / PHYSICS_HZ
        self._last_com_print_t = 0.0
        self.COM_PRINT_PERIOD_S = 1.0

        def aid(name: str) -> int:
            idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name.encode())
            if idx < 0:
                raise RuntimeError(f"Actuator '{name}' not found in XML")
            return int(idx)

        def sid(name: str) -> int:
            idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name.encode())
            if idx < 0:
                raise RuntimeError(f"Sensor '{name}' not found in XML")
            return int(idx)
        

         # -------- Actuators --------
        self.aid_bldc = [aid("drone_BLDC1"), aid("drone_BLDC2"), aid("drone_BLDC3"), aid("drone_BLDC4")]
        self.aid_servo = [
            aid("drone_servo1_pos"),
            aid("drone_servo2_pos"),
            aid("drone_servo3_pos"),
            aid("drone_servo4_pos"),
        ]
 
        self.aid_arm = [
            aid("pos_joint1"),
            aid("pos_joint2"),
            aid("pos_joint3"),
            aid("pos_joint5_L"),
            aid("pos_joint5_R"),
            aid("thruster_left"),
            aid("thruster_right"),
        ]
        self.aid_thruster_left = self.aid_arm[5]
        self.aid_thruster_right = self.aid_arm[6]



        # -------- Sensors --------
        self.sid_quat = sid("drone_body_quat")
        self.sid_gyro = sid("drone_body_gyro")
        self.sid_pos = sid("drone_base_pos")
        self.sid_vel = sid("drone_base_linvel")
        self.sid_servo_ang = [
            sid("drone_servo1_angle"),
            sid("drone_servo2_angle"),
            sid("drone_servo3_angle"),
            sid("drone_servo4_angle"),
        ]

        self.s_adr = self.model.sensor_adr
        self.s_dim = self.model.sensor_dim

        
        # -------- Input buffers --------
        self.ctrl_recv = np.zeros(8, dtype=float)
        self._delay_len = max(1, int(DELAY_TIME * PHYSICS_HZ))
        self._delay_buf = np.zeros((self._delay_len, 8), dtype=float)
        self._delay_idx = 0

        self._arm_enable = False
        self._arm_u = np.zeros(7, dtype=float)



        # -------- State memory --------
        self.prev_pub_t: Optional[float] = None
        self.prev_linvel_W: Optional[np.ndarray] = None
        self.prev_gyro_I: Optional[np.ndarray] = None

        self.sub_input = self.create_subscription(Input, "/input", self.on_input, 10)
        self.sub_arm = self.create_subscription(ArmCmd, "/arm_cmd", self.on_arm_cmd, 10)
        self.pub_state = self.create_publisher(PalletroneState, "/palletrone_state", 10)

        self._dob_enabled = False #dob publish부분
        qos_latch = QoSProfile(depth=1)
        qos_latch.reliability = ReliabilityPolicy.RELIABLE
        qos_latch.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.pub_dob_enable = self.create_publisher(Bool, "/dob_enable", qos_latch)


        self._lock = threading.Lock()
        self._stop = False

        self.viewer_thread = threading.Thread(target=self.viewer_loop, daemon=True)
        self.sim_thread = threading.Thread(target=self.sim_loop, daemon=True)
        self.viewer_thread.start()
        self.sim_thread.start()


    def on_input(self, msg: Input):
        u = np.asarray(msg.u, dtype=float)
        if u.size < 8:
            return
        with self._lock:
            self.ctrl_recv = u[:8].copy() 

    def on_arm_cmd(self, msg: ArmCmd):
        u7 = np.array(
            [msg.joint1, msg.joint2, msg.joint3, msg.joint5_l, msg.joint5_r, msg.thruster_left, msg.thruster_right],
            dtype=float,
        )
        with self._lock:
            self._arm_u = u7
            self._arm_enable = bool(msg.enable)

    def _sensing(self, sid: int) -> np.ndarray:
        adr = self.s_adr[sid]
        dim = self.s_dim[sid]
        return np.array(self.data.sensordata[adr:adr+dim], dtype=float)


    def _delay_step(self) -> np.ndarray:
        self._delay_buf[self._delay_idx] = self.ctrl_recv
        self._delay_idx = (self._delay_idx + 1) % self._delay_len
        return self._delay_buf[self._delay_idx]
    

    # -------- Simulation loop --------
    def sim_loop(self):
        next_step = time.perf_counter()
        next_pub = next_step

        while rclpy.ok() and not self._stop:
            now = time.perf_counter()

            with self._lock:
                u_drone = self._delay_step()

                for i in range(4):
                    omega = float(u_drone[i])
                    if omega < 0.0:
                        omega = 0.0
                    self.data.ctrl[self.aid_bldc[i]] = ZETA * (omega * omega)

                for i in range(4):
                    self.data.ctrl[self.aid_servo[i]] = float(u_drone[4 + i])

                if self._arm_enable:
                    for i in range(7):
                        self.data.ctrl[self.aid_arm[i]] = float(self._arm_u[i])

                while now >= next_step:
                    left = float(self.data.ctrl[self.aid_thruster_left])
                    self.data.ctrl[self.aid_thruster_right] = left

                    mujoco.mj_step(self.model, self.data)
                    next_step += 1.0 / PHYSICS_HZ

                while now >= next_pub:
                    quat_W = self._sensing(self.sid_quat)
                    pos_W = self._sensing(self.sid_pos)
                    vel_W = self._sensing(self.sid_vel)
                    gyro_I = self._sensing(self.sid_gyro)
                    rpy  = quat_to_rpy(quat_W)

                    if (now - self._last_com_print_t) >= self.COM_PRINT_PERIOD_S:
                        com_W = np.array(self.data.subtree_com[0], dtype=float) #MuJoCo 내부 계산값을 읽어서 com 얻기
                        R_WB = rpy_to_R_WB(rpy)
                        pc_B = R_WB.T @ (com_W - pos_W)
                        pcx, pcy, pcz = float(pc_B[0]), float(pc_B[1]), float(pc_B[2])
                        self.get_logger().info(f"pc_B = [{pcx:.4f}, {pcy:.4f}, {pcz:.4f}]")
                        self._last_com_print_t = now
                    	
                    servo = np.array([self._sensing(sid)[0] for sid in self.sid_servo_ang], dtype=float)

                    t = now

                    if self.prev_pub_t is None:
                        acc_W = np.zeros(3, dtype=float)
                        a_rpy = np.zeros(3, dtype=float)
                    else:
                        dt = max(1e-6, t - self.prev_pub_t)
                        acc_W = (vel_W - self.prev_linvel_W) / dt
                        a_rpy = (gyro_I - self.prev_gyro_I) / dt

                    self.prev_pub_t = t
                    self.prev_linvel_W = vel_W.copy()
                    self.prev_gyro_I = gyro_I.copy()

                    msg = PalletroneState()
                    msg.pos = pos_W.tolist()
                    msg.vel = vel_W.tolist()
                    msg.acc = acc_W.tolist()
                    msg.rpy = rpy.tolist()
                    msg.w_rpy = gyro_I.tolist()
                    msg.a_rpy = a_rpy.tolist()
                    msg.servo = servo.tolist()

                    self.pub_state.publish(msg)
                    next_pub += 1.0 / PHYSICS_HZ

            sleep_t = next_step - time.perf_counter()
            if sleep_t > 0:
                time.sleep(sleep_t)

    def viewer_loop(self):
        try:
            with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                while viewer.is_running() and rclpy.ok() and not self._stop:
                    with self._lock:
                        viewer.sync()
        except Exception as e:
            self.get_logger().warn(f"[viewer] ended: {e}")
                        
        except Exception as e:
            self.get_logger().warn(f"[viewer] ended: {e}")

    def close(self):
        self._stop = True


def main():
    rclpy.init()
    node = PlantRosNode()
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
