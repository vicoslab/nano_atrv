#!/usr/bin/env python3
import math
import time
import tf2_ros
import rclpy

from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import JointState

from vassar_feetech_servo_sdk import ServoController
from scservo_sdk import GroupSyncRead

TICKS_PER_DEG = 4096.0 / 360.0
STEER_CENTER = 2048

#Each setvo set up with https://github.com/Kotakku/FT_SCServo_Debug_Qt
LEFT_STEER_MIN  = 870    # most-clockwise position (robot-frame: steer right)
LEFT_STEER_MAX  = 2488   # most-counter-clockwise  (steer left)
RIGHT_STEER_MIN = 1618   # most-clockwise          (steer left  for right wheel)
RIGHT_STEER_MAX = 3214   # most-counter-clockwise  (steer right)
REAR_STEER_MIN  = 0
REAR_STEER_MAX  = 4096

def deg_to_ticks(degrees):
    """Convert an angle offset in degrees to a tick offset (signed)."""
    return int(round(degrees * TICKS_PER_DEG))

def angle_to_left_ticks(angle_deg):
    """
    Convert a desired steering angle (degrees, positive = steer left / CCW)
    to an absolute tick value for the LEFT steering servo (ID 2).
    Clamped to the servo's physical limits.
    """
    ticks = STEER_CENTER + deg_to_ticks(angle_deg)
    return max(LEFT_STEER_MIN, min(LEFT_STEER_MAX, ticks))


def angle_to_right_ticks(angle_deg):
    """
    Convert a desired steering angle (degrees, positive = steer left / CCW)
    to an absolute tick value for the RIGHT steering servo (ID 3).
    The right servo is mirrored, so a positive (left-turn) angle subtracts ticks.
    Clamped to the servo's physical limits.
    """
    ticks = STEER_CENTER - deg_to_ticks(angle_deg)
    return max(RIGHT_STEER_MIN, min(RIGHT_STEER_MAX, ticks))


def angle_to_rear_ticks(angle_deg):
    """
    Convert a desired rear-caster angle (degrees, positive = CW / right)
    to an absolute tick value for the REAR steering servo (ID 5).
    Subtracts offset from centre because the rear servo is mirrored.
    Clamped to ±90°.
    """
    ticks = STEER_CENTER - deg_to_ticks(angle_deg)
    return max(REAR_STEER_MIN, min(REAR_STEER_MAX, ticks))

def _nonzero(val, tol=1e-3):
    return abs(val) > tol

class DriveMode:
    STOP = "stop"
    SPIN = "spin"       # pure angular.z  → spin in place
    DIFFDRIVE = "diffdrive"  # linear.x (+ optional angular.z) → forward/curve
    CRAB = "crab"       # pure linear.y   → sideways translation

def classify_mode(v_x, v_y, omega, tol=1e-3):
    has_x = _nonzero(v_x,   tol)
    has_y = _nonzero(v_y,   tol)
    has_omega = _nonzero(omega, tol)

    if not has_x and not has_y and not has_omega:
        return DriveMode.STOP
    if has_y and not has_x and not has_omega:
        return DriveMode.CRAB
    if has_omega and not has_x and not has_y:
        return DriveMode.SPIN
    return DriveMode.DIFFDRIVE

class FeetechDiffDrive(Node):
    def __init__(self):
        super().__init__("feetech_diff_drive")

        # Serial
        self.declare_parameter("port", "/dev/ttyWHEELS")
        self.declare_parameter("baudrate", 1000000)

        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value

        # Geometry
        self.declare_parameter("wheel_diameter", 0.17)
        self.declare_parameter("track_width", 0.355)
        self.declare_parameter("wheelbase", 0.35)
        self.declare_parameter("rear_caster_offset", 0.175)
        self.declare_parameter("spin_toe_deg", -1.0)

        self.wheel_diameter = self.get_parameter("wheel_diameter").value
        self.track_width = self.get_parameter("track_width").value
        self.wheelbase = self.get_parameter("wheelbase").value
        self.rear_caster_offset = self.get_parameter("rear_caster_offset").value

        _spin_toe_raw = self.get_parameter("spin_toe_deg").value
        self.spin_toe_override_deg = _spin_toe_raw if _spin_toe_raw >= 0.0 else None

        self.spin_toe_deg = (
            self.spin_toe_override_deg
            if self.spin_toe_override_deg is not None
            else math.degrees(math.atan2(self.track_width, self.wheelbase))
        )
        self.get_logger().info(
            "Spin-in-place front wheel angle: {:.1f} deg (track={}, wheelbase={})".format(
                self.spin_toe_deg, self.track_width, self.wheelbase
            )
        )

        # Drive limits
        self.declare_parameter("max_velocity_ticks", 65)
        self.declare_parameter("accel_ticks", 100)
        self.declare_parameter("spin_radius_threshold", 0.05)

        self.max_velocity_ticks = self.get_parameter("max_velocity_ticks").value
        self.accel_ticks = self.get_parameter("accel_ticks").value
        self.spin_radius_threshold = self.get_parameter("spin_radius_threshold").value

        # Servo IDs
        self.declare_parameter("left_servo_id", 1)
        self.declare_parameter("left_steer_servo_id", 2)
        self.declare_parameter("right_steer_servo_id", 3)
        self.declare_parameter("right_servo_id", 4)
        self.declare_parameter("rear_steer_servo_id", 5)

        self.left_drive_id = self.get_parameter("left_servo_id").value
        self.left_steer_id = self.get_parameter("left_steer_servo_id").value
        self.right_steer_id = self.get_parameter("right_steer_servo_id").value
        self.right_drive_id = self.get_parameter("right_servo_id").value
        self.rear_steer_id = self.get_parameter("rear_steer_servo_id").value

        # Frames
        self.declare_parameter("base_frame", "base_link")
        self.base_frame = self.get_parameter("base_frame").value

        # Rates
        self.declare_parameter("deadman_timer_sec", 1.0)
        self.declare_parameter("rate", 30)

        self.deadman_timer_sec = self.get_parameter("deadman_timer_sec").value       
        self.rate_hz = self.get_parameter("rate").value

        self.last_cmd_time = self.get_clock().now() - Duration(seconds=self.deadman_timer_sec + 1.0)

        # Internal constants
        self.read_hz_scale = 0.0122
        self.radius = self.wheel_diameter / 2.0
        self.circumference = math.pi * self.wheel_diameter

        # State
        self.cmd_vx = 0.0 # linear.x  (m/s)
        self.cmd_vy = 0.0 # linear.y  (m/s)
        self.cmd_w = 0.0 # angular.z (rad/s)
        self.current_mode = DriveMode.STOP

        # Topics
        self.battery_pub = self.create_publisher(BatteryState, "battery_state", 5)
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)

        self.cmd_sub = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_cb, 10)

        # Servo Setup
        self.servo_setup()

        # Timers
        self.main_timer = self.create_timer(1.0 / self.rate_hz, self.update)
        self.battery_timer = self.create_timer(1.0, self.update_battery)

        self.get_logger().info("Feetech diff drive node initialised (with steering)")

    def servo_setup(self):
        self.controller = ServoController(
            servo_ids=[
                self.left_drive_id,
                self.left_steer_id,
                self.right_steer_id,
                self.right_drive_id,
                self.rear_steer_id,
            ],
            servo_type="sts",
            port=self.port,
            baudrate=self.baudrate,
        )
        self.controller.connect()

        #Caching and group reading to prevent high CPU usage
        self.prev_left_ticks = None
        self.prev_right_ticks = None

        self.prev_left_deg = None
        self.prev_right_deg = None
        self.prev_rear_deg = None

        STS_PRESENT_POSITION_L = 56
        READ_LEN = 4  # position (2) + speed (2)

        self.sync_all = GroupSyncRead(self.controller.packet_handler, STS_PRESENT_POSITION_L, READ_LEN)
        self.sync_all.addParam(self.left_drive_id)
        self.sync_all.addParam(self.right_drive_id)
        self.sync_all.addParam(self.left_steer_id)
        self.sync_all.addParam(self.right_steer_id)
        self.sync_all.addParam(self.rear_steer_id)

        # Drive servos → velocity / wheel-mode
        self.controller.packet_handler.WheelMode(self.left_drive_id)
        self.controller.packet_handler.WheelMode(self.right_drive_id)

        # Enable torque
        self.controller.packet_handler.write1ByteTxRx(self.left_drive_id, 40, 1)
        self.controller.packet_handler.write1ByteTxRx(self.right_drive_id, 40, 1)
        self.controller.packet_handler.write1ByteTxRx(self.left_steer_id, 40, 1)
        self.controller.packet_handler.write1ByteTxRx(self.right_steer_id, 40, 1)
        self.controller.packet_handler.write1ByteTxRx(self.rear_steer_id, 40, 1)

        # Park steering straight ahead
        self.set_steering(0.0, 0.0, 0.0)

    def cmd_vel_cb(self, msg):
        self.cmd_vx  = msg.linear.x
        self.cmd_vy = msg.linear.y
        self.cmd_w  = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def update(self):
        self.update_motors()
        self.publish_joints()

    def set_wheel_speeds(self, left_ticks, right_ticks):
        if left_ticks != self.prev_left_ticks:
            self.prev_left_ticks = left_ticks
            self.controller.packet_handler.WriteSpec(self.left_drive_id, -left_ticks, self.accel_ticks)

        if right_ticks != self.prev_right_ticks:
            self.prev_right_ticks = right_ticks
            self.controller.packet_handler.WriteSpec(self.right_drive_id, right_ticks, self.accel_ticks)

    def set_steering(self, left_deg, right_deg, rear_deg):
        """
        Set steering angles for front and rear wheels.
        
        For the rear freewheeling axle: angles that differ by 180° are equivalent.
        Chooses the shortest path considering 180° symmetry.
        """
        positions = {}

        if left_deg != self.prev_left_deg:
            positions[self.left_steer_id] = angle_to_left_ticks(left_deg)
            self.prev_left_deg = left_deg

        if right_deg != self.prev_right_deg:
            positions[self.right_steer_id] = angle_to_right_ticks(right_deg)
            self.prev_right_deg = right_deg

        if rear_deg != self.prev_rear_deg:
            if self.prev_rear_deg is None:
                positions[self.rear_steer_id] = angle_to_rear_ticks(rear_deg)
                self.prev_rear_deg = rear_deg
            else:
                LIMIT_MIN = -179.0
                LIMIT_MAX = 179.0

                # Identify 180-degree mirrors and normalize to stay within a reasonable wrapping range
                candidates = [rear_deg, rear_deg + 180, rear_deg - 180]
                
                # Filter candidates that are actually reachable by the hardware
                valid_candidates = [c for c in candidates if LIMIT_MIN <= c <= LIMIT_MAX]

                if not valid_candidates:
                    chosen_physical = max(LIMIT_MIN, min(LIMIT_MAX, rear_deg))
                else:
                    # Pick the candidate closest to the CURRENT physical position
                    chosen_physical = min(valid_candidates, key=lambda c: abs(c - self.prev_rear_deg))

                if abs(chosen_physical - self.prev_rear_deg) > 0.5:
                    positions[self.rear_steer_id] = angle_to_rear_ticks(chosen_physical)
                    self.prev_rear_deg = chosen_physical

        if positions:
            self.controller.write_position(positions)

    def update_motors(self):

        def linear_to_ticks(v_ms):
            """Convert a wheel surface speed (m/s) to servo tick speed."""
            rev_per_sec = v_ms / self.circumference
            return int(rev_per_sec / self.read_hz_scale)

        def speed_to_ticks(v_l, v_r):
            """Convert left/right wheel speeds (m/s) to clamped tick values."""
            tl = linear_to_ticks(v_l)
            tr = linear_to_ticks(v_r)
            tl = max(-self.max_velocity_ticks, min(self.max_velocity_ticks, tl))
            tr = max(-self.max_velocity_ticks, min(self.max_velocity_ticks, tr))
            return tl, tr

        def apply_spin(omega):
            """
            Rotate the robot around its centre of mass.
            Each wheel must be tangent to its circle around the centre, i.e.
            perpendicular to its radius vector.
            Front wheels:
            - Position: (±track/2, +wheelbase/2) from centre
            - Radius magnitude: sqrt((track/2)² + (wheelbase/2)²)
            - Steering angle from straight-ahead: atan(track_width / wheelbase)
            - Left wheel steers CCW (+), right wheel steers CW (-)
            Rear caster:
            - Position: (0, -rear_caster_offset) → purely behind centre
            - Perpendicular to that radius = exactly 90° from straight-ahead
            - For positive omega (CCW robot rotation) the caster points left: +90°
            Drive speed:
            - v_wheel = omega × radius_to_wheel
            - Front: radius = sqrt((track/2)² + (wheelbase/2)²)
            - Left wheel moves backward for positive omega (CCW), right forward.
            """
            # Steering ---------------------------------------------------------
            left_steer  = -self.spin_toe_deg   # CW  / right (toward centre)
            right_steer = -self.spin_toe_deg   # CCW / left  (toward centre)

            # Rear caster must be exactly 90° so it is tangent to its (purely
            # longitudinal) radius.
            rear_steer = 90.0 if omega >= 0 else -90.0
            self.set_steering(left_steer, right_steer, rear_steer)

            # Drive ------------------------------------------------------------
            # True radius from robot centre to front wheel contact patch
            front_radius = math.sqrt(
                (self.track_width / 2.0) ** 2 + (self.wheelbase / 2.0) ** 2
            )
            v_wheel = omega * front_radius
            left_ticks, right_ticks = speed_to_ticks(
                -v_wheel,   # left wheel backward for +omega (CCW)
                v_wheel,   # right wheel forward
            )
            self.set_wheel_speeds(left_ticks, right_ticks)

        def apply_diff_drive(v_x, omega):
            """
            Standard diff drive steering + active rear caster that matches the turn radius.
            Turn radius R = v_x / omega  (signed; infinity when omega == 0).
            """
            if _nonzero(omega):
                R = v_x / omega   # signed turn radius (m)

                # Guard against very small R
                if abs(R) < self.spin_radius_threshold:
                    apply_spin(omega)
                    return

                rear_deg = math.degrees(math.atan(self.rear_caster_offset / R))
                self.set_steering(0, 0, rear_deg)
            else:
                self.set_steering(0, 0, 0)

            v_l = v_x - (omega * self.track_width / 2.0)
            v_r = v_x + (omega * self.track_width / 2.0)
            left_ticks, right_ticks = speed_to_ticks(v_l, v_r)
            self.set_wheel_speeds(left_ticks, right_ticks)

        def apply_crab(v_y):
            """
            All three wheels turn 90° so the robot can slide sideways.
            Both drive wheels spin at the same speed in the same direction.
            Positive v_y → move left (conventional ROS frame).
            """
            self.set_steering(-90, -90, -90)
            ticks = linear_to_ticks(v_y)
            ticks = max(-self.max_velocity_ticks, min(self.max_velocity_ticks, ticks))
            # Left motor is physically mirrored, so same-direction travel = (-ticks, ticks)
            self.set_wheel_speeds(-ticks, ticks)


        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9

        # deadman timer
        if elapsed > self.deadman_timer_sec:
            v_x = v_y = omega = 0.0
        else:
            v_x = self.cmd_vx
            v_y = self.cmd_vy
            omega = self.cmd_w

        mode = classify_mode(v_x, v_y, omega)
        self.current_mode = mode

        if mode == DriveMode.STOP:
            self.set_wheel_speeds(0, 0)
        elif mode == DriveMode.SPIN:
            apply_spin(omega)
        elif mode == DriveMode.CRAB:
            apply_crab(v_y)
        else:
            apply_diff_drive(v_x, omega)


    def publish_joints(self):

        TICKS_TO_RAD = 1.0 / TICKS_PER_DEG * math.pi / 180.0

        now = self.get_clock().now()
        self.sync_all.txRxPacket()

        def get_pos_speed(sid):
            pos = self.sync_all.getData(sid, 56, 2)
            speed = self.sync_all.getData(sid, 58, 2)

            pos_rad = (pos - STEER_CENTER) * TICKS_TO_RAD
            speed_rad_s = self.controller.packet_handler.scs_tohost(speed, 15) * self.read_hz_scale * 2.0 * math.pi 

            return pos_rad, speed_rad_s

        left_drive_pos, left_drive_spd = get_pos_speed(self.left_drive_id)
        right_drive_pos, right_drive_spd = get_pos_speed(self.right_drive_id)

        left_steer_pos, left_steer_spd = get_pos_speed(self.left_steer_id)
        right_steer_pos, right_steer_spd = get_pos_speed(self.right_steer_id)
        rear_steer_pos, rear_steer_spd = get_pos_speed(self.rear_steer_id)

        js = JointState()
        js.header.stamp = now.to_msg()
        js.header.frame_id = self.base_frame
        js.name = ["left_wheel_joint", "right_wheel_joint", "left_steer_joint", "right_steer_joint", "rear_steer_joint"]
        js.position = [
            left_drive_pos,
            right_drive_pos,
            left_steer_pos, 
            right_steer_pos,
            rear_steer_pos
        ]

        js.velocity = [
            left_drive_spd,
            right_drive_spd, 
            left_steer_spd,
            right_steer_spd,
            rear_steer_spd
        ]

        self.joint_pub.publish(js)

    def update_battery(self):

        try:
            voltage = self.controller.read_voltage(self.rear_steer_id) #servo with thickest wires and is closest to the battery
        except Exception as e:
            self.get_logger().warn("Battery voltage read failed: {}".format(e))
            return

        percentage = (voltage - 9.6) / (12.45 - 9.6)
        percentage = max(0.0, min(1.0, percentage))

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = voltage
        msg.percentage = percentage
        msg.cell_voltage = [voltage / 3.0, voltage / 3.0, voltage / 3.0]
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True
        self.battery_pub.publish(msg)

    def shutdown(self):
        self.get_logger().info("Stopping servos and centering steering")
        self.set_steering(0.0, 0.0, 0.0)
        self.set_wheel_speeds(0, 0)

        time.sleep(1)

        self.controller.disconnect()

def main(args=None):
    rclpy.init(args=args)
    node = FeetechDiffDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()