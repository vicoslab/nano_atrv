#!/usr/bin/env python3

import rospy
import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Quaternion
from tf.broadcaster import TransformBroadcaster

from vassar_feetech_servo_sdk import ServoController



# ---------------------------------------------------------------------------
# Steering geometry constants (from calibration / demo script)
# ---------------------------------------------------------------------------
# 4096 ticks = 360 deg  →  TICKS_PER_DEG ≈ 11.378
TICKS_PER_DEG = 4096.0 / 360.0

# Center (straight-ahead) tick value for all three steering servos
STEER_CENTER = 2048

# Per-servo tick limits observed in the demo (used for clamping)
# Left  (ID 2): CW  = 870  (≈ –103 ticks from centre → ~–9°?  No – see below)
#               CCW = 2488  (+440 ticks → +38.6°)
# The demo labels these "max", so we trust them as hard limits.
LEFT_STEER_MIN  = 870    # most-clockwise position (robot-frame: steer right)
LEFT_STEER_MAX  = 2488   # most-counter-clockwise  (steer left)

# Right (ID 3): CW  = 1618, CCW = 3214
RIGHT_STEER_MIN = 1618   # most-clockwise          (steer left  for right wheel)
RIGHT_STEER_MAX = 3214   # most-counter-clockwise  (steer right)

# Rear  (ID 5): centre = 2048, –512 ticks = –45°, –1024 ticks = –90°
#   We steer in both directions, so allow ±1024 ticks (±90°)
REAR_STEER_MIN  = 1024   # –90°
REAR_STEER_MAX  = 3072   # +90°


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


# ---------------------------------------------------------------------------
# Drive mode detection helpers
# ---------------------------------------------------------------------------

def _nonzero(val, tol=1e-3):
    return abs(val) > tol


class DriveMode:
    STOP      = "stop"
    SPIN      = "spin"       # pure angular.z  → spin in place
    ACKERMANN = "ackermann"  # linear.x (+ optional angular.z) → forward/curve
    CRAB      = "crab"       # pure linear.y   → sideways translation


def classify_mode(v_x, v_y, omega, tol=1e-3):
    has_x     = _nonzero(v_x,   tol)
    has_y     = _nonzero(v_y,   tol)
    has_omega = _nonzero(omega, tol)

    if not has_x and not has_y and not has_omega:
        return DriveMode.STOP
    if has_y and not has_x and not has_omega:
        return DriveMode.CRAB
    if has_omega and not has_x and not has_y:
        return DriveMode.SPIN
    # Everything else (including mixed x+omega) uses Ackermann
    return DriveMode.ACKERMANN


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class FeetechDiffDrive:

    def __init__(self):

        rospy.init_node("feetech_diff_drive")

        # ── Serial ──────────────────────────────────────────────────────────
        self.port     = rospy.get_param("~port",     "/dev/ttyWHEELS")
        self.baudrate = rospy.get_param("~baudrate", 1000000)

        # ── Geometry ────────────────────────────────────────────────────────
        self.wheel_diameter  = rospy.get_param("~wheel_diameter",  0.17)
        self.track_width     = rospy.get_param("~track_width",     0.355)
        # Distance from front axle to rear-caster pivot (for Ackermann geometry)
        self.wheelbase       = rospy.get_param("~wheelbase",       0.35)
        # Signed longitudinal offset of the rear caster from robot centre.
        # Positive value = caster is behind centre (typical).
        # Used to compute the caster's Ackermann angle.
        self.rear_caster_offset = rospy.get_param("~rear_caster_offset",
                                                   self.wheelbase / 2.0)

        # ── Drive limits ────────────────────────────────────────────────────
        self.max_velocity_ticks = rospy.get_param("~max_velocity_ticks", 65)
        self.accel_ticks        = rospy.get_param("~accel_ticks",        100)

        # Minimum |R| below which we switch to pure spin-in-place to avoid
        # degenerate Ackermann geometry (very tight turn radii).
        self.spin_radius_threshold = rospy.get_param("~spin_radius_threshold",
                                                      0.15)

        # ── Spin-in-place steering angles ────────────────────────────────
        # For a true pivot-in-place each wheel must be tangent to its circle
        # around the robot centre, i.e. perpendicular to the radius that
        # connects it to the centre.
        #
        # Front wheel position (half-track laterally, half-wheelbase forward):
        #   radius vector angle from lateral axis = atan(wheelbase / track_width)
        #   → steering angle from straight-ahead   = atan(track_width / wheelbase)
        #
        # This is computed after all geometry params are loaded; see _compute_spin_angle().
        # An optional override is available if the calculated angle needs trimming.
        self.spin_toe_override_deg = rospy.get_param("~spin_toe_deg", None)

        # ── Servo IDs ───────────────────────────────────────────────────────
        self.left_drive_id   = rospy.get_param("~left_servo_id",         1)
        self.left_steer_id   = rospy.get_param("~left_steer_servo_id",   2)
        self.right_steer_id  = rospy.get_param("~right_steer_servo_id",  3)
        self.right_drive_id  = rospy.get_param("~right_servo_id",        4)
        self.rear_steer_id   = rospy.get_param("~rear_steer_servo_id",   5)

        # ── Frames ──────────────────────────────────────────────────────────
        self.base_frame  = rospy.get_param("~base_frame", "base_link")
        self.odom_frame  = rospy.get_param("~odom_frame", "odom")
        self.publish_tf  = rospy.get_param("~publish_tf", True)

        # ── Deadman ─────────────────────────────────────────────────────────
        self.cmd_timeout = rospy.get_param("~cmd_timeout", 1.0)

        # ── Loop ────────────────────────────────────────────────────────────
        self.rate_hz = rospy.get_param("~rate", 30)

        # ── Derived geometry ─────────────────────────────────────────────────
        # Spin-in-place: front wheel steering angle from straight-ahead so that
        # each wheel is tangent to its circle around the robot centre.
        #
        #   Front wheel offset from centre: (track/2  laterally,  wheelbase/2  forward)
        #   The wheel must be perpendicular to the radius → steer by atan(track/2 / wheelbase/2)
        #                                                             = atan(track / wheelbase)
        #
        # Left  wheel steers CCW (positive), right wheel steers CW (negative).
        self.spin_toe_deg = (
            self.spin_toe_override_deg
            if self.spin_toe_override_deg is not None
            else math.degrees(math.atan2(self.track_width, self.wheelbase))
        )
        rospy.loginfo(
            "Spin-in-place front wheel angle: {:.1f} deg (track={}, wheelbase={})".format(
                self.spin_toe_deg, self.track_width, self.wheelbase
            )
        )

        # ── Internal constants ───────────────────────────────────────────────
        self.read_hz_scale = 0.0122
        self.radius        = self.wheel_diameter / 2.0
        self.circumference = math.pi * self.wheel_diameter

        # ── Command state ────────────────────────────────────────────────────
        self.cmd_v     = 0.0   # linear.x  (m/s)
        self.cmd_vy    = 0.0   # linear.y  (m/s)
        self.cmd_w     = 0.0   # angular.z (rad/s)
        self.last_cmd_time = rospy.Time(0)

        # ── Drive-mode state (for smooth transitions) ────────────────────────
        self.current_mode = DriveMode.STOP

        # ── Odometry state ───────────────────────────────────────────────────
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0
        self.last_time = rospy.Time.now()

        # ── ROS interfaces ───────────────────────────────────────────────────
        self.cmd_sub  = rospy.Subscriber("cmd_vel", Twist, self.cmd_callback)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.tf_br = TransformBroadcaster()

        # ── Battery ─────────────────────────────────────────────────────────
        self.battery_servo_id  = rospy.get_param("~battery_servo_id", 5)
        self.battery_pub_hz = rospy.get_param("~battery_pub_hz", 1.0)
        self._battery_interval = rospy.Duration(1.0 / self.battery_pub_hz)
        self._last_battery_time = rospy.Time(0)
        self.battery_pub = rospy.Publisher("battery_state", BatteryState, queue_size=5)

        # ── Servo controller ─────────────────────────────────────────────────
        all_ids = [
            self.left_drive_id,
            self.left_steer_id,
            self.right_steer_id,
            self.right_drive_id,
            self.rear_steer_id,
        ]
        self.controller = ServoController(
            servo_ids=all_ids,
            servo_type="sts",
            port=self.port,
            baudrate=self.baudrate,
        )
        self.controller.connect()

        # Drive servos → velocity / wheel-mode
        self.set_velocity_control(self.left_drive_id)
        self.set_velocity_control(self.right_drive_id)

        # Steering servos → position mode (default for STS; just verify torque on)
        self._enable_torque(self.left_steer_id)
        self._enable_torque(self.right_steer_id)
        self._enable_torque(self.rear_steer_id)

        # Park steering straight ahead
        self._set_steering(0.0, 0.0, 0.0)

        rospy.loginfo("Feetech diff drive node initialised (with steering)")


    def update_battery(self):
        """Read voltage from servo 5 and publish BatteryState at a throttled rate."""
        now = rospy.Time.now()
        if (now - self._last_battery_time) < self._battery_interval:
            return
        self._last_battery_time = now

        try:
            voltage = self.controller.read_voltage(self.battery_servo_id)
        except Exception as e:
            rospy.logwarn_throttle(10.0, "Battery voltage read failed: {}".format(e))
            return

        percentage = (voltage - 9.6) / (12.45 - 9.6)
        percentage = max(0.0, min(1.0, percentage))

        msg = BatteryState()
        msg.header.stamp = now
        msg.voltage = voltage
        msg.percentage = percentage
        msg.cell_voltage = [voltage/3.0, voltage/3.0, voltage/3.0]
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True
        self.battery_pub.publish(msg)

    # ────────────────────────────────────────────────────────────────────────
    # Servo helpers
    # ────────────────────────────────────────────────────────────────────────

    def set_velocity_control(self, servo_id):
        self.controller.packet_handler.WheelMode(servo_id)
        self.controller.packet_handler.write1ByteTxRx(servo_id, 40, 1)

    def _enable_torque(self, servo_id):
        """Enable torque on a position-controlled servo."""
        self.controller.packet_handler.write1ByteTxRx(servo_id, 40, 1)

    # ────────────────────────────────────────────────────────────────────────
    # Command callback
    # ────────────────────────────────────────────────────────────────────────

    def cmd_callback(self, msg):
        self.cmd_v  = msg.linear.x
        self.cmd_vy = msg.linear.y
        self.cmd_w  = msg.angular.z
        self.last_cmd_time = rospy.Time.now()

    def deadman_active(self):
        return (rospy.Time.now() - self.last_cmd_time).to_sec() > self.cmd_timeout

    # ────────────────────────────────────────────────────────────────────────
    # Steering output (absolute angles in degrees, robot-frame convention:
    #   positive angle → steer left / CCW)
    # ────────────────────────────────────────────────────────────────────────

    def _set_steering(self, left_deg, right_deg, rear_deg):
        """Write three steering angles (degrees) to the servo controller."""
        positions = {
            self.left_steer_id:  angle_to_left_ticks(left_deg),
            self.right_steer_id: angle_to_right_ticks(right_deg),
            self.rear_steer_id:  angle_to_rear_ticks(rear_deg),
        }
        self.controller.write_position(positions)

    # ────────────────────────────────────────────────────────────────────────
    # Mode: STOP
    # ────────────────────────────────────────────────────────────────────────

    def _apply_stop(self):
        self._set_steering(0.0, 0.0, 0.0)
        self._send_drive(0, 0)

    # ────────────────────────────────────────────────────────────────────────
    # Mode: SPIN IN PLACE
    # ────────────────────────────────────────────────────────────────────────

    def _apply_spin(self, omega):
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
        # longitudinal) radius.  Sign follows robot rotation direction so it
        # doesn't fight the turn; using a fixed +90° is fine because the
        # caster is passive and will trail — but commanding it explicitly
        # removes ambiguity about which side it rests on.
        rear_steer = 90.0 if omega >= 0 else -90.0
        self._set_steering(left_steer, right_steer, rear_steer)

        # Drive ------------------------------------------------------------
        # True radius from robot centre to front wheel contact patch
        front_radius = math.sqrt(
            (self.track_width / 2.0) ** 2 + (self.wheelbase / 2.0) ** 2
        )
        v_wheel = omega * front_radius

        left_ticks, right_ticks = self._speed_to_ticks(
            -v_wheel,   # left wheel backward for +omega (CCW)
             v_wheel,   # right wheel forward
        )
        self._send_drive(left_ticks, right_ticks)

    # ────────────────────────────────────────────────────────────────────────
    # Mode: ACKERMANN (forward/curve)
    # ────────────────────────────────────────────────────────────────────────

    def _apply_ackermann(self, v_x, omega):
        """
        Standard Ackermann steering + active rear caster.

        Turn radius R = v_x / omega  (signed; infinity when omega == 0).
        For each front wheel the Ackermann angle is:
            delta = atan( wheelbase / (R ± track/2) )
        The rear caster angle is:
            delta_rear = atan( rear_caster_offset / R )
        """
        if _nonzero(omega):
            R = v_x / omega   # signed turn radius (m)

            # Guard against very small R (handled upstream, but be safe)
            if abs(R) < self.spin_radius_threshold:
                self._apply_spin(omega)
                return

            # Front Ackermann angles
            # Positive angle → turn left (CCW), consistent with positive omega
            left_deg  = math.degrees(
                math.atan(self.wheelbase / (R - self.track_width / 2.0))
            )
            right_deg = math.degrees(
                math.atan(self.wheelbase / (R + self.track_width / 2.0))
            )

            # Rear caster angle
            rear_deg = math.degrees(
                math.atan(self.rear_caster_offset / R)
            )
        else:
            # Pure straight line – all steering centred
            left_deg = right_deg = rear_deg = 0.0

        self._set_steering(0, 0, rear_deg)

        # Differential drive on the rear (existing logic)
        v_l = v_x - (omega * self.track_width / 2.0)
        v_r = v_x + (omega * self.track_width / 2.0)
        left_ticks, right_ticks = self._speed_to_ticks(v_l, v_r)
        self._send_drive(left_ticks, right_ticks)

    # ────────────────────────────────────────────────────────────────────────
    # Mode: CRAB (pure lateral translation)
    # ────────────────────────────────────────────────────────────────────────

    def _apply_crab(self, v_y):
        """
        All three wheels turn 90° so the robot can slide sideways.
        Both drive wheels spin at the same speed in the same direction.
        Positive v_y → move left (conventional ROS frame).
        """
        # All wheels perpendicular to the robot body
        # Positive v_y = move left in ROS frame → wheels point right (-90°)
        # so that spinning "forward" on both drives produces leftward motion.
        self._set_steering(-90, -90, -90)

        ticks = self._linear_to_ticks(v_y)
        ticks = max(-self.max_velocity_ticks, min(self.max_velocity_ticks, ticks))

        # Left motor is physically mirrored, so same-direction travel = (ticks, -ticks)
        self._send_drive(-ticks, ticks)

    # ────────────────────────────────────────────────────────────────────────
    # Shared helpers
    # ────────────────────────────────────────────────────────────────────────

    def _linear_to_ticks(self, v_ms):
        """Convert a wheel surface speed (m/s) to servo tick speed."""
        rev_per_sec = v_ms / self.circumference
        return int(rev_per_sec / self.read_hz_scale)

    def _speed_to_ticks(self, v_l, v_r):
        """Convert left/right wheel speeds (m/s) to clamped tick values."""
        tl = self._linear_to_ticks(v_l)
        tr = self._linear_to_ticks(v_r)
        tl = max(-self.max_velocity_ticks, min(self.max_velocity_ticks, tl))
        tr = max(-self.max_velocity_ticks, min(self.max_velocity_ticks, tr))
        return tl, tr

    def _send_drive(self, left_ticks, right_ticks):
        """Write velocity commands to the two drive servos."""
        # Left servo is physically mirrored → negate
        self.controller.packet_handler.WriteSpec(
            self.left_drive_id, -left_ticks, self.accel_ticks
        )
        self.controller.packet_handler.WriteSpec(
            self.right_drive_id, right_ticks, self.accel_ticks
        )

    # ────────────────────────────────────────────────────────────────────────
    # Main update
    # ────────────────────────────────────────────────────────────────────────

    def send_commands(self):
        if self.deadman_active():
            v_x = v_y = omega = 0.0
        else:
            v_x   = self.cmd_v
            v_y   = self.cmd_vy
            omega = self.cmd_w

        mode = classify_mode(v_x, v_y, omega)
        self.current_mode = mode

        if mode == DriveMode.STOP:
            self._apply_stop()
        elif mode == DriveMode.SPIN:
            self._apply_spin(omega)
        elif mode == DriveMode.CRAB:
            self._apply_crab(v_y)
        else:  # ACKERMANN
            self._apply_ackermann(v_x, omega)

    # ────────────────────────────────────────────────────────────────────────
    # Odometry (unchanged from original)
    # ────────────────────────────────────────────────────────────────────────

    def read_wheel_velocities(self):
        speed_l, _, _ = self.controller.packet_handler.ReadSpeed(self.left_drive_id)
        speed_r, _, _ = self.controller.packet_handler.ReadSpeed(self.right_drive_id)

        hz_l = speed_l * self.read_hz_scale
        hz_r = speed_r * self.read_hz_scale

        v_l = hz_l * self.circumference
        v_r = hz_r * self.circumference

        v_l = -v_l   # left servo is mirrored
        return v_l, v_r

    def update_odometry(self):
        now = rospy.Time.now()
        dt  = (now - self.last_time).to_sec()
        self.last_time = now

        v_l, v_r = self.read_wheel_velocities()

        v = (v_l + v_r) / 2.0
        w = (v_r - v_l) / self.track_width

        self.x     += v * math.cos(self.theta) * dt
        self.y     += v * math.sin(self.theta) * dt
        self.theta += w * dt

        q = Quaternion()
        q.z = math.sin(self.theta / 2.0)
        q.w = math.cos(self.theta / 2.0)

        if self.publish_tf:
            self.tf_br.sendTransform(
                (self.x, self.y, 0.0),
                (q.x, q.y, q.z, q.w),
                now,
                self.base_frame,
                self.odom_frame,
            )

        odom = Odometry()
        odom.header.stamp        = now
        odom.header.frame_id     = self.odom_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        odom.child_frame_id       = self.base_frame
        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

    def shutdown(self):
        rospy.loginfo("Stopping servos and centering steering")
        self._set_steering(0.0, 0.0, 0.0)
        self._send_drive(0, 0)
        time.sleep(1)
        self.controller.disconnect()

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        rospy.on_shutdown(self.shutdown)
        while not rospy.is_shutdown():
            self.send_commands()
            self.update_odometry()
            self.update_battery()
            rate.sleep()


if __name__ == "__main__":
    node = FeetechDiffDrive()
    node.run()