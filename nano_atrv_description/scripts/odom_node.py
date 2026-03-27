#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import tf2_ros


def make_quaternion_z(angle: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(angle / 2.0)
    q.w = math.cos(angle / 2.0)
    return q


class JointStateToOdom(Node):
    def __init__(self):
        super().__init__("jointstate_to_odom")

        self.declare_parameter("wheel_radius", 0.085)   # 170mm diameter
        self.declare_parameter("track_width", 0.355)
        self.declare_parameter("wheelbase", 0.35)
        self.declare_parameter("rear_caster_offset", 0.175) 
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("left_wheel_joint", "left_wheel_joint")
        self.declare_parameter("right_wheel_joint", "right_wheel_joint")
        self.declare_parameter("left_steer_joint", "left_steer_joint")
        self.declare_parameter("right_steer_joint", "right_steer_joint")

        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.track_width = self.get_parameter("track_width").value
        self.wheelbase = self.get_parameter("wheelbase").value
        self.rear_caster_offset = self.get_parameter("rear_caster_offset").value
        self.base_frame = self.get_parameter("base_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.lw_joint = self.get_parameter("left_wheel_joint").value
        self.rw_joint = self.get_parameter("right_wheel_joint").value
        self.ls_joint = self.get_parameter("left_steer_joint").value
        self.rs_joint = self.get_parameter("right_steer_joint").value

        self.x = self.y = self.theta = 0.0
        self.last_time = None

        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.tf_br = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(JointState, "joint_states", self.joint_cb, 10)

    def joint_cb(self, msg: JointState):
        idx = {n: i for i, n in enumerate(msg.name)}
        current_time = rclpy.time.Time.from_msg(msg.header.stamp)

        # Mapping joint values
        v_l = -msg.velocity[idx[self.lw_joint]] * self.wheel_radius
        v_r = msg.velocity[idx[self.rw_joint]] * self.wheel_radius
        phi_l = -msg.position[idx[self.ls_joint]]
        phi_r = msg.position[idx[self.rs_joint]]
        phi_rear = msg.position[idx["rear_steer_joint"]] # Needs to be in your JointState

        if self.last_time is None:
            self.last_time = current_time
            return
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        if dt <= 0.0: return

        # --- STEP 1: Calculate Longitudinal Velocity and Yaw Rate ---
        # We use the projections of the front wheels onto the robot's X-axis.
        # v_i_x = v_i * cos(phi_i)
        vl_x = v_l * math.cos(phi_l)
        vr_x = v_r * math.cos(phi_r)

        # In a diff-drive/swerve hybrid:
        # vr_x = vx + (yaw_rate * track_width / 2)
        # vl_x = vx - (yaw_rate * track_width / 2)
        vx_robot = (vr_x + vl_x) / 2.0
        yaw_rate = (vr_x - vl_x) / (self.track_width)

        # --- STEP 2: Solve for Lateral Velocity (vy) ---
        # We have two sources of information for vy: 
        # 1. The front wheels' own steering (if they aren't pointed straight)
        # 2. The rear caster's constraint (especially crucial for crabbing)
        
        # Projections of front wheels onto robot's Y-axis:
        vl_y = v_l * math.sin(phi_l)
        vr_y = v_r * math.sin(phi_r)
        vy_front = (vl_y + vr_y) / 2.0

        # Constraint from rear caster:
        # At the caster (x = -wheelbase, y = 0), the velocity is:
        # v_rear_x = vx
        # v_rear_y = vy + (yaw_rate * -wheelbase)
        # Since v_rear_y / v_rear_x = tan(phi_rear):
        # vy = vx * tan(phi_rear) + (yaw_rate * wheelbase)
        
        if abs(math.cos(phi_rear)) > 0.1:
            # Compute velocity at rear caster
            vy_rear = vx_robot * math.tan(phi_rear)

            # Transform to base_link
            vy_robot = vy_rear + yaw_rate * self.wheelbase/2
        else:
            # Pure lateral crabbing
            vy_robot = vr_y

        self.x += (vx_robot * math.cos(self.theta) - vy_robot * math.sin(self.theta)) * dt
        self.y += (vx_robot * math.sin(self.theta) + vy_robot * math.cos(self.theta)) * dt
        self.theta += yaw_rate * dt

        self.publish_odom(msg.header.stamp, vx_robot, vy_robot, yaw_rate)

    def publish_odom(self, stamp, vx, vy, w):
        q = make_quaternion_z(self.theta)

        tf = TransformStamped()
        tf.header.stamp    = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id  = self.base_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation = q
        self.tf_br.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()