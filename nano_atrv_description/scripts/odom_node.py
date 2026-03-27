#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import tf2_ros


""" def update_odometry(self):

    now = self.get_clock().now()
    dt  = (now - self.last_time).nanoseconds / 1e9
    self.last_time = now

    speed_l, _, _ = self.controller.packet_handler.ReadSpeed(self.left_drive_id)
    speed_r, _, _ = self.controller.packet_handler.ReadSpeed(self.right_drive_id)

    hz_l = speed_l * self.read_hz_scale
    hz_r = speed_r * self.read_hz_scale

    v_l = - hz_l * self.circumference
    v_r = + hz_r * self.circumference

    v = (v_l + v_r) / 2.0
    w = (v_r - v_l) / self.track_width

    self.x += v * math.cos(self.theta) * dt
    self.y += v * math.sin(self.theta) * dt
    self.theta += w * dt

    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(self.theta / 2.0)
    q.w = math.cos(self.theta / 2.0)

    now_msg = now.to_msg()

    if self.publish_tf:
        tf_msg = TransformStamped()
        tf_msg.header.stamp    = now_msg
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id  = self.base_frame
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = q
        self.tf_br.sendTransform(tf_msg)

    odom = Odometry()
    odom.header.stamp = now_msg
    odom.header.frame_id= self.odom_frame
    odom.pose.pose.position.x = self.x
    odom.pose.pose.position.y = self.y
    odom.pose.pose.orientation = q
    odom.child_frame_id= self.base_frame
    odom.twist.twist.linear.x = v
    odom.twist.twist.angular.z = w
    self.odom_pub.publish(odom) """

class JointStateToOdom(Node):
    def __init__(self):
        super().__init__("jointstate_to_odom")

        # Geometry
        self.declare_parameter("wheel_diameter", 0.17)
        self.declare_parameter("track_width", 0.355)
        self.declare_parameter("wheelbase", 0.35)
        self.declare_parameter("rear_caster_offset", 0.175)

        self.wheel_diameter = self.get_parameter("wheel_diameter").value
        self.track_width = self.get_parameter("track_width").value
        self.wheelbase = self.get_parameter("wheelbase").value
        self.rear_caster_offset = self.get_parameter("rear_caster_offset").value

        self.radius = self.wheel_diameter / 2.0
        self.circumference = math.pi * self.wheel_diameter

        # Frames
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("left_wheel_frame", "left_wheel")
        self.declare_parameter("right_wheel_frame", "right_wheel")
        self.declare_parameter("rear_wheel_frame", "rear_wheel")

        self.base_frame = self.get_parameter("base_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.left_wheel_frame = self.get_parameter("left_wheel_frame").value
        self.right_wheel_frame = self.get_parameter("right_wheel_frame").value
        self.rear_wheel_frame = self.get_parameter("rear_wheel_frame").value

        # Joint names
        self.declare_parameter("left_wheel_joint", "left_wheel_joint")
        self.declare_parameter("right_wheel_joint", "right_wheel_joint")
        self.declare_parameter("rear_wheel_joint", "rear_wheel_joint")
        self.declare_parameter("left_steer_joint", "left_steer_joint")
        self.declare_parameter("right_steer_joint", "right_steer_joint")
        self.declare_parameter("rear_steer_joint", "rear_steer_joint")

        self.left_wheel_joint = self.get_parameter("left_wheel_joint").value
        self.right_wheel_joint = self.get_parameter("right_wheel_joint").value
        self.rear_wheel_joint = self.get_parameter("rear_wheel_joint").value
        self.left_steer_joint = self.get_parameter("left_steer_joint").value
        self.right_steer_joint = self.get_parameter("right_steer_joint").value
        self.rear_steer_joint = self.get_parameter("rear_steer_joint").value

        #<node pkg="tf2_ros" exec="static_transform_publisher" name="tf_pub_2" output="screen" args="-0.17 0 0 0 0 0 base_link rear_joint" />
        #<node pkg="tf2_ros" exec="static_transform_publisher" name="tf_pub_3" output="screen" args="0.17 0.115 -0.1 0 0 0 base_link left_joint" />
        #<node pkg="tf2_ros" exec="static_transform_publisher" name="tf_pub_4" output="screen" args="0.17 -0.115 -0.1 0 0 0 base_link right_joint" />

        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = None

        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0

        # Latest joint states for TF publishing
        self.latest_joint_state = None

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.tf_br = tf2_ros.TransformBroadcaster(self)

        # Subscriber
        self.joint_sub = self.create_subscription(JointState, "joint_states", self.joint_cb, 10)

        self.get_logger().info("JointState to Odom node initialized")

    def joint_cb(self, msg: JointState):
        now = msg.header.stamp
        current_time = rclpy.time.Time.from_msg(now)

        # Build name to index map
        name_to_idx = {name: i for i, name in enumerate(msg.name)}

        # Extract wheel velocities (rad/s)
        v_l = msg.velocity[name_to_idx.get(self.left_wheel_joint, 0)]
        v_r = msg.velocity[name_to_idx.get(self.right_wheel_joint, 0)]

        # Extract steering angles (rad)
        steer_l = msg.position[name_to_idx.get(self.left_steer_joint, 3)]
        steer_r = msg.position[name_to_idx.get(self.right_steer_joint, 4)]
        steer_rear = msg.position[name_to_idx.get(self.rear_steer_joint, 5)]

        # Store for TF publishing
        self.latest_joint_state = {
            "stamp": now,
            "steer_l": steer_l,
            "steer_r": steer_r,
            "steer_rear": steer_rear,
        }

        # Compute odometry
        if self.last_time is not None:
            dt = (current_time - self.last_time).nanoseconds / 1e9

            # Convert wheel angular velocity to linear velocity
            # v = omega * radius
            v_l_linear = v_l * self.radius
            v_r_linear = v_r * self.radius

            # Differential drive kinematics
            v = (v_l_linear + v_r_linear) / 2.0
            w = (v_r_linear - v_l_linear) / self.track_width

            # Integrate
            self.x += v * math.cos(self.theta) * dt
            self.y += v * math.sin(self.theta) * dt
            self.theta += w * dt

            # Publish odometry
            self.publish_odom(now, v, w)
            self.publish_tfs(now, steer_l, steer_r, steer_rear)

        self.last_time = current_time

    def publish_odom(self, stamp, v, w):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(self.theta / 2.0)
        q.w = math.cos(self.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

    def publish_tfs(self, stamp, steer_l, steer_r, steer_rear):
        # odom -> base_link
        q_base = Quaternion()
        q_base.x = 0.0
        q_base.y = 0.0
        q_base.z = math.sin(self.theta / 2.0)
        q_base.w = math.cos(self.theta / 2.0)

        tf_odom = TransformStamped()
        tf_odom.header.stamp = stamp
        tf_odom.header.frame_id = self.odom_frame
        tf_odom.child_frame_id = self.base_frame
        tf_odom.transform.translation.x = self.x
        tf_odom.transform.translation.y = self.y
        tf_odom.transform.translation.z = 0.0
        tf_odom.transform.rotation = q_base

        # base_link -> left_wheel (positioned at +wheelbase/2, +track/2)
        # Wheel rotates around y-axis, steers around z-axis
        q_left = Quaternion()
        q_left.x = 0.0
        q_left.y = math.sin(steer_l / 2.0)
        q_left.z = 0.0
        q_left.w = math.cos(steer_l / 2.0)

        tf_left = TransformStamped()
        tf_left.header.stamp = stamp
        tf_left.header.frame_id = self.base_frame
        tf_left.child_frame_id = self.left_wheel_frame
        tf_left.transform.translation.x = self.wheelbase / 2.0
        tf_left.transform.translation.y = self.track_width / 2.0
        tf_left.transform.translation.z = 0.0
        tf_left.transform.rotation = q_left

        # base_link -> right_wheel (positioned at +wheelbase/2, -track/2)
        q_right = Quaternion()
        q_right.x = 0.0
        q_right.y = math.sin(steer_r / 2.0)
        q_right.z = 0.0
        q_right.w = math.cos(steer_r / 2.0)

        tf_right = TransformStamped()
        tf_right.header.stamp = stamp
        tf_right.header.frame_id = self.base_frame
        tf_right.child_frame_id = self.right_wheel_frame
        tf_right.transform.translation.x = self.wheelbase / 2.0
        tf_right.transform.translation.y = -self.track_width / 2.0
        tf_right.transform.translation.z = 0.0
        tf_right.transform.rotation = q_right

        # base_link -> rear_wheel (positioned at -rear_caster_offset, 0)
        q_rear = Quaternion()
        q_rear.x = 0.0
        q_rear.y = math.sin(steer_rear / 2.0)
        q_rear.z = 0.0
        q_rear.w = math.cos(steer_rear / 2.0)

        tf_rear = TransformStamped()
        tf_rear.header.stamp = stamp
        tf_rear.header.frame_id = self.base_frame
        tf_rear.child_frame_id = self.rear_wheel_frame
        tf_rear.transform.translation.x = -self.rear_caster_offset
        tf_rear.transform.translation.y = 0.0
        tf_rear.transform.translation.z = 0.0
        tf_rear.transform.rotation = q_rear

        self.tf_br.sendTransform([tf_odom, tf_left, tf_right, tf_rear])


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