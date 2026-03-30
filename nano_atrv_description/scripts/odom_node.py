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

		v_l = -msg.velocity[idx[self.lw_joint]] * self.wheel_radius
		v_r = msg.velocity[idx[self.rw_joint]] * self.wheel_radius
		phi_l = -msg.position[idx[self.ls_joint]]
		phi_r = msg.position[idx[self.rs_joint]]
		phi_rear = msg.position[idx["rear_steer_joint"]]

		if self.last_time is None:
			self.last_time = current_time
			return

		dt = (current_time - self.last_time).nanoseconds / 1e9
		self.last_time = current_time

		if dt <= 0.0:
			return

		if phi_l > 1.5 and phi_r > 1.5:
			#time for crab
			vx_robot = 0.0
			vy_robot = v_r
			yaw_rate = 0.0
		else:

			# Hub positions in base_link frame (from URDF)
			lx, ly = self.wheelbase / 2, self.track_width / 2
			rx, ry = self.wheelbase / 2, -self.track_width / 2

			# Each wheel's heading unit vector and its axle-normal (the constraint line direction pointing toward ICR)
			# Heading: (cos(phi), sin(phi)); axle normal (perpendicular): (-sin(phi), cos(phi))
			# Parametric line: P = hub + t * axle_normal

			# Toed-in rotation mode: left wheel steers inward (phi_l < 0), right steers inward (phi_r > 0)
			toed_in = phi_l < -0.05 and phi_r > 0.05

			if toed_in:
				# Intersect the two axle-normal lines to find the ICR.
				# Left:  (lx, ly) + t * (-sin(phi_l),  cos(phi_l))
				# Right: (rx, ry) + s * (-sin(phi_r),  cos(phi_r))
				# Solve for t:  lx - t*sin(phi_l) = rx - s*sin(phi_r)
				#               ly + t*cos(phi_l) = ry + s*cos(phi_r)
				dl_x, dl_y = -math.sin(phi_l), math.cos(phi_l)
				dr_x, dr_y = -math.sin(phi_r), math.cos(phi_r)
				# Cross-multiply to solve: (lx + t*dl_x, ly + t*dl_y) = (rx + s*dr_x, ry + s*dr_y)
				# t*dl_x - s*dr_x = rx - lx
				# t*dl_y - s*dr_y = ry - ly
				denom = dl_x * (-dr_y) - dl_y * (-dr_x)
				if abs(denom) < 1e-6:
					# Lines are parallel (wheels pointing the same way) — fall through to normal diff-drive
					toed_in = False
				else:
					dx, dy = rx - lx, ry - ly
					t = (dx * (-dr_y) - dy * (-dr_x)) / denom
					icr_x = lx + t * dl_x
					icr_y = ly + t * dl_y

			if toed_in:
				# Average wheel speed (they should be equal magnitude but opposite sign in the joint frame,
				# both contributing positive tangential velocity around the ICR)
				v_avg = (abs(v_l) + abs(v_r)) / 2.0
				r_left = math.hypot(lx - icr_x, ly - icr_y)
				r_right = math.hypot(rx - icr_x, ry - icr_y)
				r_avg = (r_left + r_right) / 2.0

				if r_avg < 1e-4:
					vx_robot = vy_robot = yaw_rate = 0.0
				else:
					# Sign of omega: if ICR is ahead of the axle (icr_y near 0), spinning left wheel
					# forward and right wheel forward (in their local frames) produces CCW rotation.
					# Use the left hub's tangential direction to determine sign.
					# Tangent at left hub is perpendicular to the radius vector from ICR, in CCW sense.
					# v_l drives the left wheel; positive v_l = forward along phi_l.
					# The component of v_l tangential to the ICR circle:
					radius_l_x, radius_l_y = lx - icr_x, ly - icr_y
					# Tangent direction (CCW): (-radius_l_y, radius_l_x) / r_left
					heading_l_x, heading_l_y = math.cos(phi_l), math.sin(phi_l)
					tangent_ccw_x, tangent_ccw_y = -radius_l_y / r_left, radius_l_x / r_left
					v_tangential = v_l * (heading_l_x * tangent_ccw_x + heading_l_y * tangent_ccw_y)
					yaw_rate = v_tangential / r_left
					# Robot body velocity = pure rotation around ICR
					# v_base = omega x r_base_from_ICR; r_base = (0,0) - (icr_x, icr_y)
					vx_robot = -yaw_rate * icr_y
					vy_robot = yaw_rate * icr_x
			else:
				vl_x = v_l * math.cos(phi_l)
				vr_x = v_r * math.cos(phi_r)
				vx_robot = (vr_x + vl_x) / 2.0
				yaw_rate = (vr_x - vl_x) / self.track_width

				vl_y = v_l * math.sin(phi_l)
				vr_y = v_r * math.sin(phi_r)

				if abs(math.cos(phi_rear)) > 0.1:
					vy_rear = vx_robot * math.tan(phi_rear)
					vy_robot = vy_rear + yaw_rate * self.wheelbase / 2
				else:
					vy_robot = (vl_y + vr_y) / 2.0

		self.x += (vx_robot * math.cos(self.theta) - vy_robot * math.sin(self.theta)) * dt
		self.y += (vx_robot * math.sin(self.theta) + vy_robot * math.cos(self.theta)) * dt
		self.theta += yaw_rate * dt

		self.publish_odom(msg.header.stamp, vx_robot, vy_robot, yaw_rate)

	def joint_cb_prev(self, msg: JointState):
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