#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import numpy as np
import ikpy.chain
from ikpy.utils import geometry
import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose
import threading
import time

JOINT_NAMES = [
	'shoulder_pan',
	'shoulder_lift',
	'elbow_flex',
	'wrist_flex',
	'wrist_roll',
	'gripper',
]
URDF_PATH = "/home/vicos/colcon_ws/src/nano_atrv/nano_atrv_arm/urdf/so101_new_calib_ikpy.urdf"
ROBOT_CALIB = np.array([-0.03, 0.003, 0])
TARGET_FRAME = "base_link_arm"
GRIPPER_LINK = "gripper_link"

GRIPPER_OPEN = 1.0
GRIPPER_CLOSED = -0.2

# z positions relative to base_link_arm
Z_FLOOR = -0.1
Z_ABOVE = 0.1

# how close gripper_link must get before we consider a move done
ARRIVAL_THRESHOLD = 0.12 
ARRIVAL_SETTLE_DELAY = 1.0


class IKCommander(Node):
	def __init__(self):
		super().__init__('ik_commander')
		self.pub = self.create_publisher(JointState, '/joint_commands_arm', 10)

		self.sub_pos = self.create_subscription(PoseStamped, '/gripper_pos_target', self.target_callback, 10)
		self.sub_grab = self.create_subscription(PoseStamped, '/grab_target', self.grab_callback, 10)
		self.sub_place = self.create_subscription(PoseStamped, '/place_target', self.place_callback, 10)

		self.tf_buffer = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

		self.chain = ikpy.chain.Chain.from_urdf_file(URDF_PATH)
		self.chain.active_links_mask[0] = False

		# prevent overlapping sequences
		self._seq_lock = threading.Lock()

	# ---- helpers ----

	def _lookup_gripper_pos(self):
		"""Returns gripper_link position in TARGET_FRAME, or None on failure."""
		try:
			t = self.tf_buffer.lookup_transform(TARGET_FRAME, GRIPPER_LINK, rclpy.time.Time())
			p = t.transform.translation
			return np.array([p.x, p.y, p.z])
		except TransformException:
			return None

	def _wait_for_arrival(self, target_pos):
		"""Blocks until gripper_link is within ARRIVAL_THRESHOLD of target_pos, then waits ARRIVAL_SETTLE_DELAY."""
		while rclpy.ok():
			pos = self._lookup_gripper_pos()
			print(f"target away: {np.linalg.norm(pos - target_pos)}")
			if pos is not None and np.linalg.norm(pos - target_pos) < ARRIVAL_THRESHOLD:
				break
			time.sleep(0.05)
		time.sleep(ARRIVAL_SETTLE_DELAY)

	def _transform_pose(self, msg: PoseStamped):
		"""Transform a PoseStamped into TARGET_FRAME. Returns pose or None."""
		try:
			transform = self.tf_buffer.lookup_transform(TARGET_FRAME, msg.header.frame_id, rclpy.time.Time())
			return do_transform_pose(msg.pose, transform)
		except TransformException as e:
			self.get_logger().warn(f"TF failed: {e}")
			return None

	def _move_to(self, xy_pos, z, gripper_val):
		"""Compute IK and publish for a given XY position (in TARGET_FRAME), z height, and gripper value.
		Returns the 3D target position used, or None on IK failure."""
		target_pos = np.array([xy_pos[0], xy_pos[1], z]) + ROBOT_CALIB
		orientation = geometry.rpy_matrix(0, np.deg2rad(180), 0)
		try:
			ik = self.chain.inverse_kinematics(
				target_position=target_pos,
				target_orientation=orientation,
				orientation_mode="all",
			)
		except Exception as e:
			self.get_logger().error(f"IK failed: {e}")
			return None

		msg_out = JointState()
		msg_out.name = JOINT_NAMES
		# IK gives arm joints; override gripper with desired value
		msg_out.position = [float(ik[i + 1]) for i in range(len(JOINT_NAMES) - 1)] + [gripper_val]
		self.pub.publish(msg_out)
		return target_pos

	# ---- original single-step callback ----

	def target_callback(self, msg: PoseStamped):
		pose = self._transform_pose(msg)
		if pose is None:
			return
		xy = np.array([pose.position.x, pose.position.y])
		self._move_to(xy, Z_FLOOR, GRIPPER_CLOSED)

	# ---- sequence runners (run in threads so spin() stays unblocked) ----

	def _run_grab(self, xy):
		if not self._seq_lock.acquire(blocking=False):
			self.get_logger().warn("Sequence already running, ignoring grab target")
			return
		try:
			self.get_logger().info("Grab: moving above target")
			target = self._move_to(xy, Z_ABOVE, GRIPPER_OPEN)
			if target is None:
				return
			self._wait_for_arrival(target)

			self.get_logger().info("Grab: lowering to floor")
			target = self._move_to(xy, Z_FLOOR, GRIPPER_OPEN)
			if target is None:
				return
			self._wait_for_arrival(target)

			self.get_logger().info("Grab: closing gripper")
			target = self._move_to(xy, Z_FLOOR, GRIPPER_CLOSED)
			if target is None:
				return
			self._wait_for_arrival(target)

			self.get_logger().info("Grab: lifting arm")
			target = self._move_to(xy, Z_ABOVE, GRIPPER_CLOSED)
			if target is None:
				return
			self._wait_for_arrival(target)

			self.get_logger().info("Grab: complete")
		finally:
			self._seq_lock.release()

	def _run_place(self, xy):
		if not self._seq_lock.acquire(blocking=False):
			self.get_logger().warn("Sequence already running, ignoring place target")
			return
		try:
			self.get_logger().info("Place: moving above target")
			target = self._move_to(xy, Z_ABOVE, GRIPPER_CLOSED)
			if target is None:
				return
			self._wait_for_arrival(target)

			self.get_logger().info("Place: lowering to floor")
			target = self._move_to(xy, Z_FLOOR+0.02, GRIPPER_CLOSED)
			if target is None:
				return
			self._wait_for_arrival(target)

			self.get_logger().info("Place: opening gripper")
			target = self._move_to(xy, Z_FLOOR+0.02, GRIPPER_OPEN)
			if target is None:
				return
			self._wait_for_arrival(target)

			self.get_logger().info("Place: lifting arm")
			target = self._move_to(xy, Z_ABOVE, GRIPPER_OPEN)
			if target is None:
				return
			self._wait_for_arrival(target)

			self.get_logger().info("Place: closing gripper")
			target = self._move_to(xy, Z_ABOVE, GRIPPER_CLOSED)
			if target is None:
				return
			self._wait_for_arrival(target)

			self.get_logger().info("Place: complete")
		finally:
			self._seq_lock.release()

	def grab_callback(self, msg: PoseStamped):
		pose = self._transform_pose(msg)
		if pose is None:
			return
		xy = np.array([pose.position.x, pose.position.y])
		threading.Thread(target=self._run_grab, args=(xy,), daemon=True).start()

	def place_callback(self, msg: PoseStamped):
		pose = self._transform_pose(msg)
		if pose is None:
			return
		xy = np.array([pose.position.x, pose.position.y])
		threading.Thread(target=self._run_place, args=(xy,), daemon=True).start()


def main():
	rclpy.init()
	node = IKCommander()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()