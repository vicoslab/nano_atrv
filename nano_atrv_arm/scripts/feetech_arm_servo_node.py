#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty

import math
import time

from vassar_feetech_servo_sdk import ServoController
from scservo_sdk import GroupSyncRead

SERVO_IDS = [1,2,3,4,5,6]

JOINT_NAMES = [
	'shoulder_pan',
	'shoulder_lift',
	'elbow_flex',
	'wrist_flex',
	'wrist_roll',
	'gripper',
]

JOINT_TO_ID = dict(zip(JOINT_NAMES, SERVO_IDS))

TICKS_PER_DEG = 4096.0 / 360.0
TICKS_TO_RAD = 1.0 / TICKS_PER_DEG * math.pi / 180.0
RAD_TO_TICKS = 1.0 / TICKS_TO_RAD
SERVO_CENTER = 2048
READ_HZ_SCALE = 0.0122

STS_PRESENT_POSITION_L = 56
READ_LEN = 4

POSES = {
    'identity': [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ],
	'stowed': [-1.5922720578252956, -1.902136176978195,  1.6858448858863198, 0.6887573737606529,  -0.08743690490948154, -0.056757289151768725],
}

class ServoJointCommander(Node):

	def __init__(self):
		super().__init__('servo_joint_publisher')

		self.speed = 200
		self.acceleration = 20

		self.publisher = self.create_publisher(JointState, '/joint_states_arm', 10)
		self.subscription = self.create_subscription(JointState, '/joint_commands_arm', self.command_callback, 10)

		self.stow_sub = self.create_subscription(Empty, '/stow_arm', self.stow_callback, 10)

		# Servo controller setup
		self.controller = ServoController(
			servo_ids=SERVO_IDS,
			servo_type="sts",
			port="/dev/ttyARM",
			baudrate=1000000
		)
		self.controller.connect()

		self.sync_all = GroupSyncRead(
			self.controller.packet_handler,
			STS_PRESENT_POSITION_L,
			READ_LEN
		)

		for sid in SERVO_IDS:
			self.sync_all.addParam(sid)

		self.timer = self.create_timer(0.02, self.update)  # 50 Hz

		self.set_torque(True)
		self.set_pose("stowed")

	def stow_callback(self, msg):
		self.set_pose("stowed")

	def set_pose(self, pose):
		msg = JointState()
		msg.name = JOINT_NAMES
		msg.position = POSES[pose]
		self.command_callback(msg)

	def set_torque(self, enabled):
		val = 1 if enabled else 0
		for i in SERVO_IDS:
			self.controller.packet_handler.write1ByteTxRx(i, 40, val)


	def command_callback(self, msg: JointState):
		positions = {}
		for name, pos_rad in zip(msg.name, msg.position):
			if name not in JOINT_TO_ID:
				continue
			sid = JOINT_TO_ID[name]
			positions[sid] = SERVO_CENTER + int(pos_rad * RAD_TO_TICKS)
		if positions:
			self.controller.write_position(positions, speed=self.speed, acceleration=self.acceleration)

	def get_pos_speed(self, sid):
		pos = self.sync_all.getData(sid, 56, 2)
		speed = self.sync_all.getData(sid, 58, 2)

		pos_rad = (pos - SERVO_CENTER) * TICKS_TO_RAD
		speed_rad_s = (
			self.controller.packet_handler.scs_tohost(speed, 15)
			* READ_HZ_SCALE * 2.0 * math.pi
		)

		return pos_rad, speed_rad_s

	def update(self):
		self.sync_all.txRxPacket()

		msg = JointState()
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.name = JOINT_NAMES

		positions = []
		velocities = []

		for sid in SERVO_IDS:
			pos, vel = self.get_pos_speed(sid)
			positions.append(pos)
			velocities.append(vel)

		msg.position = positions
		msg.velocity = velocities

		self.publisher.publish(msg)

	def destroy_node(self):
		self.controller.disconnect()
		super().destroy_node()


def main():
	rclpy.init()
	node = ServoJointCommander()

	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass

	node.destroy_node()
	rclpy.shutdown()


if __name__ == "__main__":
	main()