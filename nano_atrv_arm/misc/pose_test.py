#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

JOINT_NAMES = [
	'shoulder_pan',
	'shoulder_lift',
	'elbow_flex',
	'wrist_flex',
	'wrist_roll',
	'gripper',
]

POSES = {
    'identity': [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ],
	'forward_and_down': [-0.08436894333371027,  0.9480001269133262, -0.15339807878856412,  0.9648739155800683, -0.12578642460662257,  0.5031456984264903 ],
	'upward_and_open': [-0.0030679615757712823,-0.3083301383650139, -1.0231651855197226,  0.22242721424341796,-0.12425244381873693,  0.7455146629124216 ],
    'identity': [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ],
	'stowed': [-1.5922720578252956, -1.902136176978195,  1.6858448858863198, 0.6887573737606529,  -0.08743690490948154, -0.056757289151768725],
}

def main():
	rclpy.init()
	node = Node('joint_command_test')
	pub = node.create_publisher(JointState, '/joint_commands_arm', 10)

	# Give the subscriber time to connect
	time.sleep(1.0)

	for name, positions in POSES.items():
		node.get_logger().info(f'Sending pose: {name}')
		msg = JointState()
		msg.header.stamp = node.get_clock().now().to_msg()
		msg.name = JOINT_NAMES
		msg.position = positions
		pub.publish(msg)
		time.sleep(3.0)

	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main() 
