import time
import math
import inspect

from vassar_feetech_servo_sdk import ServoController
from scservo_sdk import GroupSyncRead

#https://github.com/vassar-robotics/feetech-servo-sdk/tree/main

SERVO_IDS = [1,2,3,4,5,6]
JOINT_NAMES = [
    'shoulder_pan',
    'shoulder_lift',
    'elbow_flex',
    'wrist_flex',
    'wrist_roll',
    'gripper',
]

TICKS_PER_DEG = 4096.0 / 360.0
TICKS_TO_RAD = 1.0 / TICKS_PER_DEG * math.pi / 180.0
SERVO_CENTER = 2048
READ_HZ_SCALE = 0.0122
STS_PRESENT_POSITION_L = 56
READ_LEN = 4  # position (2) + speed (2)

# Initialize with specific configuration
controller = ServoController(
    servo_ids=SERVO_IDS,
    servo_type="sts",  # 'sts' or 'hls'
    port="/dev/ttyARM",
    baudrate=1000000
)
controller.connect()


sync_all = GroupSyncRead(controller.packet_handler, STS_PRESENT_POSITION_L, READ_LEN)
for i in SERVO_IDS:
    sync_all.addParam(i)

print("Running... reading speed (Ctrl+C to stop)")
try:
    while True:
        sync_all.txRxPacket()

        def get_pos_speed(sid):
            pos = sync_all.getData(sid, 56, 2)
            speed = sync_all.getData(sid, 58, 2)

            pos_rad = (pos - SERVO_CENTER) * TICKS_TO_RAD
            speed_rad_s = controller.packet_handler.scs_tohost(speed, 15) * READ_HZ_SCALE * 2.0 * math.pi 

            return pos_rad, speed_rad_s

        for i in SERVO_IDS:
            pos, speed = get_pos_speed(i)
            print(f"{JOINT_NAMES[i-1]}: {pos}")
        
        time.sleep(0.5)
except KeyboardInterrupt:
    pass

controller.disconnect()