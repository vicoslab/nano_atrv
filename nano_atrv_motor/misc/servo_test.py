from vassar_feetech_servo_sdk import ServoController
import time
import inspect


#https://github.com/vassar-robotics/feetech-servo-sdk/tree/main

# Initialize with specific configuration
controller = ServoController(
    servo_ids=[2, 3],
    servo_type="sts",  # 'sts' or 'hls'
    port="/dev/ttyUSB0",
    baudrate=1000000
)
controller.connect()


def set_velocity_control(servo_id):
    # Set wheel mode (writes mode=1 directly to RAM, no EEPROM)
    controller.packet_handler.WheelMode(servo_id)
    # Enable torque
    controller.packet_handler.write1ByteTxRx(servo_id, 40, 1)


set_velocity_control(2)
set_velocity_control(3)

# WriteSpec(id, speed, acceleration)
# speed: positive=CW, negative=CCW, range ~10-1000
# acc: 0 = max acceleration
controller.packet_handler.WriteSpec(2, -10, 30)
controller.packet_handler.WriteSpec(3, -10, 30)

print("Running... reading speed (Ctrl+C to stop)")
try:
    while True:
        speed, comm_result, error = controller.packet_handler.ReadSpeed(2)
        speed, comm_result, error = controller.packet_handler.ReadSpeed(3)
        print(f"1_Hz:{speed*0.0122} 2_Hz:{speed*0.0122}", end="\r")
        time.sleep(0.05)
except KeyboardInterrupt:
    pass

controller.packet_handler.WriteSpec(2, 0, 0)
controller.packet_handler.WriteSpec(3, 0, 0)
time.sleep(1)

controller.disconnect()