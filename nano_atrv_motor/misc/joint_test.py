from vassar_feetech_servo_sdk import ServoController
import time
import inspect


#https://github.com/vassar-robotics/feetech-servo-sdk/tree/main

controller = ServoController(
    servo_ids=[1, 2, 3, 4, 5],
    servo_type="sts",  # 'sts' or 'hls'
    port="/dev/ttyUSB0",
    baudrate=1000000
)
controller.connect()

# 1 = left wheel
# 2 = left steering
# 3 = right steering
# 4 = right wheel
# 5 = rear steering
# 4096 ticks = 360 deg

#All joints forward
results = controller.write_position({
    2: 2048,
    3: 2048, 
    5: 2048
})
time.sleep(1)


#Left Steering test
results = controller.write_position({
    2: 2488, #max counterclockwise
    3: 2048, 
    5: 2048
})
time.sleep(3)


results = controller.write_position({
    2: 870, #max clockwise
    3: 2048, 
    5: 2048
})
time.sleep(3)

#Right Steering test
results = controller.write_position({
    2: 2048,
    3: 1618, #max clockwise
    5: 2048
})
time.sleep(3)


results = controller.write_position({
    2: 2048, 
    3: 3214, #max counterclockwise
    5: 2048
})
time.sleep(3)

#Rear Caster test
results = controller.write_position({
    2: 2048,
    3: 2048, 
    5: 1536 #90 deg
})
time.sleep(3)

results = controller.write_position({
    2: 2048,
    3: 2048, 
    5: 1024 #45 deg
})
time.sleep(3)

controller.disconnect()