import stretch_body.robot
import math
import time
robot=stretch_body.robot.Robot()
robot.startup()
robot.head.move_to('head_pan', math.radians(90))
robot.head.move_to('head_tilt',math.radians(-90))
robot.push_command()
time.sleep(2.0)
robot.head.move_to('head_pan', math.radians(0))
robot.head.move_to('head_tilt',math.radians(-90))
robot.push_command()
time.sleep(2.0)


