from controller import Robot

TIME_STEP = 64
MAX_SPEED = 6.28
DISTANCE_SENSORS_NUMBER = 8

DISTANCE_SENSORS_NAMES = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]
DISTANCE_SENSORS = []

# create the Robot instance.
robot = Robot()

# get the motor devices
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
# set the target position of the motors
leftMotor.setPosition(float('+inf'))
rightMotor.setPosition(float('+inf'))
leftMotor.setVelocity(MAX_SPEED)
rightMotor.setVelocity(MAX_SPEED)

# IMPORTANT this functions allows the robot to move
while robot.step(TIME_STEP) != -1:
   pass

# INITIALIZE SENSORS & LEDS
