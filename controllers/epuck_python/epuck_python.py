from controller import Robot, DistanceSensor

# Global Constants
TIME_STEP = 64
INFINITY = float('+inf')
MAX_SPEED = 6.28
FRONT_SENSORS_NAMES = ["ps0", "ps7"]
LEFT_SENSOR_IN = "ps5"
LEFT_SENSOR_OUT = "ps6"
WALL_DETECTED_VALUE = 300
TURNING_WAIT_TIME = 1

# Global Variables
frontSensors = [None] * 2
frontSensorsValues = [None] * 2

# These two sensors keep the robot closer to the left wall
leftSensorIn = None  # Left-most sensor, used to instruct the robot to turn right
leftSensorOut = None # Left-front sensor, used to instruct the robot to turn left

# Global instances
robot = Robot()
leftMotor = robot.getDevice('left wheel motor')
rightMotor =  robot.getDevice('right wheel motor')

# INITIALIZE SENSORS
def init_devices():
   global leftSensorIn
   global leftSensorOut

   # Initialize both front sensors
   for i in range(2):
      frontSensors[i] = robot.getDevice(FRONT_SENSORS_NAMES[i])
      DistanceSensor.enable(frontSensors[i], int(robot.getBasicTimeStep()))
   
   # Initialize left sensors
   leftSensorIn = robot.getDevice(LEFT_SENSOR_IN)
   leftSensorOut = robot.getDevice(LEFT_SENSOR_OUT)
   DistanceSensor.enable(leftSensorIn, int(robot.getBasicTimeStep()))
   DistanceSensor.enable(leftSensorOut, int(robot.getBasicTimeStep()))
   
   print("[INFO] Sensors activated")

   leftMotor.setPosition(INFINITY)
   rightMotor.setPosition(INFINITY)
   leftMotor.setVelocity(0)
   rightMotor.setVelocity(0)

   step()

def step():
   if robot.step(int(robot.getBasicTimeStep())) == -1:
      pass

def get_front_sensors_values():
   frontSensorsSum = 0
   for i in range(2):
      frontSensorsValues[i] = DistanceSensor.getValue(frontSensors[i])
      frontSensorsSum += int(frontSensorsValues[i])
   
   return frontSensorsSum

def get_left_sensor_in_value():
   return int(DistanceSensor.getValue(leftSensorIn))

def get_left_sensor_out_value():
   return int(DistanceSensor.getValue(leftSensorOut))

def turn_right():
   leftMotor.setVelocity(MAX_SPEED)
   rightMotor.setVelocity(-MAX_SPEED)
   print("[INFO] Turning right")
   wait(TURNING_WAIT_TIME)

def get_closer_to_wall():
   leftMotor.setVelocity(MAX_SPEED/3)
   rightMotor.setVelocity(MAX_SPEED)

def get_farther_from_wall():
   leftMotor.setVelocity(MAX_SPEED)
   rightMotor.setVelocity(MAX_SPEED/3)

def stop():
   leftMotor.setVelocity(0.0)
   rightMotor.setVelocity(0.0)
   print("[INFO] Stoping")
   wait(0.2)

def wait(sec):
   startTime = robot.getTime()
   while startTime + sec > robot.getTime():
      step()



################################################################################################
#---------------------------------EVERYTHING STARTS HERE---------------------------------------#
################################################################################################
init_devices()

while True:

   # Check for walls in front
   if get_front_sensors_values() > WALL_DETECTED_VALUE:
      print("[INFO] Wall detected")
      stop()
      turn_right()
   
   # These to conditions make the robot follow the left wall
   if get_left_sensor_in_value() < WALL_DETECTED_VALUE / 2:
      get_closer_to_wall()
   
   if get_left_sensor_out_value() > WALL_DETECTED_VALUE / 2:
      get_farther_from_wall()

   step()
