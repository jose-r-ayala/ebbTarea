from controller import Robot, DistanceSensor

# Global Constants
TIME_STEP = 64
INFINITY = float('+inf')
MAX_SPEED = 6.28
FRONT_SENSORS_NAMES = ['ps0', 'ps7']
LED_NAMES = ['led0', 'led1', 'led2', 'led3', 'led4', 'led5', 'led6', 'led7', 'led8', 'led9']
WALL_DETECTED_VALUE = 350
TURNING_WAIT_TIME = 1

# Global Variables
frontSensors = [None] * len(FRONT_SENSORS_NAMES)
frontSensorsValues = [None] * len(FRONT_SENSORS_NAMES)
leds = [None] * len(LED_NAMES)

# Global instances
robot = Robot()
leftMotor = robot.getDevice('left wheel motor')
rightMotor =  robot.getDevice('right wheel motor')
leftSensorIn = robot.getDevice('ps5')  # Left-most sensor, used to instruct the robot to turn right
leftSensorOut = robot.getDevice('ps6') # Left-front sensor, used to instruct the robot to turn left

# INITIALIZE SENSORS
def init_devices():
   global leftSensorIn
   global leftSensorOut

   # Initialize both front sensors
   for i in range(2):
      frontSensors[i] = robot.getDevice(FRONT_SENSORS_NAMES[i])
      DistanceSensor.enable(frontSensors[i], int(robot.getBasicTimeStep()))
   
   # Initialize left sensors
   DistanceSensor.enable(leftSensorIn, int(robot.getBasicTimeStep()))
   DistanceSensor.enable(leftSensorOut, int(robot.getBasicTimeStep()))

   # Initialize leds
   for i in range(len(LED_NAMES)):
      leds[i] = robot.getDevice(LED_NAMES[i])
   
   print('[INFO] Sensors activated')

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
   print('[INFO] Turning right')
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
   print('[INFO] Stoping')
   wait(0.2)

def wait(sec):
   startTime = robot.getTime()
   while startTime + sec > robot.getTime():
      step()

def turn_on_leds():
   for led in leds:
      led.set(1)

def turn_off_leds():
   for led in leds:
      led.set(0)

################################################################################################
#---------------------------------EVERYTHING STARTS HERE---------------------------------------#
################################################################################################
init_devices()

while True:

   # Check for walls in front
   if get_front_sensors_values() > WALL_DETECTED_VALUE:
      print('[INFO] Wall detected')
      turn_on_leds()
      stop()
      turn_right()
      turn_off_leds()
   
   # These to conditions make the robot follow the left wall
   if get_left_sensor_in_value() < WALL_DETECTED_VALUE / 2:
      get_closer_to_wall()
   
   if get_left_sensor_out_value() > WALL_DETECTED_VALUE / 2:
      get_farther_from_wall()

   step()
