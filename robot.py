__version__ = '0.2'
__author__ = 'Jefferson Zhang'
__github__ = 'github.com/lutet88'
__license__ = "Apache License 2.0. https://www.apache.org/licenses/LICENSE-2.0"

"""
robot.py - MicroPython robot handler for the 5 robots I built for my students.
make sure you have "import robot, os" in boot.py for this to work properly.
requires: hcsr04 (https://github.com/rsc1975/micropython-hcsr04) by rsc1975
"""

from machine import Pin, PWM
from hcsr04 import HCSR04
import time

### definitions
SERVO_MIN = 64 # out of 1024
SERVO_MID = 160 # out of 1024
SERVO_MAX = 256 # out of 1024
SERVO_FREQ = 100 # Hz
SERVO_ANGLE = 180 # degrees max

MOTOR_OFF = 0 # out of 1024 (or really, anything <256)
MOTOR_MAX = 1023 # out of 1024
MOTOR_FREQ = 20 # Hz

CLOCKWISE = 1
COUNTERCLOCKWISE = 2

### pins
MOTOR_A = 32
MOTOR_B = 33
MOTOR_C = 12
MOTOR_D = 13

FRONT_SERVO = 14

SR04_TRIG = 25
SR04_ECHO = 26

### motors
# access with robot.motors
# using class MotorController, a PWM handler
class MotorController:
  motorA = PWM(Pin(MOTOR_A))
  motorB = PWM(Pin(MOTOR_B))
  motorC = PWM(Pin(MOTOR_C))
  motorD = PWM(Pin(MOTOR_D))

  motorA.freq(MOTOR_FREQ)
  motorB.freq(MOTOR_FREQ)
  motorC.freq(MOTOR_FREQ)
  motorD.freq(MOTOR_FREQ)

  motorA.duty(0)
  motorB.duty(0)
  motorC.duty(0)
  motorD.duty(0)
  
  calibrateL = 1
  calibrateR = 1
  
  
  def __init__(self):
    self.motorA.duty(0)
    self.motorB.duty(0)
    self.motorC.duty(0)
    self.motorD.duty(0)
    self.calibrateL = 1
    self.calibrateR = 1
  
  def halt(self):
    """Halts all motors (sets duty to 0)
    """
    self.motorA.duty(0)
    self.motorB.duty(0)
    self.motorC.duty(0)
    self.motorD.duty(0)
  
  def calibrate(self, left, right):
    """Calibrates left and right motors
    :param float left: left calibration % (0-200)
    :param float right: right calibration % (0-200)
    """
    if not (0 <= left <= 200 and 0 <= right <= 200):
      raise LookupError("Calibration invalid: Must be between 0 and 200")
    self.calibrateL = left / 100
    self.calibrateR = right / 100
    print("New calibrations:", [self.calibrateL, self.calibrateR])
  
  def rightMotor(self, speed):
    """Internal function: set speed of right motor
    :param float speed: speed of motor (0-100)
    """
    if not -100 <= speed <= 100:
      raise LookupError("Speed invalid: "+str(speed)) 
    s = int(abs(speed * MOTOR_MAX / 100))
    if speed == 0:
      self.motorA.duty(0)
      self.motorB.duty(0)
    elif speed < 0:
      self.motorA.duty(min(int(s * self.calibrateR), 1023))
      self.motorB.duty(0)
    else:
      self.motorA.duty(0)
      self.motorB.duty(min(int(s * self.calibrateR), 1023))
  
  def leftMotor(self, speed):
    """Internal function: set speed of left motor
    :param float speed: speed of motor (0-100)
    """
    if not -100 <= speed <= 100:
      raise LookupError("Speed invalid: "+str(speed)) 
    s = int(abs(speed * MOTOR_MAX / 100))
    if speed == 0:
      self.motorC.duty(0)
      self.motorD.duty(0)
    elif speed < 0:
      self.motorC.duty(min(int(s * self.calibrateL), 1023))
      self.motorD.duty(0)
    else:
      self.motorC.duty(0)
      self.motorD.duty(min(int(s * self.calibrateL), 1023))
  
  def forward(self, speed, ms, halt=True):
    """Moves the robot forward.
    :param float speed: speed of movement (0-100)
    :param int ms: time in ms to move
    :param boolean halt: whether to halt after the movement
    """
    self.rightMotor(speed)
    self.leftMotor(speed)
    time.sleep(ms / 1000)
    if halt:
      self.rightMotor(0)
      self.leftMotor(0)
  
  def backward(self, speed, ms, halt=True):
    """Moves the robot backwards.
    :param float speed: speed of movement (0-100)
    :param int ms: time in ms to move
    :param boolean halt: whether to halt after the movement
    """
    self.forward(-speed, ms, halt)

  def turnLeft(self, speed, ms, in_place=False, halt=True):
    """Turns the robot to the left.
    :param float speed: speed of movement (0-100)
    :param int ms: time in ms to move
    :param boolean in_place: whether to turn in-place or rotate.
    :param boolean halt: whether to halt after the movement
    """
    if in_place:
      self.rightMotor(speed)
      self.leftMotor(-speed)
      time.sleep(ms / 1000)
      if halt:
        self.rightMotor(0)
        self.leftMotor(0)
    else:
      self.rightMotor(speed)
      time.sleep(ms / 1000)
      if halt:
        self.rightMotor(0)
  
  def turnRight(self, speed, ms, in_place=False, halt=True):
    """Turns the robot to the right.
    :param float speed: speed of movement (0-100)
    :param int ms: time in ms to move
    :param boolean in_place: whether to turn in-place or rotate.
    :param boolean halt: whether to halt after the movement
    """
    if in_place:
      self.rightMotor(-speed)
      self.leftMotor(speed)
      time.sleep(ms / 1000)
      if halt:
        self.rightMotor(0)
        self.leftMotor(0)
    else:
      self.leftMotor(speed)
      time.sleep(ms / 1000)
      if halt:
        self.leftMotor(0)
  
  def spin(self, speed, dir, ms, halt=True):
    """Makes the robot spin.
    :param float speed: speed of movement (0-100)
    :param int ms: time in ms to move
    :param int dir: robot.CLOCKWISE or robot.COUNTERCLOCKWISE
    :param boolean halt: whether to halt after the spin
    """
    if dir == CLOCKWISE:
      self.rightMotor(-speed)
      self.leftMotor(speed)
    elif dir == COUNTERCLOCKWISE:
      self.rightMotor(speed)
      self.leftMotor(-speed)
    else:
      raise Exception("invalid direction. must be robot.CLOCKWISE or robot.COUNTERCLOCKWISE")
    time.sleep(ms / 1000)
    if halt:
      self.rightMotor(0)
      self.leftMotor(0)


motors = MotorController()

### servo
# access with robot.servo
# using class FrontServo, a PWM servo handler
class Servo:
  def __init__(self):
    self.servo = PWM(Pin(FRONT_SERVO))
    self.servo.freq(SERVO_FREQ)
    self.servo.duty(SERVO_MID)
  
  def setAngle(self, angle, delay=True):
    """Sets the intended angle of the servo.
    :param float angle: intended angle of servo. range: [-(SERVO_ANGLE / 2), (SERVO_ANGLE / 2)]
    :param boolean delay: whether to delay the requisite amount of time for the servo to spin. (based on SG91R)
    """
    a = self.servo.duty()
    if not -(SERVO_ANGLE / 2) <= angle <= (SERVO_ANGLE / 2):
      raise LookupError("Angle invalid: "+str(angle)) 
    d = angle + (SERVO_ANGLE / 2)
    d *= (SERVO_MAX - SERVO_MIN) / SERVO_ANGLE
    d = int(d + 64)
    self.servo.duty(d)
    if delay:
      time.sleep(0.1 * abs(angle - a) / 60)
    return d
  
  def getAngle(self):
    """Gets the current intended angle of the servo.
    :return: the intended angle of the servo. range: [-(SERVO_ANGLE / 2), (SERVO_ANGLE / 2)]
    """
    a = self.servo.duty()
    a -= 64
    a *= SERVO_ANGLE / (SERVO_MAX - SERVO_MIN)
    a -= (SERVO_ANGLE / 2)
    return a


servo = Servo()

### ultrasonic sensor
# access with robot.sensor
# using class UltrasonicSensor, a simple extension to HCSR04 by Roberto Sanchez
class UltrasonicSensor(HCSR04):
  def __init__(self):
    super().__init__(trigger_pin=SR04_TRIG, echo_pin=SR04_ECHO)

  def getDistance(self):
    """Gets the current distance measured by the ultrasonic sensor.
    Please don't call this more than around 100 times a second.
    :return: the current distance measured by the ultrasonic sensor.
    """
    return super().distance_cm()


sensor = UltrasonicSensor()


### system-wide functions
def halt():
  """Halts all motors and moves servo to angle 0.
  """
  motors.halt()
  servo.setAngle(0)
  
