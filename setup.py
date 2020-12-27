from machine import Pin, PWM
from hcsr04 import HCSR04
import time

# definitions
SERVO_MIN = 64 # out of 1024
SERVO_MID = 160 # out of 1024
SERVO_MAX = 256 # out of 1024
SERVO_FREQ = 100 # Hz
SERVO_ANGLE = 216 # degrees max

MOTOR_OFF = 0 # out of 1024 (or really, anything <256)
MOTOR_MINPWR = 320 # out of 1024
MOTOR_MAXPWR = 1023 # out of 1024
MOTOR_FREQ = 20 # Hz

# object assignments
motorA = PWM(Pin(32))
motorB = PWM(Pin(33))
motorC = PWM(Pin(34))
motorD = PWM(Pin(35))

motorA.freq(MOTOR_FREQ)
motorB.freq(MOTOR_FREQ)
motorC.freq(MOTOR_FREQ)
motorD.freq(MOTOR_FREQ)

motorA.duty(0)
motorB.duty(0)
motorC.duty(0)
motorD.duty(0)

# servo
class FrontServo:
  def __init__(self):
    self.servo = PWM(Pin(14))
    self.servo.freq(SERVO_FREQ)
    self.servo.duty(SERVO_MID)
  
  def setAngle(self, angle):
    self.servo.duty((angle / SERVO_ANGLE + 0.5) * (SERVO_MAX - SERVO_MIN) + SERVO_MIN)
  
  def getAngle(self):
    return (self.servo.duty() - SERVO_MIN) / (SERVO_MAX - SERVO_MIN) * SERVO_ANGLE + 0.5 * SERVO_ANGLE

servo = FrontServo()

# ultrasonic sensor
class UltrasonicSensor(HCSR04):
  def __init__(self):
    super().__init__(trigger_pin=25, echo_pin=26)
  
  def getDistance(self):
    return super().distance_cm()

sensor = UltrasonicSensor()
