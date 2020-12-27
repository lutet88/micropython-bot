import robot
import time

print("calibrating. (110/100)")
robot.motors.calibrate(110, 100)

print("forward 50")
robot.motors.forward(50, 2000)

print("forward 100")
robot.motors.forward(100, 2000)

print("backward 75")
robot.motors.backward(75, 2000)

print("turnRight 50 not in place")
robot.motors.turnRight(50, 2000, False)

print("turnRight 50 in place") 
robot.motors.turnRight(50, 2000, True)

print("turnLeft 50 not in place")
robot.motors.turnLeft(50, 2000, False)

print("turnLeft 50 in place")
robot.motors.turnLeft(50, 2000, True)

print("current ultrasonic distance (over 10 getDistance calls):")
for i in range(10):
  print(robot.sensor.getDistance())
  time.sleep(0.2)
  
print("servo aerobic exercises...")
for i in range(3):
  robot.servo.setAngle(-90)
  robot.servo.setAngle(90)
robot.servo.setAngle(0)

print("starting test program...")
sensorDistance = robot.sensor.getDistance()
servoAngle = 0
servoDir = 1

while True:
  while sensorDistance > 15:
    robot.motors.forward(12, 600, False)
    sensorDistance = robot.sensor.getDistance()
    print("not yet, sensor distance is "+str(sensorDistance))
    
    robot.servo.setAngle(servoAngle, False)
    servoAngle += servoDir * 30
    if servoAngle > 60:
      servoDir = -1
    if servoAngle < -60:
      servoDir = 1
    robot.motors.halt()
    
  robot.motors.halt()
  print("turning right....")
  robot.motors.turnRight(12, 700, True)
  sensorDistance = robot.sensor.getDistance()
  robot.motors.halt()

