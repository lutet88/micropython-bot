# micropython-bot
micropython bot and software for a class I'm teaching

## requirements

 - esp32 or esp8266 (not tested)
 - https://github.com/rsc1975/micropython-hcsr04 library (included in codebase)
 - bot specs in `robot.md`

## API reference (`robot.py`)

`robot` - main library for the robot.
`import robot` - imports the robot library.
#### Global Variables
`robot.CLOCKWISE` - defines clockwise for `robot.motors.spin()`
`robot.COUNTERCLOCKWISE` - defines counterclockwise for `robot.motors.spin()`
`robot.MOTOR_A` - pin for right motor clockwise. (defined as 32)
`robot.MOTOR_B` - pin for right motor counterclockwise. (defined as 33)
`robot.MOTOR_C` - pin for left motor clockwise. (defined as 12)
`robot.MOTOR_D` - pin for left motor counterclockwise. (defined as 13)
`robot.FRONT_SERVO` - pin for front servo (defined as 14)
`robot.SR04_TRIG` - pin for the SR04 module's trigger. (defined as 25)
`robot.SR04_ECHO` - pin for the SR04 module's echo. (defined as 26)

#### Classes
`robot.motors` - preinitialized MotorController object.
`robot.servo` - preinitialized Servo object.
`robot.sensor` - preinitialized UltrasonicSensor object.

#### Methods
- `robot.halt()` - command to stop all motors and move the servo to angle 0.
- `robot.motors.halt()` - command to stop all motors.
- `robot.motors.calibrate(left, right)` - calibrates left and right motors
	- float `left`: left calibration % (0-200)
	- float `right`: right calibration % (0-200)
- `robot.motors.rightMotor(speed)` - sets speed of right motor.
	- float `speed`: speed between 0 and 100
- `robot.motors.leftMotor(speed)` - sets speed of left motor.
	-float  `speed`: speed between 0 and 100
- `robot.motors.forward(speed, ms)` - moves the robot forward for a specified period of time.
	- float `speed`: speed between 0 and 100
	- int `ms`: time to move in ms
	- boolean `halt`(optional): whether to halt after movement (default=`True`)
- `robot.motors.backward(speed, ms)`- moves the robot backwards for a specified period of time.
	- float `speed`: speed between 0 and 100
	- int `ms`: time to move in ms
	- boolean `halt`(optional): whether to halt after movement (default=`True`)
- `robot.motors.turnLeft(speed, ms)` - makes the robot turn left for a specified period of time.
	- float `speed`: speed between 0 and 100
	- int `ms`: time to move in ms
	- boolean `in_place`(optional): whether to turn in-place or rotate
	- boolean `halt`(optional): whether to halt after movement (default=`True`)
- `robot.motors.turnRight(speed, ms)` - makesthe robot turn right for a specified period of time.
	- float `speed`: speed between 0 and 100
	- int `ms`: time to move in ms
	- boolean `in_place`(optional): whether to turn in-place or rotate
	- boolean `halt`(optional): whether to halt after movement (default=`True`)
- `robot.motors.spin(speed, dir, ms)` - makes the robot spin in place
	- float `speed`: speed between 0 and 100
	- int `dir`: `robot.CLOCKWISE` or `robot.COUNTERCLOCKWISE`
	- int `ms`: time to spin in ms
	- boolean `halt`(optional): whether to halt after movement (default=`True`)
- `robot.servo.setAngle(angle)` - sets the angle of the servo
	- int `angle`: angle between -90 and 90 (by default)
	- boolean `delay`(optional): whether to delay until the servo is done moving
- `robot.servo.getAngle()` - gets the intended angle of the servo
	- `return` (int) the angle of the servo. may not exactly match `robot.servo.setAngle()`
- `robot.sensor.getDistance()` - gets the current distance measured by the ultrasonics sensor. 
	**note**: please do not call this more than around 10 times a second. accuracy is greatly diminished.
	- `return` (float) the distance measured by the sensor in centimeters (cm).

### how to use MicroPython
 Make sure you're using uPyCraft or Thonny or any other MicroPython IDE. I recommend uPyCraft. [Here's a good quick-start tutorial for it.](https://maker.pro/esp8266/tutorial/using-micropython-on-an-esp8266-with-upycraft)
 Connect to your controller (on Windows, it's a COM port, and on Linux, it's probably /dev/ttyUSBx)
 MicroPython controllers run Python in this fashion:

`reset or start` -> `boot.py` -> `main.py` -> `REPL`

I recommend putting your imports in `boot.py`. Just make sure it includes:
- `import robot`
- `import os` (in most cases)

In most cases you'll be running your code in main.py (so a REPL command isn't needed), so your code will have to go there. To reference and run another script on the MCU just `import script` inside `main.py`. 

To test the robot library you can `import test` in `main.py`. `test.py` is a testing program that tests all functionality of a basic robot outlined in `robot.md`.

### modifying `robot.py`
to make my `robot.py` work with your `robot`,  you may have to 
- change definitions:

`SERVO_MIN` - minimum duty out of 1024 

`SERVO_MID` - midpoint (0 degrees) duty out of 1024

`SERVO_MAX` - maximum duty out of 1024

`SERVO_FREQ` - frequency of PWM driving servo

`SERVO_ANGLE` - angle range of servo when converting from angle to duty

`MOTOR_OFF` - minimum duty of motor out of 1024

`MOTOR_MAX` - maximum duty of motor out of 1024

`MOTOR_FREQ` - frequency of PWM driving motors

- add more servos

create more `Servo` objects around line 245

if you're doing more than that I assume you know what you're doing.

