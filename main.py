#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor
from pybricks.parameters import Port, Direction, Button, Color
from pybricks.robotics import DriveBase
from pybricks.ev3devices import ColorSensor
from pybricks.tools import wait

ev3 = EV3Brick()
 
left_motor = Motor(Port.C)

right_motor = Motor(Port.B)

wheel_diameter = 55.5
axle_track = 175
distance2 = 1000
distance1 = 2000

angle = 80
correction_angle=5
base = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)

path_lenght=0.0
color_sensor = ColorSensor(Port.S1)
ultrasonic_sensor = UltrasonicSensor(Port.S4)
front_distance = 1000.0

def average_ultrasonic_distance(sensor, samples=10, delay_ms=50):
    total = 0.0
    for _ in range(samples):
        total += sensor.distance()
        wait(delay_ms)
    return total / samples

left_motor = Motor(Port.C)
right_motor = Motor(Port.B)

ev3.screen.print("Place sensor on")
ev3.screen.print("BLACK")
ev3.screen.print("Press center")
ev3.screen.print("button")

while Button.CENTER not in ev3.buttons.pressed():
    wait(10)
black = color_sensor.reflection()

wait(500)

ev3.screen.clear()
ev3.screen.print("Place sensor on")
ev3.screen.print("WHITE")
ev3.screen.print("Press center")
ev3.screen.print("button")

while Button.CENTER not in ev3.buttons.pressed():
    wait(10)
white = color_sensor.reflection()

threshold = (black + white) / 2

DRIVE_SPEED = 50

PROPORTIONAL_GAIN = 1.2

wait(500)

ev3.screen.clear()
ev3.screen.print("Press center")
ev3.screen.print("button to start")

while Button.CENTER not in ev3.buttons.pressed():
    wait(10)

ev3.speaker.beep()  

while average_ultrasonic_distance(ultrasonic_sensor) > 250.0:
    deviation = color_sensor.reflection() - threshold

    turn_rate = PROPORTIONAL_GAIN * deviation

    base.drive(DRIVE_SPEED, turn_rate)

    wait(10)

base.turn(-angle)

while color_sensor.color() != Color.BLACK:
    base.straight(25 * 10)

    base.turn(angle)

    distance = average_ultrasonic_distance(ultrasonic_sensor)

    if distance < 150.0:
        base.turn(-angle)

        distance = average_ultrasonic_distance(ultrasonic_sensor)

        if distance < 150.0:
            base.turn(-angle)
    else:
        continue

    wait(10)

ev3.speaker.beep()
