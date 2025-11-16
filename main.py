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

def turn_if_needed(distance):
    if distance < 150.0:
        base.turn(-angle)

while color_sensor.color() != Color.BLACK:
    base.straight(10 * 10)

    base.turn(angle)

    distance = average_ultrasonic_distance(ultrasonic_sensor)

    turn_if_needed(distance)

    wait(10)


#while True:
#    if color_sensor.reflection() < 10:
#        base.turn(-correction_angle)
#    elif color_sensor.reflection() > 10:
#        base.turn(correction_angle)
#    else:
#        base.straight(200)
#base.turn(angle)
#front_distance=ultrasonic_sensor.distance()
#hand_motor.run_angle(100, -90)
#while color_sensor.reflection() > 10:
#    wall_distance=ultrasonic_sensor.distance()
#
#    print(wall_distance)
#
#    print(path_lenght)
#    if front_distance>200.0:
#        if wall_distance>170.0 and wall_distance<230.0 or front_distance>600:
#            base.straight(300)
#            path_lenght+=300
#            
#        elif wall_distance<170.0:
#            base.turn(correction_angle)
#        elif wall_distance>300.0:
#            base.straight(200)
#            base.turn(-angle)
#            hand_motor.run_angle(100, 90)
#            front_distance=ultrasonic_sensor.distance()
#            hand_motor.run_angle(100, -90)
#            path_lenght=0
#        elif wall_distance>230.0:
#            base.turn(-correction_angle)
#        else:
#            base.straight(50)
#            path_lenght+=50
#    elif front_distance<=200.0:
#        base.turn(angle)
#        hand_motor.run_angle(100, 90)
#        front_distance=ultrasonic_sensor.distance()
#        hand_motor.run_angle(100, -90)
#        path_lenght=0
#    else:
#        base.straight(50)
#        path_lenght+=50
#
#    front_distance=front_distance-path_lenght
#
#
#
#
## Write your program here.
#ev3.speaker.beep()
