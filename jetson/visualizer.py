# Thanks, Carson of 4910!!!


import turtle
import math

WIDTH = 324   # width of field in inches
HEIGHT = 648  # height of field in inches

INCHES_PER_METER = 39.37

def field_to_screen(pose):
    return (
        -pose[1]*INCHES_PER_METER,
        pose[0]*INCHES_PER_METER - HEIGHT/2,
        (pose[2]+(math.pi/2))/math.pi*180)

START_POSE=(0, 0, 0) #initial pose in (in, in, rad)

turtle.setup(WIDTH+100, HEIGHT+100, 0, 0)   #+100 for spacing

robot=turtle.Turtle()   #create turtle object
robot.speed(0)          #robot travels faster than the speed of light

#DRAW FIELD
robot.penup()
robot.goto(-WIDTH/2, -HEIGHT/2)
robot.pendown()
robot.goto(WIDTH/2, -HEIGHT/2)
robot.goto(WIDTH/2, HEIGHT/2)
robot.goto(-WIDTH/2, HEIGHT/2)
robot.goto(-WIDTH/2, -HEIGHT/2)

#DRAW INITIAL ROBOT POSE
robot.penup()
x, y, theta = field_to_screen(START_POSE)
robot.goto(x, y)
robot.seth(theta)
robot.pendown()

lookahead = turtle.Turtle()
lookahead.speed(0)
lookahead.color('red')
lookahead.penup()
lookahead.goto(x, y)
lookahead.seth(theta)
lookahead.pendown()

def move(pose, lookahead_point):  #set new robot pose
    x, y, theta = field_to_screen(pose)

    xl, yl = x, y  # not really meaningful
    if lookahead_point is not None:
        xl, yl, _ = field_to_screen((lookahead_point[0], lookahead_point[1], 0))

    lookahead.goto(xl, yl)
    lookahead.seth(theta)

    robot.goto(x, y)
    robot.seth(theta)

#turtle.done()   #finish
