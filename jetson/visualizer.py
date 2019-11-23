# Thanks, Carson of 4910!!!


import turtle
import math

START_POSE=(0, 0, 0) #initial pose in (in, in, rad)

width=324   #width of field in inches
height=648  #height of field in inches
turtle.setup(width+100, height+100, 0, 0)   #+100 for spacing

robot=turtle.Turtle()   #create turtle object
robot.speed(0)          #robot travels faster than the speed of light

#DRAW FIELD
robot.penup()
robot.goto(-width/2, -height/2)
robot.pendown()
robot.goto(width/2, -height/2)
robot.goto(width/2, height/2)
robot.goto(-width/2, height/2)
robot.goto(-width/2, -height/2)

#DRAW INITIAL ROBOT POSE
robot.penup()
robot.goto(-START_POSE[1], START_POSE[0]-(height/2)) #pos manipulation to suit turtle coordinate system
robot.seth((START_POSE[2]+(math.pi/2))/math.pi*180)  #rad manipulation to suit turtle heading system
robot.pendown()

def move(new_pose):  #set new robot pose
    # print(f'New pose: {new_pose}')
    robot.goto(-new_pose[1], new_pose[0]-(height/2))
    robot.seth((new_pose[2]+(math.pi/2))/math.pi*180)
    # print(f'Robot Coordinates:  ({-robot.ycor()}, {robot.xcor() - height/2})')
#turtle.done()   #finish

# Begin Mr Odom code

def update(pose):
    move((pose[0]*39.3701, pose[1]*39.3701, pose[2]))
