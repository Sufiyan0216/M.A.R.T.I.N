#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time
from math import pi

leftEn = 32         #   Purple
rightEn = 33      #   Red

#leftBackward = 5    #   Blue
#leftForward = 6     #   Green
#rightForward = 16   #   Yellow
#rightBackward = 20  #   Orange

left_dir = 16
right_dir = 18

wheel_separation = 0.52           #0.625
wheel_diameter = 0.125             #0.21
wheel_radius = wheel_diameter/2
circumference_of_wheel = 2 * pi * wheel_radius
max_speed = 2.00

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

GPIO.setup(leftEn, GPIO.OUT)
GPIO.setup(rightEn, GPIO.OUT)
#GPIO.setup(leftForward, GPIO.OUT)
#GPIO.setup(leftBackward, GPIO.OUT)
#GPIO.setup(rightForward, GPIO.OUT)
#GPIO.setup(rightBackward, GPIO.OUT)
GPIO.setup(left_dir, GPIO.OUT)
GPIO.setup(right_dir, GPIO.OUT)

pwmL = GPIO.PWM(leftEn, 100)
pwmL.start(0)
pwmR = GPIO.PWM(rightEn, 100)
pwmR.start(0)

def stop():
    #print('stopping')
    pwmL.ChangeDutyCycle(0)
    #GPIO.output(leftForward, GPIO.HIGH)
    #GPIO.output(leftBackward, GPIO.HIGH)
    pwmR.ChangeDutyCycle(0)
    #GPIO.output(rightForward, GPIO.HIGH)
    #GPIO.output(rightBackward, GPIO.HIGH)
    GPIO.output(left_dir, GPIO.LOW)
    GPIO.output(right_dir, GPIO.LOW)

def forward(left_speed, right_speed):
    #print('going forward')
    lspeed = min(((left_speed/max_speed)*100),100)
    rspeed = min(((right_speed/max_speed)*100),100)
    #print(str(left_speed)+" "+str(right_speed))
    pwmL.ChangeDutyCycle(lspeed)
    pwmR.ChangeDutyCycle(rspeed)
    #GPIO.output(leftForward, GPIO.HIGH)
    #GPIO.output(rightForward, GPIO.HIGH)
    #GPIO.output(leftBackward, GPIO.LOW)
    #GPIO.output(rightBackward, GPIO.LOW)
    GPIO.output(left_dir, GPIO.HIGH)
    GPIO.output(right_dir, GPIO.LOW)

def backward(left_speed, right_speed):
    #print('going backward')
    lspeed = min(((left_speed/max_speed)*100),100)
    rspeed = min(((right_speed/max_speed)*100),100)
    #print(str(left_speed)+" "+str(right_speed))
    pwmL.ChangeDutyCycle(lspeed)
    pwmR.ChangeDutyCycle(rspeed)
    #GPIO.output(leftForward, GPIO.LOW)
    #GPIO.output(rightForward, GPIO.LOW)
    #GPIO.output(leftBackward, GPIO.HIGH)
    #GPIO.output(rightBackward, GPIO.HIGH)
    GPIO.output(left_dir, GPIO.LOW)
    GPIO.output(right_dir, GPIO.HIGH)

def left(left_speed, right_speed):
    #print('turning left')
    lspeed = min(((left_speed/max_speed)*100),100)
    rspeed = min(((right_speed/max_speed)*100),100)
    pwmL.ChangeDutyCycle(lspeed)
    pwmR.ChangeDutyCycle(rspeed)
    #GPIO.output(leftForward, GPIO.LOW)
    #GPIO.output(leftBackward, GPIO.HIGH)
    #GPIO.output(rightForward, GPIO.HIGH)
    #GPIO.output(rightBackward, GPIO.LOW)
    GPIO.output(left_dir, GPIO.HIGH)
    GPIO.output(right_dir, GPIO.HIGH)

def right(left_speed, right_speed):
    #print('turning right')
    lspeed = min(((left_speed/max_speed)*100),100)
    rspeed = min(((right_speed/max_speed)*100),100)
    pwmL.ChangeDutyCycle(lspeed)
    pwmR.ChangeDutyCycle(rspeed)
    #GPIO.output(leftForward, GPIO.HIGH)
    #GPIO.output(leftBackward, GPIO.LOW)
    #GPIO.output(rightForward, GPIO.LOW)
    #GPIO.output(rightBackward, GPIO.HIGH)
    GPIO.output(left_dir, GPIO.LOW)
    GPIO.output(right_dir, GPIO.LOW)
    
def callback(data):
    
    global wheel_radius
    global wheel_separation
    
    linear_vel = data.linear.x
    angular_vel = data.angular.z

    if(0.007 <= linear_vel < 0.139):
       linear_vel =  0.14
    if(-0.139 < linear_vel <= -0.007):
       linear_vel = -0.14
    if(linear_vel == 0):
       linear_vel = 0
   # print(str(linear_vel)+"\t"+str(angular_vel))
    
    rplusl  = ( 2 * linear_vel ) / wheel_radius
    rminusl = ( angular_vel * wheel_separation ) / wheel_radius
    
    left_omega = ( rplusl + rminusl ) / 2
    right_omega  = (rplusl - left_omega)
    
    right_vel = right_omega * wheel_radius
    left_vel  = left_omega  * wheel_radius
    
#    print (str(left_vel)+"\t"+str(right_vel))
    '''
    left_speed  = abs ( linear - ( (wheel_separation/2) * (angular) ) )
    right_speed = abs ( linear - ( (wheel_separation/2) * (angular) ) )
    '''
    
    if (left_vel == 0.0 and right_vel == 0.0):
        stop()
    elif (left_vel >= 0.0 and right_vel >= 0.0):
        forward(abs(left_vel), abs(right_vel))
    elif (left_vel <= 0.0 and right_vel <= 0.0):
        backward(abs(left_vel), abs(right_vel))
    elif (left_vel < 0.0 and right_vel > 0.0):
        left(abs(left_vel), abs(right_vel))
    elif (left_vel > 0.0 and right_vel < 0.0):
        right(abs(left_vel), abs(right_vel))
    else:
        stop()
        
def listener():
    rospy.init_node('cmdvel_listener', anonymous=False)
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.spin()

if __name__== '__main__':
    print('Int1500 Differential Drive Initialized')
    listener()
