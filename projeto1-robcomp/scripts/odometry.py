#! /usr/bin/env python
# -*- coding:utf-8 -*-

# Sugerimos rodar com:
# roslaunch turtlebot3_gazebo  turtlebot3_empty_world.launch 


from __future__ import print_function, division
import rospy
import numpy as np
import cv2
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
import math
import time
from tf import transformations


x = None
y = None
angrobot = None

contador = 0
pula = 2

def recebe_odometria(data):
    global x
    global y
    global contador
    global angrobot

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    quat = data.pose.pose.orientation
    lista = [quat.x, quat.y, quat.z, quat.w]
    angulos = np.degrees(transformations.euler_from_quaternion(lista))    
    angrobot = angulos[2]
    if contador % pula == 0:
        print("Posicao (x,y)  ({:.2f} , {:.2f}) + angulo {:.2f}".format(x, y,angulos[2]))
    contador = contador + 1

def gira():
    kappa = [Vector3(0,0,0), Vector3(0,0,0.1)]
    return kappa
def checkAngle():
    angulocorreto = np.arctan((y/x))*180/np.pi
    kappa = [Vector3(0,0,0), Vector3(0,0,0)]
    if x==0 and y>0:
        angulocorreto = -90
        while (angrobot < (angulocorreto+0.05) and angrobot > (angulocorreto-0.05)) == False:
            print(angrobot, angulocorreto)
    elif x==0 and y<0:
        angulocorreto = 90 
        while (angrobot < (angulocorreto+0.05) and angrobot > (angulocorreto-0.05)) == False:
            print(angrobot, angulocorreto)
    elif y==0 and x>0:
        angulocorreto = -180 
        while (angrobot < (angulocorreto+0.05) and angrobot > (angulocorreto-0.05)) == False:
            print(angrobot, angulocorreto)
    elif y==0 and x<0:
        angulocorreto = 0
        while (angrobot < (angulocorreto+0.05) and angrobot > (angulocorreto-0.05)) == False:
            print(angrobot, angulocorreto)
    elif x>0 and y>0:
        while (angrobot < (angulocorreto+0.05-180) and angrobot > (angulocorreto-0.05-180)) == False:
            print(angrobot, angulocorreto)
    elif x<0 and y<0:
        while (angrobot < (angulocorreto+0.05) and angrobot > (angulocorreto-0.05)) == False:
            print(angrobot, angulocorreto)
    elif x>0 and y<0:
        while (angrobot < (angulocorreto+0.05+180) and angrobot > (angulocorreto-0.05+180)) == False:
            print(angrobot, angulocorreto)
    elif x<0 and y>0:
        while (angrobot < (angulocorreto+0.05) and angrobot > (angulocorreto-0.05)) == False:
            print(angrobot, angulocorreto)
    
        
    print("foi")
    return kappa
def forward():
    kappa = [Vector3(0.1,0,0), Vector3(0,0,0)]
    return kappa
def ateChegar():
    kappa = [Vector3(0,0,0), Vector3(0,0,0)]
    while ((x>0.01 or x<-0.01) and (y>0.01 or y<-0.01)) == True:
        print(x,y)
    return kappa

if __name__=="__main__":

    rospy.init_node("exemplo_odom")

    t0 = rospy.get_rostime()


    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )

    ref_odometria = rospy.Subscriber("/odom", Odometry, recebe_odometria)

    while not rospy.is_shutdown():
        print("t0", t0)
        if t0.nsecs == 0:
            t0 = rospy.get_rostime()
            print("waiting for timer")
            continue        
        t1 = rospy.get_rostime()
        elapsed = (t1 - t0)
        print("Passaram ", elapsed.secs, " segundos")
        if elapsed.secs == 30:
            vel = gira()
            velocidade_saida.publish(vel)
            vel = checkAng()
            velocidade_saida.publish(vel)
            rospy.sleep(1)
            vel = forward()
            velocidade_saida.publish(vel)
            vel = checkXY()
            velocidade_saida.publish(vel)
        rospy.sleep(0.5)
