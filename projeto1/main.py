#!/usr/bin/python

# -*- coding: utf-8 -*-

# __author__      = "Matheus Dib, Fabio de Miranda" ==> Modificado
__author__ = "Carlos Dip, João Andrade, Lucas Fukada"


# Imports
import cv2
import numpy as np
import time
from classes import Point, Line


# Setup webcam video capture
cap = cv2.VideoCapture("lines.mp4")
time.sleep(1)

# Calculates mean line for intersection
def calculate_mean_line(linhas):
  first_point_x = int(round(np.mean([linha.point1.x for linha in linhas])))
  first_point_y = int(round(np.mean([linha.point1.y for linha in linhas])))
  second_point_x = int(round(np.mean([linha.point2.x for linha in linhas])))
  second_point_y = int(round(np.mean([linha.point2.y for linha in linhas])))
  first_point = Point(first_point_x, first_point_y)
  second_point = Point(second_point_x, second_point_y)
  return Line(first_point, second_point)

# Canny edge detection
def auto_canny(image, sigma=0.33):
    # compute the median of the single channel pixel intensities
    v = np.median(image)

    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)

    # return the edged image
    return edged

# Applies white mask for filtering sides of road.
def treatForLines(frame):
    # Shape detection using color (cv2.inRange masks are applied over orginal image)
    mask = cv2.inRange(cv2.GaussianBlur(frame,(5,5),0),np.array([0,0,0]),np.array([10,10,255]))
    morphMask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((6, 6)))
    contornos, arvore = cv2.findContours(morphMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    frame_out = cv2.drawContours(morphMask, contornos, -1, [0, 0, 255], 3)
    return frame_out

running = True
buffering = 5
lista_goodLeft = [0]*buffering
lista_goodRight = [0]*buffering

upperV = np.array([0, 200, 0], dtype=np.uint8)
lowerV = np.array([0, 255, 0], dtype=np.uint8)

upperB = np.array([200, 0, 0], dtype=np.uint8)
lowerB = np.array([255, 0, 0], dtype=np.uint8)


while running:
    ret, frame = cap.read()
    mask_green = cv2.inRange(frame, upperV, lowerV)
    mask_blue = cv2.inRange(frame, upperB, lowerB)
    mask = cv2.bitwise_or(mask_blue, mask_green)
    blur = cv2.GaussianBlur(mask, (5,5),0)
    edges = cv2.Canny(blur,50,150)
    lines = cv2.HoughLines(edges,1,np.pi/180, 75)
    if lines is not None:
        for line in lines:
            for rho, theta in line:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                pt1 = Point(int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                pt2 = Point(int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                lin = Line(pt1, pt2)

                if lin.m < -0.2:
                    lista_goodLeft.pop(0)
                    lista_goodLeft.append(lin)
                elif lin.m > 0.2:
                    lista_goodRight.pop(0)
                    lista_goodRight.append(lin)


        if 0 not in lista_goodLeft and 0 not in lista_goodRight:
            average_Left = calculate_mean_line(lista_goodLeft)
            average_Right =calculate_mean_line(lista_goodRight)
            a, b = average_Left.getPoints()
            c, d = average_Right.getPoints()
            cv2.line(frame, a, b,(255,0,0),2)
            cv2.line(frame, c, d,(255,0,0),2)
            inter = average_Left.intersect(average_Right)
            cv2.circle(frame, inter, 5,(0,255,255), 5)

        
    # Display the resulting frame
    cv2.imshow('Detector de circulos',frame)
    # Exit condition
    if cv2.waitKey(1) & 0xFF == ord('q'):
        running = False

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
