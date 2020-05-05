import numpy as np
import cv2
import time
from classes import Point, Line
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge, CvBridgeError
import visao_module
import rospy
import matplotlib.pyplot as plt


class AI:

    def __init__(self):
        self.frame = None
        self.buffering = 5
        self.lista_goodLeft = [0]*self.buffering
        self.lista_goodRight = [0]*self.buffering
        self.lydar = None
        self.__acceptableDelay__ = 1.5E9
        self.mobileNetResults = None
        self.modifiedFrame = None

    def alignToTarget(self, point):
        if self.checkFrame():
            targetX = point.x
            currentX = self.frame.shape[1]//2

            # cv2.circle(self.modifiedFrame, (self.frame.shape[1]/2,self.frame.shape[0]/2), 2,(0,0,255), 5)
            
            direction = targetX - currentX
            velArr = [Vector3(0,0,0), Vector3(0,0,0)]
            if direction >= 2: 
                velArr = [Vector3(0,0,0), Vector3(0,0,-0.1)]
            elif direction <= -2:
                velArr = [Vector3(0,0,0), Vector3(0,0,0.1)]
            return velArr
    
    def followRoad(self):
        if self.checkFrame():
            edges = self.treatForLines()
            lines = cv2.HoughLines(edges,1,np.pi/180, 75)
            # print(lines)
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

                        if lin.m < -0.1:
                            self.lista_goodLeft.pop(0)
                            self.lista_goodLeft.append(lin)
                        elif lin.m > 0.1:
                            self.lista_goodRight.pop(0)
                            self.lista_goodRight.append(lin)


                if 0 not in self.lista_goodLeft and 0 not in self.lista_goodRight:
                    average_Left = self.calculateMeanLine(self.lista_goodLeft)
                    average_Right = self.calculateMeanLine(self.lista_goodRight)
                    a, b = average_Left.getPoints()
                    c, d = average_Right.getPoints()
                    cv2.line(self.modifiedFrame, a, b,(255,0,0),2)
                    cv2.line(self.modifiedFrame, c, d,(255,0,0),2)
                    inter = average_Left.intersect(average_Right)
                    cv2.circle(self.modifiedFrame, inter.getTuple(), 5,(0,255,255), 5)
                    
                    return inter
                else:
                    return


    def identifyColor(self):
        pass

    def identifyId(self):
        pass

    def stop(self):
        pass

    def searchRotate(self):
        velArr = [Vector3(0,0,0), Vector3(0,0,0.1)]
        pass

    def centralizeCamera(self):
        pass

    def fastAdvance(self):
        velArr = [Vector3(0.3,0,0), Vector3(0,0,0)]
        return velArr

    def slowAdvance(self):
        velArr = [Vector3(0.1,0,0), Vector3(0,0,0)]
        return velArr

    def calculateMeanLine(self, linhas):
        first_point_x = int(round(np.mean([linha.point1.x for linha in linhas])))
        first_point_y = int(round(np.mean([linha.point1.y for linha in linhas])))
        second_point_x = int(round(np.mean([linha.point2.x for linha in linhas])))
        second_point_y = int(round(np.mean([linha.point2.y for linha in linhas])))
        first_point = Point(first_point_x, first_point_y)
        second_point = Point(second_point_x, second_point_y)
        return Line(first_point, second_point)

    def auto_canny(self, sigma=0.33):
        if self.checkFrame():
            # compute the median of the single channel pixel intensities
            v = np.median(self.frame.copy())

            # apply automatic Canny edge detection using the computed median
            lower = int(max(0, (1.0 - sigma) * v))
            upper = int(min(255, (1.0 + sigma) * v))
            edged = cv2.Canny(self.frame.copy(), lower, upper)

            # return the edged image
            return edged
    
    def treatForLines(self):
        # print("Chamou treatForLines")
        if self.checkFrame():
            # print("Entrou checkFrame")
            # Shape detection using color (cv2.inRange masks are applied over orginal image)
            temp = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(cv2.GaussianBlur(temp,(5,5),0),np.array([0,0,200]),np.array([180,10,255]))
            morphMask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3, 3)))
            contornos, arvore = cv2.findContours(morphMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            frame_out = cv2.drawContours(morphMask, contornos, -1, [0, 0, 255], 3)
            return frame_out

    def setFrame(self, frame):
        self.frame = frame
        self.modifiedFrame = frame.copy()
        return
    
    def showFrame(self):
        cv2.imshow('teste' , self.modifiedFrame)
        cv2.waitKey(1)
        return


    def showMask(self):
        cv2.imshow('mask' , self.treatForLines())
        cv2.waitKey(1)
        return

    def mobileNet(self):

        if self.checkFrame():
            # print("delay ", "{:.3f}".format(delay/1.0E9))
            try:
                _a_, _b_, resultados =  visao_module.processa(self.frame, showFrame=False)        
                self.mobileNetResults = resultados
                return resultados
            except CvBridgeError as e:
                print('ex', e)
                self.mobileNetResults = None
                return 

    def checkFrame(self):
        return (self.frame is not None)
        
