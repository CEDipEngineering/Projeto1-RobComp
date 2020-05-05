import numpy as np
import cv2
import time
from classes import Point, Line
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge, CvBridgeError
import visao_module
import rospy
import matplotlib.pyplot as plt
import os


class AI:

    def __init__(self):
        self.frame = None
        self.buffering = 10
        self.lista_goodLeft = [0]*self.buffering
        self.lista_goodRight = [0]*self.buffering
        self.lydar = None
        self.__acceptableDelay__ = 1.5E9
        self.mobileNetResults = None
        self.modifiedFrame = None
        self.lookDownMask = cv2.imread("Projeto1-RobComp/projeto1-robcomp/scripts/mask.png")
        self.lookDownMask = cv2.cvtColor(self.lookDownMask, cv2.COLOR_BGR2GRAY)

    def alignToTarget(self, point):
        if self.checkFrame():
            targetX = point.x

            # shape[1] == largura da janela
            currentX = self.frame.shape[1]//2

            # cv2.circle(self.modifiedFrame, (self.frame.shape[1]/2,self.frame.shape[0]/2), 2,(0,0,255), 5)
            
            direction = targetX - currentX
            velArr = [Vector3(0,0,0), Vector3(0,0,0)]
            if direction >= 3: 
                velArr = [Vector3(0,0,0), Vector3(0,0,-0.1)]
            elif direction <= -3:
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
                        

                        if lin.m < -2:
                            self.lista_goodLeft.pop(0)
                            self.lista_goodLeft.append(lin)
                            cv2.line(self.modifiedFrame, pt1.getTuple(), pt2.getTuple(),(255,0,0),2)
                        elif lin.m > 2:
                            self.lista_goodRight.pop(0)
                            self.lista_goodRight.append(lin)
                            cv2.line(self.modifiedFrame, pt1.getTuple(), pt2.getTuple(),(255,0,0),2)

                if 0 not in self.lista_goodLeft and 0 not in self.lista_goodRight:
                    average_Left = self.calculateMeanLine(self.lista_goodLeft)
                    average_Right = self.calculateMeanLine(self.lista_goodRight)
                    a, b = average_Left.getPoints()
                    c, d = average_Right.getPoints()
                    # cv2.line(self.modifiedFrame, a, b,(255,0,0),2)
                    # cv2.line(self.modifiedFrame, c, d,(255,0,0),2)
                    inter = average_Left.intersect(average_Right)
                    cv2.circle(self.modifiedFrame, inter.getTuple(), 5,(0,255,255), 5)
                    
                    return inter
                else:
                    return


    def identifyColor(self, color):
        colorDict = {
            "blue":[np.array([90,80,80]), np.array([155,255,255])],
            "green":[np.array([45,80,80]), np.array([90,255,255])],
            "magenta":[np.array([140,80,80]), np.array([180,255,255])]
        }
        color = color.lower()
        try:
            colorRange = colorDict[color]
        except KeyError as e:
            print("Cor fornecida '" + str(e) + "' nao e valida.")
            return 

        point, area = self._identifica_cor(colorRange[0], colorRange[1])
        cv2.circle(self.modifiedFrame, (point[0], point[1]), 3, (0,0,0), 2)
        point = Point(point[0], point[1])
        
        if area > 100:
            return point
        return    



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
            # print(str(mask.shape) + " : " + str(self.lookDownMask.shape))
            # mask = cv2.bitwise_and(mask, self.lookDownMask)
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
        
    def _identifica_cor(self, low , high):

        frame = cv2.GaussianBlur(self.frame, (5,5), cv2.BORDER_DEFAULT)
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        cor_menor = np.array(low)
        cor_maior = np.array(high)
        segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)
        frame_masked = cv2.morphologyEx(segmentado_cor, cv2.MORPH_CLOSE,np.ones((10, 10)))
        contornosMask, arvore = cv2.findContours(frame_masked.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    

        centro = (frame.shape[1]//2, frame.shape[0]//2)

        areas = [cv2.contourArea(x) for x in contornosMask]
        maior_contorno = None
        maior_contorno_area = 0
        for i,e in enumerate(areas):
            if e > maior_contorno_area:
                maior_contorno = contornosMask[i]
                maior_contorno_area = e

        if not maior_contorno is None :
            cv2.drawContours(frame, [maior_contorno], -1, [0, 0, 255], 5)
            maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
            media = maior_contorno.mean(axis=0)
            media = media.astype(np.int32)
            cv2.circle(frame, (media[0], media[1]), 5, [0, 255, 0])
            #print("Fazendo contornos")
        else:
            media = (0, 0)

        # Representa a area e o centro do maior contorno no frame
        font = cv2.FONT_HERSHEY_COMPLEX_SMALL
        cv2.putText(frame,"{:d} {:d}".format(*media),(20,100), 1, 4,(255,255,255),2,cv2.LINE_AA)
        cv2.putText(frame,"{:0.1f}".format(maior_contorno_area),(20,50), 1, 4,(255,255,255),2,cv2.LINE_AA)

        # cv2.imshow('video', frame)
        #cv2.imshow('frame', frame)
        #cv2.waitKey(1)

        return media, maior_contorno_area