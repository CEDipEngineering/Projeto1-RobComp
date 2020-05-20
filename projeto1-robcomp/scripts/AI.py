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
        self.buffering = 5
        self.lista_goodLeft = [0]*self.buffering
        self.lista_goodRight = [0]*self.buffering
        self.leftBuffer = 0
        self.rightBuffer = 0
        self.lydar = None
        self.__acceptableDelay__ = 1.5E9
        self.mobileNetResults = None
        self.modifiedFrame = None
        #self.lookDownMask = cv2.cvtColor(cv2.imread("Projeto1-RobComp/projeto1-robcomp/scripts/mask.png"), cv2.COLOR_BGR2GRAY)
        self.target = []
        self.x_0 = None
        self.y_0 = None
        self.x   = None
        self.y   = None
        self.markers = []
        self.angulo = None
        self.angulo_0 = None
        with open("Projeto1-RobComp/projeto1-robcomp/scripts/camParams.txt", "r") as fileObj:
            self.cameraInfo = np.fromstring(fileObj.read(), dtype=np.float32, sep=',')
            self.K = self.cameraInfo.reshape((3,3))
            self.D = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
            print(self.cameraInfo)
        self.counters = {
            "FramesSemAlinhar": 0,
            "FramesSemAlinharEstrada": 0
        }

    def alignToTarget(self, point):
        if self.checkFrame():
            targetX = point.x

            # shape[1] == largura da janela
            currentX = self.frame.shape[1]//2

            # cv2.circle(self.modifiedFrame, (self.frame.shape[1]/2,self.frame.shape[0]/2), 2,(0,0,255), 5)
            
            direction = targetX - currentX
            velArr = [Vector3(0,0,0), Vector3(0,0,0)]
            
            tol = 5
            if direction >= tol: 
                velArr = [Vector3(0,0,0), Vector3(0,0,-0.08)]
            elif direction <= -tol:
                velArr = [Vector3(0,0,0), Vector3(0,0,0.08)]
            return velArr
    
    def followRoad(self):
        if self.checkFrame():
            edges = self.treatForLines()
            minLineLength = 220
            maxLineGap = 8
            lines = cv2.HoughLinesP(edges,1,np.pi/180,180,minLineLength,maxLineGap)
            # print(lines)

            # Conta numero de retas reais em cada lista
            tempLenLeft = self.lista_goodLeft[:]
            tempLenRight = self.lista_goodRight[:]
            
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    pt1 = Point(x1,y1)
                    # print(pt1.x, pt1.y)
                    pt2 = Point(x2,y2)
                    # print(pt2.x, pt2.y)
                    lin = Line(pt1,pt2)
                    # print(lin.m)
                    # cv2.line(self.modifiedFrame, pt1.getTuple(), pt2.getTuple(),(255,0,0),2)
                    # print("m = " + str(lin.m))
                    if lin.m <= -0.2:
                        self.lista_goodLeft.pop(0)
                        self.lista_goodLeft.append(lin)
                    elif lin.m >= 0.2:
                        self.lista_goodRight.pop(0)
                        self.lista_goodRight.append(lin)



                # # Se alguma das listas nao mudou, incrementa o buffer daquele lado, se mudou, reinicia o buffer.
                # if tempLenLeft == self.lista_goodLeft:
                #     self.leftBuffer += 1
                # else:
                #     self.leftBuffer = 0

                # if tempLenRight == self.lista_goodRight:
                #     self.rightBuffer += 1
                # else:
                #     self.rightBuffer = 0
                
                # # Se nao mudam a muito tempo, reinicia as listas
                # if self.leftBuffer >= 10:
                #     self.leftBuffer = 0
                #     self.lista_goodLeft = [0]*self.buffering
                # if self.rightBuffer >= 10:
                #     self.rightBuffer = 0
                #     self.lista_goodRight = [0]*self.buffering



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
        colorDict = {
            "blue":[np.array([90,80,80]), np.array([130,255,255])],
            "green":[np.array([45,200,150]), np.array([90,255,255])],
            "magenta":[np.array([140,150,150]), np.array([160,255,255])]
        }
        color = self.target[0].lower()
        try:
            colorRange = colorDict[color]
        except KeyError as e:
            print("Cor fornecida '" + str(e) + "' nao e valida.")
            return 

        point, area = self._identifica_cor(colorRange[0], colorRange[1])
        cv2.circle(self.modifiedFrame, (point[0], point[1]), 3, (0,0,0), 2)
        point = Point(point[0], point[1])
        
        if area > 200:
            return point
        return    

    def identifyId(self):
        if len(self.markers) != 0:
            for marker in self.markers:
                foundID = marker.id
                marker = self.markers[0].pose.pose.position
                point = np.array([marker.x, marker.y, marker.z])
                z = point[0]
                x = -point[1]
                y = -point[2]
                px = x*self.K[0][0]/z + self.K[0][2]
                py = y*self.K[1][1]/z + self.K[1][2]
                pt = np.array([px, py], dtype=int)
                cv2.circle(self.modifiedFrame, (pt[0], pt[1]), 3, (0,0,255), 3)
                cv2.putText(self.modifiedFrame, ("ID: %i"%foundID), (pt[0] + 5, pt[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 2)
                if foundID == self.target[1]: 
                    return Point(px,py)

    def searchRotate(self):
        return [Vector3(0,0,0), Vector3(0,0,0.1)]
        
    def fastAdvance(self):
        velArr = [Vector3(0.15,0,0), Vector3(0,0,0)]
        return velArr

    def slowAdvance(self):
        velArr = [Vector3(0.04,0,0), Vector3(0,0,0)]
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
    
    def identifyDeposit(self):
        targetClass = self.target[2]
        # print(self.mobileNetResults)
        targetTuple = None

        for i,e in enumerate(self.mobileNetResults):
            cv2.rectangle(self.modifiedFrame, e[2], e[3], (0,0,0),3)
            if e[0] == targetClass:
                targetTuple = e
                c, p, pt1, pt2 = targetTuple
                pt1 = Point(pt1[0], pt1[1])
                pt2 = Point(pt2[0], pt2[1])
                outPt = Point((pt1.x + pt2.x)/2, (pt1.y + pt2.y)/2)
                return outPt
        
        return

    def treatForLines(self):
        # print("Chamou treatForLines")
        if self.checkFrame():
            # print("Entrou checkFrame")
            # Shape detection using color (cv2.inRange masks are applied over orginal image)
            temp = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(cv2.GaussianBlur(temp.copy(),(3,3),0),np.array([0,0,200]),np.array([180,10,255]))
            # print(str(mask.shape) + " : " + str(self.lookDownMask.shape))
            #mask = cv2.bitwise_and(mask, self.lookDownMask)
            morphMask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3, 3)))
            contornos, arvore = cv2.findContours(morphMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            frame_out = cv2.drawContours(morphMask, contornos, -1, [0, 0, 255], 3)
            return frame_out

    def setFrame(self, frame):
        self.frame = frame
        self.modifiedFrame = frame.copy()
        return
    
    def showFrame(self):
        if self.checkFrame():
            cv2.imshow('teste' , self.modifiedFrame)
            cv2.waitKey(1)
        return

    def showMask(self):
        if self.checkFrame():
            cv2.imshow('mask' , self.treatForLines())
            cv2.waitKey(1)
        return

    def mobileNet(self):

        if self.checkFrame():
            # print("delay ", "{:.3f}".format(delay/1.0E9))
            try:
                _a_, _b_, resultados =  visao_module.processa(self.frame, showFrame=False)        
                self.mobileNetResults = resultados
                print(resultados)
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
    
    def pointToReturn(self):
        angulocorreto = np.arctan(((self.y-self.y_0)/(self.x-self.x_0)))*180/np.pi


        #Magnifico
        if abs(angulocorreto-180)>180:
            angulocorreto += 180
        else:
            angulocorreto -= 180



        kappa = [Vector3(0,0,0), Vector3(0,0,0)]

        print("alvo: {0}; atual: {1}".format(angulocorreto, self.angulo))

        # if dx > 0 (esquerda), gire para um lado, para direita, gire para outro
        if angulocorreto-self.angulo >= 2:
            kappa = [Vector3(0,0,0), Vector3(0,0,0.1)]
            return kappa

        elif angulocorreto-self.angulo <= -2:
            kappa = [Vector3(0,0,0), Vector3(0,0,-0.1)]
            return kappa
             
        return kappa

    def goReturningPoint(self):
        kappa = [Vector3(0,0,0), Vector3(0,0,0)]
        print("Estou em: ({0},{1})".format(self.x, self.y))
        print("Indo para: ({0},{1})".format(self.x_0, self.y_0))
        if abs(self.x-self.x_0)>0.3 or abs(self.y-self.y_0)>0.3:
            kappa = [Vector3(0.1,0,0), Vector3(0,0,0)]
            return kappa
        return kappa

    def setReturningPoint(self):
        self.x_0 = self.x
        self.y_0 = self.y
        self.angulo_0 = self.angulo

    def CurrPos(self, x_instant, y_instant, ang_instant):
        self.x = x_instant
        self.y = y_instant
        self.angulo = ang_instant

    def detectProximity(self):
        arc = range(-15,15)
        if self.lydar is not None:
            close = [x <= 0.40 for x in self.lydar]
            # print([close[i] for i in arc])
            return np.any([close[i] for i in arc])
        return

    def setDistance(self, target):
        if self.lydar is not None:
            velArr = [Vector3(0,0,0),Vector3(0,0,0)]
            # First align to smallest value in 30 degree arc; 
            arc = range(-15,15)
            arr = [self.lydar[i] for i in arc]
            targetIndex = self._getIndexOfFirstAppearence(arr, min(arr))
            if targetIndex > 18:
                velArr = [Vector3(0,0,0), Vector3(0,0,0.1)]
            elif targetIndex < 14: 
                velArr = [Vector3(0,0,0), Vector3(0,0,-0.1)]
            else:
                # Then adjust distance
                if self.lydar[0] >= target + 0.03:
                    velArr = [Vector3(0.01,0,0),Vector3(0,0,0)]
                elif self.lydar[0] <= target - 0.03:
                    velArr = [Vector3(-0.01,0,0),Vector3(0,0,0)]                 
            
            return velArr
        return

    def stop(self):
        return [Vector3(0,0,0), Vector3(0,0,0)]

    def resetAngle(self):
        if abs(self.angulo_0-self.angulo) >= 10:
            kappa = [Vector3(0,0,0), Vector3(0,0,0.4)]
            return kappa
        else:
            return self.stop()
    
    def _getIndexOfFirstAppearence(self, arr, num):
        for i,e in enumerate(arr):
            if e == num:
                return i
        return

    def drawStates(self, stateDict):
        if self.checkFrame():
            counter = 1
            stepSize = 24
            for k,v in stateDict.items():
                if v:
                    COLOR = (0,0,0)
                    if k == "IDConfirmado":
                        COLOR = (0,0,255)
                    cv2.putText(self.modifiedFrame, k, (0, stepSize*counter), cv2.FONT_HERSHEY_SIMPLEX, 1, COLOR, 2)
                    counter += 1