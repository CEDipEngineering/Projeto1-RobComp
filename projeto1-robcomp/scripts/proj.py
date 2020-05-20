#! /usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import tf
import math
import cv2
import time
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from tf import transformations
# from tf import TransformerROS
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from numpy import linalg
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped

from AI import AI
import visao_module
from sensor_msgs.msg import LaserScan
from garra_demo import *


bridge = CvBridge()
ai = AI()

cv_image = None
media = []
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos
from nav_msgs.msg import Odometry

area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

# Criacao de uma variavel global para guardar os resultados vistos

x = 0
y = 0
z = 0 
xOdom = 0
yOdom = 0
contador = 0
angrobot = 0
pula = 2
angleStatus = 0 #resetar dps que voltar ao ponto inicial

frame = "camera_link"
# frame = "head_camera"  # DESCOMENTE para usar com webcam USB via roslaunch tag_tracking usbcam

tfl = 0

tf_buffer = tf2_ros.Buffer()

def recebeAlvar(msg):
	ai.markers = []
	for marker in msg.markers:
		ai.markers.append(marker)


def recebe_odometria(data):
	xOdom = data.pose.pose.position.x
	yOdom = data.pose.pose.position.y
	quat = data.pose.pose.orientation
	lista = [quat.x, quat.y, quat.z, quat.w]
	angulos = np.degrees(transformations.euler_from_quaternion(lista))    
	angrobot = angulos[2]
	ai.CurrPos(xOdom, yOdom, angrobot)
		


def roda_todo_frame(imagem):
	frame  = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
	ai.setFrame(frame)	

def pegaDadosLydar(data):
	ai.lydar = data.ranges

    
if __name__=="__main__":

	rospy.init_node("cor")
	topico_imagem = "/camera/rgb/image_raw/compressed"
	topico_imagem2 = "/raspicam/rgb/image_raw/compressed"	
	recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
	recebe_scan = rospy.Subscriber("/scan", LaserScan, pegaDadosLydar)
	recebedor = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, recebeAlvar) # Para recebermos notificacoes de que marcadores foram vistos
	# print("Usando ", topico_imagem)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
	ref_odometria = rospy.Subscriber("/odom", Odometry, recebe_odometria)

	t0 = rospy.get_rostime()
	girando = 0
	chegou = 0
	tolerancia = 25

	# Exemplo de categoria de resultados
	# [('chair', 86.965459585189819, (90, 141), (177, 265))]
	# [(categoria, probabilidade (certeza), canto sup.esq do retângulo, canto inf.dir do retângulo)]

	try:

		goal1 = ["blue", 11, "cat"]
		goal2 = ["green", 21, "dog"]
		goal3 = ["magenta", 12, "bicycle"]

		ai.target = goal1



		stateMachine = {
			"Vagando": 1,
			"AlinhandoCor": 1,
			"AvancandoCor": 0,
			"Parado": 0,
			"Pegando": 0,
			"Voltando": 0,
			"IDConfirmado": 0,
			"AlinhandoDeposito": 0,
			"AvancandoEstrada": 0,
			"CorrigindoDistancia": 0,
			"AvancandoDeposito":0,
			"ReturnPointSet": 0,
			"RetomandoAngulo": 0
		}

		

		tutorial = MoveGroupPythonIntefaceTutorial()
		tutorial.go_to_initial_position()
		tutorial.open_gripper()

		while not rospy.is_shutdown():
			velArr = [Vector3(0,0,0),Vector3(0,0,0)]

			# print(ai.detectProximity())
			# if ai.detectProximity():
			# 	ai.setDistance(0.25)
			# 	print(ai.lydar[0])
			# else:
			# 	velArr = ai.fastAdvance()
			
			if ai.checkFrame():
				# print("Markers encontrados: ", ai.markers)
				if 1 not in stateMachine.values():
					print("Fiquei entediado")
					stateMachine["Vagando"] = 1


				if stateMachine["Parado"]:
					velArr = [Vector3(0,0,0),Vector3(0,0,0)]
				else:
					# print("nao estou parado")
					if stateMachine["Vagando"]:
						# print("estou vagando")
						streetPoint = ai.followRoad()
						if streetPoint is not None:
							velArr = ai.alignToTarget(streetPoint)
							stateMachine["AvancandoEstrada"] = 1
							if stateMachine["AvancandoEstrada"]:
								velArr = ai.slowAdvance()
								ai.counters["FramesSemAlinharEstrada"] += 1
								# print(ai.counters)
								if ai.counters["FramesSemAlinharEstrada"] >= 5:
									ai.counters["FramesSemAlinharEstrada"] = 0
									stateMachine["AvancandoEstrada"] = 0
									velArr = ai.alignToTarget(streetPoint)
						# else:
						# 	velArr = ai.searchRotate()
						if stateMachine["AlinhandoCor"]:
							colorPoint = ai.identifyColor()
							IDPoint = ai.identifyId()
							if colorPoint is not None and IDPoint is not None:
								if abs(colorPoint.x - IDPoint.x) <= 5:
									stateMachine["Vagando"] = 0
									stateMachine["IDConfirmado"] = 1
						
						if stateMachine["AlinhandoDeposito"]:
							ai.mobileNet()
							depositPoint = ai.identifyDeposit()
							if depositPoint is not None:
								stateMachine["Vagando"] = 0
							
					
					else:
						if stateMachine["AlinhandoCor"]:
							# print("estou alinhando cor")
							if not stateMachine["ReturnPointSet"]:
								ai.setReturningPoint()
								stateMachine["ReturnPointSet"] = 1
							colorPoint = ai.identifyColor()
							if colorPoint is not None:
								# print("Achei a cor")
								velArr = ai.alignToTarget(colorPoint)
								if velArr[1] == Vector3(0,0,0):
									stateMachine["AlinhandoCor"] = 0
									stateMachine["AvancandoCor"] = 1
						if stateMachine["AvancandoCor"]:
							if ai.detectProximity():
								stateMachine["AvancandoCor"] = 0
								stateMachine["CorrigindoDistancia"] = 1
							else: 
								velArr = ai.fastAdvance()
								ai.counters["FramesSemAlinhar"] += 1
								# print(ai.counters)
								if ai.counters["FramesSemAlinhar"] >= 15:
									ai.counters["FramesSemAlinhar"] = 0
									stateMachine["AvancandoCor"] = 0
									stateMachine["AlinhandoCor"] = 1
						

						if stateMachine["AvancandoDeposito"]:
							if ai.detectProximity():
								stateMachine["AvancandoDeposito"] = 0
								tutorial.open_gripper()
							else: 
								velArr = ai.fastAdvance()
									
						
						if stateMachine["CorrigindoDistancia"]:
							velArr = ai.setDistance(0.16)
							if velArr == [Vector3(0,0,0),Vector3(0,0,0)]:
								stateMachine["CorrigindoDistancia"] = 0
								stateMachine["Pegando"] = 1

						if stateMachine["Pegando"]:
							# Garra
							tutorial.close_gripper()
							# rospy.sleep()
							tutorial.go_to_final_position()



							# When done:
							stateMachine["Pegando"] = 0
							stateMachine["Voltando"] = 1
							print("")
						
						if stateMachine["Voltando"]:
							# Voltar por Odom
							velArr = ai.pointToReturn()
							if velArr[1] == Vector3(0,0,0):
								velArr = ai.goReturningPoint()
								if velArr == ai.stop():
									stateMachine["Voltando"] = 0
									stateMachine["RetomandoAngulo"] = 1
						
						if stateMachine["RetomandoAngulo"]:
							velArr = ai.resetAngle()
							if velArr == ai.stop():
								stateMachine["RetomandoAngulo"] = 0
								stateMachine["AlinhandoDeposito"] = 1
								stateMachine["Vagando"] = 1
								# stateMachine["ReturnPointSet"] = 0



						if stateMachine["AlinhandoDeposito"]:
							# print("estou alinhando deposit")

							# Nao vamos voltar
							# if not stateMachine["ReturnPointSet"]:
							# 	ai.setReturningPoint()
							# 	stateMachine["ReturnPointSet"] = 1
							ai.mobileNet()
							depositPoint = ai.identifyDeposit()
							if depositPoint is not None:
								velArr = ai.alignToTarget(depositPoint)
								if velArr[1] == Vector3(0,0,0):
									stateMachine["AlinhandoDeposito"] = 0
									stateMachine["AvancandoDeposito"] = 1
		
			
			vel = Twist(velArr[0], velArr[1])
			velocidade_saida.publish(vel)
			# print(velArr)
			# [print("{0}: {1}".format(k,v)) for k,v in stateMachine.items()]
			ai.drawStates(stateMachine)
			ai.showFrame()
			# ai.showMask()
			rospy.sleep(0.5)


	except rospy.ROSInterruptException:
		print("Ocorreu uma exceção com o rospy")


