#! /usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import tf
import math
import cv2
import time
from sensor_msgs.msg import Image, CompressedImage
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
id = 0
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

def recebe(msg):
	global x # O global impede a recriacao de uma variavel local, para podermos usar o x global ja'  declarado
	global y
	global z
	global id
	for marker in msg.markers:
		id = marker.id
		marcador = "ar_marker_" + str(id)

		# print(tf_buffer.can_transform(frame, marcador, rospy.Time(0)))
		header = Header(frame_id=marcador)
		# Procura a transformacao em sistema de coordenadas entre a base do robo e o marcador numero 100
		# Note que para seu projeto 1 voce nao vai precisar de nada que tem abaixo, a 
		# Nao ser que queira levar angulos em conta
		trans = tf_buffer.lookup_transform(frame, marcador, rospy.Time(0))
		
		# Separa as translacoes das rotacoes
		x = trans.transform.translation.x
		y = trans.transform.translation.y
		z = trans.transform.translation.z
		# ATENCAO: tudo o que vem a seguir e'  so para calcular um angulo
		# Para medirmos o angulo entre marcador e robo vamos projetar o eixo Z do marcador (perpendicular) 
		# no eixo X do robo (que e'  a direcao para a frente)
		t = tf.transformations.translation_matrix([x, y, z])
		# Encontra as rotacoes e cria uma matriz de rotacao a partir dos quaternions
		r = tf.transformations.quaternion_matrix([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
		m = np.dot(r,t) # Criamos a matriz composta por translacoes e rotacoes
		z_marker = [0,0,1,0] # Sao 4 coordenadas porque e'  um vetor em coordenadas homogeneas
		v2 = np.dot(m, z_marker)
		v2_n = v2[0:-1] # Descartamos a ultima posicao
		n2 = v2_n/linalg.norm(v2_n) # Normalizamos o vetor
		x_robo = [1,0,0]
		cosa = np.dot(n2, x_robo) # Projecao do vetor normal ao marcador no x do robo
		angulo_marcador_robo = math.degrees(math.acos(cosa))
		# Terminamos
		# print("id: {} x {} y {} z {} angulo {} ".format(id, x,y,z, angulo_marcador_robo))

def recebe_odometria(data):
	global xOdom
	global yOdom
	global contador
	global angrobot

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
	# recebedor = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, recebe) # Para recebermos notificacoes de que marcadores foram vistos
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

		ai.target = ['blue', 0 , 'dog']
		#goal1 = ["blue", 11, "cat"]
		#goal2 = ["green", 21, "dog"]
		#goal3 = ["magenta", 12, "bike"]



		stateMachine = {
			"Vagando": 1,
			"AlinhandoCor": 1,
			"Avancando": 0,
			"Parado": 0,
			"Pegando": 0,
			"Voltando": 0,
			"ProcurandoDeposito": 0,
			"CorrigindoDistancia": 0,
			"ReturnPointSet": 0
		}
		tutorial = MoveGroupPythonIntefaceTutorial()
		tutorial.open_gripper()
		tutorial.go_to_initial_position()
		



		while not rospy.is_shutdown():
			
			velArr = [Vector3(0,0,0),Vector3(0,0,0)]

			# print(ai.detectProximity())
			# if ai.detectProximity():
			# 	ai.setDistance(0.25)
			# 	print(ai.lydar[0])
			# else:
			# 	velArr = ai.fastAdvance()


			
			if ai.checkFrame():

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

						if stateMachine["AlinhandoCor"]:
							colorPoint = ai.identifyColor(ai.target[0])
							if colorPoint is not None:
								stateMachine["Vagando"] = 0
						
						if stateMachine["ProcurandoDeposito"]:
							# Teste mobileNet
							# ai.mobileNet()
							# resultados = ai.mobileNetResults
							# for r in resultados:
							# 	print(r)
							print("")
					
					else:
						if stateMachine["AlinhandoCor"]:
							# print("estou alinhando cor")
							if not stateMachine["ReturnPointSet"]:
								ai.setReturningPoint()
								stateMachine["ReturnPointSet"] = 1
							colorPoint = ai.identifyColor(ai.target[0])
							if colorPoint is not None:
								# print("Achei a cor")
								velArr = ai.alignToTarget(colorPoint)
								if velArr[1] == Vector3(0,0,0):
									stateMachine["AlinhandoCor"] = 0
									stateMachine["Avancando"] = 1
						if stateMachine["Avancando"]:
							if ai.detectProximity():
								stateMachine["Avancando"] = 0
								stateMachine["CorrigindoDistancia"] = 1
							else: 
								velArr = ai.fastAdvance()
								ai.counters["FramesSemAlinhar"] += 1
								# print(ai.counters)
								if ai.counters["FramesSemAlinhar"] >= 10:
									ai.counters["FramesSemAlinhar"] = 0
									stateMachine["Avancando"] = 0
									stateMachine["AlinhandoCor"] = 1
						
						if stateMachine["CorrigindoDistancia"]:
							velArr = ai.setDistance(0.20)
							if velArr == [Vector3(0,0,0),Vector3(0,0,0)]:
								stateMachine["CorrigindoDistancia"] = 0
								stateMachine["Pegando"] = 1

						if stateMachine["Pegando"]:
							# Garra
							tutorial.close_gripper()




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
									velArr = ai.resetAngle()
									if velArr == ai.stop():
										stateMachine["Voltando"] = 0
										stateMachine["ProcurandoDeposito"] = 1
									
							# print(ai.frame.shape)
						
							
					# 			velArr = ai.fastAdvance()
					# 	else:
					# 		velArr = [Vector3(0,0,0),Vector3(0,0,0.07)]
					# else:
					# 	velArr = ai.alignToTarget(colorPoint)
					# 	if velArr[1] == Vector3(0,0,0):
					# 		if not ai.detectProximity():
					# 			velArr = ai.fastAdvance()
					# 		elif velArr != [Vector3(0,0,0),Vector3(0,0,0)]:
					# 			velArr = ai.setDistance(0.25)
					# 		else:
					# 			print("acabei")


			


			 	# ai.mobileNet()
			 	# resultados = ai.mobileNetResults
			 	# for r in resultados:
			 	# 	print(r)



			# print("t0", t0)
			# if t0.nsecs == 0:
			# 	t0 = rospy.get_rostime()
			# 	print("waiting for timer")
			# 	continue        
			# t1 = rospy.get_rostime()
			# elapsed = (t1 - t0)
			# print("Passaram ", elapsed.secs, " segundos")
			# if elapsed.secs == 1:
			# 	raw_input("Manda um enter pra eu salvar a posicao")
			# 	ai.setReturningPoint(xOdom, yOdom)
			# 	# t0 = rospy.get_rostime()
			# if elapsed.secs >= 15:
			# 	if girando == 0:
			# 		velArr, girando = ai.checkAngle(girando)
				# if chegou == 0:
				# 	print("Cheguei! WooHoo")
				# 	velArr, chegou = ai.ateChegar(chegou)

			# print(ai.x, ai.y, "Estou aqui")
			# print(ai.x_0, ai.y_0, "Voltarei para ca")
			
			
			
			
			
			
			vel = Twist(velArr[0], velArr[1])
			velocidade_saida.publish(vel)
			# print(velArr)
			[print("{0}: {1}".format(k,v)) for k,v in stateMachine.items()]
			ai.showFrame()
			rospy.sleep(0.5)


	except rospy.ROSInterruptException:
		print("Ocorreu uma exceção com o rospy")


