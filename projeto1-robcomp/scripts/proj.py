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
# from tf import transformations
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


def roda_todo_frame(imagem):
	frame  = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
	ai.setFrame(frame)	



    
if __name__=="__main__":

	rospy.init_node("cor")
	topico_imagem = "/camera/rgb/image_raw/compressed"
	topico_imagem2 = "/raspicam/rgb/image_raw/compressed"
	recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
	# recebedor = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, recebe) # Para recebermos notificacoes de que marcadores foram vistos
	# print("Usando ", topico_imagem)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)


	tolerancia = 25

	# Exemplo de categoria de resultados
	# [('chair', 86.965459585189819, (90, 141), (177, 265))]
	# [(categoria, probabilidade (certeza), canto sup.esq do retângulo, canto inf.dir do retângulo)]

	try:
		while not rospy.is_shutdown():
			velArr = [Vector3(0,0,0),Vector3(0,0,0)]
			if ai.checkFrame():
				# print(ai.frame.shape)
				streetPoint = ai.followRoad()
				if streetPoint is not None:
					velArr = ai.alignToTarget(streetPoint)
					if velArr[1] == Vector3(0,0,0):
						velArr = ai.fastAdvance()
				else:
					velArr = [Vector3(0,0,0),Vector3(0,0,0.1)]





				# ai.mobileNet()
				# resultados = ai.mobileNetResults
				# for r in resultados:
				# 	print(r)

				# ai.identifyColor('magenta')
				ai.showFrame()
				
				ai.showMask()

			vel = Twist(velArr[0],velArr[1])
			velocidade_saida.publish(vel)
			rospy.sleep(0.1)

	except rospy.ROSInterruptException:
		print("Ocorreu uma exceção com o rospy")


