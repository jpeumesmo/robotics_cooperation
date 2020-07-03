#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy
from nav_msgs.msg import Odometry # configuração do robo
from geometry_msgs.msg import Twist, Pose2D # para enviar velocidade e definir ponto de objetivo
from sensor_msgs.msg import LaserScan # dados do laser do robo
import tf
import math #biblioteca matematica
import threading

robots_distance = -1
odomMsg = None
laserMsg = None
pos_r_laser = Pose2D()
goal = Pose2D()
allow_error = 0
error = 1
state = 0 #variavel estado
rotate_goal= None
mutex = threading.Lock()

#funcao para calcular a distancia euclidiana
def distance(a,b):
	return math.sqrt(pow((a.x - b.x),2)+pow((a.y - b.y),2))

#funcao de rotate da maquina de estados
def rotate():
	global pos_r_laser
	global odomMsg
	global robots_distance
	global goal
	global state
	global rotate_goal

	flag = -1

	if (pos_r_laser is None or odomMsg is None or robots_distance is -1):
		return 0,0
	else:
		if (goal.x < odomMsg.pose.pose.position.x and goal.y < odomMsg.pose.pose.position.y):
			state = 1
			flag = 0
		elif (goal.x > odomMsg.pose.pose.position.x and goal.y > odomMsg.pose.pose.position.y):
			state = 1
			flag = 1
		else:
			state = 0
			return 0,0

		if (rotate_goal is None and flag == 0):
			rotate_goal = Pose2D()
			rotate_goal.x = odomMsg.pose.pose.position.x + 2*abs(odomMsg.pose.pose.position.x - pos_r_laser.x)
			rotate_goal.y = odomMsg.pose.pose.position.y + 2*abs(odomMsg.pose.pose.position.y - pos_r_laser.y)
			return 0,0
		if (rotate_goal is None and flag == 1):
			rotate_goal = Pose2D()
			rotate_goal.x = odomMsg.pose.pose.position.x - 2*abs(odomMsg.pose.pose.position.x - pos_r_laser.x)-0.6
			rotate_goal.y = odomMsg.pose.pose.position.y - 2*abs(odomMsg.pose.pose.position.y - pos_r_laser.y)-0.6
			return 0,0
		else:
			if flag == 0:
				if (odomMsg.pose.pose.position.x < rotate_goal.x):
					return 0.2,0
				elif (odomMsg.pose.pose.position.y < rotate_goal.y):
					return 0,0.2
				else:
					state = 0
					return 0,0
			if flag == 1:
				print pos_r_laser.x,rotate_goal.x,odomMsg.pose.pose.position.x-pos_r_laser.x#pos_r_laser.y,rotate_goal.y
				if (odomMsg.pose.pose.position.x > rotate_goal.x):
					return -0.2,0
				elif (odomMsg.pose.pose.position.y > rotate_goal.y):
					return 0,-0.2
				else:
					state = 0
					return 0,0

#funcao que calcula as velocidade o robo de odometria
def robo_odom_move():
	global odomMsg
	global pos_r_laser
	global robots_distance

	treshold = 0.5
	goal = Pose2D()

	goal.x = -(odomMsg.pose.pose.position.x - pos_r_laser.x)
	goal.y = -(odomMsg.pose.pose.position.y - pos_r_laser.y)
	if (robots_distance < treshold):
		goal.x = -goal.x
		goal.y = -goal.y


	return goal.x,goal.y

#funcao de atracao utilizada para o robo com laser
def atracao():
	global odomMsg
	global pos_r_laser
	fx = -(pos_r_laser.x - goal.x)
	fy = -(pos_r_laser.y - goal.y)
	return fx,fy

#funcao de repulsao utilizada para o robo com laser
def repulsao():
	global laserMsg
	global robots_distance
	fx = 0
	fy = 0
	treshold = 1.5
	for idx,value in enumerate(laserMsg.ranges):
		if (value < treshold) and (value > robots_distance + 0.2):

			d = value-treshold+robots_distance
			fx += -0.1*(1/d - 1/treshold)*(1/d**2)*(-math.cos(math.radians(idx/2)))
			fy += -0.1*(1/d - 1/treshold)*(1/d**2)*(-math.sin(math.radians(idx/2)))

	return fx,fy

#campos potenciais utilizado pelo robo com laser
def potential_fields():
	global robots_distance

	if (robots_distance < 1.0):
		andar = 0
		fax,fay = atracao()
		frx,fry = repulsao()

		frex,frey = fax+frx,fay+fry

		fx = frex/abs(frex)*robots_distance
		fy = frey/abs(frey)*robots_distance
		return fx,fy
	return 0,0

#callback do laser
def LaserCallback(msg):
	global laserMsg
	laserMsg = msg

#callback da odometria
def OdomCallback(msg):
	global odomMsg
	global robots_distance
	global pos_r_laser
	odomMsg = msg
	global mutex
	#recalcula a distancia operacao bloqueante
	mutex.acquire()
	try:
		robots_distance = distance(pos_r_laser,msg.pose.pose.position)
	finally:
		mutex.release()

#funcao para estimar a posicao do rbo com laser com base nos dados da odometria do outro robo
def posicao_robo_laser():
	global odomMsg
	global laserMsg
	global pos_r_laser
	global robots_distance

 	laserD = laserMsg.ranges
	theta = math.radians(laserD.index(min(laserD))/2)
	pos_r_laser.x =	odomMsg.pose.pose.position.x + math.cos(theta)*min(laserD)+0.19#0.19 raio dos robos
	pos_r_laser.y = odomMsg.pose.pose.position.y + math.sin(theta)*min(laserD)+0.19#0.19 raio dos robos
	pos_r_laser.theta = 0
	robots_distance = distance(pos_r_laser,odomMsg.pose.pose.position)

#funcao executada pela thread que representa o robo com odometria
def robot_odom():
	pub = rospy.Publisher('robot_0/cmd_vel',Twist,queue_size =10)
	rospy.Subscriber('robot_0/odom',Odometry,OdomCallback)
	rate = rospy.Rate (1)
	cmd_vel = Twist()
	global odomMsg
	global robots_distance
	global error
	global allow_error
	while (not rospy.is_shutdown() and error > allow_error):
		if (odomMsg == None) :
			rate.sleep ()
			continue
		if (state == 1):
			vx,vy = rotate()
		else:
			vx,vy = robo_odom_move()
		cmd_vel.linear.x = vx
		cmd_vel.linear.y = vy
		pub.publish(cmd_vel)
	cmd_vel.linear.x = 0
	cmd_vel.linear.y = 0
	pub.publish(cmd_vel)

#funcao executada pela thread que representa o robo com laser
def robot_laser():
	pub = rospy.Publisher('robot_1/cmd_vel',Twist,queue_size =10)
	rospy.Subscriber('robot_1/base_scan',LaserScan,LaserCallback)
	rate = rospy.Rate (1)
	cmd_vel = Twist()
	global odomMsg
	global laserMsg
	global robots_distance
	global allow_error
	global pos_r_laser
	global goal
	global error
	global state
	error = 1
	while (not rospy.is_shutdown() and error > allow_error):
		if (laserMsg == None) or (odomMsg == None) :
			rate.sleep ()
			continue
		mutex.acquire()
		try:
			posicao_robo_laser()#estimar posicao do robo com o laser
		finally:
			mutex.release()
		error = distance(pos_r_laser,goal)
		if (state == 1):
			vx,vy = 0,0
		else:
			vx,vy = potential_fields()#andar com o robo
		cmd_vel.linear.x = vx
		cmd_vel.linear.y = vy
		pub.publish(cmd_vel)
	cmd_vel.linear.x = 0
	cmd_vel.linear.y = 0
	pub.publish(cmd_vel)
	print pos_r_laser.x,',',pos_r_laser.y,',','0 &',abs(goal.x-pos_r_laser.x),',',abs(goal.y - pos_r_laser.y),',','0'

#funcao principal que cria o no e chama as threads para cada robo
def run():
	rospy.init_node('projeto',anonymous = True )
	robo1 = threading.Thread(target=robot_odom)
	robo0 = threading.Thread(target=robot_laser)
	global goal
	global allow_error
	pose  = rospy.get_param("~pose")
	pose = pose.split(',')
	goal.x = float(pose[0])
	goal.y = float(pose[1])
	goal.theta = float(pose[2])
	allow_error = float(rospy.get_param("~error"))
	robo0.start()
	robo1.start()

if __name__ == '__main__':
	try:
		run()
	except rospy.ROSInterruptException:
		pass
