#!/usr/bin/env python
import os
import rospy
import time
import sensor_msgs.msg
import numpy as np
from math import sin, cos, atan, pi, exp
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

obs_dist = [0 for i in range(360)]
obs_point = np.zeros((360,2))

#my name is daniel
#function of caculate att_Force
def Fatt(p,v,nRT,nVRT):
	#(p,v)=pure number (nRT,nVrt)=vector
	m=3; n=2
	aP=5; aV=1 
	Fatt1 = (m*aP*(abs(p)**(m-1)))*nRT
	Fatt2 = (n*aV*(abs(v)**(n-1)))*nVRT
	Fatt_tal = Fatt1+Fatt2
	return(Fatt_tal)


#function of caculate rep_Force
def Frep(Ss,Sm,So,nRo,nROp,vRO,vROp,amax):
	n = 0.5#positive constant
	if Ss-Sm>=So or vRO<=0:
		Frep_tal = np.array([0,0])
		return(Frep_tal)
	if Ss-Sm>0 and Ss-Sm<So and vRO>0:
		Frep1 = (-n/((Ss-Sm)**2))*(1+vRO/amax)*nRo
		Frep2 = ((n*vRO*vROp)/Ss*amax*((Ss-Sm)**2))*nROp
		return(Frep1+Frep2)
	if vRO>0 and Sm>Ss:
		Frep_tal = np.array([-100,-100])
		return()


#change xy_plane to pole
def xy_ploe(Farray):
	x = Farray[0]
	y = Farray[1]
	rad = (x**2+y**2)**0.5
	if y>0:
		if x>0:
			angle=atan(y/x)*(180/pi)
		elif x<0:
			angle=180+atan(y/x)*(180/pi)
		else:
			angle=90
	elif y<0:
		if x>0:
			angle=360+atan(y/x)*(180/pi)
		elif x<0:
			angle=180+atan(y/x)*(180/pi)
		else:
			angle=270   
	else:#y=0
		if x>0:
			angle=0
		elif x<0:
			angle=180
		else:
			angle=90
	return(np.array([angle,rad])) 

#deal with lidar callback data
def callback(data):
	global obs_point#xy_plane lidar information
	global obs_dist#pole lidar information

	#change to x_y plane
	for i in range(0,360):
		#區分實際上的i與degree其實存在90度的差異
		degree = i
		if degree<270:
			degree=degree+90
		else:
			degree=degree-270

		obs_dist[i] = data.ranges[i]
		if obs_dist[i]>=3.5:
			obs_dist[i] = 3.5

		x = obs_dist[i]*cos(degree/180*pi)
		if abs(x)<0.00005:
			x = 0
		y = obs_dist[i]*sin(degree/180*pi)
		if abs(y)<0.00005:
			y = 0
		obs_point[i][0] =x
		obs_point[i][1] =y


#Main Function
if __name__=="__main__":
	rospy.init_node('turtlebots_auto', anonymous=True)
	rospy.Subscriber("scan",LaserScan,callback)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

	#initial velocity
	linear_x = 0
	anguar_z = 0

	try:
		while(not rospy.is_shutdown()):
			#main_function_begin***************************************************************************************

			#find the longest distance direction
			tor_derec_angle = 0
			tor_derec_value = 0
			tor_derec_range = 0	
			for i in range(1,180):
				d = 270+i
				if d>=360:
					d = d-360
				#choose the large
				if obs_dist[d]>=tor_derec_value:
					tor_derec_angle = d
					if obs_dist[d] == tor_derec_value:
						tor_derec_range = tor_derec_range+1
					else:
						tor_derec_range = 0
					tor_derec_value = obs_dist[d]
			tor_derec_angle = tor_derec_angle-int(tor_derec_range*0.5)
			if tor_derec_angle<0:
				tor_derec_angle = tor_derec_angle+360
			tor_derec_value = obs_dist[tor_derec_angle]	
			#print(tor_derec_angle , tor_derec_value , tor_derec_range*0.5)
			tor_derec_angle = tor_derec_angle+90
			x_dist = tor_derec_value*cos(tor_derec_angle/180*pi)
			y_dist = tor_derec_value*sin(tor_derec_angle/180*pi)
			xy_tor_dist = np.array([x_dist,y_dist,0])#turn to xy_plane
			#print(xy_tor_dist)

			#以下的算法都是假設機器人本身靜止※暗示環境在動

			#find tor_Fatt
			#unit distance vector
			nRT = np.array([cos(tor_derec_angle/180*pi),sin(tor_derec_angle/180*pi)])
			Vtar = np.array([0,-linear_x,0])+np.cross(np.array([0,0,-anguar_z]),xy_tor_dist)
			nVtar = Vtar/(Vtar[0]**2+Vtar[1]**2)**0.5#unit velocity vector
			nVRT = np.array([Vtar[0],Vtar[1]])#form xyz to xy
			pure_Vtar = (Vtar[0]**2+Vtar[1]**2)**0.5
			tor_Fatt = Fatt(tor_derec_value,pure_Vtar,nRT,nVRT)
			#print(tor_Fatt)		
			

			#findobs_Frep
			So = 0.7
			amax  = 500#max acceleration
			obs_Frep = np.array([0,0])
			for j in range(0,360):
				nRO = obs_point[j]/obs_dist[j]#unit vector from robot to obstacle
				angular_vel=np.array([0,0,-anguar_z])
				radius = np.array([obs_point[j][0],obs_point[j][1],0])
				xyzVobs = np.array([0,-linear_x,0])+np.cross(angular_vel,radius)
				Vobs = np.array([xyzVobs[0],xyzVobs[1]])
				vRO = np.dot(-Vobs,nRO.T)#right now velocity
				vROp = (vRO**2-vRO**2)**0.5
				Ss = obs_dist[j]
				Sm = vRO**2/amax #safe distance
				v = (xyzVobs[0]**2+xyzVobs[1]**2)**0.5
				nROp = -(Vobs+vRO*nRO)
				other_Frep = Frep(Ss,Sm,So,nRO,nROp,vRO,vROp,amax)
				if other_Frep is None:
					other_Frep=np.array([0,0])
				obs_Frep = obs_Frep + other_Frep
			#print(obs_Frep)


			#Deal with the force to contro
			Ftotal = 2.5*tor_Fatt+obs_Frep
			Pole_Ftotal = xy_ploe(Ftotal)
			Angle = Pole_Ftotal[0]-90
			if Angle<0:Angle+=360
			Force = Pole_Ftotal[1]#Force max=183
			if Force<0:Force = 0
			print('Force=',Force,'Angle=',Angle)

			linear_x = 0.25-(1/Force)**0.5
			print(linear_x)
			if linear_x>=0.2:linear_x=0.2
			if linear_x<=0.05:linear_x=0.05
			if Angle<=90:
				anguar_z=Angle/120
			elif Angle<=180:
				anguar_z=1
			elif Angle<=270:
				anguar_z=-1
			else:
				anguar_z=-(360-Angle)/120

			print('Ftotal=',Ftotal,'angular=',anguar_z,', velocity=',linear_x)

			
			#main_function_finish************************************************************************************
			twist = Twist()
			twist.linear.x = linear_x; twist.linear.y = 0; twist.linear.z = 0
			twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = anguar_z
			pub.publish(twist)
			time.sleep(0.005)

	#except:
	#	print("e")

	finally:
		os.system("rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'") 
		os.system("rosservice call /gazebo/reset_simulation")
		print("the turtlebot shoutdown")