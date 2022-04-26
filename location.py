import rospy
import sys
import math
import time
from math import pi
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, Twist

# Robot location
xk = 0.0 # x position at k timestep in meters
yk = 0.0 # y position at k timestep in meters
theta_k = 0.0 # angle at k timestep in radians #### theta r es igual a theta k
'''	Values of theta must be contained within a single circle: EITHER: -pi <= theta < pi OR: 0 <= theta < 2pi'''

# Robot constants
r = 0.05 # Wheel radius, 0.05m
l = 0.19 # Distance between robot wheels 0.19m

# Measured variables
w_l, w_r = 0.0, 0.0 # Wheels velocity (rad/s)
dt = 0.0055 # Time between samples (s) #0.00525 es preciso en fisico pero no en codigo # 0.0055 es preciso en codigo pero no en fisico

final_pos_x = 0.0 #Constante que nos da la posicion final del archivo
final_pos_y = 0.0

e_theta = 0.07
e_d = 0.0

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10, latch=True)
angle_finished = 0
distancia_recorrida = 0.0

def getPos(posArray):
	return odom_callback(posArray, 'final_pos')


def odom_l(wl):
	return odom_callback(wl, 'wl')


def odom_r(wr):
	return odom_callback(wr, 'wr')


def correct_angle(error):
	global cmd_vel_pub
	w_to_cmd_vel = Twist()
	vel_w = 0.08 * error
	#print("vel w", vel_w, "Error: ", error)
	if(-0.05 < error < 0.05):
		w_to_cmd_vel.angular.z = 0.0
		cmd_vel_pub.publish(w_to_cmd_vel)
		#print("stopppp")
		return True
	if(error >= 0.8):
		#print("gooooo")
		w_to_cmd_vel.angular.z = vel_w
		cmd_vel_pub.publish(w_to_cmd_vel)
		return False
		

def reach_goal(distancia_total, distancia_faltante,distancia_recorrida):
	global cmd_vel_pub
	v_to_cmd_vel = Twist()
	vel_v = 0.4
	'''
	if(-0.5 < distancia_faltante < 0.5):
		print("STOPPPPPPP")
		print("distancia faltante", distancia_faltante)
		v_to_cmd_vel.linear.x = 0.0
		cmd_vel_pub.publish(v_to_cmd_vel)
		print(distancia_recorrida)
		#return True

	if(distancia_faltante >= 0.8):
		print("GOOOOOOOO")
		v_to_cmd_vel.linear.x = vel_v
		cmd_vel_pub.publish(v_to_cmd_vel)
		print(distancia_recorrida)
		return False'''
		
	distancia_recorrida =  distancia_total - distancia_faltante
	print("Recorrida",distancia_recorrida, "Total", distancia_total, "Faltante", distancia_faltante)
	error_distancia = distancia_total/2
	if(distancia_recorrida < distancia_total-error_distancia):
		print("GOOOOOOOO")
		v_to_cmd_vel.linear.x = vel_v
		cmd_vel_pub.publish(v_to_cmd_vel)
		print(distancia_recorrida)
		return False
	if(distancia_recorrida >= distancia_total-error_distancia): 
		print("STOPPPPPPP")
		v_to_cmd_vel.linear.x = 0.0
		cmd_vel_pub.publish(v_to_cmd_vel)
		print(distancia_recorrida)
		return True
		

def odom_callback(data, topic):
	global xk, yk, theta_k, degree
	global w_l, w_r
	global final_pos_x, final_pos_y
	global e_theta
	global distancia_recorrida, angle_finished
	
	if(topic == 'final_pos'):
		final_pos_x = data.position.x
		final_pos_y = data.position.y
		e_theta = math.atan2(final_pos_y, final_pos_x) - theta_k
	
	if (topic == "wr"):
		w_r = data.data
		
	if (topic == "wl"):
		w_l= data.data

	theta_k = theta_k + r*((w_r - w_l)/l)*dt
	degree = theta_k*180/math.pi

	xk = xk + (r*((w_r + w_l)/2)*dt*math.cos(theta_k))#/1.9
	yk = yk + (r*((w_r + w_l)/2)*dt*math.sin(theta_k))#/1.9
        
	d_t = math.sqrt((final_pos_x)**2 + (final_pos_y)**2) # Distancia total por recorrer
	e_d = math.sqrt((final_pos_x - xk)**2 + (final_pos_y - yk)**2)
	
	if(theta_k > 2*math.pi):
		theta_k = 0
	if(theta_k < 0):
		theta_k = 2*math.pi
	
	print("Valor de x: ", xk)
	print("Valor de y: ", yk)
	#print("Valor de theta: ", theta_k)
	print("Grados: ", degree)
	#print("Valor de e_theta: ", e_theta)
	# print("Valor de ed: ", e_d)
	
	if(angle_finished == 0):
		#print("entro a ejecutar")
		if(correct_angle(e_theta) == True):
			angle_finished = 1

	if(angle_finished == 1):
		if(reach_goal(d_t , e_d, distancia_recorrida) == True):
			angle_finished = 2
			print("Se alcanzo el goal")

		        
def main():
	rospy.init_node('prueba_localizacion_1p0')
	r = rospy.Rate(100)
	wl_subscriber = rospy.Subscriber("/wl", Float32, odom_l)
	wr_subscriber = rospy.Subscriber("/wr", Float32, odom_r)
	desired_position = rospy.Subscriber("/final_pos", Pose, getPos)
	while not rospy.is_shutdown():
		r.sleep()


if __name__ == "__main__":

    main()

# TO DO:
# Comprobar si al llamarlo 2 veces causa problemas o no afecta en el procesamiento  PROBADO
# RECIBIR LOS VALORES DE LOS SUBSCRIBERS EN LA FUNCION DE LOCALIZACION              FUNCIONA
# CALCULO CONSTANTE DE LA POSICION (no se si se reinicia cada que llama al main)    FUNCIONA
# DAR SALIDA DE LOS 3 VALORES (prints de los valores)                               FUNCIONA
# HACER QUE EL VALOR DE THETA OSCILE ENTRE LOS RANGOS DE VALORES DE THETA           FUNCIONA
# PROBAR DIFERENTES dt                                                              FUNCIONA PERO PUEDE MEJORAR
# DEFINICION DE LAS VARIABLES FUERA DE LA FUNCION                                   FUNCIONA
# PUBLICAR LOS VALORES (SOLO LOS TENEMOS CON PRINTS)                                POR HACER
# 
