import rospy
import sys
import math
from math import pi
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist


# Robot location
xk = 0.0 # x position at k timestep in meters
yk = 0.0 # y position at k timestep in meters
thetak = 0.0 # angle at k timestep in radians
'''	Values of theta must be contained within a single circle:
	EITHER: -pi <= theta < pi OR: 0 <= theta < 2pi'''

# Robot constants
r = 0.05 # Wheel radius, 0.05m
l = 0.19 # Distance between robot wheels 0.19m

# Measured variables
w_l, w_r = 0.0, 0.0 # Wheels velocity (rad/s)
dt = 0.00525 # Time between samples (s)

final_pos_x = 0.0
final_pos_y = 0.0

e_theta = 0.0
theta_t = 0.0
# theta r es igual a theta k
e_d = 0.0

finished_change_Angle = False 
isFinished = False
goal_reached = False
distancia_recorrida = 0.0

x = 0

def getPos(posArray):
	return odom_callback(posArray, 'final_pos')


def odom_l(wl):
	return odom_callback(wl, 'wl')


def odom_r(wr):
	return odom_callback(wr, 'wr')

'''
# Recibe angulo actual, angulo deseado y error
def correctAngleRichie(theta_r, theta_t, e_theta):
	pub_z = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	data = Twist()
	data_to_send_z = 0.0
	
	if(e_theta > 0.5):
		data_to_send_z = 0.2
		data.angular.z = data_to_send_z
	
	elif(e_theta < -0.5):
		data_to_send_z = -0.2
		data.angular.z = data_to_send_z
	else:
		data_to_send_z = 0.0
		data.angular.z = data_to_send_z
	pub_z.publish(data)'''

'''
def correct_Angle(Actual, Deseado, terminado):
	global finished_change_Angle
	print("dentro de correct angle")
        pub_Angle = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	w_to_cmdVel  = Twist()
	velo_W = 0.0
	stop = 0.0
	print("Actual: ", Actual, "Deseado: ",Deseado, "velo_w: ", velo_W)
	
	if(Actual > Deseado):
		w_to_cmdVel.angular.z = stop
		pub_Angle.publish(w_to_cmdVel)
		terminado = True
		finished_change_Angle = True
		return terminado
		
	if(Actual < Deseado):
		velo_W = 0.1*Deseado
		w_to_cmdVel.angular.z = velo_W
		pub_Angle.publish(w_to_cmdVel) '''


def correct_Angle(Actual, Deseado):
        pub_Angle = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	w_to_cmdVel  = Twist()
	velo_W = 0.25*Deseado
	print("Actual: ", Actual, "Deseado: ",Deseado, "velo_w: ", velo_W)
	
	if(Actual > Deseado):
		w_to_cmdVel.angular.z = 0.0
		pub_Angle.publish(w_to_cmdVel)
		return True
		
	if(Actual < Deseado):
		w_to_cmdVel.angular.z = velo_W
		pub_Angle.publish(w_to_cmdVel) 
		return False


def reachGoal(distancia_total, distancia_faltante, isFinished):
	global distancia_recorrida, goal_reached
	print("Entrando a reach goal")
	pub_Distance = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	v_to_cmdVel = Twist()
	velo_V = 0.0
	stop = 0.0
	distancia_recorrida = distancia_recorrida + (distancia_total - distancia_faltante)
	print(distancia_recorrida)
	
	if(distancia_recorrida >= distancia_total):
		v_to_cmdVel.linear.x = stop
		pub_Distance.publish(v_to_cmdVel)
		print("Ya debio de haberse detenido pa")
		isFinished = True
		goal_reached = True
		return goal_reached
		
	if(distancia_recorrida < distancia_total):
		velo_V = 0.05*distancia_faltante
		v_to_cmdVel.linear.x = velo_V
		pub_Distance.publish(v_to_cmdVel)
		print("Sigue sin llegar pa")

def funcion(x):

	x = x + 101 
	print(x)
	if(x>100):
		return True
	else:
		return False  

def odom_callback(data, topic):
	global xk, yk, thetak, degree
	global w_l, w_r
	global final_pos_x, final_pos_y
	global e_theta, theta_t
	global finished_change_Angle
	global goal_reached
	global x 
	
	if(topic == 'final_pos'):
		final_pos_x = data.position.x
		final_pos_y = data.position.y
	print("--------------------------------")
	print("pos deseada x: ", final_pos_x)
	print("pos deseada y: ", final_pos_y)
	print("--------------------------------")

	# print(topic, data) # debug
	if (topic == "wr"):
		w_r = data.data 
	if (topic == "wl"):
		w_l= data.data

	thetak = thetak + r*((w_r - w_l)/l)*dt

	degree = thetak*180/math.pi
	#degree = thetak * 3.1416

	xk = xk + r*((w_r + w_l)/2)*dt*math.cos(thetak)
	yk = yk + r*((w_r + w_l)/2)*dt*math.sin(thetak)
	
	theta_t = math.atan2(final_pos_y, final_pos_x)
	e_theta = theta_t - thetak

	d_t = math.sqrt((final_pos_x)**2 + (final_pos_y)**2) # Distancia total por recorrer
	e_d = math.sqrt((final_pos_x - xk)**2 + (final_pos_y - yk)**2)

	if(thetak > math.pi):
		thetak = -math.pi
	if(thetak < -math.pi):
		thetak = math.pi
	
	print("Valor de x: ", xk)
	print("Valor de y: ", yk)
	print("Valor de theta: ", thetak)
	print("Grados: ", degree)
	print("Valor de e_theta: ", e_theta)
	print("--------------------------------")
	# print("Valor de ed: ", e_d)
	#correctAngleRichie(thetak, theta_t, e_theta)
	#print("finished angle change", finished_change_Angle)
	#if(finished_change_Angle == False):
        #correct_Angle(thetak, e_theta, finished_change_Angle)
	#print("finished", finished) ########### da error despues de que el finished_change_Angle se hace true porque ya no se crea esa variable entonces ya no la encuentra
	
	if(correct_Angle(thetak, e_theta) == True):
		#reachGoal(d_t, e_d, isFinished)
		print("correccion terminada CUCK")
		
	#if(funcion(x) == True):
		#print("se hizo me vengo en tu ojo")
	#if(finished_change_Angle == True and goal_reached == False):
	#if(goal_reached == False and finished_change_Angle == True):
		#print("finished_change_Angle ya es true we")
		#reached = reachGoal(d_t, e_d, isFinished)
	#print("Reached: ", reached)


def main():
	rospy.init_node('prueba_localizacion_1p0')
	r = rospy.Rate(100)
	# Si se llama 2 veces, ejecuta las 2 suscripciones cada spin
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
# PROBAR DIFERENTES dt                                                              FALLA
# DEFINICION DE LAS VARIABLES FUERA DE LA FUNCION                                   FUNCIONA
# PUBLICAR LOS VALORES (SOLO LOS TENEMOS CON PRINTS)                                POR HACER
# 

