import rospy
import sys
import math
import time
from math import pi, atan2
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
dt = 0.01 # Time between samples (s) #0.00525 es preciso en fisico pero no en codigo # 0.0055 es preciso en codigo pero no en fisico

final_pos_x = 0.0 #Constante que nos da la posicion final del archivo
final_pos_y = 0.0

e_theta = 0.07
e_d = 0.0

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10, latch=True)
angle_finished = 0
lost = 0

#Wait for target
halt = True

def get_target(data):
        global halt, final_pos_x, final_pos_y
        final_pos_x = data.position.x
        final_pos_y = data.position.y
        halt = False


def odom_l(data):
        global w_l
        w_l = data.data	


def odom_r(data):
        global w_r
        w_r = data.data	


def correct_angle(error):
	global cmd_vel_pub
	w_to_cmd_vel = Twist()
	vel_w = 0.08 * error
	#print("vel w", vel_w, "Error: ", error)
	if(-0.05 < error < 0.05):
		w_to_cmd_vel.angular.z = 0.0
		cmd_vel_pub.publish(w_to_cmd_vel)
		return True
	if(error >= 0.8):
		w_to_cmd_vel.angular.z = vel_w
		cmd_vel_pub.publish(w_to_cmd_vel)
		return False
		

def reach_goal(distancia_total, distancia_faltante, lost):
	global cmd_vel_pub
        v_to_cmd_vel = Twist()
        #minimum of 0
        if(distancia_total != 0):
	        vel_v = max(0, 0.9*(distancia_faltante/distancia_total))
        else:
                vel_v = 0

        if(lost < 0):
                vel_v = -vel_v

	distancia_recorrida = distancia_total - distancia_faltante
        if(distancia_recorrida >= distancia_total or distancia_faltante < 0.0095): 
		v_to_cmd_vel.linear.x = 0.0
		cmd_vel_pub.publish(v_to_cmd_vel)
		return True
        else:
		v_to_cmd_vel.linear.x = vel_v
		cmd_vel_pub.publish(v_to_cmd_vel)
		return False
        return False


def actuate():
        global theta_k, xk, yk, d_t, e_d, lost
        global angle_finished

        e_theta = atan2(final_pos_y, final_pos_x) - theta_k
	theta_k = theta_k + r*((w_r - w_l)/l)*dt
	degree = theta_k*180/pi

	xk = xk + r*((w_r + w_l)/2)*dt*math.cos(theta_k)
	yk = yk + r*((w_r + w_l)/2)*dt*math.sin(theta_k)
        
	d_t = math.sqrt((final_pos_x)**2 + (final_pos_y)**2) # Distancia total por recorrer
	e_d = math.sqrt((final_pos_x - xk)**2 + (final_pos_y - yk)**2)

        lost = atan2(final_pos_y - yk, final_pos_x - xk)*180/pi
	
	if(theta_k > 2*math.pi):
		theta_k = 0
	if(theta_k < 0):
		theta_k = 2*math.pi

        sys.stdout.write("x: %+06.2f\ty: %+06.2f\tLost: %+06.2f\tDegree: %+06.2f" % ((xk, yk, lost, degree)))
        sys.stdout.write("\t\td_r: %+06.2f\td_t: %+06.2f\td_f: %+06.2f | %d\n" % (d_t-e_d, d_t, e_d, angle_finished))
        
	if(angle_finished == 0 and correct_angle(e_theta)):
	        angle_finished = 1
	elif(angle_finished > 0 and reach_goal(d_t, e_d, lost)):
                angle_finished = 2


def main():
	global cmd_vel_pub
	rospy.init_node('prueba_localizacion_1p0')
	r = rospy.Rate(100)
	wl_subscriber = rospy.Subscriber("/wl", Float32, odom_l)
	wr_subscriber = rospy.Subscriber("/wr", Float32, odom_r)
	desired_position = rospy.Subscriber("/final_pos", Pose, get_target)
        while not rospy.is_shutdown():
                if(not halt):
                    actuate()
                r.sleep()


if __name__ == "__main__":
        main()

# TO DO:
# Comprobar si al llamarlo 2 veces causa problemas o no afecta en el procesamiento  PROBADO
# RECIBIR LOS VALORES DE LOS SUBSCRIBERS EN LA FUNCION DE LOCALIZACION              FUNCIONA
# CALCULO CONSTANTE DE LA POSICION (no se si se reinicia cada que llama al main)    FUNCIONA
# DAR SALIDA DE LOS 3 VALORES (prints de los valores)                               FUNCIONA
# HACER QUE EL VALOR DE THETA OSCILE ENTRE LOS RANGOS DE VALORES DE THETA           FUNCIONA
# PROBAR DIFERENTES dt                                                              FUNCIONA
# DEFINICION DE LAS VARIABLES FUERA DE LA FUNCION                                   FUNCIONA
# PUBLICAR LOS VALORES (SOLO LOS TENEMOS CON PRINTS)                                POR HACER
