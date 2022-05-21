import rospy
import time
import cv2 as cv2 
import numpy as np 
from cv_bridge import CvBridge,CvBridgeError #Cv bridge nos ayuda a pasar la imagen a un formato procesable para opencv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

linear_velocity = 0.1
max_angular_velocity = 1.0
k = max_angular_velocity / 5
vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

command = Twist()
command.linear.y = 0 
command.linear.z = 0 
command.angular.y = 0 
command.angular.x = 0 

Bridge = CvBridge()

BGR_LOW = (0,0,0)
BGR_HIGH = (128,85,128)

def color_detection(frame):
    resize = cv2.resize(frame, (1280/4,720/4)) # Image resizing
    cropped_image = resize[90:180, 60:260]
    height, width, _, = cropped_image.shape
    gray = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
    mask = cv2.inRange(cropped_image, BGR_LOW, BGR_HIGH)
    line_img = gray & mask
    return cropped_image, line_img, width


def contour_detection(cropped_image, line_img, width):
    contours, _ = cv2.findContours(np.uint8(line_img),cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    if len(contours) > 0 :
        line = max(contours,key=cv2.contourArea)
        if cv2.contourArea(line) > 500:
            moments = cv2.moments(line)
            #componentes "x" y "y" del centro del poligono mas grande
            cx = int(moments['m10']/moments['m00'])
            cy = int(moments['m01']/moments['m00'])
            cv2.drawContours(cropped_image,contours,-1,(180,255,180),1)
            cv2.drawContours(cropped_image,line,-1,(0,255,0),1)
            cv2.circle(cropped_image,(cx,cy),4,(255,0,0),-1)
        else:
            cx = -1
            return 0
    else:
        cx = -1
        return 0

    if(cx != -1):
        error = width/2 - cx # que tan lejos esta el centro del width del centro de la linea
        return error


def control(error, width):
    #valor del error entre 0 y 1 (0 indicandonos que no hay error)
    normal_error = error / (width / 2.0) 
    command.linear.x = linear_velocity
    command.angular.z = k * normal_error
    vel_pub.publish(command)


def image_callback(data):
    cv_image = Bridge.imgmsg_to_cv2(data, "bgr8")
    cropped_image, line_img, width = color_detection(cv_image)
    error =  contour_detection(cropped_image, line_img, width)
    control(error, width)
    
    cv2.imshow("Imagen_Linea", cropped_image)
    cv2.imshow("color detection", line_img)
    cv2.waitKey(3)
    

def main():
    rospy.init_node('image_converter')
    image_sub = rospy.Subscriber("/video_source/raw",Image,image_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
