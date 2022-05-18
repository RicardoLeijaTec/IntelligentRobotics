import rospy
from std_msgs.msg import String
import cv2 as cv2 
import numpy as np 
import time 
#Cv bridge nos ayuda a pasar la imagen a un formato procesable para opencv
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image



Bridge = CvBridge()
bgr_low = (19,255,255)
bgr_high = (119,255,255)


def image_callback(data):
    cv_image = Bridge.imgmsg_to_cv2(data, "bgr8")
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    mask = cv2.inRange(cv_image,bgr_low,bgr_high)
    line_img = gray & mask
    contours, _ = cv2.findContours(np.uint8(line_img),cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    if len(contours) > 0 :
        line = max(contours,key=cv2.contourArea)
        if cv2.contourArea(line) > 50:
            print("adentro")
            moments = cv2.moments(line)
            #componentes "x" y "y" del centro del poligono mas grande
            cx = int(moments['m10']/moments['m00'])
            cy = int(moments['m01']/moments['m00'])
            cv2.drawContours(cv_image,contours,-1,(180,255,180),3)
            cv2.drawContours(cv_image,line,-1,(0,255,0),3)
            cv2.circle(cv_image,(cx,cy),4,(255,0,0),-1)
        else:
            cx = -1
    else:
        cx = -1
            
    cv2.imshow("Imagen_Linea", cv_image)
    cv2.waitKey(3)
    

def main():
    rospy.init_node('image_converter')
    image_sub = rospy.Subscriber("/camera/image",Image,image_callback)
    rospy.spin()


if _name_ == '_main_':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
