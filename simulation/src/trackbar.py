#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from std_msgs.msg import Int32MultiArray

class TrackbarNode:
    def __init__(self):
        # Inicializa a janela e os trackbars
        cv2.namedWindow('RGB Trackbars')
        cv2.createTrackbar('R', 'RGB Trackbars', 146, 255, self.nothing)
        cv2.createTrackbar('G', 'RGB Trackbars', 71, 255, self.nothing)
        cv2.createTrackbar('B', 'RGB Trackbars', 0, 255, self.nothing)
        cv2.createTrackbar('Tolerancia', 'RGB Trackbars', 20, 100, self.nothing)
        
        # Inicializa o publisher
        self.pub = rospy.Publisher('/trackbar_values', Int32MultiArray, queue_size=10)
        rospy.init_node('trackbar_node', anonymous=False)
        self.rate = rospy.Rate(10)  # Publica a cada 10 Hz
    
    def nothing(self, x):
        pass
    
    def run(self):
        while not rospy.is_shutdown():
            # Obtém os valores dos trackbars
            r = cv2.getTrackbarPos('R', 'RGB Trackbars')
            g = cv2.getTrackbarPos('G', 'RGB Trackbars')
            b = cv2.getTrackbarPos('B', 'RGB Trackbars')
            tolerancia = cv2.getTrackbarPos('Tolerancia', 'RGB Trackbars')
            
            # Cria uma mensagem para publicar
            msg = Int32MultiArray(data=[r, g, b, tolerancia])
            self.pub.publish(msg)
            
            # Atualiza a janela do OpenCV
            img = np.zeros((300, 512, 3), np.uint8)
            img[:] = [b, g, r]
            cv2.imshow('RGB Trackbars', img)
            cv2.waitKey(1)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        TrackbarNode().run()
    except rospy.ROSInterruptException:
        pass
