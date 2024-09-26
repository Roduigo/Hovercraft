#!/usr/bin/env python3
# Rodrigo Kurosawa
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
from cv_bridge import CvBridge

class ImageConverter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.setpoint_pub = rospy.Publisher("/setpoint", Point, queue_size=10)
        self.image_pub = rospy.Publisher("/camera/image_processed", Image, queue_size=10)
        self.distancia_pub = rospy.Publisher("/distancia", Int32, queue_size=10)

    def callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
        # Converter o frame para o espaço de cor HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Definir os limites inferiores e superiores para a cor laranja
        lower_orange = np.array([10, 100, 100])   # Limite inferior do laranja
        upper_orange = np.array([25, 255, 255])   # Limite superior do laranja

        # Criar a máscara para a cor laranja
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # Exibir a máscara da cor laranja
        cv2.imshow('Orange Object Mask', mask)
        cv2.waitKey(1)
        
        sum_mask = np.sum(mask)  # Soma todos os pixeis da máscara
        rospy.loginfo(f"Soma dos pixeis laranja: {sum_mask}")
        self.distancia_pub.publish(sum_mask)  # Publica a soma dos pixeis laranja
        
        # Calcular os momentos da imagem binária
        moments = cv2.moments(mask)

        # Verificar se há algum pixel laranja detectado (área > 0)
        if moments["m00"] != 0:
            # Calcular o centro de massa dos pixels laranja
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])

            # Desenhar um ponto vermelho no centro de massa
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        
        # Obtém as dimensões da imagem (altura, largura)
        altura, largura, _ = frame.shape
    
        # Calcula o centro da imagem
        centro_x = largura // 2
        centro_y = altura // 2
    
        setpoint = Point()
        setpoint.x = cx - centro_x # Defina a coordenada x do ponto 
        setpoint.y = cy - centro_y # Defina a coordenada y do ponto
        
        # Publica o setpoint no tópico
        rospy.loginfo(f"Publicando setpoint: x={setpoint.x}, y={setpoint.y}")
        self.setpoint_pub.publish(setpoint)
        
        # Exibir a imagem com o centro de massa detectado
        cv2.imshow('Orange Object Center Detection', frame)
        cv2.waitKey(1)

        # Converter o frame de volta para formato ROS e publicar
        frame = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

if __name__ == "__main__":
    rospy.init_node('image_converter', anonymous=False)
    rospy.loginfo("Iniciando o node image_converter")
    ic = ImageConverter()
    rospy.spin() 
