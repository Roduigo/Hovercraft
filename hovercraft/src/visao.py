#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
from cv_bridge import CvBridge

class ImageConverter:
    def __init__(self):
        # Cria a janela e os trackbars para os valores de RGB
        cv2.namedWindow('RGB Trackbars')
        cv2.createTrackbar('R', 'RGB Trackbars', 0, 255, self.trackbar_pos)
        cv2.createTrackbar('G', 'RGB Trackbars', 0, 255, self.trackbar_pos)
        cv2.createTrackbar('B', 'RGB Trackbars', 0, 255, self.trackbar_pos)
        cv2.createTrackbar('Tolerancia', 'RGB Trackbars', 20, 100, self.trackbar_pos)
        # Define os valores iniciais para cada trackbar
        cv2.setTrackbarPos('R', 'RGB Trackbars', 213)
        cv2.setTrackbarPos('G', 'RGB Trackbars', 10)
        cv2.setTrackbarPos('B', 'RGB Trackbars', 40)
        cv2.setTrackbarPos('Tolerancia', 'RGB Trackbars', 68)
        # cria uma imagem preta com height = 300, width = 512
        self.img = np.zeros((300, 512, 3), np.uint8)
        self.lower_color = 0
        self.upper_color = 0
        
        self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.image = cv2.VideoCapture(-1)
        
        # Definir a resolução da câmera
        largura = 640  # Largura desejada
        altura = 480   # Altura desejada

        self.image.set(cv2.CAP_PROP_FRAME_WIDTH, largura)
        self.image.set(cv2.CAP_PROP_FRAME_HEIGHT, altura)
        self.setpoint_pub = rospy.Publisher("/setpoint", Point, queue_size=30)
        self.image_pub = rospy.Publisher("/camera/image_processed", Image, queue_size=30)
        self.distancia_pub = rospy.Publisher("/distancia", Int32, queue_size=30)
        
        self.mostrar_imagem = True
          
    def trackbar_pos(self,x):
        # Obter valores das trackbars de cor
        r = cv2.getTrackbarPos('R', 'RGB Trackbars')
        g = cv2.getTrackbarPos('G', 'RGB Trackbars')
        b = cv2.getTrackbarPos('B', 'RGB Trackbars')
        tolerancia = cv2.getTrackbarPos('Tolerancia', 'RGB Trackbars')

        self.img[:] = [b, g, r]  # Preencher a imagem com a cor selecionada
        
        # Definir os limites da cor baseada no valor da trackbar e tolerância
        self.lower_color = np.array([b - tolerancia, g - tolerancia, r - tolerancia])
        self.upper_color = np.array([b + tolerancia, g + tolerancia, r + tolerancia])

    def processa_imagem(self):
        ret, frame = self.image.read()
        if not ret:
            rospy.logwarn("Não foi possível capturar a imagem da câmera")
            return

        # Criar a máscara para a cor selecionada
        mask = cv2.inRange(frame, self.lower_color, self.upper_color)

        # Exibir a máscara
        if(self.mostrar_imagem):
            cv2.imshow('RGB Trackbars', self.img)
            cv2.imshow('Object Mask', mask)
            cv2.waitKey(1)  # Importante para atualizar as janelas do OpenCV


        sum_mask = np.sum(mask)  # Soma todos os pixeis da máscara
        rospy.loginfo(f"Soma dos pixeis laranja: {sum_mask}")
        self.distancia_pub.publish(sum_mask)  # Publica a soma dos pixeis laranja
        
        # Calcular os momentos da imagem binária
        moments = cv2.moments(mask)
        cx = 0
        cy = 0 
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
        if(self.mostrar_imagem):
            cv2.imshow('Orange Object Center Detection', frame)
            cv2.waitKey(1)

        # Converter o frame de volta para formato ROS e publicar
        frame = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        
        self.image_pub.publish(frame)

if __name__ == "__main__":
    rospy.init_node('image_converter', anonymous=False)
    rospy.loginfo("Iniciando o node image_converter")
    ic = ImageConverter()
    while not rospy.is_shutdown():
        ic.processa_imagem()  # Processa a captura da câmera
        # Pressione 'q' para sair do loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            ic.mostrar_imagem = not (ic.mostrar_imagem)
            cv2.destroyAllWindows()
    cv2.waitKey()
    cv2.destroyAllWindows()
