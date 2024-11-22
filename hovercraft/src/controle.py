#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point
from PID import PID
import math

class ContVelocidade:
    def __init__(self):
        self.vel = Float32MultiArray()
        self.pub = rospy.Publisher('/velocidade', Float32MultiArray, queue_size=3000)
        self.sub = rospy.Subscriber('/setpoint', Point, self.angular)
        self.distancia_sub = rospy.Subscriber('/distancia', Int32, self.distancia)
        self.yaw = rospy.Subscriber("/mavros/imu/data", Imu, self.vel_control)
        self.rate = rospy.Rate(10)
        self.dist = 0
        self.rot = 0
        self.frente = 0
        self.referencia = 10000000 # distancia em quantidade de pixels para parar o robo
        self.min_pixels = 10000 #valor minínimo para o robo ainda identificar objeto
        self.setpoint_relativo = 0
        # parametros da camera
        self.freq_cam = 30
        self.altura = 480
        self.largura = 640
        self.fov = 53.423480767872312502265 # campo de visão da camera calculado na diagonal
        
        self.k_graus = self.fov / self.largura #constante graus / pixeis
        
        self.PID_giro = PID(1/600,0, 0, 1/self.freq_cam)
        self.PID_frente = PID(7/800000,0,0,1/self.freq_cam,0 ,100) 
        #valores entre 1/80000 e 1/800000
    
    def vel_control(self, data): #callback do yaw da px4
        
        self.vel = Float32MultiArray()#cria um objeto do tipo Twist para alterar a velocidade
        
        ########## FRENTE ###########
        setpoint_frente = self.referencia - self.dist

        self.frente = self.PID_frente.compute(setpoint_frente)
        
        if(self.dist < self.min_pixels): # limite minimo de pixeis que precisa para robo ir para frente
            self.vel.data = [0,0]
            rospy.loginfo(f"Nenhum objeto detectado. Quantidade de pixels:{self.min_pixels}")
        else:
            self.vel.data = [self.frente,self.frente]
            rospy.loginfo(f"Ajustando a velocidade linear para frente={self.frente}")
    
        ########## GIRO ###########
    
        self.angulo_pix = convert_graus(data) #posição da pix em graus
        
        setpoint_giro = self.setpoint_relativo - self.angulo_pix
        
        # o PID vai retornar a direção em que o robo deverá girar
        self.rot = self.PID_giro.compute(setpoint_giro) # calcula o o setpoint em graus e joga no PID 
        
        if(self.rot>0):
            self.vel.data[0] -= abs(self.rot)
        else:
            self.vel.data[1] -= abs(self.rot)
            
        ######### PUBLICA #########
        
        self.pub.publish(self.vel)
        rospy.loginfo(f"distancia ao centro da tela{setpoint_giro}")
        rospy.loginfo(f"Ajustando rotação: angular.z = dir {self.vel.data[0]} esq = {self.vel.data[1]}")
    
    def convert_graus(self, valor):
        orientation = valor.orientation
        quaternion = [orientation.x,orientation.y,orientation.z,orientation.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        yaw *= 360/(2*math.pi)
        return yaw

    # função para apenas inicializar a variável distancia. Obs: a atribuição é a quantidade de pixeis
    def distancia(self,data):
        dist32 = data
        self.dist = int(dist32.data)
        
    def angular(self,data):
        setpoint_camera = data.x * self.k_graus #setpoint da camera dado em graus
        self.setpoint_relativo =  self.angulo_pix + setpoint_camera # valor relativo da posição do objeto calculado a partir da pix
        
if __name__ == '__main__':
    rospy.init_node('move_and_turn', anonymous=False) #Inicializa o nó do ROS
    sv = ContVelocidade()
    rospy.spin() #mantém o nó ativo 
    #felipepombo

