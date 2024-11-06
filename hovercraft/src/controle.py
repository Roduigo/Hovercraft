#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
from PID import PID

class ContVelocidade:
    def __init__(self):
        self.vel = Int32MultiArray()
        self.pub = rospy.Publisher('/velocidade', Int32MultiArray, queue_size=10)
        self.sub = rospy.Subscriber('/setpoint', Point, self.callback)
        self.distancia_sub = rospy.Subscriber('/distancia', Int32, self.distancia)
        self.rate = rospy.Rate(10)
        self.dist = 0
        self.referencia = 10000000 # distancia em quantidade de pixels para parar o robo
        
        self.PID_giro = PID(1/600,1/600, 1/1500, 1/20) #Ku = 1/50, Tu = 2s
        self.PID_frente = PID(1/8000000,0,0,1/30,0, 100) #Ku = 1/2500000, Tu = 2s
      #1/5000000 6/25000000, 3/50000000
        
    def callback(self, data):
        #setpoint = data
        setpoint_giro = data.x
        
        self.vel = Int32MultiArray() #cria um objeto do tipo Twist para alterar a velocidade
        
        self.vel.angular.z = self.PID_giro.compute(setpoint_giro)

        rospy.loginfo(f"distancia ao centro da tela{setpoint_giro}")
        rospy.loginfo(f"Ajustando rotação: angular.z={self.vel.angular.z}")
        self.pub.publish(self.vel)
        
        setpoint_frente = self.referencia - self.dist

        self.msg_vel.linear.x = self.PID_frente.compute(setpoint_frente)
        
        if self.dist < self.referencia:
            rospy.loginfo(f"Ajustando a velocidade linear para frente={self.msg_vel.angular.x}")
            self.pub.publish(self.msg_vel)
        else: 
            rospy.loginfo("Parando o robô")
            self.pub.publish(self.msg_vel)
        
        #---------
        
        # setpoint = data
        
        # self.vel = Int32MultiArray()
        
        # if abs(data.x) > self.tolerance:
        #     # Calcula a velocidade angular proporcional à distância do setpoint ao centro
        #     if(setpoint<0):
        #         dir = 30
        #         esq = 0
        #     else:
        #         dir = 0
        #         esq = 30
        #         #vai ser do PID e COMO????
        #     # Publica a nova velocidade angular
        #     rospy.loginfo(f"distancia ao centro da tela{setpoint}")
        #     self.vel = [dir,esq]
        #     self.pub.publish(self.vel)
        # else:
        #     # Se o setpoint estiver centralizado, parar o robô
        #     dir = 30
        #     esq = 30
        #     rospy.loginfo("Setpoint centralizado, parando o robô")
        #     self.vel = [dir,esq]
        #     self.pub.publish(self.vel)
        
    # função para apenas inicializar a variável distancia. Obs: a atribuição é a quantidade de pixeis
    def distancia(self,data):
        dist32 = data
        self.dist = int(dist32.data)
    
        
if __name__ == '__main__':
    rospy.init_node('move_and_turn', anonymous=False) #Inicializa o nó do ROS
    sv = ContVelocidade()
    rospy.spin() #mantém o nó ativo 
    #felipepombo
