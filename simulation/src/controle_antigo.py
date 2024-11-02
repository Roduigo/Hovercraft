#!/usr/bin/env python3
# Rodrigo Kurosawa
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from geometry_msgs.msg import Point

class ContVelocidade:
    def __init__(self):
        self.msg_vel = Twist()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/setpoint', Point, self.callback)
        self.distancia_sub = rospy.Subscriber('/distancia', Int32, self.callback_distancia)
        self.rate = rospy.Rate(10)
        self.tolerance = 20
        self.distancia = 4000000 # distancia em quantidade de pixels para parar o robo
        
    def callback(self, data):
        setpoint = data
        
        self.msg_vel = Twist() #cria um objeto do tipo Twist para alterar a velocidade
        
        # Define a velocidade linear e angular para ajustar o setpoint do robo
        
        if abs(data.x) > self.tolerance:
            # Calcula a velocidade angular proporcional à distância do setpoint ao centro
            self.msg_vel.angular.z = (data.x) / 700  # Fator de escala para alterar a velocidade angular

            # Publica a nova velocidade angular
            rospy.loginfo(f"distancia ao centro da tela{data.x}")
            rospy.loginfo(f"Ajustando rotação: angular.z={self.msg_vel.angular.z}")
            self.pub.publish(self.msg_vel)
        else:
            # Se o setpoint estiver centralizado, parar o robô
            self.msg_vel.angular.z = 0
            rospy.loginfo("Setpoint centralizado, parando o robô")
            self.pub.publish(self.msg_vel)
        
    def callback_distancia(self, data): #calcula a distancia com base na quantidade de pixels na tela
        distancia = data.data
        self.msg_vel = Twist()
        if distancia < self.distancia:
            self.msg_vel.linear.x = 2
            rospy.loginfo("Ajustando a velocidade linear para frente")
            self.pub.publish(self.msg_vel)
        else: 
            self.msg_vel.linear.x = 0
            rospy.loginfo("Parando o robô")
            self.pub.publish(self.msg_vel)
    
if __name__ == '__main__':
    rospy.init_node('move_and_turn', anonymous=False) #Inicializa o nó do ROS
    sv = ContVelocidade()
    rospy.spin() #mantém o nó ativo
