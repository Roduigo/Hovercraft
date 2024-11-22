#!/usr/bin/env python3
import pigpio 
import rospy
from std_msgs.msg import Float32MultiArray

class Hover:

    def __init__(self, pin1, pin2, frequencia):
        self.esc1 = ESC(pin1, freq=frequencia)
        self.esc2 = ESC(pin2, freq=frequencia)
        self.velocidade = rospy.Subscriber("/velocidade", Float32MultiArray , self.callback)

        #quando interromper o ros, irá dar um cleanup
        rospy.on_shutdown(self.cleanup)
        
    def callback(self, velocidade):
        
        #velocidade [0] - direita
        #velocidade [1] - esquerda
        dir = velocidade.data[0]
        esq = velocidade.data[1]
        rospy.loginfo(f"direita ={self.velocidade.data[0]} esquerda ={self.velocidade.data[1]}")

        #Motor esquerda (pino 1)
        self.esc1.set_speed(esq)
        #motor direita (pino 2)
        self.esc2.set_speed(dir)
            
    def cleanup(self):
        self.esc1.cleanup()
        self.esc2.cleanup()
        
class ESC:
    def __init__(self, pwm_pin, freq=50):
        self.pi = pigpio.pi()
        self.pwm_pin = pwm_pin
        self.freq = freq

        # Cria um objeto PWM no pino pwm_pin com a frequência especificada
        self.pi.set_PWM_range(self.pwm_pin, 100)
        self.pi.set_mode(self.pwm_pin, pigpio.OUTPUT)
        self.pi.set_PWM_frequency(self.pwm_pin, self.freq)  # Inicializa com o PWM em 0%
        
    def set_speed(self, speed):
        # Converte a velocidade (0 a 100%) para o sinal PWM correspondente
        # Pulsos de 1ms a 2ms são mapeados para valores de duty cycle entre 5%(parado) e 10%(vel max) em uma frequência de 50Hz.
        #Isso corresponde a pulsos de 2ms e 1ms para a freq de 50Hz. Geralmente, esses são os pulsos esperados pela ESC.
        pwm_value = self.map_speed_to_pwm(speed)
        self.pi.set_PWM_dutycycle(self.pwm_pin,pwm_value)

    def stop(self):
        # Para o ESC ajustando o duty cycle para 0
        self.pi.set_PWM_dutycycle(self.pwm_pin, 0)

    def map_speed_to_pwm(self, speed):
        # Garante que a velocidade esteja entre 0 e 100
        if speed > 100:
            speed = 100
        if speed < 0:
            speed = 0

        # Mapeia a velocidade de 0 a 100 para duty cycle entre 5% e 10%
        # 5% duty cycle -> 1ms (ESC parada)
        # 10% duty cycle -> 2ms (ESC a toda velocidade)
        return int((5 * speed / 100) + 5)

    def cleanup(self):
        # Limpa a configuração dos pinos 
        self.pi.stop()
        #quando mudar biblioteca, fazer método de desligar hover

if __name__ == '__main__':
    rospy.init_node('ESC_Motores', anonymous=False) #Inicializa o nó do ROS
    pao = Hover(18, 20, 50) #pino 1, pino 2, freq 
    rospy.spin() #mantém o nó ativo 
    
    #luisa_santello