import RPi.GPIO as GPIO
import time
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Point

class Hover:

    def __init__(self, pin1, pin2, frequencia):
        self.esc1 = ESC(pin1, freq=frequencia)
        self.esc2 = ESC(pin2, freq=frequencia)
        self.velocidade = rospy.Subscriber("/velocidade", Int32MultiArray , self.callback)

    def callback(self, velocidade):

        #set_speed
        # Converte a velocidade (0 a 100%) para o sinal PWM correspondente
        # Pulsos de 1ms a 2ms são mapeados para valores de duty cycle entre 5%(parado) e 10%(vel max) em uma frequência de 50Hz.
        #Isso corresponde a pulsos de 2ms e 1ms para a freq de 50Hz. Geralmente, esses são os pulsos esperados pela ESC.
        pwm_value = self.map_speed_to_pwm(velocidade)
        self.pwm.ChangeDutyCycle(pwm_value)

        msg = Int32MultiArray()

        #COMO FAZER PARA DIFERENCIAR ESC1 E ESC2 NO SET_SPEED???????
        if msg.dir > msg.esq:
            #Motor esquerda (pino 1)
            self.esc1.set_speed(70)
            #motor direita (pino 2)
            self.esc2.set_speed(30)
        elif msg.esq > msg.dir:
            #Motor esquerda (pino 1)
            self.esc1.set_speed(30)
            #motor direita (pino 2)
            self.esc2.set_speed(70)
        else:
            #Motor esquerda (pino 1)
            self.esc1.set_speed(100)
            #motor direita (pino 2)
            self.esc2.set_speed(100)



class ESC:
    def __init__(self, pwm_pin, freq=50):
       
        # Configura o pino GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pwm_pin, GPIO.OUT)

        # Cria um objeto PWM no pino pwm_pin com a frequência especificada
        self.pwm = GPIO.PWM(self.pwm_pin, self.freq)
        self.pwm.start(0)  # Inicializa com o PWM em 0%

    def calibrate(self):
        
        print("Calibrando ESC - Máximo...")
        self.set_speed(100)  # Define a velocidade máxima para calibrar
        input("Conecte a bateria")

        time.sleep(2)

        print("Calibrando ESC - Mínimo...")
        self.set_speed(0)  # Define a velocidade mínima para calibrar
        time.sleep(2)
        input("Desconecte a bateria")
        print("ESC calibrado")

    def stop(self):
        # Para o ESC ajustando o duty cycle para 0
        self.pwm.ChangeDutyCycle(0)

    def map_speed_to_pwm(self, speed):
        # Garante que a velocidade esteja entre 0 e 100
        if speed > 100:
            speed = 100
        if speed < 0:
            speed = 0

        # Mapeia a velocidade de 0 a 100 para duty cycle entre 5% e 10%
        # 5% duty cycle -> 1ms (ESC parada)
        # 10% duty cycle -> 2ms (ESC a toda velocidade)
        return (5 * speed / 100) + 5

    def cleanup(self):
        # Limpa a configuração dos pinos GPIO
        self.pwm.stop()
        GPIO.cleanup()
        #quando mudar biblioteca, fazer método de desligar hover

if __name__ == '__main__':
    rospy.init_node('ESC Motores', anonymous=False) #Inicializa o nó do ROS
    pao = Hover(18, 19, 50) #pino 1, pino 2, freq 
    rospy.spin() #mantém o nó ativo 
    #luisa_santello