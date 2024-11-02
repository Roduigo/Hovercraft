import RPi.GPIO as GPIO
from time import sleep

IN1 = 18
#IN2 = 27

GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
#GPIO.setup(IN2, GPIO.OUT)

def motor_on():
    motor = GPIO.PWM(IN1, 50)

    # #pino e saida
    # GPIO.output(IN1, GPIO.HIGH)
    # GPIO.output(IN2, GPIO.LOw)
    print("motor ligado")

def motor_off():
    # GPIO.output(IN1, GPIO.LOW)
    # GPIO.output(IN2, GPIO.LOW)
    print("motor desligado")

try:
    while True:
        motor_on()
        sleep(5)
        motor_off()
        sleep(5)
except KeyboardInterrupt:
    print("Encerrando o programa")
finally:
    GPIO.cleanup()