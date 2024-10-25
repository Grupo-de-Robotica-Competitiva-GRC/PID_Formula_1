import RPi.GPIO as GPIO
import time

# Definição dos pinos pwm
IN1=12
IN3=13

# Constantes PID
Ki = 0
Kp = 35
Kd = 35

P = 0
I = 0
D = 0
PID = 0
velEsq = 0
velDir = 0
erroAnterior = 0
erro = 0

# Velocidades padrão
aVelo = 210
bVelo = 210

# Configuração dos sensores
sensor_pins = [17,22,23,24]

def calculaPID():
    global P, I, D, PID, erro, erroAnterior
    if erro == 0:
        I = 0
    P = erro
    I += erro
    if I > 255:
        I = 255
    elif I < -255:
        I = -255
    D = erro - erroAnterior
    PID = (Kp * P) + (Ki * I) + (Kd * D)
    erroAnterior = erro

def controlaMotor():
    global velEsq, velDir
    if PID >= 0:
        velEsq = bVelo
        velDir = aVelo - PID
    else:
        velEsq = bVelo + PID
        velDir = aVelo
    
    IN1_pwm.ChangeDutyCycle(velEsq / 2.1)  # RPi.GPIO PWM usa valores de 0 a 100
    IN3_pwm.ChangeDutyCycle(velDir / 2.1)  # Converte 0-255 para 0-100


def lerSensores():
    global erro
    # Cria uma lista para armazenar os valores lidos de cada sensor
    sensor_values = [0] * 4
    for i in range(4):
        # Lê o valor de cada pino (0 ou 1)
        sensor_values[i] = GPIO.input(sensor_pins[i])
    
    if (sensor_values == [0,1,1,0]): erro = 0
    elif (sensor_values == [0,1,0,0]): erro = -1
    elif (sensor_values == [0,0,1,0]): erro = 1
    elif (sensor_values == [1,1,0,0]): erro = -2
    elif (sensor_values == [0,0,1,1]): erro = 2
    elif (sensor_values == [1,0,0,0]): erro = -3
    elif (sensor_values == [0,0,0,1]): erro = 3
    else: erro = 0                                      # revisar 
    
    #print(sensor_values,erro,sep='\t')
    


def setup():
    # Configuração do RPi.GPIO
    GPIO.setmode(GPIO.BCM)
    
    GPIO.setup(IN1,GPIO.OUT)
    GPIO.setup(IN3,GPIO.OUT)

    # Configuração dos sensores 
    for pin in sensor_pins:
        GPIO.setup(pin, GPIO.IN)
    
        


def loop():
    global erro, velEsq, velDir
    try:
        while True:
            # Lê os valores dos sensores
            lerSensores()
            calculaPID()
            controlaMotor()

            # Exibe os valores
            print(f'VelEsq: {velEsq}\tVelDir: {velDir}\tKp: {Kp}\tKd: {Kd}\tKi: {Ki}')

            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        IN1_pwm.ChangeDutyCycle(0) # this prevents jitter
        IN3_pwm.ChangeDutyCycle(0)
        
        #parando pwm
        IN1_pwm.stop()
        IN3_pwm.stop()
        
        # Limpeza dos GPIOs
        GPIO.cleanu;

if __name__ == '__main__':
    setup()
    
    # Configuração de PWM freq 1kHz
    IN1_pwm = GPIO.PWM(IN1,500)
    IN3_pwm = GPIO.PWM(IN3,500)
    
    IN1_pwm.start(0)
    IN3_pwm.start(0)
    try:
        loop()
    except KeyboardInterrupt:
        IN1_pwm = GPIO.PWM(IN1,500)
        IN3_pwm = GPIO.PWM(IN3,500)
    
        IN1_pwm.start(0)
        IN3_pwm.start(0)
        GPIO.cleanup()
        print("Programa interrompido e GPIO limpo.")
