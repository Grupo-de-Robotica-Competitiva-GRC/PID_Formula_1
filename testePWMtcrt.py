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
velo = 255
aVelo = velo
bVelo = velo 


# Configuração dos sensores
sensor_pins = [5,6,16,17,22,23,24,25]

def calculaPID():
    global P, I, D, PID, erro, erroAnterior
    global velo
    if erro == 0:
        I = 0
    P = erro
    I += erro
    if I > velo:
        I = velo
    elif I < -velo:
        I = -velo
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
    
    IN1_pwm.ChangeDutyCycle(velEsq / 2.55)  # RPi.GPIO PWM usa valores de 0 a 100
    IN3_pwm.ChangeDutyCycle(velDir / 2.55)  # Converte 0-255 para 0-100

def calcular_erro(sensor_values):
    erros = {
        (0, 0, 0, 1, 1, 0, 0, 0): 0,
        (0, 0, 0, 1, 0, 0, 0, 0): -0.5,
        (0, 0, 1, 1, 0, 0, 0, 0): -1,
        (0, 0, 1, 0, 0, 0, 0, 0): -1.5,
        (0, 1, 1, 0, 0, 0, 0, 0): -2,
        (0, 1, 0, 0, 0, 0, 0, 0): -2.5,
        (1, 1, 0, 0, 0, 0, 0, 0): -3,
        (1, 0, 0, 0, 0, 0, 0, 0): -3.5,
        (0, 0, 0, 0, 1, 0, 0, 0): 0.5,
        (0, 0, 0, 0, 1, 1, 0, 0): 1,
        (0, 0, 0, 0, 0, 1, 0, 0): 1.5,
        (0, 0, 0, 0, 0, 1, 1, 0): 2,
        (0, 0, 0, 0, 0, 0, 1, 0): 2.5,
        (0, 0, 0, 0, 0, 0, 1, 1): 3,
        (0, 0, 0, 0, 0, 0, 0, 1): 3.5,
    }
    return erros.get(tuple(sensor_values), 0) # Talvez trocar esse none por algum valor

def lerSensores():
    global erro
    # Cria uma lista para armazenar os valores lidos de cada sensor
    sensor_values = [0] * 8
    for i in range(8):
        # Lê o valor de cada pino (0 ou 1)
        sensor_values[i] = GPIO.input(sensor_pins[i])
    
    erro = calcular_erro(sensor_values)
    print(erro)
    #print(sensor_values,erro,sep='\t')


def setup():
    # Configuração do RPi.GPIO
    GPIO.setmode(GPIO.BCM)
    
    GPIO.setup(IN1,GPIO.OUT)
    GPIO.setup(IN3,GPIO.OUT)

    # Configuração dos sensores 
    for pin in sensor_pins:
        if pin not in [5,6,24,25]:
            GPIO.setup(pin, GPIO.IN)
        else: GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    

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
        GPIO.cleanup();

if __name__ == '__main__':
    setup()
    
    # Configuração de PWM freq 1kHz
    IN1_pwm = GPIO.PWM(IN1,16000)
    IN3_pwm = GPIO.PWM(IN3,16000)
    
    IN1_pwm.start(0)
    IN3_pwm.start(0)
    try:
        loop()
    except KeyboardInterrupt:
        IN1_pwm = GPIO.PWM(IN1,16000)
        IN3_pwm = GPIO.PWM(IN3,16000)
    
        IN1_pwm.start(0)
        IN3_pwm.start(0)
        GPIO.cleanup()
        print("Programa interrompido e GPIO limpo.")
    finally:
        IN1_pwm.ChangeDutyCycle(0) # this prevents jitter
        IN3_pwm.ChangeDutyCycle(0)
        
        #parando pwm
        IN1_pwm.stop()
        IN3_pwm.stop()
        
        # Limpeza dos GPIOs
        GPIO.cleanup();
