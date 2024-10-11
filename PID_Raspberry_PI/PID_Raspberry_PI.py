import RPi.GPIO as GPIO
import time

# Definição dos pinos
LED_VERDE = 3
LED_VERMELHO = 2
B_DIR = 11
ENABLE_B = 10
A_DIR = 6
ENABLE_A = 5

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

# Configuração de PWM
pwm_A = GPIO.PWM(ENABLE_A, 1000)  # Frequência de 1000 Hz
pwm_B = GPIO.PWM(ENABLE_B, 1000)  # Frequência de 1000 Hz

pwm_A.start(0)
pwm_B.start(0)

# Configuração dos sensores
sensor_pins = [5, 6, 13, 19, 26]

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

    pwm_A.ChangeDutyCycle(velEsq / 2.55)  # RPi.GPIO PWM usa valores de 0 a 100
    pwm_B.ChangeDutyCycle(velDir / 2.55)  # Converte 0-255 para 0-100


def lerSensores():
    # Cria uma lista para armazenar os valores lidos de cada sensor
    sensor_values = [0] * 5
    for i in range(5):
        # Lê o valor de cada pino (0 ou 1)
        sensor_values[i] = GPIO.input(sensor_pins[i])
    
    if (sensor_values[0] == 1) and (sensor_values[1] == 1) and (sensor_values[2] == 1) and (sensor_values[3] == 0) and (sensor_values[4] == 1) and (sensor_values[5] == 1) and (sensor_values[6] == 1):
        erro = 0
    elif (sensor_values[0] == 1) and (sensor_values[1] == 1) and (sensor_values[2] == 1) and (sensor_values[3] == 0) and (sensor_values[4] == 0) and (sensor_values[5] == 1) and (sensor_values[6] == 1):
        erro = 1
    elif (sensor_values[0] == 1) and (sensor_values[1] == 1) and (sensor_values[2] == 0) and (sensor_values[3] == 0) and (sensor_values[4] == 1) and (sensor_values[5] == 1) and (sensor_values[6] == 1):
        erro = -1
    elif (sensor_values[0] == 1) and (sensor_values[1] == 1) and (sensor_values[2] == 1) and (sensor_values[3] == 1) and (sensor_values[4] == 0) and (sensor_values[5] == 1) and (sensor_values[6] == 1):
        erro = 1.5
    elif (sensor_values[0] == 1) and (sensor_values[1] == 1) and (sensor_values[2] == 0) and (sensor_values[3] == 1) and (sensor_values[4] == 1) and (sensor_values[5] == 1) and (sensor_values[6] == 1):
        erro = -1.5
    elif (sensor_values[0] == 1) and (sensor_values[1] == 1) and (sensor_values[2] == 1) and (sensor_values[3] == 1) and (sensor_values[4] == 0) and (sensor_values[5] == 0) and (sensor_values[6] == 1):
        erro = 2
    elif (sensor_values[0] == 1) and (sensor_values[1] == 1) and (sensor_values[2] == 0) and (sensor_values[3] == 1) and (sensor_values[4] == 1) and (sensor_values[5] == 1) and (sensor_values[6] == 1):
        erro = -2
    elif (sensor_values[0] == 1) and (sensor_values[1] == 1) and (sensor_values[2] == 1) and (sensor_values[3] == 1) and (sensor_values[4] == 1) and (sensor_values[5] == 0) and (sensor_values[6] == 1):
        erro = 2.5
    elif (sensor_values[0] == 1) and (sensor_values[1] == 0) and (sensor_values[2] == 1) and (sensor_values[3] == 1) and (sensor_values[4] == 1) and (sensor_values[5] == 1) and (sensor_values[6] == 1):
        erro = -2.5


def setup():
    # Configuração do RPi.GPIO
    GPIO.setmode(GPIO.BOARD)

    GPIO.setup(LED_VERDE, GPIO.OUT)
    GPIO.setup(LED_VERMELHO, GPIO.OUT)
    GPIO.setup(B_DIR, GPIO.OUT)
    GPIO.setup(ENABLE_B, GPIO.OUT)
    GPIO.setup(A_DIR, GPIO.OUT)
    GPIO.setup(ENABLE_A, GPIO.OUT)

    # Configuração dos sensores 
    for pin in sensor_pins:
        GPIO.setup(pin, GPIO.IN)

    # Inicializa LEDs
    GPIO.output(LED_VERMELHO, GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(LED_VERMELHO, GPIO.LOW)
    GPIO.output(LED_VERDE, GPIO.HIGH)

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
        # Limpeza dos GPIOs
        pwm_A.stop()
        pwm_B.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    setup()
    try:
        loop()
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Programa interrompido e GPIO limpo.")
