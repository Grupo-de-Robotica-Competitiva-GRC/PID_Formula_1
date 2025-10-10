#include <Arduino.h>
#include <QTRSensors.h>
// #include <esp32-hal-lesdc.h>
#include "driver/ledc.h"


#define EMITTER_PIN    15
#define LED_CALIBRANDO 2
#define LED_SEGUIDOR   4
#define BOTAO_CALIBRAR 12
#define BOTAO_SEGUIR   13
#define MOTOR_A1       22
#define MOTOR_A2       21
#define MOTOR_B1       19
#define MOTOR_B2       18


QTRSensors qtr;
bool calibrado = false;
bool modoSeguidor = false;
bool calibrando = false;
uint8_t  sensorsPin[8] = {34, 35, 32, 33, 27, 26, 25, 14};
uint16_t sensorValues[8];
float Ki = 0; // para não acumular tanto erro
int Kp = 38;  //ajustar paramento
int Kd = 60;  //ajusta tamebem algem ve tutorial como ajustar.

int P = 0, I = 0, D = 0, PID = 0; // variáveis PID
int16_t velEsq = 0, velDir = 0, erroAnterior = 0; // controle dos motores
int16_t erro = 0;  // cálculo do erro
int bVelo = 200, aVelo = 200;

void move_motorA(int16_t vel)
{
  vel *= -vel;
  if (vel >= 0)
  {
    ledcWrite(MOTOR_A1, vel);
    ledcWrite(MOTOR_A2, 0);
  }
  else
  {
    vel = abs(vel);
    ledcWrite(MOTOR_A1, 0);
    ledcWrite(MOTOR_A2, vel);
  }

}

void move_motorB(int16_t vel)//vel e duty
{
  vel *= (-vel * 0.44);//o motor B  o da esquerda olhando de fren tem menos torque por isso o fator. 
  if (vel >= 0)
  {
    ledcWrite(MOTOR_B1, vel);
    ledcWrite(MOTOR_B2, 0);
  }
  else
  {
    vel = abs(vel);
    ledcWrite(MOTOR_B1, 0);
    ledcWrite(MOTOR_B2, vel);
  }

}

void pararMotores()
{
  move_motorA(0);
  move_motorB(0);
}

void calculaPID() {
  //if (erro == 0) { I = 0; }
  P = erro;
  I = I + erro; // Acúmulo de erro (somatório)
  if (I > 255) { I = 255; }
  else if (I < -255) { I = -255; }
  D = erro - erroAnterior; // PID discreto
  PID = (Kp * P) + (Ki * I) + (Kd * D);
  erroAnterior = erro;
}


void calcula_erro()
{

  /* americo ou quem for  olahr esse codigo esse array de erro pode/temque ser ajsutado com base em dados empiricos ma sde inicio acho é suficite
   pS: nao descomenta esse demtro*/
  erro = 0;
  // float erro_pesos[8] = {-(1 / 50), -(1 / 70), (-1 / 100), -(1 / 120),  (1 / 120), (1 / 100), (1 / 70), (1 / 50) };
  float erro_pesso[8] = {-20, -16, -9, -2, 2, 9, 16, 20};
  qtr.read(sensorValues);
  for (uint8_t i = 0; i < 8; i++)
  {
    if (sensorValues[i] > 3900)
    {
      erro += erro_pesso[i];  
    }
  }
  Serial.println(erro);
  // for(uint8_t i = 0; i < 8; i++)  
  // {
  //   erro += erro_pesso[i] * sensorValues[i];
  // }
}


void controlaMotor() {
  if (PID >= 0) {
    velEsq = bVelo;
    velDir = aVelo - PID;
  } else {
    velEsq = bVelo + PID;
    velDir = aVelo;
  }

  if(velEsq < 0) velEsq = 0;
  if(velDir < 0) velDir = 0;
  //en teiirua e so colocar os valores de vel em seus respectivos motores

}

void moveCalibrar(void *pvParams) {
  int16_t vel = 512;
  digitalWrite(LED_CALIBRANDO, HIGH);
  for(int i = 0; i <= 200; i++) {
    //move motor A
    move_motorA(500);
    //move motor B
    move_motorB(-500);

    pararMotores();
    vTaskDelay(200 / portTick_PERIOD_MS);

    move_motorA(-500);
    move_motorB(500);
    pararMotores();

    vTaskDelay(200 / portTICK_PERIOD_MS);
  }

  pararMotores();
  digitalWrite(LED_CALIBRANDO, LOW);
  calibrado = true;
  calibrando = false;
  vTaskDelete(NULL);  
}


void calibrar(void *pvParams) {

  for(int i = 0; i <= 200; i++){
       qtr.calibrate();
       vTaskDelay(20 / portTICK_PERIOD_MS);
       Serial.println("iuuuu");
  }
       vTaskDelete(NULL);
}


void seguidor(void *pvParams) {
  digitalWrite(LED_SEGUIDOR, HIGH);
  while (modoSeguidor) {
    qtr.read(sensorValues);
    calcula_erro();
    calculaPID();
    move_motorA(velEsq);
    move_motorB(velDir);
    if (digitalRead(LED_SEGUIDOR) == HIGH) 
    {
      modoSeguidor = false
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  digitalWrite(LED_SEGUIDOR, LOW);
  pararMotores();
  vTaskDelete(NULL);
}

void setup() {  
    Serial.begin(115200);    // GPIO18 no canal 0
    // ledcWrite(18, 128);            // 50% duty cycle4
    qtr.setTypeAnalog();
    qtr.setSensorPins(sensorsPin, 8);
    qtr.setEmitterPin(EMITTER_PIN);
    pinMode(LED_CALIBRANDO, OUTPUT);
    pinMode(LED_SEGUIDOR, OUTPUT);
    pinMode(BOTAO_CALIBRAR, INPUT);
    pinMode(BOTAO_SEGUIR, INPUT);
    // pinMode(EMITTER_PIN, OUTPUT); 
    Serial.println("presine o botao para calibra");
    ledcAttachChannel(MOTOR_A1, 2000, 10, 0);
    ledcAttachChannel(MOTOR_A2, 2000, 10, 1);
    ledcAttachChannel(MOTOR_B1, 2000, 10, 2);
    ledcAttachChannel(MOTOR_B2, 2000, 10, 3);
    // ledcAttachChannel(2, 250, 8, 4);
    // digitalWrite(EMITTER_PIN, HIGH);
    // ledcAttachChannel(15, 250 , 8, 5);
    // ledcWrite(15, 120);


      
  }


  void loop() {

    if (digitalRead(BOTAO_CALIBRAR) == HIGH && !calibrando) {
      calibrando = true;
      calibrado = false;
      
      qtr.read(sensorValues);
      xTaskCreatePinnedToCore(moveCalibrar, "Mover", 4096, NULL, 1, NULL, 0);
      xTaskCreatePinnedToCore(calibrar, "Calibrar", 4096, NULL, 1, NULL, 1);
  

    }
      Serial.println();

 
    // calcula_erro();
    // calculaPID();
    // controlaMotor();
    move_motorA(-500);
    move_motorB(-220);
    delay(1000);
    pararMotores();


    // }
    // for (uint8_t i = 0; i < 255; i++) { Serial.println(i); delay(200); }

    if (digitalRead(BOTAO_SEGUIR) == HIGH && calibrado && !modoSeguidor) {
      modoSeguidor = true;
      xTaskCreatePinnedToCore(seguidor, "Seguidor", 4096, NULL, 1, NULL, 1);
      delay(500);
    }


  }