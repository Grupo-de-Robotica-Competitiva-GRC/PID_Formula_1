#include <Arduino.h>
#include <QTRSensors.h>
#include "BluetoothSerial.h"
// #include <esp32-hal-lesdc.h>
#include "driver/ledc.h"

BluetoothSerial SerialBT;
String inputString = "";

#define EMITTER_PIN    15
#define LED_CALIBRANDO 2
#define LED_SEGUIDOR   4
#define BOTAO_CALIBRAR 12
#define BOTAO_SEGUIR   13
#define MOTOR_A1       22
#define MOTOR_A2       21
#define MOTOR_B1       19
#define MOTOR_B2       18

enum class Direcao 
{
  RETO,
  DIREITA,
  ESQUERDA
};


QTRSensors qtr;
Direcao direcao = Direcao::RETO;
bool calibrado = false;
bool modoSeguidor = false;
bool calibrando = false;
uint8_t  sensorsPin[8] = {34, 35, 32, 33, 27, 26, 25, 14};
uint16_t sensorValues[8];
int8_t sensorValuesDigital[8];
int16_t bit_pesos[8] = {1, 2, 4, 8, 16, 32, 64, 128};
int SumSensoresAtivos;
int bitValue = 0;
bool podeParar = false;
float Ki = 0; // para não acumular tanto erro
int Kp = 30;  //ajustar paramento
int Kd = 38;  //ajusta tamebem algem ve tutorial como ajustar.

int P = 0, I = 0, D = 0, PID = 0; // variáveis PID
int16_t velEsq = 0, velDir = 0, erroAnterior = 0; // controle dos motores
int16_t erro = 0;  // cálculo do erro
int bVelo = 400, aVelo = 400;

void move_motorA(int16_t vel)
{

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
  //o motor B  o da esquerda olhando de fren tem menos torque por isso o fator. 
  //TODO tem que fazer um umento de potencia gradual nesse motor  para ele nao sair no supetao para esquerda o torque dele  e alto.
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

// void transforma_digital()
// {
//   for (size_t i = 0; i <= numSensors; i++)
//   {
//     if (sensorValues[i] >= 3900)
//     {
//       sensorValuesDigital[i] = 1;
//     }
//     else 
//     {
//       sensorValuesDigital[i] = 0;
//     }
//   }
// }

void define_direcao()
{
  if (bitValue == 0b11111000 || bitValue == 0b11111100 || bitValue == 0b11110000)
  {
    direcao = Direcao::ESQUERDA;
  }
  else if (bitValue == 0b00011111 || bitValue == 0b00001111 || bitValue == 0b00111111)
  {
    direcao = Direcao::DIREITA;
  }
}

void vira_direita(int16_t velA, int16_t velB)
{
  while (sensorValuesDigital[4] != 1)
  {
    calcula_erro();
    move_motorA(velA);
    move_motorB(-velB);
  }
  direcao = Direcao::RETO;
}

void vira_esquerda(int16_t velA, int16_t velB)
{
  while (sensorValuesDigital[3] != 1)
  {
    calcula_erro();
    move_motorA(-velA);
    move_motorB(velB);
  }
  direcao = Direcao::RETO;
}


void curva_90()
{
  int16_t velA, velB;
  velA = 300; velB = 300;
  if (direcao == Direcao::DIREITA)
  {
    vira_direita(velA, velB);
  }
  else if (direcao == Direcao::ESQUERDA)
  {
    vira_esquerda(velA, velB);
  }
}


void calcula_erro()
{

  /* americo ou quem for  olahr esse codigo esse array de erro pode/temque ser ajsutado com base em dados empiricos ma sde inicio acho é suficite
   pS: nao descomenta esse demtro*/
  erro = 0;
  // float erro_pesos[8] = {-(1 / 50), -(1 / 70), (-1 / 100), -(1 / 120),  (1 / 120), (1 / 100), (1 / 70), (1 / 50) };
  float erro_pesso[8] = {-20, -7, -7, 0, 0, 7, 7, 20};
  qtr.read(sensorValues);
  bitValue = 0;
  SumSensoresAtivos = 0;

  for (uint8_t i = 0; i < 8; i++)
  {
    if (sensorValues[i] >= 3900)
    {
      sensorValuesDigital[i] = 1;
      erro += erro_pesso[i];  
      SumSensoresAtivos += sensorValuesDigital[i];
      bitValue += bit_pesos[i] * sensorValuesDigital[i];
    }
    else
    {
      sensorValuesDigital[i] = 0;
    }
  }
}


void controlaMotor() {
  if (PID >= 0) {
    velEsq = bVelo - PID; // PID;
    velDir = aVelo + PID;
  } else {
    velEsq = bVelo - PID;
    velDir = aVelo + PID;
  }

  // if(velEsq < 0) velEsq = 0;
  // if(velDir < 0) velDir = 0;
  //en teiirua e spararMotores();o colocar os valores de vel em seus respectivos motores

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
    vTaskDelay(200 / portTICK_PERIOD_MS);

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


void calibrar() {

  for(int i = 0; i <= 200; i++){
       qtr.calibrate();
       vTaskDelay(20 / portTICK_PERIOD_MS);
       Serial.println("iuuuu");
  }
}

void lendoMensagem(String cmd) {
  cmd.trim();

  if (cmd.startsWith("kp=")) {
    Kp = cmd.substring(3).toFloat();
    Serial.print("Novo Kp: "); Serial.println(Kp);
  } else if (cmd.startsWith("ki=")) {
    Ki = cmd.substring(3).toFloat();
    Serial.print("Novo Ki: "); Serial.println(Ki);
  } else if (cmd.startsWith("kd=")) {
    Kd = cmd.substring(3).toFloat();
    Serial.print("Novo Kd: "); Serial.println(Kd);
  } else if (cmd == "mostrar") {  
    SerialBT.printf("Kp=%.3f Ki=%.3f Kd=%.3f\n", Kp, Ki, Kd);
  } else {
    SerialBT.printf("Comando invalido! \n Os comandos validos são:\n mostrar - Mostra os valores das variaveis;\n ki=<valor> - atualiza o valor de Ki;\n kd=<valor> - atualiza o valor de Kd;\n kp=<valor> - atualiza o valor de Kp");
  }
}

void atualiza_variaveis(){
  while (SerialBT.available()) { 
    char c = SerialBT.read();     //Lê bit a bit
    
    //Esse if é pra quando chegar no fim da mensagem (\n é o Enter)
    if (c == '\n') { 
      lendoMensagem(inputString);
      inputString = "";
    } else {
      inputString += c;
    }
  }
}

void seguidor() {
  // delay(400);
  digitalWrite(LED_SEGUIDOR, HIGH);
  while (modoSeguidor) {
    
    qtr.read(sensorValues);
    calcula_erro();
    calculaPID();
    controlaMotor();
    move_motorA(velEsq);
    move_motorB(velDir);
    define_direcao();
    if (SumSensoresAtivos == 0)
    {
      delay(30);
      curva_90();
    }
    SerialBT.print("Velesq: ");
    SerialBT.println(velEsq);
    SerialBT.print("Veldir: ");
    SerialBT.println(velDir);
    SerialBT.print("erro ");
    SerialBT.println(erro);
    if (digitalRead(BOTAO_SEGUIR) == HIGH && podeParar) 
    {
      modoSeguidor = false;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
    podeParar = true;
      SerialBT.print("PID ");
      SerialBT.println(PID);
      SerialBT.print("bit_value ");
      SerialBT.println(bitValue, BIN);
      SerialBT.print("sum_sensores ");
      SerialBT.println(SumSensoresAtivos);
      // SerialBT.print("direcao ");
      // if (direcao == Direcao::RETO)
      //   SerialBT.println("reto");
      // else if (direcao == Direcao::ESQUERDA)
      //   SerialBT.println("esquerda");
      // else 
      //   SerialBT.println("direita");
      // for (int i = 0; i < 8; i++)
      // {
      //   SerialBT.printf("%d ", sensorValues[i]);
      // }
      // SerialBT.println(); 
  }
  digitalWrite(LED_SEGUIDOR, LOW);
  pararMotores();
  // vTaskDelete(NULL);
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

    SerialBT.begin("Barão_Vermelho");
    Serial.println("Bluetooth iniciado 'Barão_Vermelho'");
      
  }


  void loop() {
    
    if (digitalRead(BOTAO_CALIBRAR) == HIGH && !calibrando) {
      calibrando = true;
      calibrado = true;

      digitalWrite(LED_CALIBRANDO, HIGH);
      
      qtr.read(sensorValues);
      //xTaskCreatePinnedToCore(moveCalibrar, "Mover", 4096, NULL, 1, NULL, 0);
      //xTaskCreatePinnedToCore(calibrar, "Calibrar", 4096, NULL, 1, NULL, 1);
      calibrar();

      digitalWrite(LED_CALIBRANDO, LOW);
      
      
  

    }
    // qtr.read()
    // for (uint8_t i = 0; i < 8; i++) {Serial.printf("%d ", sensorValues[i]);}
    // delay(200);
    // modoSeguidor = false; 2
    // calibrado = true;
    if (digitalRead(BOTAO_SEGUIR) == HIGH && calibrado && !modoSeguidor) {
      modoSeguidor = true;
      // xTaskCreatePinnedToCore(seguidor, "Seguidor", 4096, NULL, 1, NULL, 1);
      // delay(500);
      seguidor();

    }



    for (int i = 0; i < 8; i++)
    {
      Serial.printf("%d ", sensorValues[i]);
    }
    Serial.println();
    Serial.printf("-------------------------------\n");


    if(SerialBT.available()){
      atualiza_variaveis();
    }
  }
