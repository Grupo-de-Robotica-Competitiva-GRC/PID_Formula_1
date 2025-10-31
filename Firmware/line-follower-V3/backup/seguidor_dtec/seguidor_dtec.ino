//BIBLIOTECAS
#include <Arduino.h>
#include <QTRSensors.h>
#include "BluetoothSerial.h"
#include "driver/ledc.h"

//BLUETOOTH
BluetoothSerial SerialBT;
String inputString = "";

//PINOS
#define EMITTER_PIN    15
#define LED_CALIBRANDO 2
#define LED_SEGUIDOR   4
#define BOTAO_CALIBRAR 12
#define BOTAO_SEGUIR   13
#define MOTOR_A1       22
#define MOTOR_A2       21
#define MOTOR_B1       18
#define MOTOR_B2       19


QTRSensors qtr;
bool taskCriada = false;
bool calibrado = false;
bool modoSeguidor = false;
bool calibrando = false;
uint8_t  sensorsPin[8] = {34, 35, 32, 33, 27, 26, 25, 14};
uint16_t sensorValues[8];
float Ki = 0; 
int Kp = 21;  
int Kd = 35;  

int P = 0, I = 0, D = 0, PID = 0; // variáveis PID
int16_t velEsq = 0, velDir = 0, erroAnterior = 0; // controle dos motores
int16_t erro = 0;  // cálculo do erro
int bVelo = 700, aVelo =700;


//================FUNÇÕES PARA MOVIMENTAR OS MOTORES================
void move_motorA(int16_t vel)
{
  vel *= 0.65;
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

//================CALCULO DO PID================
void calculaPID() {
  //if (erro == 0) { I = 0; }
  I = I + erro; // Acúmulo de erro (somatório)
  if (I > 255) { I = 255; }
  else if (I < -255) { I = -255; }
  
  P = erro;
  D = erro - erroAnterior; // PID discreto
  PID = (Kp * P) + (Ki * I) + (Kd * D);
  erroAnterior = erro;
}


void calcula_erro()
{
  erro = 0;
  // float erro_pesos[8] = {-(1 / 50), -(1 / 70), (-1 / 100), -(1 / 120),  (1 / 120), (1 / 100), (1 / 70), (1 / 50) };
  
  float erro_pesso[8] = {-100, -16, -7, 0, 0, 7, 16, 100};
  qtr.read(sensorValues);
  for (uint8_t i = 0; i < 8; i++)
  {
    if (sensorValues[i] > 3900)
    {
      erro += erro_pesso[i];  
    }
  }
  Serial.println(erro);
}


void controlaMotor() {
  if (PID >= 0) {
    velEsq = bVelo - PID;
    velDir = aVelo  + PID;
  } else {
    velEsq = bVelo - PID;
    velDir = aVelo + PID;
  }

}

//================CALIBRAÇÃO================
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


void calibrar(void *pvParams) {

  for(int i = 0; i <= 200; i++){
       qtr.calibrate();
       vTaskDelay(20 / portTICK_PERIOD_MS);
       Serial.println("iuuuu");
  }
}


//================COMUNICAÇÃO BLUETOOTH================
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


//================SEGUIDOR================
void seguidor() {
  delay(400);
  digitalWrite(LED_SEGUIDOR, HIGH);
  while (modoSeguidor) {
    
    qtr.read(sensorValues);
    calcula_erro();
    calculaPID();
    controlaMotor();
    move_motorA(velEsq);
    move_motorB(velDir);
    Serial.print("Velesq: ");
    Serial.println(velEsq);
    Serial.print("Veldir: ");
    Serial.println(velDir);
    if (digitalRead(BOTAO_SEGUIR) == HIGH) 
    {
      modoSeguidor = false;
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
      vTaskDelay(5 / portTICK_PERIOD_MS);
  }
  // encerra task
  taskCriada = false;  
  vTaskDelete(NULL);
}

void setup() {  
    Serial.begin(115200);    // GPIO18 no canal 0

    qtr.setTypeAnalog();
    qtr.setSensorPins(sensorsPin, 8);
    qtr.setEmitterPin(EMITTER_PIN);
    pinMode(LED_CALIBRANDO, OUTPUT);
    pinMode(LED_SEGUIDOR, OUTPUT);
    pinMode(BOTAO_CALIBRAR, INPUT);
    pinMode(BOTAO_SEGUIR, INPUT);
   
    ledcAttachChannel(MOTOR_A1, 2000, 10, 0);
    ledcAttachChannel(MOTOR_A2, 2000, 10, 1);
    ledcAttachChannel(MOTOR_B1, 2000, 10, 2);
    ledcAttachChannel(MOTOR_B2, 2000, 10, 3);

    SerialBT.begin("Barão_Vermelho");
    Serial.println("Bluetooth iniciado 'Barão_Vermelho'");
      
  }


  void loop() {
    
    if (digitalRead(BOTAO_CALIBRAR) == HIGH && !calibrando) {
      calibrando = true;
      calibrado = false;

      digitalWrite(LED_CALIBRANDO, HIGH);
      
      qtr.read(sensorValues);
      xTaskCreatePinnedToCore(moveCalibrar, "Mover", 4096, NULL, 1, NULL, 1);
      xTaskCreatePinnedToCore(calibrar, "Calibrar", 4096, NULL, 1, NULL, 0);
      // calibrar();
      // for(int i=0; i<=400; i++){
      //   moveCalibrar(TaskParameters_t);
      //   calibrar();
      //   vTaskDelay(10 / portTICK_PERIOD_MS);
      // }

      digitalWrite(LED_CALIBRANDO, LOW);
      
      
  

    }
    // qtr.read()
    // for (uint8_t i = 0; i < 8; i++) {Serial.printf("%d ", sensorValues[i]);}
    // delay(200);
    calibrado = true;
    // bool taskCriada = false;  // controle para não criar a task várias vezes
    if (digitalRead(BOTAO_SEGUIR) == HIGH && calibrado && !taskCriada) {
      modoSeguidor = true;
      taskCriada = true;
      // xTaskCreatePinnedToCore(seguidor, "Seguidor", 4096, NULL, 1, NULL, 0);
      seguidor();
      delay(300);
    }


    if(SerialBT.available()){
      atualiza_variaveis();
    }
  }
