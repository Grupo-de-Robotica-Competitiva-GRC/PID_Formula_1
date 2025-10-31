/* GRUPO DE ROBÓICA COMPETITVA

LINE FOLLOWER V3 - RED BARON

Membros:
  Kauê Lucas (Capitão)

  # Equipe do Controle #

  Maria Kézia (Sensores)
  Vicente Laio (PID)
  Americo Vitor (Conectividade)

*/

/*  Includes  */

#include <Arduino.h>
#include <QTRSensors.h>
#include "BluetoothSerial.h"
#include "driver/ledc.h"
// #include "Main.h"

/* Bluetooth */

BluetoothSerial SerialBT;
String inputString = "";

/*  Pinouts  */

#define EMITTER_PIN    15
#define LED_CALIBRANDO 2
#define LED_SEGUIDOR   4
#define BOTAO_CALIBRAR 12
#define BOTAO_SEGUIR   13
#define MOTOR_A1       22
#define MOTOR_A2       21
#define MOTOR_B1       19
#define MOTOR_B2       18

/*  Booleans Values  */

bool taskCriada = false;
bool calibrado = false;
bool modoSeguidor = false;
bool calibrando = false;

/*  QTRSensor  */

QTRSensors qtr;
#define numSensors 8
uint8_t  sensorsPin[numSensors] = {34, 35, 32, 33, 27, 26, 25, 14};
uint16_t sensorValues[numSensors];

/*  PID Constants and Variables  */

float Ki = 0; 
int Kp = 21;  
int Kd = 35;  

int P = 0, I = 0, D = 0, PID = 0; // variáveis PID
int16_t velEsq = 0, velDir = 0, erroAnterior = 0; // controle dos motores
int16_t erro = 0;  // cálculo do erro

/*  Motors velocity  */

int bVelo = 700, aVelo =700;

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
