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

/* Bluetooth */
luetoothSerial SerialBT;
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

/*  QTRSensor  */

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
