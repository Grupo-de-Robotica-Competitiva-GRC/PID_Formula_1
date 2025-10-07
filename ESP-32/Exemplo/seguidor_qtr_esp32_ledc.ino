#include <Arduino.h>
#include <QTRSensors.h>

// ====== PINAGEM (ajuste conforme seu hardware) ======
#define LED_CALIBRANDO 2
#define LED_SEGUIDOR   4
#define BOTAO_CALIBRAR 34
#define BOTAO_SEGUIR   35

#define MOTOR_A1 25
#define MOTOR_A2 26
#define MOTOR_B1 27
#define MOTOR_B2 14

#define emitterPin 15
uint8_t  sensorsPin[8] = {32, 33, 34, 35, 36, 39, 38, 37};
uint16_t sensorValues[8];

QTRSensors qtr;


// ====== CONFIGURAÇÃO LEDC ======
#define LEDC_TIMER_BIT     8
#define LEDC_BASE_FREQ     5000
#define LEDC_TIMER         LEDC_TIMER_0
#define LEDC_MODE          LEDC_HIGH_SPEED_MODE

ledc_channel_config_t motorA_ch1 = { .gpio_num = MOTOR_A1, .speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_0, .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER, .duty = 0, .hpoint = 0 };
ledc_channel_config_t motorA_ch2 = { .gpio_num = MOTOR_A2, .speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_1, .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER, .duty = 0, .hpoint = 0 };
ledc_channel_config_t motorB_ch1 = { .gpio_num = MOTOR_B1, .speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_2, .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER, .duty = 0, .hpoint = 0 };
ledc_channel_config_t motorB_ch2 = { .gpio_num = MOTOR_B2, .speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_3, .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER, .duty = 0, .hpoint = 0 };

bool calibrado = false;
bool modoSeguidor = false;
bool calibrando = false;

void motorEsquerdo(int vel) {
  if (vel > 0) {
    ledc_set_duty(LEDC_MODE, motorA_ch1.channel, vel);
    ledc_update_duty(LEDC_MODE, motorA_ch1.channel);
    ledc_set_duty(LEDC_MODE, motorA_ch2.channel, 0);
    ledc_update_duty(LEDC_MODE, motorA_ch2.channel);
  } else {
    vel = -vel;
    ledc_set_duty(LEDC_MODE, motorA_ch1.channel, 0);
    ledc_update_duty(LEDC_MODE, motorA_ch1.channel);
    ledc_set_duty(LEDC_MODE, motorA_ch2.channel, vel);
    ledc_update_duty(LEDC_MODE, motorA_ch2.channel);
  }
}

void motorDireito(int vel) {
  if (vel > 0) {
    ledc_set_duty(LEDC_MODE, motorB_ch1.channel, vel);
    ledc_update_duty(LEDC_MODE, motorB_ch1.channel);
    ledc_set_duty(LEDC_MODE, motorB_ch2.channel, 0);
    ledc_update_duty(LEDC_MODE, motorB_ch2.channel);
  } else {
    vel = -vel;
    ledc_set_duty(LEDC_MODE, motorB_ch1.channel, 0);
    ledc_update_duty(LEDC_MODE, motorB_ch1.channel);
    ledc_set_duty(LEDC_MODE, motorB_ch2.channel, vel);
    ledc_update_duty(LEDC_MODE, motorB_ch2.channel);
  }
}

void pararMotores() {
  motorEsquerdo(0);
  motorDireito(0);
}

void calibrar(void *pvParams) {
  digitalWrite(LED_CALIBRANDO, HIGH);
  for (int i = 0; i < 250; i++) {
    motorEsquerdo(100);
    motorDireito(-100);
    qtr.calibrate();
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
  pararMotores();
  digitalWrite(LED_CALIBRANDO, LOW);
  calibrado = true;
  calibrando = false;
  vTaskDelete(NULL);
}

void seguidor(void *pvParams) {
  digitalWrite(LED_SEGUIDOR, HIGH);
  while (modoSeguidor) {
    qtr.read(sensorValues);
    unsigned int pos = qtr.readLineBlack(sensorValues);
    int erro = ((int)pos) - 3500;
    int velBase = 200;
    int Kp = 0.05 * erro;
    motorEsquerdo(velBase + Kp);
    motorDireito(velBase - Kp);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  digitalWrite(LED_SEGUIDOR, LOW);
  pararMotores();
  vTaskDelete(NULL);
}

void setup() {
  qtr.setTypeAnalog();
  qtr.setSensorPins(sensorsPin, 8);
  qtr.setEmitterPin(emitterPin);

  Serial.begin(115200);
  pinMode(LED_CALIBRANDO, OUTPUT);
  pinMode(LED_SEGUIDOR, OUTPUT);
  pinMode(BOTAO_CALIBRAR, INPUT);
  pinMode(BOTAO_SEGUIR, INPUT);

  ledc_timer_config_t ledc_timer = {
      .speed_mode = LEDC_MODE,
      .duty_resolution = LEDC_TIMER_BIT,
      .timer_num = LEDC_TIMER,
      .freq_hz = LEDC_BASE_FREQ,
      .clk_cfg = LEDC_AUTO_CLK};
  ledc_timer_config(&ledc_timer);

  ledc_channel_config(&motorA_ch1);
  ledc_channel_config(&motorA_ch2);
  ledc_channel_config(&motorB_ch1);
  ledc_channel_config(&motorB_ch2);

  Serial.println("Sistema pronto. Pressione o botão para calibrar.");
}

void loop() {
  if (digitalRead(BOTAO_CALIBRAR) == HIGH && !calibrando) {
    calibrando = true;
    calibrado = false;
    xTaskCreatePinnedToCore(calibrar, "Calibrar", 4096, NULL, 1, NULL, 0);
    delay(500);
  }

  if (digitalRead(BOTAO_SEGUIR) == HIGH && calibrado && !modoSeguidor) {
    modoSeguidor = true;
    xTaskCreatePinnedToCore(seguidor, "Seguidor", 4096, NULL, 1, NULL, 1);
    delay(500);
  }
}
