// Definição dos pinos para controle dos motores com a ponte H L298
#define M1_PWM 9   // Pino PWM para o 1º Motor
#define M2_PWM 6   // Pino PWM para o 2º Motor

// Definição dos pinos dos sensores de linha
#define pin_S1 7   // Sensor 1
#define pin_S2 2   // Sensor 2
#define pin_S3 3   // Sensor 3
#define pin_S4 4   // Sensor 4
#define pin_S5 5   // Sensor 4
#define pin_S6 12   // Sensor 4

//----- CONSTANTES -----
float Ki = 0; // para não acumular tanto erro
int Kp = 50;   // valor padrão para nós
int Kd = 98;   // ir ajustando até ficar bom

int P = 0, I = 0, D = 0, PID = 0; // variáveis PID
int velEsq = 0, velDir = 0, erroAnterior = 0; // controle dos motores
int erro = 0;  // cálculo do erro

// Velocidades padrão/max
int aVelo = 250;
int bVelo = 250;
int velocidadeBase = 150;
int sensores_ativos = 0;

int sensor_valores[6] = {0,0,0,0,0,0};

void setup() {
  // Configuração dos pinos dos motores
  pinMode(M1_PWM, OUTPUT);  // PWM do Motor 1
  pinMode(M2_PWM, OUTPUT);  // PWM do Motor 2

  // Configura os pinos dos sensores como entrada
  pinMode(pin_S1, INPUT);
  pinMode(pin_S2, INPUT);
  pinMode(pin_S3, INPUT);
  pinMode(pin_S4, INPUT);
  pinMode(pin_S5, INPUT);
  pinMode(pin_S6, INPUT);

  // Inicia o serial para monitoramento
  Serial.begin(9600);
}

void calculaPID() {
  //if (erro == 0) { I = 0; }
  P = erro;
  I = I + erro; // Acúmulo de erro (somatório)
  //if (I > 255) { I = 255; }
  //else if (I < -255) { I = -255; }
  D = erro - erroAnterior; // PID discreto
  PID = (Kp * P) + (Ki * I) + (Kd * D);
  erroAnterior = erro;
}

void calcular_erro(){
  erro = 0;
  int pesos[6] = {-10, -4, -1, 1, 4, 10};
  int erro_total = 0;
  sensores_ativos = 0;

  for(int i = 0; i < 6; i++){
    if(sensor_valores[i] == 1){
      erro_total += pesos[i];
      sensores_ativos += 1;
    }
  }

  if(sensores_ativos > 0)
    //erro = erro_total / sensores_ativos;
    erro = erro_total;
  else
    erro = erroAnterior;
}
/*
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
  // Ajuste da velocidade dos motores com os pinos PWM
  analogWrite(M1_PWM, velEsq);  // Ajusta a velocidade do motor 1
  analogWrite(M2_PWM, velDir);  // Ajusta a velocidade do motor 2
}*/

void controlaMotor() {
  velEsq = velocidadeBase + PID;
  velDir = velocidadeBase - PID;

  // Limitar os valores de velocidade entre 0 e 255
  velEsq = constrain(velEsq, 0, 255);
  velDir = constrain(velDir, 0, 255);

  // Ajustar a velocidade dos motores
  analogWrite(M1_PWM, velEsq);
  analogWrite(M2_PWM, velDir);
}

// Função para ler os sensores de linha
void lerSensores(){
    // Leitura dos sensores
    sensor_valores[0] = digitalRead(pin_S1);
    sensor_valores[1] = digitalRead(pin_S2);
    sensor_valores[2] = digitalRead(pin_S3);
    sensor_valores[3] = digitalRead(pin_S4);
    sensor_valores[4] = digitalRead(pin_S5);
    sensor_valores[5] = digitalRead(pin_S6);
}

void loop() {
  // Lê os valores dos sensores de linha
  lerSensores();
  
  // Calcula o erro com base na posição
  calcular_erro();

  // Calcula o PID para controlar os motores
  calculaPID();

  // Controla os motores com base no PID calculado
  controlaMotor();

  
  // Imprime os valores para monitoramento
  Serial.print(erro);
  Serial.print("Sen: ");
  for(int i = 0; i < 6; i++)
    Serial.print(sensor_valores[i]);
  Serial.print("\tEsq: ");
  Serial.print(velEsq);
  Serial.print("\tDir: ");
  Serial.print(velDir);
  Serial.print("\tKp: ");
  Serial.print(Kp);
  Serial.print("\tKd: ");
  Serial.print(Kd);
  Serial.print("\tKi: ");
  Serial.println(Ki);


  if((erro < -14 || erro >= 14) && sensores_ativos>=4){
    delay(300);
  }else{
    delay(20);
  }
}