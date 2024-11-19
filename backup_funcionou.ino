// Definição dos pinos para controle dos motores com a ponte H L298
#define M1_PWM 9   // Pino PWM para o 1º Motor
#define M2_PWM 6   // Pino PWM para o 2º Motor
#define dir1A 10    // Direção do 1º Motor A
#define dir1B 11    // Direção do 1º Motor B
#define dir2A 7     // Direção do 2º Motor A
#define dir2B 8     // Direção do 2º Motor B

// Definição dos pinos dos sensores de linha
#define pin_S1 2   // Sensor 1
#define pin_S2 3   // Sensor 2
#define pin_S3 4   // Sensor 3
#define pin_S4 5   // Sensor 4

//----- CONSTANTES -----
float Ki = 0; // para não acumular tanto erro
int Kp = 35;   // valor padrão para nós
int Kd = 160;   // ir ajustando até ficar bom

int P = 0, I = 0, D = 0, PID = 0; // variáveis PID
int velEsq = 0, velDir = 0, erroAnterior = 0; // controle dos motores
int erro = 0;  // cálculo do erro

// Velocidades padrão/max
int aVelo = 150;
int bVelo = 150;

bool pare;

int sensor_valores[4] = {0,0,0,0};

void setup() {
  // Configuração dos pinos dos motores
  pinMode(M1_PWM, OUTPUT);  // PWM do Motor 1
  pinMode(M2_PWM, OUTPUT);  // PWM do Motor 2
  pinMode(dir1A, OUTPUT);   // Direção do Motor 1 A
  pinMode(dir1B, OUTPUT);   // Direção do Motor 1 B
  pinMode(dir2A, OUTPUT);   // Direção do Motor 2 A
  pinMode(dir2B, OUTPUT);   // Direção do Motor 2 B

  digitalWrite(dir1A, HIGH);
  digitalWrite(dir1B, LOW);
  digitalWrite(dir2A, HIGH);
  digitalWrite(dir2B, LOW);

  // Configura os pinos dos sensores como entrada
  pinMode(pin_S1, INPUT);
  pinMode(pin_S2, INPUT);
  pinMode(pin_S3, INPUT);
  pinMode(pin_S4, INPUT);

  // Inicia o serial para monitoramento
  Serial.begin(9600);
}

void calculaPID() {
  if (erro == 0) { I = 0; }
  P = erro;
  I = I + erro; // Acúmulo de erro (somatório)
  if (I > 255) { I = 255; }
  else if (I < -255) { I = -255; }
  D = erro - erroAnterior; // PID discreto
  PID = (Kp * P) + (Ki * I) + (Kd * D);
  erroAnterior = erro;
}

void calcular_erro(){
  int pesos[4] = {-3, -2, 2, 3};
  int erro_total = 0;
  int sensores_ativos = 0;

  for(int i = 0; i < 4; i++){
    if(sensor_valores[i] == 1){
      erro_total += pesos[i];
      sensores_ativos += 1;
    }
  }

  if(sensores_ativos > 0){
    erro = erro_total / sensores_ativos;
    pare = false;
  }
  else{
    pare = true;
  }
}

void controlaMotor() {
  if (PID >= 0) {
    velEsq = bVelo;
    velDir = aVelo - PID;
  } else {
    velEsq = bVelo + PID;
    velDir = aVelo;
  }

  // Ajuste da velocidade dos motores com os pinos PWM
  analogWrite(M1_PWM, velEsq);  // Ajusta a velocidade do motor 1
  analogWrite(M2_PWM, velDir);  // Ajusta a velocidade do motor 2
}

// Função para ler os sensores de linha
void lerSensores(){
    // Leitura dos sensores
    sensor_valores[0] = digitalRead(pin_S1);
    sensor_valores[1] = digitalRead(pin_S2);
    sensor_valores[2] = digitalRead(pin_S3);
    sensor_valores[3] = digitalRead(pin_S4);
}

void loop() {
  // Lê os valores dos sensores de linha
  lerSensores();
  
  // Calcula o erro com base na posição
  calcular_erro();
  if(pare == false){
    // Calcula o PID para controlar os motores
    calculaPID();

    // Controla os motores com base no PID calculado
    controlaMotor();
  }
  else{
    analogWrite(M1_PWM, 0);  // Ajusta a velocidade do motor 1
    analogWrite(M2_PWM, 0);  // Ajusta a velocidade do motor 2
  }
  
  // Imprime os valores para monitoramento
  Serial.print("Sen: ");
  for(int i = 0; i < 4; i++)
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

  delay(20);
}
