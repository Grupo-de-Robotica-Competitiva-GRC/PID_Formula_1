//Esta eh uma implementacao de controle PID para o carrinho seguidor de linha 'V3' do GRCFC
//Autor: Luis Ferlim - Engenharia de Computacao - UFC - GRC - 2024

//formula PID: Kp * P + Ki * I + Kd * D
// obs: PID continuo: Kp* e[k] + Ki *sum(e[n]) + Kd *(e[k] - e[k-1]) usado aqui*
#include <QTRSensors.h>

#define LED_VERDE 3
#define LED_VERMELHO 2
#define B_dir 11
#define ENABLE_B 10
#define A_dir 5
#define ENABLE_A 6

//----- CONSTANTES -----
float Ki = 0; //para nao acumularmos tanto erro
int Kp = 35; //valor padrao pra nos
int Kd = 35; //ir ajustando ateh ficar bom

int P =  0,I = 0, D = 0, PID = 0; //inicializando as variaveis que vao sofrer variacao conforme o erro
int velEsq = 0, velDir = 0, erroAnterior = 0; //estas sao as variaveis que irao fazer os motores PWM se moverem de fato
int erro = 0;  //calculo do erro

//variaveis para a conexao bluetooth
uint8_t val; 
uint8_t cnt = 0;
float v[3];

//cria um objeto QTR para usarmos os métodos da biblioteca
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

//velocidades padrao/max
int aVelo = 210;
int bVelo = 210;

void setup() {

  //setar os pinos do sensor
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);

  //setar os pinos do motor
  pinMode(ENABLE_B, OUTPUT);
  pinMode(B_dir, OUTPUT);
  pinMode(ENABLE_A, OUTPUT);
  pinMode(A_dir, OUTPUT);

  //leds
  pinMode(LED_VERDE, OUTPUT);
  pinMode(LED_VERMELHO, OUTPUT);

  //liga o led para entendermos que ele está calibrando
  delay(500);
  digitalWrite(LED_VERMELHO, HIGH);

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 8 sensors
  // * 10 reads per calibrate() call = ~32 ms per calibrate() call.
  // Call calibrate() 313 times to make calibration take about 10 seconds.
  Serial.begin(9600);
  for (uint16_t i = 0; i < 313; i++)
  {
    qtr.calibrate();
  }
  
  digitalWrite(LED_VERMELHO, LOW);
  digitalWrite(LED_VERDE, HIGH);

  //definido a rotação dos motores 
  analogWrite(B_dir,0); // o modulo da diferença dos dois da a velocidade no teste empirico, o que tiver numero maior tem sua direcao
  analogWrite(A_dir,0);

  delay(1000);
  //inicia o serial para observamos os valores do sensor
}

void calculaPID(){

  if (erro == 0) {I = 0;}
  P = erro;
  I = I + erro; // acumulo de erro (somatorio)
  if (I > 255) { I = 255;}
  else if (I < -255) { I = -255;}
  D = erro - erroAnterior; //PID discreto
  PID = (Kp * P) + (Ki * I) + (Kd * D);
  erroAnterior = erro;
}

//talvez precisemos de um map, verificar a rapidez de execucao
void calculaErro(int position){
  if (position < 100) erro = - 6;
  else if (position < 500) erro = - 5;
  else if (position < 1600) erro  = - 4;
  else if (position < 2100) erro  = - 3;  
  else if (position < 3000) erro  = - 2;
  else if (position < 3500) erro  = - 1;
  else if (position < 4010) erro = 0;
  else if (position < 4500) erro  =  1;
  else if (position < 5050) erro  =  2;
  else if (position < 5500) erro  =  3;
  else if (position < 6000) erro  =  4;
  else if (position < 6500) erro  =  5;
  else if (position <= 7000) erro  =  6;
}

void controlaMotor(){
  if (PID >= 0)
  {
    velEsq = bVelo;
    velDir = aVelo - PID;
  }
  else
  {
    velEsq = bVelo + PID;
    velDir = aVelo;
  }

  analogWrite(ENABLE_A, velEsq);
  analogWrite(ENABLE_B, velDir);
}

//a seguinte funcao le as duas informacoes passadas por bluetooth pelo nosso app
void ler_serial(){
  val = Serial.read();
  cnt++;
  v[cnt] = val;
  if (cnt == 2)

    cnt = 0;
}

//a seguinte funcao identifica qual o id passado pelo bluetooth e logo apos aplica na constante k 'respectiva'
void processa_serial() {
  int a = v[1]; // 1o bit lido, valor
  if (a == 1) { //2o bit lido, variavel
    Kp = v[2]; 
  }

  if (a == 2) {
    Kd = v[2];
  }
  if (a == 3) {
    Ki = v[2]/200;

  }
}

void loop() {

  // atualizacao de Kp,Kd e Ki via bluetooth
   if (Serial.available())
  {
    while(Serial.available() == 0);
    ler_serial();
    processa_serial();
  }

  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 7000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);
  calculaErro(position);
  calculaPID();
  controlaMotor();

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }

  //printa os valores da posicao e das constantes
  Serial.print(position);
  Serial.print('\t');
  Serial.print(velEsq);
  Serial.print('\t');
  Serial.print(velDir);
  Serial.print('\t');
  Serial.print(Kp);
  Serial.print('\t');
  Serial.print(Kd);
  Serial.print('\t');
  Serial.println(Ki);

  delay(20);

}
