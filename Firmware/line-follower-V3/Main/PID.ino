#include "Main.ino"

void calculaPID();
void calcula_erro();


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

void transforma_digital()
{
  for (size_t i = 0; i <= numSensors; i++)
  {
    if (sensorValues[i] >= 3900)
    {
      sensorValuesDigital[i] = 1;
    }
    else 
    {
      sensorValuesDigital[i] = 0;
    }
  }
}

void calcula_erro()
{
  erro = 0;
  // float erro_pesos[8] = {-(1 / 50), -(1 / 70), (-1 / 100), -(1 / 120),  (1 / 120), (1 / 100), (1 / 70), (1 / 50) };

  qtr.read(sensorValues);
  for (uint8_t i = 0; i < 8; i++)
  {
    if (sensorValuesDigital[i] == 1)
    {
      erro += erro_pesso[i];  
    }
    qtdSesores_lidos += sensorValuesDigital[i];
  }
  Serial.println(erro);
}