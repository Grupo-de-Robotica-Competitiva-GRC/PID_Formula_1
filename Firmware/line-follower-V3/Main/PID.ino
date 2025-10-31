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
