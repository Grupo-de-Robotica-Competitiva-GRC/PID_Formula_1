// #include "Motores.ino"
#include "PID.ino"
#include "Direcao.ino"
extern sensorValuesDigital[8];

void curva_direita(int velA, int velB)
{
  while(sensorValuesDigital[4] != 1)
  {
    move_motorA(velA);
    move_motorB(velB);
    calcula_erro(); 
  }

}