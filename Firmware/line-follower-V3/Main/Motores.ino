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

void controlaMotor() {
  if (PID >= 0) {
    velEsq = bVelo - PID;
    velDir = aVelo  + PID;
  } else {
    velEsq = bVelo - PID;
    velDir = aVelo + PID;
  }

}
