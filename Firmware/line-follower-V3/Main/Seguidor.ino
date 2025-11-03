void seguidor();

extern bool modoSeguidor;
extern uint16_t sensorValues;
extern int velEsq, velDir;

void seguidor() {
  delay(400);
  digitalWrite(LED_SEGUIDOR, HIGH);
  while (modoSeguidor) {
    
    qtr.read(sensorValues);
    calcula_erro();
    calculaPID();
    controlaMotor();
    move_motorA(velEsq);
    move_motorB(velDir);
    Serial.print("Velesq: ");
    Serial.println(velEsq);
    Serial.print("Veldir: ");
    Serial.println(velDir);
    if (digitalRead(BOTAO_SEGUIR) == HIGH) 
    {
      modoSeguidor = false;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  digitalWrite(LED_SEGUIDOR, LOW);
  pararMotores();
  // vTaskDelete(NULL);
}