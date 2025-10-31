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
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
      vTaskDelay(5 / portTICK_PERIOD_MS);
  }
  // encerra task
  taskCriada = false;  
  vTaskDelete(NULL);
}