void moveCalibrar(void *pvParams) {
  int16_t vel = 512;
  digitalWrite(LED_CALIBRANDO, HIGH);
  for(int i = 0; i <= 200; i++) {
    //move motor A
    move_motorA(500);
    //move motor B
    move_motorB(-500);

    pararMotores();
    vTaskDelay(200 / portTICK_PERIOD_MS);

    move_motorA(-500);
    move_motorB(500);
    pararMotores();

    vTaskDelay(200 / portTICK_PERIOD_MS);
  }

  pararMotores();
  digitalWrite(LED_CALIBRANDO, LOW);
  calibrado = true;
  calibrando = false;
  vTaskDelete(NULL);  
}


void calibrar(void *pvParams) {

  for(int i = 0; i <= 200; i++){
       qtr.calibrate();
       vTaskDelay(20 / portTICK_PERIOD_MS);
       Serial.println("iuuuu");
  }
}
