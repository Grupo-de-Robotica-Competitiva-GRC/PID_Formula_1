#include "Main.ino"

// extern float erro_pesso[];  // diz ao compilador que essa variável existe em outro arquivo

void lendoMensagem(String cmd);
void atualiza_variaveis();

void lendoMensagem(String cmd) {
  cmd.trim();

  if (cmd.startsWith("kp=")) {
    Kp = cmd.substring(3).toFloat();
    Serial.print("Novo Kp: "); Serial.println(Kp);
  } else if (cmd.startsWith("ki=")) {
    Ki = cmd.substring(3).toFloat();
    Serial.print("Novo Ki: "); Serial.println(Ki);
  } else if (cmd.startsWith("kd=")) {
    Kd = cmd.substring(3).toFloat();
    Serial.print("Novo Kd: "); Serial.println(Kd);
  } else if (cmd == "mostrar") {  
    SerialBT.printf("Kp=%.3f Ki=%.3f Kd=%.3f\n", Kp, Ki, Kd);
    for (int i = 0; i < numSensors; i++){

      SerialBT.printf(" %d ",erro_pesso[i]);
    }
  } 
  else if(cmd.startsWith("i=")) {
      int i;
      i = cmd[2] - '0';
      erro_pesso[i] = cmd.substring(4).toInt();
    }

  else {
    SerialBT.printf("Comando invalido! \n Os comandos validos são:\n mostrar - Mostra os valores das variaveis;\n ki=<valor> - atualiza o valor de Ki;\n kd=<valor> - atualiza o valor de Kd;\n kp=<valor> - atualiza o valor de Kp");
  }
}

void atualiza_variaveis(){
  while (SerialBT.available()) { 
    char c = SerialBT.read();     //Lê bit a bit
    
    //Esse if é pra quando chegar no fim da mensagem (\n é o Enter)
    if (c == '\n') { 
      lendoMensagem(inputString);
      inputString = "";
    } else {
      inputString += c;
    }
  }
}