#include <PID_v2.h>
#define TRIGGER   2 //pino de trigger do sensor
#define ECHO      3 //pino de echo do sensor
#define VEL_SOMu  0.000340 //velocidade do som para calculos
#define M1F  5 //pino de tensão direta no motor 1
#define M2F  6 //pino de tensão direta no motor 2
#define M1R  9 //pino de tensão reversa no motor 1
#define M2R  10 //pino de tensão reversa no motor 2
#define TAM_FILTRO  10 //tamanho do vetor de filtro

//     tensão aplicada nos  tensão aplicada nos  distancia  distancia
//     motores para avanço  motores para ré      calculada  desejada
double tensao_saidaf = 100, tensao_saidar = 100, distancia, setpoint = 0.1,

//coeficientes: proporcional integral   derivativo  vetor de medições   soma p/ calculo  
                Kp = 3,      Ki = 0.75, Kd = 3,     filtro[TAM_FILTRO], soma; //do filtro

char parametro;         //auxiliar para
long cont = 0;          //comunicação serial

//objeto que guardará as informações do PID de avanço
PID pid(&distancia, &tensao_saidaf, &setpoint, Kp, Ki, Kd, DIRECT);

//objeto que guardará as informações do PID de ré
PID pidr(&distancia, &tensao_saidar, &setpoint, Kp, Ki, Kd, REVERSE);

void setup() {
  pinMode(TRIGGER, OUTPUT);
  digitalWrite(TRIGGER, LOW);
  pinMode(ECHO, INPUT);
  pinMode(M1F, OUTPUT);  
  pinMode(M2F, OUTPUT);
  pinMode(M1R, OUTPUT);  
  pinMode(M2R, OUTPUT);
  Serial.begin(115200);
  
  pid.SetMode(AUTOMATIC);   //inicia
  pidr.SetMode(AUTOMATIC);  //os PID
}

void loop() {

  if(Serial.available() > 0) parametro = Serial.read();       //guarda o primeiro byte da serial
                                                              //em "parametro".
  if(parametro == 'p')        Kp = Serial.parseFloat();       //de acordo com o caracter recebido,
  else  if(parametro == 'i')  Ki = Serial.parseFloat();       //atualiza as variáveis dos
  else  if(parametro == 'd')  Kd = Serial.parseFloat();       //coeficientes ou do setpoint
  else  if(parametro == 's')  setpoint = Serial.parseFloat(); //do sistema
  
  pid.SetTunings(Kp, Ki, Kd);   //atualiza os
  pidr.SetTunings(Kp, Ki, Kd);  //coeficientes


  for(int i = TAM_FILTRO - 2; i >= 0 ; i --)                    //passa cada item do filtro para a
    filtro[i] = filtro[i + 1];                                  //posição anterior, apagando o mais antigo
                                                                //
  digitalWrite(TRIGGER, HIGH);                                  //envia uma onda ultrassônica
  delayMicroseconds(10);                                        //e, quando recebe o pulso de ECHO
  digitalWrite(TRIGGER, LOW);                                   //armazena a distancia calculada na ultima
  filtro[TAM_FILTRO - 1] = pulseIn(ECHO, HIGH) * VEL_SOMu / 2;  //posição do vetor de filtro
                                                                //
  soma = 0;                                                     //então, a média do vetor é
  for(int i = 0; i < TAM_FILTRO; i ++)                          //calculada e enviada para a
    soma += filtro[i];                                          //variável distancia
                                                                //
  distancia = soma / TAM_FILTRO;                                //
  
  
  pid.Compute();  //calcula os novos valores de tensao_saida, com base nos coeficientes e na
  pidr.Compute(); //distancia calculada. saídas, entradas e setpoint estão ligados ao algoritmo
                  //pelos ponteiros para seus endereços, passados na inicialização da instancia.
                  //já os coeficientes foram atualizados no método SetTunings

  if(cont > 1000) {                                                     //
    Serial.println("Kp    Ki    Kd    set   distancia   tensao_saida"); //
    Serial.print(Kp); Serial.print("  ");                               //a cada 1000 ciclos da void loop
    Serial.print(Ki); Serial.print("  ");                               //as informações sobre o sistema
    Serial.print(Kd); Serial.print("  ");                               //são enviadas ao monitor serial.
    Serial.print(setpoint); Serial.print("  ");                         //aqui, o objetivo é somente
    Serial.print(distancia); Serial.print("        ");                  //acompanhar os dados do sistema
    Serial.println(tensao_saidaf);                                      //
    cont = 0;                                                           //
  } cont ++;                                                            //

  if(distancia < setpoint - 0.02) {   //se a distancia for menor do que dois
    analogWrite(M1F, tensao_saidaf);  //centimetros a menos do que o setpoint,
    analogWrite(M2F, tensao_saidaf);  //os motores de avanço são acionados
    analogWrite(M1R, 0);              //com a tensão calculada pelo PID
    analogWrite(M2R, 0);              //
  }
  else if(distancia < setpoint + 0.02 &&  //se a distancia estiver
          distancia > setpoint - 0.02) {  //a menos de 2 centimetros
    analogWrite(M1R, 0);                  //do setpoint, os motores
    analogWrite(M2R, 0);                  //freiam
    analogWrite(M1F, 0);                  //
    analogWrite(M2F, 0);                  //
  }
  else {
    analogWrite(M1R, tensao_saidar);  //se a distancia for maior do que dois
    analogWrite(M2R, tensao_saidar);  //centimetros a mais do que o setpoint,
    analogWrite(M1F, 0);              //os motores de ré são acionados
    analogWrite(M2F, 0);              //com a tensão calculada pelo PID
  }
}