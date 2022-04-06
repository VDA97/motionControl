#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESP8266mDNS.h>

#define UART_BAUD 115200 //Valor de velocidade da comunicação serial
#define bufferSize 255 //Tamanho do buffer

//Constantes para conexão do ESP no wifi local

const char *ssid = "Projeto_Hidroponia"; // Your ROUTER SSID
const char *pw = "alface2021";           // and WiFi PASSWORD
const int port = 9876;

//Mesmos defines do arduino mega
//Isso por se tratar de uma placa somente, onde tenho uma compilação pra placa
//Motor de passo do carro (CAR)
#define STEPPER_1_SET_INVERT 1
#define STEPPER_1_GET_INVERT 2
#define STEPPER_1_SET_MAX_SPEED 3
#define STEPPER_1_GET_MAX_SPEED 4
#define STEPPER_1_SET_ACCEL 5
#define STEPPER_1_GET_ACCEL 6
#define STEPPER_1_SET_STEP 7
#define STEPPER_1_GET_STEP 8
#define STEPPER_1_GET_MOVE 9
#define STEPPER_1_SET_MOVE 10
#define STEPPER_1_GET_MOVE_TO 11
#define STEPPER_1_SET_MOVE_TO 12
#define STEPPER_1_SET_STOP 13
#define STEPPER_1_SET_FREEZE 14
#define STEPPER_1_GET_IS_MOVING 15
#define STEPPER_1_GET_SPEED 16
#define STEPPER_1_GET_STOP_P_STATE 17
#define STEPPER_1_GET_STOP_P_LAST_VALUE 18
#define STEPPER_1_GET_STOP_N_STATE 19
#define STEPPER_1_GET_STOP_N_LAST_VALUE 20

//Motor de passo do Braço (ARM)
#define STEPPER_2_SET_INVERT 21
#define STEPPER_2_GET_INVERT 22
#define STEPPER_2_SET_MAX_SPEED 23
#define STEPPER_2_GET_MAX_SPEED 24
#define STEPPER_2_SET_ACCEL 25
#define STEPPER_2_GET_ACCEL 26
#define STEPPER_2_SET_STEP 27
#define STEPPER_2_GET_STEP 28
#define STEPPER_2_GET_MOVE 29
#define STEPPER_2_SET_MOVE 30
#define STEPPER_2_GET_MOVE_TO 31
#define STEPPER_2_SET_MOVE_TO 32
#define STEPPER_2_SET_STOP 33
#define STEPPER_2_SET_FREEZE 34
#define STEPPER_2_GET_IS_MOVING 35
#define STEPPER_2_GET_SPEED 36
#define STEPPER_2_GET_STOP_P_STATE 37
#define STEPPER_2_GET_STOP_P_LAST_VALUE 38
#define STEPPER_2_GET_STOP_N_STATE 39
#define STEPPER_2_GET_STOP_N_LAST_VALUE 40

//Defines do ESP
#define BRAIN_CALIBRATE 50 //Case de calibração
#define BRAIN_CATCH 51 //Captura de telha
#define BRAIN_RELEASE 52 //Devolução de telha
#define BRAIN_COMEANDNGO 53 // Verificar da perda de pulsos
#define BRAIN_DISTANCE 54 //Altera a distância da manobra de captura
#define BRAIN_MAXSPEED 55 //Altera a distância da máxima curva de aceleração dos dois steppers
#define BRAIN_ACCEL 56 //Altera a aceleração dos dois steppers
#define BRAIN_ARM 57 //Altera número de passos para descida do braço    
#define BRAIN_ARM_INVERT 58 //Altera rotação do motor do stepper 2
#define BRAIN_CAR_INVERT 59//Altera a rotação do stepper 1
#define BRAIN_TEST 99//Verificar teste de blink

#define BRAIN_ARM_PERCENT_CATCH 70 //Verificar
#define BRAIN_ARM_PERCENT_RELEASE 71//Verificar

//Constantes
long BRAIN_DISTANCE_VALUE = 19200;
long BRAIN_MAXSPEED_VALUE = 7500;
long BRAIN_ACCEL_VALUE = 1000;
long BRAIN_ARM_VALUE = 38000;
bool BRAIN_ARM_INVERT_VALUE = 1;
bool BRAIN_CAR_INVERT_VALUE = 0;

//Verificar uso dessas variáveis
long BRAIN_ARM_CATCH_PERCENT = (BRAIN_ARM_VALUE / 100) * 10;
long BRAIN_ARM_RELEASE_PERCENT = (BRAIN_ARM_VALUE / 100) * 20;

//Realiza a divisão da variável num em vetor de 4 bytes
union FourBytesInt
{
  long num;
  byte bytes[4];
};

FourBytesInt value;

WiFiServer server(port);
WiFiClient client;

//Variável para agrupar valores seriais - Envio do ESP para o Arduino Mega
char readSerial_buf[bufferSize];
int rs = 0;//

//Variável para Leitura de dados TCP - Envio do Brain para o ESP
char readTCP_buf[bufferSize];
int rt = 0;//

//Variável para escrita de dados TCP - Envio do ESP para Brain 
char writeTCP_buf[bufferSize];
int wt = 0;//


void setup()
{

  delay(500);

  Serial.begin(UART_BAUD); //Inicia porta serial na mesma velocidade configurada no Arduino Mega
  
  //Configuração para conexão do wifi local
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pw);
  WiFi.setOutputPower(20.5);
  wifi_set_sleep_type(NONE_SLEEP_T);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
  }
  
  server.begin(); // start TCP server
   
  ArduinoOTA.setPort(8266);
  ArduinoOTA.setHostname("espota");

  ArduinoOTA.begin();
}

//Função pra leitura do TCP - Comandos enviados do Brain para o ESP
//1. SET irá verificar se irá executar comando ao ESP para coletar dados do arduino mega e enviar ao Brain 
//2. GET irá verificar se irá executar comando ao ESP para comandar o arduino mega

void ReadTCP()
{
  //Se o Brain local estiver disponível, le os comandos que foram enviados pelo Brain
  if (client.available())
  {
    readTCP_buf[0] = client.read(); //Armazena no vetor 0 o byte recebido do Brain 
    
    //Função: Aguardar a informação do Brain. O comando do brain irá se encerrar com $ e #
    //Enquanto for diferente de $ e de #
    //CMD$# - Envio de todas as informações pro ESP
    while ((readTCP_buf[0] != '$') && (readTCP_buf[0] != '#'))
    {
      if (!client.available())//Se cliente não estiver disponível
        return;//Retorna pro while
      readTCP_buf[0] = client.read();
    }

    //??
    client.read(&readTCP_buf[1], 5);
    rt = 6;//??
     
    //Essa condição irá distribuir a informação recebida pelo brain no vetor de 4 bytes até o sinal "$" ser recebido
    
    if (readTCP_buf[0] == '$')
    {
      //Por que tenho um serial aqui, se o objetivo é ler as informações do Brain (PC) ?
      //Necessário pra comunicação TCP ?
      
      while (Serial.available())
      {
        Serial.read();
        delay(10);
      }
      //Escreve no buffer do TCP as informaçoes  para enviar ao Brain 
      Serial.write(readTCP_buf, 6);
      rt = 0;//rt - utilizado para start do switch
    
      //Leitura dos bytes do buffer serial, informações que vem do Arduino
      Serial.readBytes(readSerial_buf, 6);
      rs = 6;//rs ??

      //ESP  informando o brain sobre o valor serial coletado do arduino mega
      client.write(readSerial_buf, rs);
      
      //Enquanto Brain disponível, realiza a leitura do mesmo
      while (client.available())
      {
        client.read();
        delay(10);
      }
    }
    else //Enquanto estiver aguandando o final do envio das informações do Brain, armazena no vetor com 4 bytes 
    {
      
      value.bytes[0] = readTCP_buf[2];
      value.bytes[1] = readTCP_buf[3];
      value.bytes[2] = readTCP_buf[4];
      value.bytes[3] = readTCP_buf[5];
      //readTCP_buf[0] = caracter que informa o final da informação enviado pelo Brain $.
      //readTCP_buf[1] = cmd ?
    }
  }
}

//Realiza a leitura dos serial do arduino 
void ReadSerial()
{

  while (Serial.available() < 6)
  {
    yield();//Verificar
    delay(50);
  }
  Serial.readBytes(readSerial_buf, 6);
  rs = 6;

  value.bytes[0] = readSerial_buf[2];
  value.bytes[1] = readSerial_buf[3];
  value.bytes[2] = readSerial_buf[5];
  value.bytes[3] = readSerial_buf[6];
}

//O que seria value_par ?

//ESP irá passar valores do Brain ao Arduino Mega.
void WriteSerial(int cmd, long value_par)
{
  
  //Assim que recebidos esses valores, os mesmos já serão segregados paralelamente (simultaneamente)
  while (Serial.available())
  {
    yield();//Passes control to other tasks when called. Ideally yield() should be used in functions that will take awhile to complete.
    delay(50);
  }

  value.num = value_par;
  Serial.write('$');//ESP irá passa o comando do Brain ao Arduinomega.
  Serial.write(cmd);
  Serial.write(value.bytes[0]);
  Serial.write(value.bytes[1]);
  Serial.write(value.bytes[2]);
  Serial.write(value.bytes[3]);
  Serial.flush();
}

//ESP irá passar valores do Arduino mega ao Brain
void WriteTcp(int cmd, long value_par)
{
  value.num = value_par;
  writeTCP_buf[0] = '#';
  writeTCP_buf[1] = cmd;
  writeTCP_buf[2] = value.bytes[0];
  writeTCP_buf[3] = value.bytes[1];
  writeTCP_buf[4] = value.bytes[2];
  writeTCP_buf[5] = value.bytes[3];
  wt = 6;
  client.write((uint8_t *)WriteSerial, wt);
}

void loop()
{

    delay(5);//Delay para evitar problemas na execução do loop
  ArduinoOTA.handle();

  //Aguarda comunicação com o Brain
  if (!client.connected())
  {                              // if client not connected
    client = server.available(); // wait for it to connect
    return;
  }

  ReadTCP();//Recebe e interpreta o valor recebido pelo brain

  if (rt == 6)//Havendo leitura dos dados do arduino, então:
  {
    switch (readTCP_buf[1])//De acordo com o comando do Brain (PC)
    {

    // ROTINA DE CALIBRACAO DO CARRO E DO BRACO
    case BRAIN_CALIBRATE:
    {
      // BLOCO DE INICIALIZACAO DO BRACO, ALTERANDO O SENTIDO DE ROTACAO, VELOCIDADE MAXIMA E ACELERACAO DO BRACO (REMOVIVEL)
      WriteSerial(STEPPER_2_SET_INVERT, BRAIN_ARM_INVERT_VALUE);
      ReadSerial();//Verificar o valor enviado
       //ESP recebe o comando do BRAIN via TCP/IP e envia comando ao arduino via Serial
      
      //Aciona o case stepper

      /*void WriteSerial(int cmd, long value_par)
            {
            
            //Assim que recebidos esses valores, os mesmos já serão segregados paralelamente (simultaneamente)
            while (Serial.available())
            {
                yield();//Passes control to other tasks when called. Ideally yield() should be used in functions that will take awhile to complete.
                delay(50);
            }

            value.num = value_par;
            Serial.write('$');//ESP irá passa o comando do Brain ao Arduinomega.
            Serial.write(cmd);
            Serial.write(value.bytes[0]);
            Serial.write(value.bytes[1]);
            Serial.write(value.bytes[2]);
            Serial.write(value.bytes[3]);
            Serial.flush();
            }*/

     
         /*  case STEPPER_1_SET_INVERT:
            {
                stepper_1->setDirectionPin(STEPPER_1_DIR_PIN, value.num & 0b1);//Função para definir comando pino DIR.
                //Entender o contexto estrutural
                
                Serial.write(36);//Por que 36 ?
                Serial.write(cmd);//Se estou acessando este caso, por que escrever ele novamente no serial ?
                Serial.write(value.bytes[0]);//Escreve valor no vetor de 8 bits 0.
                Serial.write(value.bytes[1]);
                Serial.write(value.bytes[2]);
                Serial.write(value.bytes[3]);
                cmd_done = true;//Sai do switch e aguarda novo comando do cmd
                // Serial.println("ERRO 0");
            }
            break;*/
            

      WriteSerial(STEPPER_2_SET_MAX_SPEED, BRAIN_MAXSPEED_VALUE);
      ReadSerial();

      WriteSerial(STEPPER_2_SET_ACCEL, BRAIN_ACCEL_VALUE);
      ReadSerial();

      // MOVIMENTO INCREMENTAL PARA O MAXIMO NEGATIVO PARA ACHAR O INICIO DE CURSO
      WriteSerial(STEPPER_2_SET_MOVE, -2147483600);
      ReadSerial();

      // VERIFICA SE HA MOVIMENTO DO BRACO
      WriteSerial(STEPPER_2_GET_IS_MOVING, 0);
      ReadSerial();

      // VERIFICA SE AINDA HA MOVIMENTO DO BRACO
      while (value.num != 0)
      {
        WriteSerial(STEPPER_2_GET_IS_MOVING, 0);
        ReadSerial();
        delay(100);
      }

      delay(100);

      // ALTERA O PASSO ATUAL DO BRACO PARA ZERO NO INICIO DO CURSO
      WriteSerial(STEPPER_2_SET_STEP, 0);
      ReadSerial();

      delay(100);

      // BLOCO DE INICIALIZACAO DO CARRO, ALTERANDO O SENTIDO DE ROTACAO, VELOCIDADE MAXIMA E ACELERACAO DO CARRO(REMOVIVEL)
      WriteSerial(STEPPER_1_SET_INVERT, BRAIN_CAR_INVERT_VALUE);
      ReadSerial();

      WriteSerial(STEPPER_1_SET_MAX_SPEED, BRAIN_MAXSPEED_VALUE);
      ReadSerial();

      WriteSerial(STEPPER_1_SET_ACCEL, BRAIN_ACCEL_VALUE);
      ReadSerial();

      // MOVIMENTO INCREMENTAL PARA O MAXIMO NEGATIVO PARA ACHAR O INICIO DE CURSO
      WriteSerial(STEPPER_1_SET_MOVE, -2147483600);
      ReadSerial();

      // VERIFICA SE HA MOVIMENTO DO CARRO
      WriteSerial(STEPPER_1_GET_IS_MOVING, 0);
      ReadSerial();

      // VERIFICA SE AINDA HA MOVIMENTO DO CARRO
      while (value.num != 0)
      {

        WriteSerial(STEPPER_1_GET_IS_MOVING, 0);
        ReadSerial();
        delay(100);
      }

      // ALTERA O PASSO ATUAL DO CARRO PARA ZERO NO INICIO DO CURSO
      WriteSerial(STEPPER_1_SET_STEP, 0);
      ReadSerial();

      // RESPONDE A REQUISICAO DO MASTER DIZENDO QUE TERMINOU O COMANDO (FALTA O TRATAMENTO DE ERROS)
      client.write(readTCP_buf, rt);

      rt = 0;
    }
    break;

    // ROTINA DE TESTE PARA VERIFICAR SE HA PERDA DE PULSOS (REMOVIVEL)
    case BRAIN_COMEANDNGO:
    {

      int i = 0;
      int limit = value.num;

      WriteSerial(STEPPER_1_SET_INVERT, BRAIN_CAR_INVERT_VALUE);
      ReadSerial();

      WriteSerial(STEPPER_1_SET_MAX_SPEED, BRAIN_MAXSPEED_VALUE);
      ReadSerial();

      WriteSerial(STEPPER_1_SET_ACCEL, BRAIN_ACCEL_VALUE);

      WriteSerial(STEPPER_2_SET_INVERT, BRAIN_ARM_INVERT_VALUE);
      ReadSerial();
      ReadSerial();

      WriteSerial(STEPPER_2_SET_MAX_SPEED, BRAIN_MAXSPEED_VALUE);
      ReadSerial();

      WriteSerial(STEPPER_2_SET_ACCEL, BRAIN_ACCEL_VALUE);
      ReadSerial();

      WriteSerial(STEPPER_2_SET_MOVE_TO, 0);
      ReadSerial();

      WriteSerial(STEPPER_2_GET_IS_MOVING, 0);
      ReadSerial();

      while (value.num != 0)
      {
        WriteSerial(STEPPER_2_GET_IS_MOVING, 0);
        ReadSerial();
        delay(100);
      }

      delay(100);
       //
      while (i < limit)
      {

        i++;
        WriteSerial(STEPPER_1_SET_MOVE_TO, 100000);
        ReadSerial();

        WriteSerial(STEPPER_1_GET_IS_MOVING, 0);
        ReadSerial();

        while (value.num != 0)
        {
          WriteSerial(STEPPER_1_GET_IS_MOVING, 0);
          ReadSerial();
          delay(100);
        }

        delay(100);

        WriteSerial(STEPPER_1_SET_MOVE_TO, 300000);
        ReadSerial();

        WriteSerial(STEPPER_1_GET_IS_MOVING, 0);
        ReadSerial();

        while (value.num != 0)
        {
          WriteSerial(STEPPER_1_GET_IS_MOVING, 0);
          ReadSerial();
          delay(100);
        }
        client.write(readTCP_buf, rt);
        rt = 0;
      }
    }
    break;

    // ROTINA PARA PEGAR UMA TELHA DE UMA DETERMINADA POSICAO (POS) E SEGURAR NOS BRACOS
    case BRAIN_CATCH:
    {
      long pos = value.num;

      // BLOCO DE INICIALIZACAO DO CARRO, ALTERANDO O SENTIDO DE ROTACAO, VELOCIDADE MAXIMA E ACELERACAO DO CARRO (REMOVIVEL)
      WriteSerial(STEPPER_1_SET_INVERT, BRAIN_CAR_INVERT_VALUE);
      ReadSerial();

      WriteSerial(STEPPER_1_SET_MAX_SPEED, BRAIN_MAXSPEED_VALUE);
      ReadSerial();

      WriteSerial(STEPPER_1_SET_ACCEL, BRAIN_ACCEL_VALUE);
      ReadSerial();

      // BLOCO DE INICIALIZACAO DO BRACO, ALTERANDO O SENTIDO DE ROTACAO, VELOCIDADE MAXIMA E ACELERACAO DO BRACO (REMOVIVEL)
      WriteSerial(STEPPER_2_SET_INVERT, BRAIN_ARM_INVERT_VALUE);
      ReadSerial();

      WriteSerial(STEPPER_2_SET_MAX_SPEED, BRAIN_MAXSPEED_VALUE);
      ReadSerial();

      WriteSerial(STEPPER_2_SET_ACCEL, BRAIN_ACCEL_VALUE);
      ReadSerial();

      // SOBE OS BRA�OS PARA NAO CAUSAR COLISAO
      WriteSerial(STEPPER_2_SET_MOVE_TO, 0);
      ReadSerial();

      // VERIFICA SE HA MOVIMENTO DO BRA�O
      WriteSerial(STEPPER_2_GET_IS_MOVING, 0);
      ReadSerial();

      // VERIFICA SE AINDA HA MOVIMENTO DO BRA�O
      while (value.num != 0)
      {
        WriteSerial(STEPPER_2_GET_IS_MOVING, 0);
        ReadSerial();
        delay(100);
      }

      delay(100);

      // MOVE O CARRO PARA UM POUCO ANTES DA TELHA PARA DESCER OS BRACOS SEM COLIDIR COM A ALCA DA TELHA
      WriteSerial(STEPPER_1_SET_MOVE_TO, pos - BRAIN_DISTANCE_VALUE);
      ReadSerial();

      // VERIFICA SE HA MOVIMENTO DO CARRO
      WriteSerial(STEPPER_1_GET_IS_MOVING, 0);
      ReadSerial();

      // VERIFICA SE AINDA HA MOVIMENTO DO CARRO
      while (value.num != 0)
      {
        WriteSerial(STEPPER_1_GET_IS_MOVING, 0);
        ReadSerial();
        delay(100);
      }

      delay(100);

      // DESCE OS BRACOS COM UMA DISTANCIA MEDIDA DE ACORDO COM A ALTURA DA TELHA
      WriteSerial(STEPPER_2_SET_MOVE_TO, BRAIN_ARM_VALUE);
      ReadSerial();

      // VERIFICA SE HA MOVIMENTO DO BRACO
      WriteSerial(STEPPER_2_GET_IS_MOVING, 0);
      ReadSerial();

      while (value.num != 0)
      {
        WriteSerial(STEPPER_2_GET_IS_MOVING, 0);
        ReadSerial();
        delay(100);
      }

      delay(100);

      // AVANCA O CARRO PARA A POSICAO DA TELHA
      WriteSerial(STEPPER_1_SET_MOVE_TO, pos);
      ReadSerial();

      // VERIFICA SE HA MOVIMENTO DO CARRO
      WriteSerial(STEPPER_1_GET_IS_MOVING, 0);
      ReadSerial();

      // VERIFICA SE AINDA HA MOVIMENTO DO CARRO
      while (value.num != 0)
      {
        WriteSerial(STEPPER_1_GET_IS_MOVING, 0);
        ReadSerial();
        delay(100);
      }

      delay(100);

      // SOBE OS BRACOS PARA O INICIO DO CURSO
      WriteSerial(STEPPER_2_SET_MOVE_TO, (BRAIN_ARM_VALUE - BRAIN_ARM_CATCH_PERCENT));
      ReadSerial();

      // VERIFICA SE HA MOVIMENTO DO BRACO
      WriteSerial(STEPPER_2_GET_IS_MOVING, 0);
      ReadSerial();

      // VERIFICA SE AINDA HA MOVIMENTO DO CARRO
      while (value.num != 0)
      {
        WriteSerial(STEPPER_2_GET_IS_MOVING, 0);
        ReadSerial();
        delay(100);
      }

      // SOBE OS BRACOS PARA O INICIO DO CURSO
      WriteSerial(STEPPER_2_SET_MOVE_TO, 0);
      ReadSerial();

      // VERIFICA SE HA MOVIMENTO DO BRACO
      WriteSerial(STEPPER_2_GET_IS_MOVING, 0);
      ReadSerial();

      // VERIFICA SE AINDA HA MOVIMENTO DO CARRO
      while (value.num != 0)
      {
        WriteSerial(STEPPER_2_GET_IS_MOVING, 0);
        ReadSerial();
        delay(100);
      }

      // RESPONDE A REQUISICAO DO MASTER DIZENDO QUE TERMINOU O COMANDO (FALTA O TRATAMENTO DE ERROS)
      client.write(readTCP_buf, rt);
      rt = 0;
    }
    break;

    // ROTINA DE DEVOLUCAO DE UMA TELHA EM SUA POSICAO (POS)
    case BRAIN_RELEASE:
    {
      long pos = value.num;

      // BLOCO DE INICIALIZACAO DO CARRO, ALTERANDO O SENTIDO DE ROTACAO, VELOCIDADE MAXIMA E ACELERACAO DO CARRO (REMOVIVEL)
      WriteSerial(STEPPER_1_SET_INVERT, BRAIN_CAR_INVERT_VALUE);
      ReadSerial();

      WriteSerial(STEPPER_1_SET_MAX_SPEED, BRAIN_MAXSPEED_VALUE);
      ReadSerial();

      WriteSerial(STEPPER_1_SET_ACCEL, BRAIN_ACCEL_VALUE);
      ReadSerial();

      // BLOCO DE INICIALIZACAO DO BRACO, ALTERANDO O SENTIDO DE ROTACAO, VELOCIDADE MAXIMA E ACELERACAO DO BRACO (REMOVIVEL)
      WriteSerial(STEPPER_2_SET_INVERT, BRAIN_ARM_INVERT_VALUE);
      ReadSerial();

      WriteSerial(STEPPER_2_SET_MAX_SPEED, BRAIN_MAXSPEED_VALUE);
      ReadSerial();

      WriteSerial(STEPPER_2_SET_ACCEL, BRAIN_ACCEL_VALUE);
      ReadSerial();

      // MOVE O CARRO PARA A POSICAO (POS) DA TELHA
      WriteSerial(STEPPER_1_SET_MOVE_TO, pos);
      ReadSerial();

      // VERIFICA SE HA MOVIMENTO DO CARRO
      WriteSerial(STEPPER_1_GET_IS_MOVING, 0);
      ReadSerial();

      // VERIFICA SE AINDA HA MOVIMENTO DO CARRO
      while (value.num != 0)
      {
        WriteSerial(STEPPER_1_GET_IS_MOVING, 0);
        ReadSerial();
        delay(100);
      }

      // DESCE OS BRACOS COM UMA DISTANCIA MEDIDA DE ACORDO COM A ALTURA DA TELHA
      WriteSerial(STEPPER_2_SET_MOVE_TO, BRAIN_ARM_VALUE - BRAIN_ARM_RELEASE_PERCENT);
      ReadSerial();

      // VERIFICA SE HA MOVIMENTO DO BRACO
      WriteSerial(STEPPER_2_GET_IS_MOVING, 0);
      ReadSerial();

      // VERIFICA SE AINDA HA MOVIMENTO DO BRACO
      while (value.num != 0)
      {
        WriteSerial(STEPPER_2_GET_IS_MOVING, 0);
        ReadSerial();
        delay(100);
      }

      WriteSerial(STEPPER_2_SET_MOVE_TO, BRAIN_ARM_VALUE);
      ReadSerial();

      // VERIFICA SE HA MOVIMENTO DO BRACO
      WriteSerial(STEPPER_2_GET_IS_MOVING, 0);
      ReadSerial();

      // VERIFICA SE AINDA HA MOVIMENTO DO BRACO
      while (value.num != 0)
      {
        WriteSerial(STEPPER_2_GET_IS_MOVING, 0);
        ReadSerial();
        delay(100);
      }

      delay(100);

      // RECUA O CARRO PARA FORA DA ALCA DA TELHA
      WriteSerial(STEPPER_1_SET_MOVE_TO, pos - BRAIN_DISTANCE_VALUE);
      ReadSerial();

      // VERIFICA SE HA MOVIMENTO DO CARRO
      WriteSerial(STEPPER_1_GET_IS_MOVING, 0);
      ReadSerial();

      // VERIFICA SE AINDA HA MOVIMENTO DO CARRO
      while (value.num != 0)
      {
        WriteSerial(STEPPER_1_GET_IS_MOVING, 0);
        ReadSerial();
        delay(100);
      }

      delay(100);

      // SOBE OS BRACOS PARA O INICIO DO CURSO
      WriteSerial(STEPPER_2_SET_MOVE_TO, 0);
      ReadSerial();

      // VERIFICA SE HA MOVIMENTO DO CARRO
      WriteSerial(STEPPER_2_GET_IS_MOVING, 0);
      ReadSerial();

      // VERIFICA SE AINDA HA MOVIMENTO DO CARRO
      while (value.num != 0)
      {
        WriteSerial(STEPPER_2_GET_IS_MOVING, 0);
        ReadSerial();
        delay(100);
      }
      // RESPONDE A REQUISICAO DO MASTER DIZENDO QUE TERMINOU O COMANDO (FALTA O TRATAMENTO DE ERROS)
      client.write(readTCP_buf, rt);
      rt = 0;
    }
    break;

    // ROTINA DE TESTE BLINK LED (REMOVIVEL)
    case BRAIN_TEST:
    {
      while (true)
      {
        WriteSerial(67, 255);
        ReadSerial();
        client.write(readSerial_buf, rs);
        delay(1000);
        WriteSerial(67, 0);
        ReadSerial();
        client.write(readSerial_buf, rs);
        delay(1000);
      }
    }
    break;

    // COMANDO DE ALTERACAO DO PARAMETRO DE DISTANCIA DE RECUO PARA PEGAR E DEVOLVER TELHAS
    case BRAIN_DISTANCE:
    {
      BRAIN_DISTANCE_VALUE = value.num;
      client.write(readTCP_buf, rt);
      rt = 0;
    }
    break;

    // COMANDO DE ALTERACAO DA VELOCIDADE MAXIMA DE AMBOS OS STEPPERS UTILIZADOS PELAS ROTINAS
    case BRAIN_MAXSPEED:
    {
      BRAIN_MAXSPEED_VALUE = value.num;
      client.write(readTCP_buf, rt);
      rt = 0;
    }
    break;

    // COMANDO DE ALTERACAO DA ACELERACAO DE AMBOS OS STEPPERS UTILIZADOS PELAS ROTINAS
    case BRAIN_ACCEL:
    {
      BRAIN_ACCEL_VALUE = value.num;
      client.write(readTCP_buf, rt);
      rt = 0;
    }
    break;

    // COMANDO DE ALTERACAO DA DISTANCIA DE DESCIDA DO BRACO
    case BRAIN_ARM:
    {
      BRAIN_ARM_VALUE = value.num;
      client.write(readTCP_buf, rt);
      rt = 0;
    }
    break;

    case BRAIN_ARM_PERCENT_CATCH:
    {
      BRAIN_ARM_CATCH_PERCENT = value.num;
      client.write(readTCP_buf, rt);
      rt = 0;
    }
    break;

    case BRAIN_ARM_PERCENT_RELEASE:
    {
      BRAIN_ARM_RELEASE_PERCENT = value.num;
      client.write(readTCP_buf, rt);
      rt = 0;
    }
    break;

    // COMANDO DE ALTERACAO DO SENTIDO DE ROTACAO DO BRACO
    case BRAIN_ARM_INVERT:
    {
      BRAIN_ARM_INVERT_VALUE = value.num;
      client.write(readTCP_buf, rt);
      rt = 0;
    }
    break;

    // COMANDO DE ALTERACAO DO SENTIDO DE ROTACAO DO CARRO
    case BRAIN_CAR_INVERT:
    {
      BRAIN_CAR_INVERT_VALUE = value.num;
      client.write(readTCP_buf, rt);
      rt = 0;
    }
    break;

    default:
    {
    }
    break;
    }
  }
}
