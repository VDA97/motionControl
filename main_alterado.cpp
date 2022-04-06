#include "FastAccelStepper.h" //Biblioteca para controle de servomotor 
#include "AVRStepperPins.h" //Biblioteca solicitado pelos microcontroladores AVR, usando em Arduino.

//1. Confirmar SET e GET.
//SET e GET - Realizar e comparar - Feedback para verificar se o valor enviado foi o executado.
//SET - Parametro setado
//GET - Coleta o valor setado 

//teste git pro visual studio + 1 +1
//Processo alterado teste 1


//1. Carro irá definir sentido de rotação e acelerar até próximo ao local onde a telha será coletada
//2. Carro irá desacelerar e parar próximo ao local de coleta da telha
//3. Braço irá acelerar com movimento inferior até parar
//4. Carro irá acelerar lentamente afim de encaixar o braço na alça da telha
//5. Braço irá alterar sentido de rotação e acelerar com movimento superior.
//6. Braço irá desacelerar e parar a rotação
//7. Carro irá alterar sentido de rotação e acelerar até o local para deixar a telha 
//8. Carro irá desacelerar e parar próximo ao local de deixar a telha
//9. Braço irá alterar sentido de rotação e acelerar com movimento inferior
//10. Braço irá acelerar com movimento inferior até parar
//11. Carro irá alterar sentido de rotação e acelerar afim de se distanciar da alça da telha
//12. Carro irá parar e aguardar próximo deslocamento
//13. Braço irá alterar sentido de rotação e acelerar com movimento superior
//14. Braço irá parar 

//STEPPER_1 - Motor do braço ou carro ?

//2. Confirmar o processo de Define
//#define comumente utilizei pra "rotular" os pinos do arduino/ARM
//Necesse caso é referente ao valor atribuído para cada case, ou seja, no comando (cmd) acesso os respectivos cases para teste do motor

#define STEPPER_1_SET_INVERT 1 //Inverter rotação 
#define STEPPER_1_GET_INVERT 2 
#define STEPPER_1_SET_MAX_SPEED 3 //Velocidade máxima
#define STEPPER_1_GET_MAX_SPEED 4
#define STEPPER_1_SET_ACCEL 5 //Aceleração
#define STEPPER_1_GET_ACCEL 6
#define STEPPER_1_SET_STEP 7 //Quantidade de passos
#define STEPPER_1_GET_STEP 8
#define STEPPER_1_GET_MOVE 9 
#define STEPPER_1_SET_MOVE 10 //Setar movimento 
#define STEPPER_1_GET_MOVE_TO 11  
#define STEPPER_1_SET_MOVE_TO 12 //Qual a diferença para o Move ?
#define STEPPER_1_SET_STOP 13 //Parar motor
#define STEPPER_1_SET_FREEZE 14 //Pegar a referência do motor parado
#define STEPPER_1_GET_IS_MOVING 15 
#define STEPPER_1_GET_SPEED 16 
#define STEPPER_1_GET_STOP_P_STATE 17 
#define STEPPER_1_GET_STOP_P_LAST_VALUE 18 
#define STEPPER_1_GET_STOP_N_STATE 19 
#define STEPPER_1_GET_STOP_N_LAST_VALUE 20

//
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

//3. O que seria o último valor ? Calibração do carro e do braço, estabelece os limites de referência do sistema
long STEPPER_1_STOP_P_LAST_VALUE = 2147483647;//Valor máximo que cabe dentro da long
#define STEPPER_1_STOP_P_PIN 22 //4.Força a parada do motor para referencia 0. 
long STEPPER_1_STOP_N_LAST_VALUE = -2147483648;
#define STEPPER_1_STOP_N_PIN 23 //5. Força a parada do motor no sentido anti-horário

long STEPPER_2_STOP_P_LAST_VALUE = 2147483647;
#define STEPPER_2_STOP_P_PIN 26 //6. O número 26 esta sendo usado STEPPER_2_GET_ACCEL, isso não gera algum conflito ?
long STEPPER_2_STOP_N_LAST_VALUE = -2147483648;
#define STEPPER_2_STOP_N_PIN 27

//Pinos do hardware para comunicação com o driver dos motores 
//Driver A4988
#define STEPPER_1_STE_PIN 7 //Pino referente ao envio do sinal PWM para o driver do motor de passo
#define STEPPER_1_DIR_PIN 6 //Pino pra alterar o posicionamento
#define STEPPER_1_ENA_PIN 25 //Pino para habilitar o driver do motor de passo
#define STEPPER_1_ALM_PIN 24 //ALM seria pino para alarmes gerados  

#define STEPPER_2_STE_PIN 8
#define STEPPER_2_DIR_PIN 9
#define STEPPER_2_ENA_PIN 29
#define STEPPER_2_ALM_PIN 28

//Passa o comando pra biblioteca para utilizar a função FastAccelStepperEngine();
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper_1 = NULL; //Habilitar o uso da função para o motor 1
FastAccelStepper *stepper_2 = NULL; //Habilitar o uso da função para o motor 2

int cmd;               // Variável para enviar comandos para acessar os cases
bool cmd_done = false; //Variável para habilitar o envio dos comandos para execução dos cases.

//*Entender esse contexto


//Função para dividir num para 4  bytes 
union FourBytesInt 
{
    long num; //Variável num com todos os 4 bytes 
    byte bytes[4]; //Vetor com 4 bytes 
};
//No caso os vetores são referentes a enviar valores para os pinos STE_DIR_ENA_ALM ?
//Em que momento irei inserir o valor desse vetor ?
//Irei inserir junto com o cmd ?

FourBytesInt value; //Criação do objeto value que irá ser preenchido com os valores num

void setup() //Os parametros do setup podem ser alterados no loop ?
{
    Serial.begin(115200); //seta velocidade em 115200

    engine.init(); //Habilitar o engine da FastAccelStepperEngine engine = FastAccelStepperEngine();

    stepper_1 = engine.stepperConnectToPin(STEPPER_1_STE_PIN); //Funcionaria semelhante ao pinMode no caso. pinMode(STP, OUTPUT);
    stepper_2 = engine.stepperConnectToPin(STEPPER_2_STE_PIN); 

    if (stepper_1 && stepper_2) //Condição para comunicação com o driver dos motores 1 e 2
    {
           /* 
        #define STEPPER_2_STE_PIN 8
        #define STEPPER_2_DIR_PIN 9
        #define STEPPER_2_ENA_PIN 29
        #define STEPPER_2_ALM_PIN 28

           */
           stepper_1->setDirectionPin(STEPPER_1_DIR_PIN, 0); // pinMode(STEPPER_1_DIR_PIN, OUTPUT); digitalWrite(STEPPER_1_DIR_PIN,0);
           //No caso a direção inicia em antihorário
           stepper_1->setEnablePin(STEPPER_1_ENA_PIN);//pinMode(STEPPER_1_ENA_PIN, OUTPUT); 
           // stepper_1->setAutoEnable(true);

           stepper_1->setSpeedInUs(100);   // the parameter is us/step !!!  //Velocidade em Microssegundos ?
           stepper_1->setAcceleration(20); // Aceleração em microssegundos

           stepper_2->setDirectionPin(STEPPER_2_DIR_PIN, 1);
           stepper_2->setEnablePin(STEPPER_2_ENA_PIN);
           // stepper_2->setAutoEnable(true);

           stepper_2->setSpeedInUs(100); // the parameter is us/step !!!
           stepper_2->setAcceleration(20);//Setou aceleração, porém STEPPER_1_SET_ACCEL, chama a função pra colocar uma nova aceleração.

           stepper_1->enableOutputs();//Utilizado para habilitar o driver
           stepper_2->enableOutputs();
    }
    else //Segurança para não enviar dados pros drivers dos motores ? Assim escrevendo 000

    {
        for (int i = 0; i < 6; i++)
            Serial.write(0);

        Serial.flush();//Espera a transmissão de dados seriais enviados terminar.
        delay(500);
    }

    //Definição dos pinos stop como pull up, ou seja, entrando nível lógico 1.
    //Forçando a parada de ambos motores em quaisquer direções.
    //Motor irá acionar com bit 0 neste caso.
    pinMode(STEPPER_1_STOP_N_PIN, INPUT_PULLUP); 
    pinMode(STEPPER_1_STOP_P_PIN, INPUT_PULLUP);

    pinMode(STEPPER_2_STOP_N_PIN, INPUT_PULLUP);
    pinMode(STEPPER_2_STOP_P_PIN, INPUT_PULLUP);
}

void loop()
{
    delay(10);//Delay por conta do bootloader, instabilidade do sistema durante incio.

    //Em caso de alarme, realiza um autoreset no driver

    if (!digitalRead(STEPPER_1_ALM_PIN)) //Caso acionamento de 1 alarme, no caso seria alarmes fornecidos pelo driver
    {
        stepper_1->disableOutputs(); //Desabilita  o driver 
        stepper_1->enableOutputs(); //Habilita o driver sáidas 
        // Serial.println("ERRO Alarm 1");
    }

    if (!digitalRead(STEPPER_2_ALM_PIN))
    {
        stepper_2->disableOutputs();
        stepper_2->enableOutputs();
        // Serial.println("ERRO Alarm 2");
    }

    //Condição para forçar a parada do motor

    //Se a parada do motor estiver acionado pro sentido horário (bit 0) e estiver rodando em sentido horário > 0
    //Força a parada do motor - Segurança - Redundância
    if (((!digitalRead(STEPPER_1_STOP_P_PIN)) && (stepper_1->getCurrentSpeedInUs() > 0)))
    {
        //O último valor da posição, será igual o valor da posição atual, seria pra saber qual a posição de referência do motor.
        STEPPER_1_STOP_P_LAST_VALUE = stepper_1->getCurrentPosition();
        stepper_1->forceStopAndNewPosition(STEPPER_1_STOP_P_LAST_VALUE);
        //Para o motor.
        // A função perdea a referência de posição anterior, sendo necessário estabelecer nova posição.

        // Serial.println("ERRO Stop P 1");
    }

    //Condição para saber o posicionamento em tempo real do motor de passo 2, no sentido anti-horário.
    //Semelhante ao bloco acima, porém para o sentido de rotação invertido
   
    if (((!digitalRead(STEPPER_1_STOP_N_PIN)) && (stepper_1->getCurrentSpeedInUs() < 0)))
    {

        STEPPER_1_STOP_N_LAST_VALUE = stepper_1->getCurrentPosition();
        stepper_1->forceStopAndNewPosition(STEPPER_1_STOP_N_LAST_VALUE);
        // Serial.println("ERRO Stop N 1");
    }

    //Idêntico ao motor 1 IDLE  - Estado de espera

    if (((!digitalRead(STEPPER_2_STOP_P_PIN)) && (stepper_2->getCurrentSpeedInUs() > 0)))
    {
        STEPPER_2_STOP_P_LAST_VALUE = stepper_2->getCurrentPosition();
        stepper_2->forceStopAndNewPosition(STEPPER_2_STOP_P_LAST_VALUE);
        // Serial.println("ERRO Stop P 2");
    }

    if (((!digitalRead(STEPPER_2_STOP_N_PIN)) && (stepper_2->getCurrentSpeedInUs() < 0)))
    {
        STEPPER_2_STOP_N_LAST_VALUE = stepper_2->getCurrentPosition();
        stepper_2->forceStopAndNewPosition(STEPPER_2_STOP_N_LAST_VALUE);
        // Serial.println("ERRO Stop N 2");
    }

    //Condição para saber se ambos motores estâo habilitados, do contrário as informações enviadas pelo cmd do computador para o Arduino serão 000


    if (stepper_1 && stepper_2)
    {

        //Condição para caso os dados do cmd forem enviados, cmd = true, assim habiltando o switch/case.
        if (!cmd_done) 
        {
            switch (cmd) 
            {
                //Case utilizado para setar a direção do motor de passo 1
                //Stepper_1_SET_Invert é o rótulo utilizado para o pino 1 do motor de passo 1.
                //Estou enviando pelo cmd uma informação, um bit na entrada 1 ou 0 na entrada do pino 1.
                //Caso pino 1 receba um 0 ou 1 então muda a direção do motor.

            case STEPPER_1_SET_INVERT:
            {
                stepper_1->setDirectionPin(STEPPER_1_DIR_PIN, value.num & 0b1);//Função para definir comando pino DIR.
                //Entender o contexto estrutural - A estrura abaixo fornece um feedback de que o comando foi corretamente recebido
                
                Serial.write(36);//Por que 36 ? Pra demonstrar que foi acionado o arduino mega
                Serial.write(cmd);
                Serial.write(value.bytes[0]);
                Serial.write(value.bytes[1]);
                Serial.write(value.bytes[2]);
                Serial.write(value.bytes[3]);
                cmd_done = true;//Sai do switch e aguarda novo comando do cmd
                // Serial.println("ERRO 0");
            }
            break;
            
                //Verificar se a rotação foi invertida e para o motor
                
            case STEPPER_1_GET_INVERT:
            {
                Serial.write(36);
                Serial.write(cmd);
                Serial.write(stepper_1->directionPinHighCountsUp());//Escreve na porta serial a contagem do nível lógico 1 
                Serial.write(0);
                Serial.write(0);
                Serial.write(0);
                cmd_done = true;
                // Serial.println("ERRO 1");
            }
            break;

             //Seta o valor da velocidade em hertz

            case STEPPER_1_SET_MAX_SPEED:
            {
                stepper_1->setSpeedInHz(value.num);//Value.num é utilizado nos cases abaixo, o que garante que esse valor reset ?
                Serial.write(36);
                Serial.write(cmd);
                Serial.write(value.bytes[0]);
                Serial.write(value.bytes[1]);
                Serial.write(value.bytes[2]);
                Serial.write(value.bytes[3]);
                cmd_done = true;
                // Serial.println("ERRO 2");
            }
            break;

            //Retornar o valor setado de velocidade.

            case STEPPER_1_GET_MAX_SPEED:
            {
                value.num = long(TICKS_PER_S / stepper_1->getSpeedInTicks());//Retornar o valor setado de velocidade.

                Serial.write(36);
                Serial.write(cmd);
                Serial.write(value.bytes[0]);
                Serial.write(value.bytes[1]);
                Serial.write(value.bytes[2]);
                Serial.write(value.bytes[3]);
                cmd_done = true;
                // Serial.println("ERRO 3");
            }
            break;
            //Setpoint de aceleração
            case STEPPER_1_SET_ACCEL:
            {
                stepper_1->setAcceleration(value.num);//Determina valor de aceleração
                stepper_1->applySpeedAcceleration();//Caso o motor estiver rodando, posso aumentar o valor da velocidade.
                Serial.write(36);
                Serial.write(cmd);
                Serial.write(value.bytes[0]);
                Serial.write(value.bytes[1]);
                Serial.write(value.bytes[2]);
                Serial.write(value.bytes[3]);
                cmd_done = true;
                // Serial.println("ERRO 4");
            }
            break;
            //Verifica o motor até o valor setado de aceleração
            case STEPPER_1_GET_ACCEL:
            {
                value.num = stepper_1->getAcceleration();//Retonra o valor da aceleração atual
                Serial.write(36);
                Serial.write(cmd);
                Serial.write(value.bytes[0]);
                Serial.write(value.bytes[1]);
                Serial.write(value.bytes[2]);
                Serial.write(value.bytes[3]);
                cmd_done = true;
                // Serial.println("ERRO 5");
            }
            break;
            //Seta Valor de steps 
            case STEPPER_1_SET_STEP:
            {
                stepper_1->setCurrentPosition(value.num);
                Serial.write(36);
                Serial.write(cmd);
                Serial.write(value.bytes[0]);
                Serial.write(value.bytes[1]);
                Serial.write(value.bytes[2]);
                Serial.write(value.bytes[3]);
                cmd_done = true;
                //  Serial.println("ERRO 6");
            }
            break;
             //Verifica a quantidade de step
            case STEPPER_1_GET_STEP:
            {
                value.num = stepper_1->getCurrentPosition();
                Serial.write(36);
                Serial.write(cmd);
                Serial.write(value.bytes[0]);
                Serial.write(value.bytes[1]);
                Serial.write(value.bytes[2]);
                Serial.write(value.bytes[3]);
                cmd_done = true;
                //  Serial.println("ERRO 7");
            }
            break;

            //Seta o valor pro motor referente ao movimento em quantidade de passos.

            case STEPPER_1_SET_MOVE:
            {
                //Se houver valor positivo
                if (value.num > 0L) //Por que 0L ?
                {

                    if (digitalRead(STEPPER_1_STOP_P_PIN))//Aciono motor caso esteja parado. Sentido horário
                    {
                        stepper_1->move(value.num);
                        cmd_done = true;//Por que cmd_done vem antes ao invés de estar no final ?
                        Serial.write(36);
                        Serial.write(cmd);
                        Serial.write(value.bytes[0]);
                        return;
                        Serial.write(value.bytes[1]);
                        Serial.write(value.bytes[2]);
                        Serial.write(value.bytes[3]);
                        // Serial.println("ERRO 8");
                        
                    }
                } 
                else //Se houver valor negativo
                {
                    if (digitalRead(STEPPER_1_STOP_N_PIN)) //Mesma coisa, porém altera em sentido anti-horário
                    {
                        stepper_1->move(value.num);
                        cmd_done = true;//Por que cmd_done vem antes ao invés de estar no final ?
                        Serial.write(36);
                        Serial.write(cmd);
                        Serial.write(value.bytes[0]);
                        Serial.write(value.bytes[1]);
                        Serial.write(value.bytes[2]);
                        Serial.write(value.bytes[3]);

                        // Serial.println("ERRO 9");
                    }
                    else //Aqui  limpa o valor value.num, porém no positivo não, há algum motivo em especifico para limpar somente no negativo ?
                    {
                        value.num = 0;
                        Serial.write(36);
                        Serial.write(cmd);
                        Serial.write(value.bytes[0]);
                        Serial.write(value.bytes[1]);
                        Serial.write(value.bytes[2]);
                        Serial.write(value.bytes[3]);
                        cmd_done = true;
                        //  Serial.println("ERRO 10");
                    }
                }
            }
            break;
            //Verifica o valor de move setado para o move que está em andamento

            case STEPPER_1_GET_MOVE:
            {
                value.num = stepper_1->targetPos();
                Serial.write(36);
                Serial.write(cmd);
                Serial.write(value.bytes[0]);
                Serial.write(value.bytes[1]);
                Serial.write(value.bytes[2]);
                Serial.write(value.bytes[3]);
                cmd_done = true;
                //  Serial.println("ERRO 11");
            }
            break;

            //Seta parada  do motor
            case STEPPER_1_SET_STOP:
            {
                stepper_1->stopMove();
                Serial.write(36);
                Serial.write(cmd);
                Serial.write(value.bytes[0]);
                Serial.write(value.bytes[1]);
                Serial.write(value.bytes[2]);
                Serial.write(value.bytes[3]);
                cmd_done = true;
                // Serial.println("ERRO 12");
            }
            break;
            //Verifica onde o motor parou e o habilita para receber comando para nova movimentação.
            case STEPPER_1_SET_FREEZE:
            {
                stepper_1->forceStopAndNewPosition(stepper_1->getCurrentPosition());
                Serial.write(36);
                Serial.write(cmd);
                Serial.write(value.bytes[0]);
                Serial.write(value.bytes[1]);
                Serial.write(value.bytes[2]);
                Serial.write(value.bytes[3]);
                cmd_done = true;
                // Serial.println("ERRO 13");
            }
            break;
            //Verifica se o motor está em movimento
            case STEPPER_1_GET_IS_MOVING:
            {
                Serial.write(36);
                Serial.write(cmd);
                Serial.write(stepper_1->isMotorRunning());
                Serial.write(value.bytes[1]);
                Serial.write(value.bytes[2]);
                Serial.write(value.bytes[3]);
                cmd_done = true;
                // Serial.println("ERRO 14");
            }
            break;
            //Verifica velocidade do motor em milihertz
            case STEPPER_1_GET_SPEED:
            {
                //Retorna o valor da velocidade em Hz = mili*1000 = 1 Hz.
                value.num = long(stepper_1->getCurrentSpeedInMilliHz() * 1000);
                
                Serial.write(36);
                Serial.write(cmd);
                Serial.write(value.bytes[0]);
                Serial.write(value.bytes[1]);
                Serial.write(value.bytes[2]);
                Serial.write(value.bytes[3]);
                cmd_done = true;
                //  Serial.println("ERRO 16");
            }
            break;
            //Diferente da linha 356 STEPPER_1_SET_MOVE_TO

            case STEPPER_1_SET_MOVE_TO:
            {
                //Se o valor for maior que a posição atual do motor de passo 1
                if (value.num > stepper_1->getCurrentPosition())
                {
                    //Motor avança para a próxima telha, ou seja, evitaria erros do operador selecionar uma posição já selecionada previamente.
                    if (digitalRead(STEPPER_1_STOP_P_PIN))
                    {
                        stepper_1->moveTo(value.num);
                        cmd_done = true;
                        Serial.write(36);
                        Serial.write(cmd);
                        Serial.write(value.bytes[0]);
                        Serial.write(value.bytes[1]);
                        Serial.write(value.bytes[2]);
                        Serial.write(value.bytes[3]);
                        //  Serial.println("ERRO 17");
                    }
                }
                else //Mesma coisa, porém em direção contrária
                {
                    if (digitalRead(STEPPER_1_STOP_N_PIN))
                    {
                        stepper_1->moveTo(value.num);
                        cmd_done = true;
                        Serial.write(36);
                        Serial.write(cmd);
                        Serial.write(value.bytes[0]);
                        Serial.write(value.bytes[1]);
                        Serial.write(value.bytes[2]);
                        Serial.write(value.bytes[3]);
                        // Serial.println("ERRO 18");
                    }
                    else
                    {
                        value.num = 0;
                        cmd_done = true;
                        Serial.write(36);
                        Serial.write(cmd);
                        Serial.write(value.bytes[0]);
                        Serial.write(value.bytes[1]);
                        Serial.write(value.bytes[2]);
                        Serial.write(value.bytes[3]);
                        //  Serial.println("ERRO 19");
                    }
                }
            }
            break;
            //Verifica posição atual
            case STEPPER_1_GET_MOVE_TO:
            {
                value.num = stepper_1->targetPos();
                Serial.write(36);
                Serial.write(cmd);
                Serial.write(value.bytes[0]);
                Serial.write(value.bytes[1]);
                Serial.write(value.bytes[2]);
                Serial.write(value.bytes[3]);
                cmd_done = true;
                // Serial.println("ERRO 20");
            }
            break;
            //Verifica valor de parada do motor em sentido anti-horário
            case STEPPER_1_GET_STOP_N_STATE:
            {

                bool STEPPER_1_STOP_N_LAST_VALUE_STATE = digitalRead(STEPPER_1_STOP_P_PIN);//Neste caso não seria STEPPER_1_STOP_N_PIN ?
                cmd_done = true;

                if (STEPPER_1_STOP_N_LAST_VALUE_STATE)
                    STEPPER_1_STOP_N_LAST_VALUE = long(stepper_1->getCurrentPosition());//Verifica posição de parada

                Serial.write(36);
                Serial.write(cmd);
                Serial.write(STEPPER_1_STOP_N_LAST_VALUE_STATE);
                Serial.write(0);
                Serial.write(0);
                Serial.write(0);
                // Serial.println("ERRO 21");
            }
            break;
             //Diferenças
            case STEPPER_1_GET_STOP_N_LAST_VALUE:
            {

                bool STEPPER_1_STOP_N_LAST_VALUE_STATE = digitalRead(STEPPER_1_STOP_P_PIN);
                cmd_done = true;

                if (STEPPER_1_STOP_N_LAST_VALUE_STATE)
                    STEPPER_1_STOP_N_LAST_VALUE = long(stepper_1->getCurrentPosition());

                value.num = STEPPER_1_STOP_N_LAST_VALUE;//Por que value.num aqui ?

                Serial.write(36);
                Serial.write(cmd);
                Serial.write(value.bytes[0]);//Altera
                Serial.write(value.bytes[1]);//Altera
                Serial.write(value.bytes[2]);//Altera
                Serial.write(value.bytes[3]);//Altera
                // Serial.println("ERRO 22");
            }
            break;
            //Idem, porém no sentido horário
            case STEPPER_1_GET_STOP_P_STATE:
            {

                bool STEPPER_1_STOP_P_LAST_VALUE_STATE = digitalRead(STEPPER_1_STOP_N_PIN);
                cmd_done = true;
                if (STEPPER_1_STOP_P_LAST_VALUE_STATE)
                    STEPPER_1_STOP_P_LAST_VALUE = long(stepper_1->getCurrentPosition());

                Serial.write(36);
                Serial.write(cmd);
                Serial.write(STEPPER_1_STOP_P_LAST_VALUE_STATE);
                Serial.write(0);
                Serial.write(0);
                Serial.write(0);
                // Serial.println("ERRO 23");
            }
            break;

            case STEPPER_1_GET_STOP_P_LAST_VALUE:
            {

                bool STEPPER_1_STOP_P_LAST_VALUE_STATE = digitalRead(STEPPER_1_STOP_N_PIN);
                cmd_done = true;
                if (STEPPER_1_STOP_P_LAST_VALUE_STATE)
                    STEPPER_1_STOP_P_LAST_VALUE = long(stepper_1->getCurrentPosition());

                value.num = STEPPER_1_STOP_P_LAST_VALUE;

                Serial.write(36);
                Serial.write(cmd);
                Serial.write(value.bytes[0]);
                Serial.write(value.bytes[1]);
                Serial.write(value.bytes[2]);
                Serial.write(value.bytes[3]);
                // Serial.println("ERRO 24");
            }
            break;
            //Idem ao motor 1

            case STEPPER_2_SET_INVERT:
            {
                stepper_2->setDirectionPin(STEPPER_2_DIR_PIN, value.num & 0b1);
                Serial.write(36);
                Serial.write(cmd);
                Serial.write(value.bytes[0]);
                Serial.write(value.bytes[1]);
                Serial.write(value.bytes[2]);
                Serial.write(value.bytes[3]);
                cmd_done = true;
                // Serial.println("ERRO 25");
            }
            break;

            case STEPPER_2_GET_INVERT:
            {
                Serial.write(36);
                Serial.write(cmd);
                Serial.write(stepper_2->directionPinHighCountsUp());
                Serial.write(0);
                Serial.write(0);
                Serial.write(0);
                cmd_done = true;
                // Serial.println("ERRO 26");
            }
            break;

            case STEPPER_2_SET_MAX_SPEED:
            {
                stepper_2->setSpeedInHz(value.num);
                Serial.write(36);
                Serial.write(cmd);
                Serial.write(value.bytes[0]);
                Serial.write(value.bytes[1]);
                Serial.write(value.bytes[2]);
                Serial.write(value.bytes[3]);
                cmd_done = true;
                //  Serial.println("ERRO 27");
            }
            break;

            case STEPPER_2_GET_MAX_SPEED:
            {
                value.num = long(TICKS_PER_S / stepper_2->getSpeedInTicks());

                Serial.write(36);
                Serial.write(cmd);
                Serial.write(value.bytes[0]);
                Serial.write(value.bytes[1]);
                Serial.write(value.bytes[2]);
                Serial.write(value.bytes[3]);
                cmd_done = true;
                // Serial.println("ERRO 28");
            }
            break;

            case STEPPER_2_SET_ACCEL:
            {
                stepper_2->setAcceleration(value.num);
                stepper_2->applySpeedAcceleration();
                Serial.write(36);
                Serial.write(cmd);
                Serial.write(value.bytes[0]);
                Serial.write(value.bytes[1]);
                Serial.write(value.bytes[2]);
                Serial.write(value.bytes[3]);
                cmd_done = true;
                // Serial.println("ERRO 29");
            }
            break;

            case STEPPER_2_GET_ACCEL:
            {
                value.num = stepper_2->getAcceleration();
                Serial.write(36);
                Serial.write(cmd);
                Serial.write(value.bytes[0]);
                Serial.write(value.bytes[1]);
                Serial.write(value.bytes[2]);
                Serial.write(value.bytes[3]);
                cmd_done = true;
                // Serial.println("ERRO 30");
            }
            break;

            case STEPPER_2_SET_STEP:
            {
                stepper_2->setCurrentPosition(value.num);
                Serial.write(36);
                Serial.write(cmd);
                Serial.write(value.bytes[0]);
                Serial.write(value.bytes[1]);
                Serial.write(value.bytes[2]);
                Serial.write(value.bytes[3]);
                cmd_done = true;
                // Serial.println("ERRO 31");
            }
            break;

            case STEPPER_2_GET_STEP:
            {
                value.num = stepper_2->getCurrentPosition();
                Serial.write(36);
                Serial.write(cmd);
                Serial.write(value.bytes[0]);
                Serial.write(value.bytes[1]);
                Serial.write(value.bytes[2]);
                Serial.write(value.bytes[3]);
                cmd_done = true;
                // Serial.println("ERRO 32");
            }
            break;

            case STEPPER_2_SET_MOVE:
            {

                if (value.num > stepper_2->getCurrentPosition())
                {

                    if (digitalRead(STEPPER_2_STOP_P_PIN))
                    {
                        stepper_2->move(value.num);
                        cmd_done = true;
                        Serial.write(36);
                        Serial.write(cmd);
                        Serial.write(value.bytes[0]);
                        Serial.write(value.bytes[1]);
                        Serial.write(value.bytes[2]);
                        Serial.write(value.bytes[3]);
                        // Serial.println("ERRO 33");
                    }
                }
                else
                {
                    if (digitalRead(STEPPER_2_STOP_N_PIN))
                    {
                        stepper_2->move(value.num);
                        cmd_done = true;
                        Serial.write(36);
                        Serial.write(cmd);
                        Serial.write(value.bytes[0]);
                        Serial.write(value.bytes[1]);
                        Serial.write(value.bytes[2]);
                        Serial.write(value.bytes[3]);
                        // Serial.println("ERRO 34");
                    }
                    else
                    {
                        value.num = 0;
                        cmd_done = true;
                        Serial.write(36);
                        Serial.write(cmd);
                        Serial.write(0);
                        Serial.write(0);
                        Serial.write(0);
                        Serial.write(0);
                        // Serial.println("ERRO 35");
                    }
                }
            }
            break;

            case STEPPER_2_GET_MOVE:
            {
                value.num = stepper_2->targetPos();
                Serial.write(36);
                Serial.write(cmd);
                Serial.write(value.bytes[0]);
                Serial.write(value.bytes[1]);
                Serial.write(value.bytes[2]);
                Serial.write(value.bytes[3]);
                cmd_done = true;
                // Serial.println("ERRO 36");
            }
            break;

            case STEPPER_2_SET_STOP:
            {
                stepper_2->stopMove();
                Serial.write(36);
                Serial.write(cmd);
                Serial.write(value.bytes[0]);
                Serial.write(value.bytes[1]);
                Serial.write(value.bytes[2]);
                Serial.write(value.bytes[3]);
                cmd_done = true;
                // Serial.println("ERRO 37");
            }
            break;

            case STEPPER_2_SET_FREEZE:
            {
                stepper_2->forceStopAndNewPosition(stepper_2->getCurrentPosition());
                Serial.write(36);
                Serial.write(cmd);
                Serial.write(value.bytes[0]);
                Serial.write(value.bytes[1]);
                Serial.write(value.bytes[2]);
                Serial.write(value.bytes[3]);
                cmd_done = true;
                // Serial.println("ERRO 38");
            }
            break;

            case STEPPER_2_GET_IS_MOVING:
            {

                Serial.write(36);
                Serial.write(cmd);
                Serial.write(stepper_2->isMotorRunning());
                Serial.write(value.bytes[1]);
                Serial.write(value.bytes[2]);
                Serial.write(value.bytes[3]);
                cmd_done = true;
                // Serial.println("ERRO 39");
            }
            break;

            case STEPPER_2_GET_SPEED:
            {

                value.num = long(stepper_2->getCurrentSpeedInMilliHz() * 1000);

                Serial.write(36);
                Serial.write(cmd);
                Serial.write(value.bytes[0]);
                Serial.write(value.bytes[1]);
                Serial.write(value.bytes[2]);
                Serial.write(value.bytes[3]);
                cmd_done = true;
                // Serial.println("ERRO 40");
            }
            break;

            case STEPPER_2_SET_MOVE_TO:
            {

                if (value.num > stepper_2->getCurrentPosition())
                {

                    if (digitalRead(STEPPER_2_STOP_P_PIN))
                    {
                        stepper_2->moveTo(value.num);
                        cmd_done = true;
                        Serial.write(36);
                        Serial.write(cmd);
                        Serial.write(value.bytes[0]);
                        Serial.write(value.bytes[1]);
                        Serial.write(value.bytes[2]);
                        Serial.write(value.bytes[3]);
                        // Serial.println("ERRO 41");
                    }
                }
                else
                {
                    if (digitalRead(STEPPER_2_STOP_N_PIN))
                    {
                        stepper_2->moveTo(value.num);
                        cmd_done = true;
                        Serial.write(36);
                        Serial.write(cmd);
                        Serial.write(value.bytes[0]);
                        Serial.write(value.bytes[1]);
                        Serial.write(value.bytes[2]);
                        Serial.write(value.bytes[3]);
                        // Serial.println("ERRO 42");
                    }
                    else
                    {
                        value.num = 0;
                        cmd_done = true;
                        Serial.write(36);
                        Serial.write(cmd);
                        Serial.write(value.bytes[0]);
                        Serial.write(value.bytes[1]);
                        Serial.write(value.bytes[2]);
                        Serial.write(value.bytes[3]);
                        // Serial.println("ERRO 43");
                    }
                }
            }
            break;

            case STEPPER_2_GET_MOVE_TO:
            {
                value.num = stepper_2->targetPos();
                Serial.write(36);
                Serial.write(cmd);
                Serial.write(value.bytes[0]);
                Serial.write(value.bytes[1]);
                Serial.write(value.bytes[2]);
                Serial.write(value.bytes[3]);
                cmd_done = true;
                // Serial.println("ERRO 44");
            }
            break;

            case STEPPER_2_GET_STOP_N_STATE:
            {

                bool STEPPER_2_STOP_N_LAST_VALUE_STATE = digitalRead(STEPPER_2_STOP_P_PIN);
                cmd_done = true;

                if (STEPPER_2_STOP_N_LAST_VALUE_STATE)
                    STEPPER_2_STOP_N_LAST_VALUE = long(stepper_2->getCurrentPosition());

                Serial.write(36);
                Serial.write(cmd);
                Serial.write(STEPPER_2_STOP_N_LAST_VALUE_STATE);
                Serial.write(0);
                Serial.write(0);
                Serial.write(0);
                // Serial.println("ERRO 45");
            }
            break;

            case STEPPER_2_GET_STOP_N_LAST_VALUE:
            {

                bool STEPPER_2_STOP_N_LAST_VALUE_STATE = digitalRead(STEPPER_1_STOP_P_PIN);
                cmd_done = true;

                if (STEPPER_2_STOP_N_LAST_VALUE_STATE)
                    STEPPER_2_STOP_N_LAST_VALUE = long(stepper_2->getCurrentPosition());

                value.num = STEPPER_2_STOP_N_LAST_VALUE;

                Serial.write(36);
                Serial.write(cmd);
                Serial.write(value.bytes[0]);
                Serial.write(value.bytes[1]);
                Serial.write(value.bytes[2]);
                Serial.write(value.bytes[3]);
                // Serial.println("ERRO 46");
            }
            break;
            case STEPPER_2_GET_STOP_P_STATE:
            {

                bool STEPPER_2_STOP_P_LAST_VALUE_STATE = digitalRead(STEPPER_2_STOP_N_PIN);
                cmd_done = true;
                if (STEPPER_2_STOP_P_LAST_VALUE_STATE)
                    STEPPER_2_STOP_P_LAST_VALUE = long(stepper_2->getCurrentPosition());

                Serial.write(36);
                Serial.write(cmd);
                Serial.write(STEPPER_2_STOP_P_LAST_VALUE_STATE);
                Serial.write(0);
                Serial.write(0);
                Serial.write(0);
                // Serial.println("ERRO 47");
            }
            break;

            case STEPPER_2_GET_STOP_P_LAST_VALUE:
            {

                bool STEPPER_2_STOP_P_LAST_VALUE_STATE = digitalRead(STEPPER_2_STOP_N_PIN);
                cmd_done = true;
                if (STEPPER_2_STOP_P_LAST_VALUE_STATE)
                    STEPPER_2_STOP_P_LAST_VALUE = long(stepper_2->getCurrentPosition());

                value.num = STEPPER_2_STOP_P_LAST_VALUE;

                Serial.write(36);
                Serial.write(cmd);
                Serial.write(value.bytes[0]);
                Serial.write(value.bytes[1]);
                Serial.write(value.bytes[2]);
                Serial.write(value.bytes[3]);
                // Serial.println("ERRO 48");
            }
            break;

            case 67:
                analogWrite(LED_BUILTIN, value.num);
                Serial.write(36);
                Serial.write(cmd);
                Serial.write(value.bytes[0]);
                Serial.write(value.bytes[1]);
                Serial.write(value.bytes[2]);
                Serial.write(value.bytes[3]);
                cmd_done = true;
                // Serial.println("ERRO 49");
                break;
            }
        }
    }
    else//Caso motores 1 e 2 estejem desativados, imrpimir 0 na serial - Segurança
    {
        // Serial.println("ERRO GERAL");
        Serial.write(0);
        for (int i = 0; i < 5; i++)
            Serial.write(0);

        Serial.flush();
        delay(500);
    }
}

void serialEvent() //Função para resetar os valores da variável cmd
{

    while (Serial.available() > 5)
    {
        while (Serial.read() != '$')
            ;

        cmd = Serial.read();
        Serial.readBytes(value.bytes, 4);

        cmd_done = false;
    }
}
