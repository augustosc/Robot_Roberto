/*
 * **************************  Projeto do Robot Roberto ******************************************************************
 * Autor: Vovô Augusto Sherman
 * Início dos trabalhos: 22/10/2020
 * Versão Final 2, 8/01/2021.
 * 
 * O ROBOT ROBERTO foi construído para Ana Clara, Giovanna e todos os netos que vierem. As filhas e Cida poderão
 * tirar uma casquinha.
 * O objetivo ainda era de proporcionar a Ana Clara e família, no Canadá, o carinho de parte de sua família que está no Brasil.
 * Para isso, foram inseridas inúmeras mensagens de "bom dia" dos familiares para ela, de modo que ela tenha
 * certeza de que, mesmo longe, estamos perto dela. Ela pode ouvir essas mensagens sempre que quiser por meio do Roberto. 
 * Imagino que esse objetivo alcança também Deborah e Gustavo.
 * 
 * É também uma recordação o vovô Sherman deixa, de seu trabalho, para seus netos. Cada solda, cada parafuso, cada
 * componente, cada fio, cada linha deste de código teve momentos de esforço, imaginação,reflexão e estudo do vovô, 
 * para vencer as dificuldades de construir o Roberto. 
 * 
 * Além de recordação, fica a mensagem do vovô para os netos de que para se construir algo, mesmo que seja 
 * um pequeno robot, esforço, trabalho, obstinação é preciso, além de reflexão, imaginação e estudo. Gostaria que
 * meus netos tivessem essa consciência. 
 * 
 *                                                                vovô AUGUSTO SHERMAN, 8/01/2021
 * 
 * 
 * 
 * **************************  Registro da evolução das versões de teste do código  ************************************
 * 
 * Versão teste 3: Incorporou novo algoritmo para melhorar direção automática, com leituras de distâncias periódicas
 *           frontal, laterais e diagonais. Mesmo assim ainda há falhas do sensor.
 * Versão teste 4: Essa versão contempla o módulo DFplayer mini MP3, que permite a emissão de mensagens de voz.
 * Versão teste 5: Implementa o controle remoto de Roberto. É possível alternar entre a direção automática e
 *           a direção controlada. É também possível ligar e desligar a direção em modo automático, sem 
 *           sair desse modo. Contempla os sensores infravermelhos para verificar a existência de degrau à frente.
 *           Foi incorporado o led laser frontal.
 * Versão teste 6: Essa versão implementa o controle por Bluetooth
 * Versão teste 7: Atualiza os GPIOs para a nova mecânica e aprimora algoritmo de direção automática, leitura de distâncias e 
 * verificação de caminho à frente e diagonal. Implementa o tocador de MP3, o tocador de mensagens e o MENU.
 * Versão teste 9: Controle Remoto Keyes/LAFVIN.
 * Versão teste 10: Contole por Bluetooth;
 * Versão final 2: Implementação do play das músicas em loop.
 * Versão final 2v1: icrementa de 100 para 150ms o delay da função PAUSE,
 *                  alteração de msgSusto para msgStop, mensagens de inicialização no Bluetooth.
 * *******************************************************************************************************************
 * 
 * 
 */

//  --------------------------------------------------- Bibliotecas
  #include <Servo.h>
  #include "SoftwareSerial.h"
  #include "DFRobotDFPlayerMini.h"
  #include "IRremote.h" 
 
  
// Constantes
  //  --------------------------------------------------- teclas do controle de IR
  // Teclas do Controle IR KEYS/LAFVIN
const unsigned long _um = 0xFF6897;
const unsigned long _dois = 0xFF9867;
const unsigned long _tres = 0xFFB04F;
const unsigned long _quatro = 0xFF30CF;
const unsigned long _cinco = 0xFF18E7;
const unsigned long _seis = 0xFF7A85;
const unsigned long _sete = 0xFF10EF;
const unsigned long _oito = 0xFF38C7;
const unsigned long _nove = 0xFF5AA5;
const unsigned long _zero = 0xFF4AB5;
const unsigned long _asterisco = 0xFF42BD;
const unsigned long _velha = 0xFF52AD;
const unsigned long _ok = 0xFF02FD;
const unsigned long _frente = 0xFF629D;
const unsigned long _tras = 0xFFA857;
const unsigned long _direita = 0xFFC23D;
const unsigned long _esquerda = 0xFF22DD;
const unsigned long _dup =        0xFFFFFFFF;

  //  --------------------------------------------------- teclas do controle do Bluetooth
  const char _fwd = 'f';
  const char _bwd = 'b';
  const char _right = 'r';
  const char _left = 'l';
  const char _stop = 'x';
  const char _go = '*';
  const char _bt = 't';
  const char _volMinus = 'v';
  const char _volPlus = 'V';
  const char _radio = '#';
  const char _musMinus = 'm';
  const char _musPlus = 'M';
  const char _pause = 'p';
  const char _msgMinus = 'g';
  const char _msgPlus = 'G';
  const char _menu = 'o';
  const char _velMinus = 'w';
  const char _velPlus = 'W';

  // --------------------------------------------------- DEBUG
  const bool debug = 0;
  const bool ble = 0;
  
  // --------------------------------------------------- temporização de varredura
const unsigned long tempoVarreduraDfp = 3000;
const unsigned long tMapDiag = 750;


  //  --------------------------------------------------- pino do Receptor de IR
  const int receptorPin = 4;

  // --------------------------------------------------- pinos do motor
  const int mEpin1 =A3;                                  
  const int mEpin2 =A2;
  const int mDpin1 =A4;                                  
  const int mDpin2 =A5;
  const int mDhab = 5;
  const int mEhab = 6;

  //  --------------------------------------------------- pinos do sensor de distancia
  const int trigPin =3;                                  
  const int echoPin =2;
  
  // --------------------------------------------------- pino do servomotor
  const int servoPin=7;

  //  --------------------------------------------------- pinos do DFplayer mini MP3
  const int busyPin=8;

//  //  --------------------------------------------------- pinos dos sensores de obstáculos infravermelhos
//  const int sensorDegrauDireito=A1;
//  const int sensorDegrauEsquerdo=A0;

  //  --------------------------------------------------- pino do LED de sinalização de controle remoto
  // LOW = controle remoto e HIGH = automático
  const int ledControle = 11;
   
  //  --------------------------------------------------- indicadores de direção para o servomotor
  const int frente=90;
  const int direita=1;
  const int diagDireita= 40;
  const int esquerda=179;
  const int diagEsquerda=140;
  
  //  --------------------------------------------------- tempo de giro
  const int tempoGiro=800;
  
  //  --------------------------------------------------- limites de distancia
  const unsigned long limiteDist=30;
  const unsigned long limiteDistLateral=25;

  //  --------------------------------------------------- distancia máxima do sensor de distância
  const unsigned long distMax = 150;

  //  --------------------------------------------------- limite máximo de stops, para o contador da mensagem de susto
  int maxStops=5;
  
  //  --------------------------------------------------- velocidade dos motores
  int velocM=130;
  const int velocGiro=150;

  //  --------------------------------------------------- Status do Roberto
  enum carState {controleRemoto, bluetooth, automatico};
  carState robertoState = automatico;

  
//  --------------------------------------------------- Variáveis
  unsigned long distFrente=0;
  unsigned long distDireita=0;
  unsigned long distDiagDireita=0;
  unsigned long distDiagEsquerda=0;
  unsigned long distEsquerda=0;
  unsigned long altura=0;
  unsigned long timeoutMapDiag;
  int musicaAtual=1;
  int volumeAtual = 25;
  bool pauseRadio=0;
  bool notBusy;
  long nArqvMsgBomDia;
  long nArqvMsgStop;
  long nArqvMp3;
  long nArqvMenu;
  int contaStops=0;
  unsigned long timeOutRadio=0;
  bool radioON = 0;
  bool radioContinuo=1;
  bool fazCurva = 0;
  int msgBomDia=1;
  int msgAtual;
  char comandoBT;
  decode_results comando; 
  
//  --------------------------------------------------- Objetos   
   Servo cabeca;
   DFRobotDFPlayerMini dfp;
   SoftwareSerial dfpSerial(10, 9);
   SoftwareSerial BT(13,12);                   
   IRrecv receptor(receptorPin); 
   

// *********************************************************************** 
//                                             SETUP
// *********************************************************************** 
  
void setup() {
  int seed;

  if (debug) {
        Serial.println("setup()");
        BT.println("setup()");
      }
  
  // inicializa os pinos de E/S
  pinMode(mDpin1,OUTPUT);
  pinMode(mDpin2,OUTPUT);
  pinMode(mEpin1,OUTPUT);
  pinMode(mEpin2,OUTPUT);
  pinMode(mDhab,OUTPUT);
  pinMode(mEhab,OUTPUT);
  pinMode(busyPin,INPUT);
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
//  pinMode(sensorDegrauDireito,INPUT);
//  pinMode(sensorDegrauEsquerdo,INPUT);
  pinMode(ledControle,OUTPUT);

  // inicia Roberto em modo autônomo
  robertoState=automatico;
  digitalWrite(ledControle,HIGH);   

 // inicializa porta serial para debug
  Serial.begin(9600); 

  //inicializa comunicação com o módulo Bluetooth
  BT.begin(9600);

  //Sinaliza inicializacao dO ROBERTO
  Serial.println ("ROBERTO inicializando ...");
  BT.println ("ROBERTO inicializando ...");

  //inicializa receptor do Controle Remoto
  receptor.enableIRIn(); 
  Serial.println ("Inicializou IR ...");
  BT.println ("Inicializou IR ...");


          
  // inicializa o servomotor
  cabeca.attach(servoPin);                      
  cabeca.write(frente);
  
  // Aguarda DFPlayer
  delay(2000);

  // inicializa comunicação com DFPlayer
  dfpSerial.begin(9600);
  
 
  // Verifica se DFPlayer está conectado e com cartão
  
  int tentativa=1;
  while (!dfp.begin(dfpSerial) && tentativa<4) {                     //Use softwareSerial to communicate with mp3.
    Serial.println(F("erro na conexão do SD"));
    BT.println(F("erro na conexão do SD"));
    tentativa++;
    delay(1000);
  }
  if (tentativa == 4 && !dfp.begin(dfpSerial)) {
    Serial.println("DFplayer OFF");
    BT.println("DFplayer OFF");
  }
  else {
    Serial.println(F("DFPlayer online ..."));
    BT.println(F("DFPlayer online ..."));
   
    // Conclui inicialização do DFplayer
    dfp.setTimeOut(500);                                 //Set serial communictaion time out 500ms
    dfp.volume(volumeAtual);                             //Set volume value (0~30)
    dfp.EQ(DFPLAYER_EQ_NORMAL);
    dfp.outputDevice(DFPLAYER_DEVICE_SD);
 

    // verifica o número de "mensagens de STOP"
    nArqvMsgStop=-1;
    int j=0;
    while (nArqvMsgStop == -1 && j < 20)  {
      nArqvMsgStop= dfp.readFileCountsInFolder(2);
      j++;
      delay(200);
    }
    if (nArqvMsgStop == -1) nArqvMsgStop=21;
    Serial.print("n. mensagens de Stop: ");Serial.println(nArqvMsgStop);
    BT.print("n. mensagens de Stop: ");BT.println(nArqvMsgStop);

     
    // verifica número de "mensagens de Bom Dia"
    nArqvMsgBomDia=-1;
    j=0;
    while (nArqvMsgBomDia == -1 && j < 20)  {
      nArqvMsgBomDia= dfp.readFileCountsInFolder(1);
      j++;
      delay(200);
    }
    if (nArqvMsgBomDia == -1) nArqvMsgBomDia=13;
    Serial.print("n. mensagens de Bom Dia: ");Serial.println(nArqvMsgBomDia);
    BT.print("n. mensagens de Bom Dia: ");BT.println(nArqvMsgBomDia);
 
    
    // verifica número de "musicas na pasta mp3"
    nArqvMp3=-1;
    j=0;
    while (nArqvMp3 == -1 && j < 20)  {
      nArqvMp3= dfp.readFileCountsInFolder(3);
      j++;
      delay(200);
    }
    if (nArqvMp3 == -1) nArqvMp3=12;
    Serial.print("n. musicas mp3: ");Serial.println(nArqvMp3);
    BT.print("n. musicas mp3: ");BT.println(nArqvMp3);
  
        // verifica mensagem do Menu"
    nArqvMenu=-1;
    j=0;
    while (nArqvMenu == -1 && j < 20)  {
      nArqvMenu= dfp.readFileCountsInFolder(4);
      j++;
      delay(200);
    }
    if (nArqvMenu == -1) nArqvMenu=1;
    Serial.print("Menu: ");Serial.println(nArqvMenu);
    BT.print("Menu: ");BT.println(nArqvMenu);
 
    
    randomSeed(micros());
   
   // inicializa timeoutMapDiag
   timeoutMapDiag=millis();

    
    //toca "mensagem de Bom Dia"
    sendMsgBomDia();
    delay(500);
  }
  BT.listen();
                                  
}


// *********************************************************************** 
//                                             LOOP
// *********************************************************************** 
 
void loop() {

   if (debug) {
        //Serial.println("loop()");
        BT.println("loop()");
      }
      
  radioEmLoop();                            // Controla o modo contínuo do Radio
  verificaBT ();                            // verifica se há comando do Bluetooth
  verificaCR();                             // verifica se há comando do Controle Remoto                                                                        
  mapeiaFrente();                           // mapeia obstáculos à frente
  moveRobertoFrente();                      // move Roberto para frente
  mapeiaDiagonal();                         // mapeia obstáculos em posição diagonal, em intervalos determinados
  moveRobertoFrente();                      // move Roberto para frente

}

// Funções
   //  --------------------------------------------------- move Roberto para frente 
    void moveRobertoFrente(){ 
     if (debug) {
      //Serial.println("moveRobertoFrente()");
      BT.println("moveRobertoFrente()");
     }
     
     analogWrite(mDhab, velocM);
     analogWrite(mEhab,velocM);

     digitalWrite(mDpin1,HIGH);
     digitalWrite(mDpin2, LOW);
     digitalWrite(mEpin1, HIGH);
     digitalWrite(mEpin2, LOW);
    }   
    
    //  --------------------------------------------------- move Roberto para tras
    void moveRobertoTras(){ 
     if (debug) {
      //Serial.println("moveRobertoTras()");
      BT.println("moveRobertoTras()");
     }
     
     analogWrite(mDhab, velocM);
     analogWrite(mEhab,velocM);

     digitalWrite(mDpin1,LOW);
     digitalWrite(mDpin2, HIGH);
     digitalWrite(mEpin1,LOW);
     digitalWrite(mEpin2,HIGH);
    }
    
    //  --------------------------------------------------- para Roberto
     void stopRoberto(){ 
     if (debug) {
      //Serial.println("stopRoberto()");
      BT.println("stopRoberto()");
     }
     
     digitalWrite(mDhab,LOW);
     digitalWrite(mDpin1,LOW);
     digitalWrite(mDpin2, LOW);
     
     digitalWrite(mEhab,LOW);
     digitalWrite(mEpin1, LOW);
     digitalWrite(mEpin2, LOW);
    }

   //  --------------------------------------------------- gira Roberto para a direita
   void giraRobertoDireita() {
    if (debug) {
      //Serial.println("giraRobertoDireita()");
      BT.println("giraRobertoDireita()");
     }
    
    stopRoberto();
    delay(500);
    analogWrite(mDhab,velocGiro);
    analogWrite(mEhab, velocGiro);

    digitalWrite(mDpin1, LOW);
    digitalWrite(mDpin2, HIGH);
     digitalWrite(mEpin1,HIGH);
    digitalWrite(mEpin2, LOW);
    delay(tempoGiro);
    
    analogWrite(mDhab,velocM);
    analogWrite(mEhab, velocM);
    
    }
   
   //  --------------------------------------------------- gira Roberto para a esquerda
   void giraRobertoEsquerda() {
    if (debug) {
      //Serial.println("giraRobertoEsquerda()");
      BT.println("giraRobertoEsquerda()");
     } 
    
    stopRoberto();
    delay(500);
    analogWrite(mDhab,velocGiro);
    analogWrite(mEhab, velocGiro);

    digitalWrite(mDpin1, HIGH);
    digitalWrite(mDpin2, LOW);
    digitalWrite(mEpin1,LOW);
    digitalWrite(mEpin2, HIGH);
    delay(tempoGiro);

    analogWrite(mDhab,velocM);
    analogWrite(mEhab, velocM);
   }

   // --------------------------------------------------- MENSAGEM DE STOP
    void msgStop() {
      long sustoMsg=random(1,nArqvMsgStop+1);

      if (debug) {
      //Serial.println("msgStop()");
      BT.println("msgStop()");
     }
     
      // Se Roberto estiver ouvindo musica, toca a mensagem de susto da pasta Advertisement
      if (radioON) {       
        dfp.advertise(sustoMsg);
      }
      else {
        notBusy=digitalRead(busyPin);
        if (notBusy) {
          dfp.playFolder(2,sustoMsg);
        }
      }
    }

   // --------------------------------------------------- MENSAGEM DE BOM DIA
    void sendMsgBomDia() {
      if (debug) {
      //Serial.println("sendMsgBomDia()");
      BT.println("sendMsgBomDia()");
     }
      
      msgAtual=random(1,nArqvMsgBomDia+1);
      notBusy=digitalRead(busyPin);     
      if (notBusy) dfp.playFolder(1,msgAtual);

   }


   // --------------------------------------------------- LÊ E INFORMA DISTÂNCIA DE OBSTÁCULO
      unsigned long leDistancia () { 
      unsigned long dist;
      
//      if (debug) {
//      Serial.println("leDistancia ()");
//      BT.println("leDistancia ()");
//      }
      
      digitalWrite(trigPin,LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin,HIGH);                          
      delayMicroseconds(10);
      digitalWrite(trigPin,LOW);
      dist=pulseIn(echoPin,HIGH)*0.017;      
      if (dist > distMax) dist=30; 
      
      return dist;              
   }

   
   // --------------------------------------------------- MAPEIA DISTÂNCIAS FRONTAIS E DIAGONAIS
 
   void mapeiaDistancias () {
    if (debug) {
      //Serial.println("mapeiaDistancias ()");
      //BT.println("mapeiaDistancias ()");
      }
    
    cabeca.write(diagDireita);                            // diagonal direita
    delay(100);
    distDiagDireita=leDistancia();
    if (distDiagDireita < limiteDistLateral) { stopRoberto();contaStops++; fazCurva=true;}
    
    cabeca.write(diagEsquerda);                           // diagonal esquerda
    delay(250);
    distDiagEsquerda=leDistancia();
    if (distDiagEsquerda < limiteDistLateral) { stopRoberto();contaStops++;fazCurva=true;}
    
    cabeca.write(frente);                                 // frente
    delay(100);
    distFrente=leDistancia();
    if (distFrente < limiteDist) { stopRoberto();contaStops++;fazCurva=true; }

    return;
  }

  //  --------------------------------------------------- mapeia distancia lateral
  void mapeiaDistLateral() {
    if (debug) {
      //Serial.println("mapeiaDistLateral()");
      //BT.println("mapeiaDistLateral()");
      }
    
    cabeca.write(diagDireita);                              // direita
    delay(200);
    distDiagDireita=leDistancia();

    cabeca.write(direita);
    delay(300);
    distDireita=leDistancia();
    
    
    cabeca.write(diagEsquerda);                             // esquerda
    delay(500);
    distDiagEsquerda=leDistancia();

    cabeca.write(esquerda);                                 // esquerda
    delay(300);
    distEsquerda=leDistancia();

    if (distEsquerda > distDiagEsquerda) distEsquerda=distDiagEsquerda;               // Escolhe o menor valor lido à direita e à esquerda  
    if (distDireita > distDiagDireita) distDireita=distDiagDireita;

    cabeca.write(frente);
    delay(200);
   
  }
  


//  --------------------------------------------------- VERIFICA DISTÂNCIA DE OBSTÁCULOS FRONTAIS
void mapeiaFrente () {
  if (debug) {
      //Serial.println("mapeiaFrente ()");
      BT.println("mapeiaFrente ()");
      }
      
  distFrente=leDistancia();
  if (distFrente < limiteDist) {                                                        // confirma distancia inferior ao limite                                                    
    stopRoberto();
    contaStops++;
    moveRobertoTras();
    delay(600);
    stopRoberto();
    mapeiaDistLateral();                                                              // mapeia distâncias lateriais                                                                
    if (distEsquerda > distDireita) {                                                 // gira Roberto na melhor direção 
      giraRobertoEsquerda();              
    }
    else {
      giraRobertoDireita();
    }
  }
  if (contaStops>maxStops) {contaStops=0; msgStop();}
}

// --------------------------------------------------- VERIFICA DISTÂNCIA DE OBSTÁCULOS DIAGONAIS
void mapeiaDiagonal () {
  if (debug) {
      //Serial.println("mapeiaDiagonal ()");
      BT.println("mapeiaDiagonal ()");
      }
  
  if (millis() - timeoutMapDiag > tMapDiag) {
  timeoutMapDiag=millis();
      fazCurva=0;
      mapeiaDistancias();
      if (fazCurva) {
        fazCurva=0;
        mapeiaDistLateral();
        if (distEsquerda > distDireita) {
            giraRobertoEsquerda();              
        }
        else {
            giraRobertoDireita();
        }
   
      }
  }
     
}


  //  ---------------------------------------------------  EXECUTA OS COMANDOS DO CONTROLE REMOTO
  void execComandoCR () {
    if (debug) {
      //Serial.println("execComandoCR ()");
      BT.println("execComandoCR ()");
      }
    
    switch (comando.value) { 

        case _asterisco:                                                // Se o camndo == '*', liga/desliga modo de controle remoto
          if (debug) {
            Serial.println("'*' liga/desliga modo automatico");
            BT.println("'*' liga/desliga modo automatico");
          }
          if (robertoState == automatico) robertoState=controleRemoto;
          else robertoState=automatico;
          if (robertoState == controleRemoto) {
            stopRoberto();
            digitalWrite(ledControle,LOW);
          }
          else digitalWrite(ledControle,HIGH);
        break;
        
        case _velha:                                                    //se comando == #, liga/desliga radio de Roberto
          if (debug) {
            Serial.println("'#' liga/desliga radio");
            BT.println("'#' liga/desliga radio");
          }
          musicaAtual=random(1,nArqvMp3+1);
          radioON=!radioON;
          if (radioON) {
            dfp.playMp3Folder(musicaAtual);
          }
          else {
            dfp.pause();
          }
        break;

        case _um:                                                     // Volume -
          if (debug) {
            Serial.println("'1' Volume -");
            BT.println("'1' Volume -");
          }
          if (volumeAtual > 5) {
            volumeAtual--;
            dfp.volumeDown();
          }
        break;
       
        case _tres:                                                    // Volume +
          if (debug) {
            Serial.println("'3' Volume+");
            BT.println("'3' Volume+");
          }
          if (volumeAtual < 30) {
            volumeAtual++;
            dfp.volumeUp();
          }
        break;

        case _quatro:                                                   // prev musica
          if (debug) {
            Serial.println("'4' prev musica");
            BT.println("'4' prev musica");
          }
          if (musicaAtual > 1) {
            musicaAtual--;
          }
          else {
            musicaAtual=nArqvMp3;
          }
          dfp.playMp3Folder(musicaAtual);
          radioON=1;
        break;

        case _cinco:                                                    // pause music
          if (debug) {
            Serial.println("'5' pause musica");
            BT.println("'5' pause musica");
          }
          pauseRadio=!pauseRadio;
          if (pauseRadio) {
            dfp.pause();
            
          }
          else {
            dfp.start();
            delay(150);
            
          }
          radioON=1;
        break;

        case _seis:                                                     // prox musica
          if (debug) {
            Serial.println("'6' prox musica");
            BT.println("'6' prox musica");
          }
          if (musicaAtual < nArqvMp3) {
            musicaAtual++;          
          }
          else {
            musicaAtual=1;
          }
          dfp.playMp3Folder(musicaAtual);
          radioON=1;
        break;
          
        
        case _sete:                                                     // msg -
          if (debug) {
            Serial.println("'7' msg -");
            BT.println("'7' msg -");
          }
          if (msgBomDia > 1) msgBomDia--;
          else msgBomDia=nArqvMsgBomDia;
          dfp.playFolder(1,msgBomDia); 
        break;

        case _oito:                                                     // Ouvir o menu
            if (debug) {
              Serial.println("'8' MENU");
              BT.println("'8' MENU");
            }
            dfp.playFolder(4,1);
        break;
        
        case _nove:                                                     // msg +
          if (debug) {
            Serial.println("'9' msg +");
            BT.println("'9' msg +");
          }
          if (msgBomDia < nArqvMsgBomDia) msgBomDia++;
          else msgBomDia=1;
          dfp.playFolder(1,msgBomDia);
        break;
        
        case _ok:                                                      // Se comando = OK, para Roberto
          if (debug) {
            Serial.println("'ok' stop");
            BT.println("'ok' stop");
          }
          stopRoberto();
        break ;
        
        case _frente:                                                  // Se comando = frente, move Roberto para frente
          if (debug) {
            Serial.println("frente");
            BT.println("frente");
          }
          moveRobertoFrente();
        break ;

        case _tras:                                                    // Se comando = tras, move Roberto para trás
          if (debug) {
            Serial.println("tras");
            BT.println("tras");
          }
          moveRobertoTras();
        break ;
        
        case _direita:                                                 // Se comando = direita, gira Roberto para direita e segue para frente
          if (debug) {
            Serial.println("'->' direita");
            BT.println("'->' direita");
          }
          giraRobertoDireita();
          moveRobertoFrente();
        break ;
        
        case _esquerda:                                                // Se comando = esquerda, gira Roberto para esquera e segue para frente
          if (debug) {
            Serial.println("'<-' esquerda");
            BT.println("'<-' esquerda");
          }
          giraRobertoEsquerda();
          moveRobertoFrente();
         break ;
                
        case _dup:
        break;
                
        default:                                                        //// ignora qualquer outro comando
          if (debug) {
            Serial.print("outro: 0x");
            Serial.println(comando.value,HEX);
            BT.print("outro: 0x");
            BT.println(comando.value,HEX);
          }
          
        break ;
      }
      
}

  // --------------------------------------------------- VERIFICA CR
  void verificaCR() {
    if (debug) {
      //Serial.println("verificaCR()");
      BT.println("verificaCR()");
      }
    
    
    if (robertoState != bluetooth) {
      if (receptor.decode(&comando)){
        if (comando.value != _dup) execComandoCR ();
        receptor.resume();  
      }
    
    while(robertoState == controleRemoto){
      if (receptor.decode(&comando)){
        if (comando.value != _dup) execComandoCR(); 
        receptor.resume(); 
      }
      radioEmLoop();
    }
   } 
  }


  // --------------------------------------------------- EXECUTA COMANDO BT
  void execComandoBT () {
      if (debug) {
        //Serial.println("execComandoBT");
        BT.println("execComandoBT");
      }
        
        
        switch (comandoBT) { 
                 
          case _go:                                               // se comando == GO, ativa modo automatico
            robertoState = automatico;
            digitalWrite(ledControle,HIGH);
            if (debug) {
              Serial.println("'*' Go direcao automatica");
              BT.println("'*' Go direcao automatica");
            }
          break;

          case _fwd:                                               // se comando == FWD, move Roberto para frente
            if (debug) {
              Serial.println("'f' frente");
              BT.println("'f' frente");
            }
            moveRobertoFrente();
          break;

          case _bwd:                                               // se comando == BWD, move Roberto para trás
            if (debug) {
              Serial.println("'b' tras");
              BT.println("'b' tras");
            }
            moveRobertoTras();
          break;

          case _right:                                               // se comando == RIGHT, move Roberto para direita
            if (debug) {
              Serial.println("'r' direita");
              BT.println("'r' direita");
            }
            giraRobertoDireita();
            stopRoberto();
          break;

          case _left:                                               // se comando == LEFT, move Roberto para esquerda
            if (debug) {
              Serial.println("'l' esquerda");
              BT.println("'l' esquerda");
            }
            giraRobertoEsquerda();
            stopRoberto();
          break;

          case _stop:                                               // se comando == STOP, para Roberto
            if (debug) {
              Serial.println("'x' stop");
              BT.println("'x' stop");
            }
            stopRoberto();
          break;

          case _radio:                                              // se comando == RADIO, ativa/desativa o radio
            if (debug) {
              Serial.println("'#' radio");
              BT.println("'#' radio");
            }
          musicaAtual=random(1,nArqvMp3+1);
          radioON=!radioON;
          if (radioON) {
            dfp.playMp3Folder(musicaAtual);
          }
          else {
            dfp.pause();
          }
          break;

          case _volMinus:                                               // se comando == VOL-, diminui o volume
            if (debug) {
              Serial.println("'v' Volume -");
              BT.println("'v' Volume -");
            }
          if (volumeAtual > 5) {
            volumeAtual--;
            dfp.volumeDown();
          }
          break;

          case _volPlus:                                               // se comando == VOL+, aumenta o volume
           if (debug) {
            Serial.println("'V' Volume+");
            BT.println("'V' Volume+");
           }
          if (volumeAtual < 30) {
            volumeAtual++;
            dfp.volumeUp();
          }
          break;

          case _musMinus:                                             // se comando == MUS-, toca musica anterior
            if (debug) {
              Serial.println("'m' prev musica");
              BT.println("'m' prev musica");
            }
          if (musicaAtual > 1) {
            musicaAtual--;
          }
          else {
            musicaAtual=nArqvMp3;
          }
          dfp.playMp3Folder(musicaAtual);
          radioON=1;
          break;

          case _musPlus:                                             // se comando == MUS+, toca próxima musica
            if (debug) {
              Serial.println("'M' prox musica");
              BT.println("'M' prox musica");
            }
          if (musicaAtual < nArqvMp3) {
            musicaAtual++;          
          }
          else {
            musicaAtual=1;
          }
          dfp.playMp3Folder(musicaAtual);
          radioON=1;
          break;

          case _pause:                                             // se comando == PAUSE, ativa/desativa pausa
            if (debug) {
              Serial.println("'p' pause musica");
              BT.println("'p' pause musica");
            }
          pauseRadio=!pauseRadio;
          if (pauseRadio) {
            dfp.pause();
            
          }
          else {
            dfp.start();
            delay(150);
            
          }
          radioON=1;
          break;

          case _msgMinus:                                             // se comando == MSG-, toca msg Bom Dia anterior
            if (debug) {
              Serial.println("'g' msg -");
              BT.println("'g' msg -");
            }
          if (msgBomDia > 1) msgBomDia--;
          else msgBomDia=nArqvMsgBomDia;
          dfp.playFolder(1,msgBomDia); 
          break;

          case _msgPlus:                                             // se comando == MSG+, toca próxima msg Bom Dia
            if (debug) {
              Serial.println("'G' msg +");
              BT.println("'G' msg +");
            }
          if (msgBomDia < nArqvMsgBomDia) msgBomDia++;
          else msgBomDia=1;
          dfp.playFolder(1,msgBomDia);
          break;

          case _menu:                                               // apresenta o MENU de opções
            if (debug) {
              Serial.println("'0' MENU");
              BT.println("'0' MENU");
            }
            dfp.playFolder(4,1);
          break;

          case _velMinus:                                           // se comando == VEL-, diminui velocidade
            if (debug) {
              Serial.println("'s' Vel-");
              BT.println("'s' Vel-");
            }
            if (velocM > 110) velocM=velocM-10;
          break;

          case _velPlus:                                            // se comando == VEL+, aumenta velocidade          
            if (debug) {
              Serial.println("'S' Vel+");
              BT.println("'S' Vel+");
            }
            if(velocM < 200) velocM=velocM+10;
          break;

          default:                                                        //// ignora qualquer outro comando
          if (debug) {
            Serial.print("outro: ");
            Serial.println(comandoBT);
            BT.print("outro: ");
            BT.println(comandoBT);
          }
          
          break ;
        
  }
 }

  
  // --------------------------------------------------- VERIFICA BT
  void verificaBT() {
    if (debug) {
        //Serial.println("verificaBT()");
        BT.println("verificaBT()");
      }
                        
    if (BT.available()) {               
     comandoBT=BT.read();
     if (comandoBT == _bt) {
        if (robertoState != bluetooth) stopRoberto();
        robertoState=bluetooth;
        digitalWrite(ledControle,LOW);
                      
     }
     execComandoBT (); 
    }
    while (robertoState == bluetooth) {
      if (BT.available ()) {
        comandoBT=BT.read();
        execComandoBT();
      }
      radioEmLoop();
    }
    
  }

//  --------------------------------------------------- radio em loop
void radioEmLoop (){
  if (debug) {
        //Serial.println("radioEmLoop ()");
        BT.println("radioEmLoop ()");
      }
  
  if (radioContinuo && !pauseRadio && radioON && millis() - timeOutRadio > 3000) {

                if (debug) {
                  //Serial.print("type: 0x");Serial.print(dfp.readType(),HEX);
                  //BT.print("type: 0x");Serial.print(dfp.readType(),HEX);
                }
                if (debug) {
                  //Serial.print(" --- Busy State: ");Serial.println(digitalRead(busyPin));
                  //BT.print(" --- Busy State: ");Serial.println(digitalRead(busyPin));
                }
    if (digitalRead(busyPin))  {
      comando.value=_seis;
      execComandoCR();
    }
    timeOutRadio=millis();
  }

}
  
