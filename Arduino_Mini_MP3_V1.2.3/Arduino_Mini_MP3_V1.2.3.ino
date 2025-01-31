#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include "RotaryEncoder.h"
#include <avr/sleep.h>
RotaryEncoder Vol(A2, A3);  // (DT, CLK) ; encoder volume
RotaryEncoder ChoiceMp3(A4, A5);  // (DT, CLK) encoder choix histoire

SoftwareSerial mySoftwareSerial(11, 10); // RX, TX
DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);

/*********Mesure du temps pour mise en veille de l'arduino**************/
unsigned long inactivityTime = 600000; // time in milliseconds = 10min
unsigned long currentTime; 
unsigned long previousTime;

/*********variable des boutons**************/
const int buttOK = 2; // bouton de validation OK
const int buttHome = 3; // bouton de  retour/home
const int buttPause = 4; // bouton de Pause

int buttOkPushed=1; //variable de stockage appui bouton OK VERT : validation histoire
int buttHomePushed=1; //variable de stockage appui bouton Home ROUGE : retour au démarrage
int buttPausePushed=1; //variable de stockage appui bouton Pause JAUNE : start et pause

int newPause=1; //variable stockage de l'état de lecture

/*********variable du volume**************/
int newVol=0; //variable stockage du volume
int volMin = 2; // volume minimum
int volMax = 30; // volume maximum
int OnOff=0; // non utilisé

/*********variable des encoders volume et choix histoires**************/
static int pos = 0;
static int choice = 0;
int newPos;
int newchoice;

/*********variable de sélection des histoires**************/
int choiceOk=0;
int selTitle=1; //permet de stocker le choix du titre de l'histoire (nombre impair)
int selStory=2; //permet de stocker le choix de l'histoire (nombre pair, choix du titre+1)
int fileCount = 0; //permet de stocker le nombre d'histoires

int readState=0; //Etat de lecture du système : "1"= mode choix et "0"=Mode lecture
int home=0; //Retour au mode de démarrage : "1"= retour au mode choix initial et "0"= ne fait rien
int sleepMode = 0; //Etat de l'arduino veille: "1"= mode veille et "0"=Mode actif

void setup()
{
  mySoftwareSerial.begin(9600);
  Serial.begin(115200);

  pinMode(buttOK, INPUT_PULLUP);
  pinMode(buttHome, INPUT_PULLUP);
  pinMode(buttPause, INPUT_PULLUP);

  Serial.println();
  Serial.println(F("StoryTeller Box :)"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  
  if (!myDFPlayer.begin(mySoftwareSerial)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true);
  }
  Serial.println(F("DFPlayer Mini online."));

  myDFPlayer.setTimeOut(2000); //Set serial communictaion time out 500ms
  myDFPlayer.EQ(DFPLAYER_EQ_NORMAL); //set the equalizer
  myDFPlayer.volume(20);  //Set volume value. From 0 to 30
  fileCount = myDFPlayer.readFileCounts(); //read all file counts in SD card

  Serial.println(F("MP3 play"));
  myDFPlayer.play(selTitle);
  delay(2000);

  previousTime = millis(); //démarrage du compteur pour le mode veille
}

void loop(){
  buttOkFunct();
  buttHomeFunct();
  buttPauseFunct();
  VolChoice();
  MP3Choice();
  IdleCheck();
  
  if (myDFPlayer.available()){
    printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
  }
}

void VolChoice(){
  Vol.tick();
  newPos = Vol.getPosition();
  
  if (pos != newPos) {
      pos = newPos;
      newVol=pos+2;
      if (newVol > volMax){ 
          newVol = volMax;
        }
      if (newVol < volMin){ 
          newVol = volMin;
      }
    Serial.print("volume:");  
    Serial.println(newVol); 
    myDFPlayer.volume(newVol);  //Set volume value. From 0 to 30 
  }
}

void MP3Choice(){
  if (home == 1){
    Serial.println(F("MP3 home"));
    Serial.println(selTitle);
    myDFPlayer.play(selTitle);
    readState = 0;
    choiceOk=0;
    home=0;
    previousTime = millis();
  }
  
  if (readState == 0){
  ChoiceMp3.tick();
  newchoice = ChoiceMp3.getPosition();

  if (choice != newchoice) {
    if (newchoice > choice){
      selTitle=selTitle+2;
        if (selTitle>fileCount){
      selTitle=1;
      }
      myDFPlayer.play(selTitle);
    } else if (newchoice < choice){
      selTitle=selTitle-2;
      if (selTitle<1){
      selTitle=fileCount-1;
      }
        myDFPlayer.play(selTitle);
      }
    choice=newchoice;
    Serial.print("title:");
    Serial.println(selTitle);
    }
  if (choiceOk == 1){
    Serial.print("title:");
    Serial.println(selTitle);
    Serial.print("story:");
    myDFPlayer.play(selStory);
    delay(100);
    readState = 1;
    delay(3000);
  } 
  }
}

void IdleCheck(){
  currentTime = millis();

  if (readState == 0 && buttOkPushed == 1 && buttHomePushed == 1 && buttPausePushed == 1){
    //Serial.print("inactivityTime:");
    //Serial.println(currentTime - previousTime);
    if (currentTime - previousTime > inactivityTime) {
      sleep();
    } 
  } else if (readState == 1 or buttOkPushed == 0 or buttHomePushed == 0 or buttPausePushed == 0){
    previousTime = currentTime;
    }
}

void buttPauseFunct(){
  buttPausePushed = digitalRead(buttPause);

  if (buttPausePushed == 0) {
    newPause = myDFPlayer.readState();
    delay(100);
    if (newPause==1){
      Serial.println("pause");
      myDFPlayer.pause();
      delay(2000);
    } else if (newPause==2){
        Serial.println("start");
        myDFPlayer.start();
        delay(2000);
      } 
  } 
}

void buttOkFunct(){
  buttOkPushed = digitalRead(buttOK);

  if (buttOkPushed == 0){
    Serial.println("Ok pushed");
    delay(1500);

    if (readState == 0 && choiceOk == 0){     //lis l'histoire choisie une fois, si lecture en cours, appui n'a aucun effet
      Serial.println("validation choix");
      selStory=selTitle+1;
      choiceOk=1;
      delay(100);
    }
  }
}

void buttHomeFunct(){
  buttHomePushed = digitalRead(buttHome);
  
  if (buttHomePushed == 0 or sleepMode == 1){
    Serial.println("Home pushed");
    delay(100); 

    myDFPlayer.stop();
    home=1;
    sleepMode=0;
    delay(1500);
  }
}

void sleep(){
  Serial.println("entrée en mode veille");
  delay(100);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
  attachInterrupt(digitalPinToInterrupt(buttHome), wakeUp, FALLING); //xternal interrupt touch type is falling edge; adjust_affichage is interrupt service function ISR
  sleep_enable();          // enables the sleep bit in the mcucr register
  sleep_cpu();
  sleep_mode();            // here the device is actually put to sleep!!
}

void wakeUp(){
  delay(100);
  sleep_disable();         // first thing after waking from sleep: disable sleep...
  delay(1000);
  Serial.println("sortie du mode veille");
  detachInterrupt(digitalPinToInterrupt(buttHome)); //disables the sleep bit in the mcucr register
  sleepMode = 1;
  delay(100);
}

void printDetail(uint8_t type, int value){
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      myDFPlayer.reset();     //Reset the module
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      if (readState == 1) {
        home=1;
      }
      previousTime = millis();
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}

 /* myDFPlayer.setTimeOut(500); //Set serial communictaion time out 500ms
  
  //----Set volume----
  myDFPlayer.volume(10);  //Set volume value (0~30).
  myDFPlayer.volumeUp(); //Volume Up
  myDFPlayer.volumeDown(); //Volume Down
  
  //----Set different EQ----
  myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
//  myDFPlayer.EQ(DFPLAYER_EQ_POP);
//  myDFPlayer.EQ(DFPLAYER_EQ_ROCK);
//  myDFPlayer.EQ(DFPLAYER_EQ_JAZZ);
//  myDFPlayer.EQ(DFPLAYER_EQ_CLASSIC);
//  myDFPlayer.EQ(DFPLAYER_EQ_BASS);
  
  //----Set device we use SD as default----
//  myDFPlayer.outputDevice(DFPLAYER_DEVICE_U_DISK);
  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
//  myDFPlayer.outputDevice(DFPLAYER_DEVICE_AUX);
//  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SLEEP);
//  myDFPlayer.outputDevice(DFPLAYER_DEVICE_FLASH);
  
  //----Mp3 control----
//  myDFPlayer.sleep();     //sleep
//  myDFPlayer.reset();     //Reset the module
//  myDFPlayer.enableDAC();  //Enable On-chip DAC
//  myDFPlayer.disableDAC();  //Disable On-chip DAC
//  myDFPlayer.outputSetting(true, 15); //output setting, enable the output and set the gain to 15
  
  //----Mp3 play----
  myDFPlayer.next();  //Play next mp3
  delay(1000);
  myDFPlayer.previous();  //Play previous mp3
  delay(1000);
  myDFPlayer.play(1);  //Play the first mp3
  delay(1000);
  myDFPlayer.loop(1);  //Loop the first mp3
  delay(1000);
  myDFPlayer.pause();  //pause the mp3
  delay(1000);
  myDFPlayer.start();  //start the mp3 from the pause
  delay(1000);
  myDFPlayer.playFolder(15, 4);  //play specific mp3 in SD:/15/004.mp3; Folder Name(1~99); File Name(1~255)
  delay(1000);
  myDFPlayer.enableLoopAll(); //loop all mp3 files.
  delay(1000);
  myDFPlayer.disableLoopAll(); //stop loop all mp3 files.
  delay(1000);
  myDFPlayer.playMp3Folder(4); //play specific mp3 in SD:/MP3/0004.mp3; File Name(0~65535)
  delay(1000);
  myDFPlayer.advertise(3); //advertise specific mp3 in SD:/ADVERT/0003.mp3; File Name(0~65535)
  delay(1000);
  myDFPlayer.stopAdvertise(); //stop advertise
  delay(1000);
  myDFPlayer.playLargeFolder(2, 999); //play specific mp3 in SD:/02/004.mp3; Folder Name(1~10); File Name(1~1000)
  delay(1000);
  myDFPlayer.loopFolder(5); //loop all mp3 files in folder SD:/05.
  delay(1000);
  myDFPlayer.randomAll(); //Random play all the mp3.
  delay(1000);
  myDFPlayer.enableLoop(); //enable loop.
  delay(1000);
  myDFPlayer.disableLoop(); //disable loop.
  delay(1000);

  //----Read imformation----
  Serial.println(myDFPlayer.readState()); //read mp3 state
  Serial.println(myDFPlayer.readVolume()); //read current volume
  Serial.println(myDFPlayer.readEQ()); //read EQ setting
  Serial.println(myDFPlayer.readFileCounts()); //read all file counts in SD card
  Serial.println(myDFPlayer.readCurrentFileNumber()); //read current play file number
  Serial.println(myDFPlayer.readFileCountsInFolder(3)); //read file counts in folder SD:/03
  }*/