#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include "RotaryEncoder.h"
RotaryEncoder StartAndVol(A2, A3);  // (DT, CLK)
RotaryEncoder ChoiceMp3(A4, A5);  // (DT, CLK)

SoftwareSerial mySoftwareSerial(11, 12); // RX, TX
DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);

const int buttOK = 2; // bouton de validation OK
const int buttHome = 1; // bouton de de retour/home

int vol=0;
int volMin = 5;
int volMax = 25;
int LuniiOn=0;

static int pos = 0;
static int choice = 0;
int newPos;
int newchoice;
int selMP3=1;
int selFolder=1;

int buttOkPushed=1;
int buttHomePushed=1;

void setup()
{
  pinMode(buttHome, INPUT);
  pinMode(buttOK, INPUT_PULLUP);  //the button pin is set to input pull-up mode
  attachInterrupt(digitalPinToInterrupt(buttOK), buttOkInterrupt, FALLING); //xternal interrupt touch type is falling edge; adjust_affichage is interrupt service function ISR
  attachInterrupt(A2, OnOffVolInterrupt, CHANGE); //xternal interrupt touch type is falling edge; adjust_affichage is interrupt service function ISR

  StartAndVol.tick();
  newPos = StartAndVol.getPosition();

  mySoftwareSerial.begin(9600);
  Serial.begin(115200);

  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

  myDFPlayer.setTimeOut(2000); //Set serial communictaion time out 2000ms
  
  if (!myDFPlayer.begin(mySoftwareSerial)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true);
  }
  Serial.println(F("DFPlayer Mini online."));

  myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
}

void loop(){
  OnOffVol();
  MP3Choice();

  if (myDFPlayer.available()) {
    printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
  }
}

void OnOffVol(){
  StartAndVol.tick();
  newPos = StartAndVol.getPosition();
  
  if (pos != newPos) {
      pos = newPos;
      vol=pos*5;
      Serial.println("entrée dans le if");
      if (vol == volMin && LuniiOn == 0){
          LuniiOn=1;
          Serial.println(F("MP3 play"));
          myDFPlayer.playFolder(selFolder,13);  //play specific mp3 in SD:/15/004.mp3; Folder Name(1~99); File Name(1~255)
        }
      if (vol > volMax)
        { 
          Serial.println("vol+ ok");
          vol = volMax;
        } 
      if (vol == 0 && LuniiOn == 1) 
        {
          LuniiOn = 0; 
          myDFPlayer.stop();
          myDFPlayer.sleep();     //sleep
        }
    myDFPlayer.volume(vol);  //Set volume value. From 0 to 30
  }
}

void MP3Choice(){
  int fileCount=0;
  int selPerso=0;
  int selLieu=0;
  int selObjet=0;

  buttOkPushed = digitalRead(buttOK);

  if (LuniiOn==1 && selPerso == 0 && selLieu == 0 && selObjet == 0 && buttOkPushed == 0){
      delay(200);
      buttOkPushed = 1;

      selFolder = ++selFolder;
      fileCount = myDFPlayer.readFileCountsInFolder(selFolder); //read file counts in folder SD:/03
      myDFPlayer.playFolder(2,1);

      while (buttOkPushed == 1){
        ChoiceMp3.tick();
        newchoice = ChoiceMp3.getPosition();

        if (choice != newchoice) {
          if (newchoice > choice){
            selMP3=++selMP3;
            if (selMP3 > fileCount){
              selMP3 = 1;
            }
          } else if (newchoice < choice){
              selMP3=--selMP3;
                if (selMP3 < 1){
                  selMP3 = fileCount;
                }
            }
          myDFPlayer.playFolder(selFolder,selMP3);
          choice=newchoice;
          delay(500);
          }
      }
      selPerso=1;
  } 

  if (LuniiOn==1 && selPerso == 1 && selLieu == 0 && selObjet == 0){
    Serial.println("selection Lieu");
      delay(200);
      buttOkPushed = 1;

      selFolder = ++selFolder;
      selMP3 = 4;

      myDFPlayer.readFileCountsInFolder(selFolder); //read file counts in folder SD:/03
      fileCount = myDFPlayer.readFileCountsInFolder(selFolder);

      myDFPlayer.playFolder(selFolder,3);
      delay(3500);
      myDFPlayer.playFolder(selFolder,4);

      while (buttOkPushed == 1){
        ChoiceMp3.tick();
        newchoice = ChoiceMp3.getPosition();

        if (choice != newchoice) {
          if (newchoice > choice){
            selMP3=++selMP3;
            if (selMP3 > fileCount+2){
              selMP3 = 4;
            }
          } else if (newchoice < choice){
              selMP3=--selMP3;
                if (selMP3 < 4){
                  selMP3 = fileCount+2;
                }
            }
          myDFPlayer.playFolder(selFolder,selMP3);
          choice=newchoice;
          delay(500);
          }
      }
      selLieu=1;
      Serial.println("selection Lieu OK");
  }

  if (LuniiOn==1 && selPerso == 1 && selLieu == 1 && selObjet == 0){
    Serial.println("selection objet");
      delay(200);
      buttOkPushed = 1;

      selFolder = ++selFolder;
      selMP3 = 7;

      myDFPlayer.readFileCountsInFolder(selFolder); //read file counts in folder SD:/03
      fileCount = myDFPlayer.readFileCountsInFolder(selFolder);

      myDFPlayer.playFolder(selFolder,6);
      delay(3500);
      myDFPlayer.playFolder(selFolder,7);

      while (buttOkPushed == 1){
        ChoiceMp3.tick();
        newchoice = ChoiceMp3.getPosition();

        if (choice != newchoice) {
          if (newchoice > choice){
            selMP3=++selMP3;
            if (selMP3 > fileCount+4){
              selMP3 = 7;
            }
          } else if (newchoice < choice){
              selMP3=--selMP3;
                if (selMP3 < 7){
                  selMP3 = fileCount+4;
                }
            }
          myDFPlayer.playFolder(selFolder,selMP3);
          choice=newchoice;
          delay(500);
          }
      }
      selObjet=1;
      Serial.println("selection Objet OK");
  }

  if (LuniiOn==1 && selPerso == 1 && selLieu == 1 && selObjet == 1){
    buttOkPushed = 1;
    //if (heros == 001 && lieu == 002 && objet ==003) {}
    selFolder = ++selFolder;
    Serial.println("lecture histoire composée");
    Serial.println(selFolder);
    myDFPlayer.playFolder(6,12);
  }


  /*buttHomePushed = digitalRead(buttHome);
  if (buttHomePushed == 0){
      myDFPlayer.reset();     //Reset the module
  }*/

  if (myDFPlayer.available() && myDFPlayer.readType() == DFPlayerPlayFinished) {
    //myDFPlayer.playFolder(1,13);
  }
}

void printDetail(uint8_t type, int value){
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
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

void OnOffVolInterrupt() {
  StartAndVol.tick();
  newPos = StartAndVol.getPosition();
  
  if (pos != newPos) {
      pos = newPos;
      vol=pos*5;
      Serial.println("entrée dans le if");
      if (vol == volMin && LuniiOn == 0){
          LuniiOn=1;
          Serial.println(F("MP3 play"));
          myDFPlayer.playFolder(selFolder,13);  //play specific mp3 in SD:/15/004.mp3; Folder Name(1~99); File Name(1~255)
        }
      if (vol > volMax)
        { 
          Serial.println("vol+ ok");
          vol = volMax;
        } 
      if (vol == 0 && LuniiOn == 1) 
        {
          LuniiOn = 0; 
          myDFPlayer.stop();
          myDFPlayer.sleep();     //sleep
        }
    myDFPlayer.volume(vol);  //Set volume value. From 0 to 30
  }
}

void buttOkInterrupt() {
    buttOkPushed=0;
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