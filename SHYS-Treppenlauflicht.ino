#include <Ethernet.h>
#include <SPI.h>
#include <Shys_Sensor.h>

//--------------------------------------
// Configuration Start
//--------------------------------------

// PIR-Data-Pins
#define PIR_TOP_PIN 3
#define PIR_BOTTOM_PIN 4

// All LEDs on Switch
#define ALL_ON_SWITCH_PIN 5

#define RELAIS_PIN 6

// Helligkeitssensor-Pin
int lightPin = 7;

// Schieberegister-Pins
int latchPin = 8;
int dataPin = 11;
int clockPin = 12;




// Allgemeine Einstellungen
//---------------------------------
// Gibt an, ob eine Animation erfolgen soll, 
// solange die Treppe aktiv ist. (Während alle Stufen an sind)
boolean activeAnimation = true;

// Gibt ab, ob ein Lichtsensor angeschlossen ist und verwendet werden soll
boolean lightSensorActive = false;

// Gibt an, ob ein Dauer-An Schalter angeschlossen wurde 
// Sollte hier ohne Schalter am Pin hier trotzdem true eingestellt sein, 
// blinken die LEDs zufällig, da der INPUT-Pin ohne Pull-Down einen Zufallswert liefert!
boolean allOnSwitchActive = false;

// Gibt an, ob ein Relais verwendet werden soll, um das 12V Netzteil für die LEDs abzuschalten, 
// wenn die Treppe deaktiviert ist. (Dann benötigt der Arduino zwingend eine unabhängige Stromquelle!)
boolean useRelaisForPowerSupply = false;

// Gibt an, ob die Werte an den Server gesendet werden sollen
boolean sensorValueSendToServerActive = false;




// Bewegungsmelder Einstellungen
//---------------------------------
// Gibt an, ob der Lichtsensor bei Bewegung HIGH oder LOW zurück gibt
// Wenn HIGH eine erkannte Bewegung bedeutet, muss der Wert auf true stehen, sonst auf false.
boolean pirSensorSignalInverted = false;

//PIR Timings
// Dauer der Initialisierung
int calibrationTime = 10;        

// Wie viele ms nach der letzten erkannten Bewegung 
// soll der Bewegungsmelder wieder auf LOW schalten
long unsigned int pirPause = 2000;  

// PIR-Sensor-IDs (nur notwendig wenn sensorValueSendToServerActive=true)
long pirTopSensorId = 22637;
long pirBottomSensorId = 22638;


// Hellligkeitssensor Settings  *optional 
// (nur notwendig wenn lightSensorActive=true)
//-----------------------------------------
// Gibt an, ob der Lichtsensor bei voller Helligkeit den maximal oder minimalwert angibt
// Wenn der maximalwert (HIGH bei digital, 1023 bei analog) volle Helligkeit bedeutet, 
// muss der Wert auf false stehen. Gibt der Sensor bei voller Helligkeit 0 zurueck muuss hier true gesetzt werden.
boolean lightSensorSignalInverted = true;
// Gibt an ob der Sensor digital oder analog betrieben wird
boolean lightSensorDigital = true;

// Der maximale Helligkeitswert unter dem die Beleuchtung automatisch aktiviert wird. 
// bei analog: 0-1023 / bei digitalen immer 1 
int lightValueMax = 300;

// Gibt an, wie viele ms Pause zwischen den Helligkeitsmessungen liegen soll
int lightRefreshTime = 5000;



// Geschwindigkeits-Einstellungen
//---------------------------------
// gibt an, wie lang die Verzögerung zwischen den einzelnen Stufen beim einschalten in ms ist
int switchOnDelayTimeInit = 100;
// gibt an, wie lang die Verzögerung zwischen den einzelnen Stufen beim ausschalten in ms ist
int switchOffDelayTimeInit = 500;
// gibt an, wie lang die Verzögerung zwischen den einzelnen Stufen während der Animation in ms ist
int animationDelayTime = 70;


//Maximale Wartezeit bis zur automatischen 
//Abschaltung der LEDs falls der andere
//Bewegungsmelder nicht aktiviert wird.
int maxTimeLightsOn = 30000;

// Dauer in ms, die nach der Bewegungserkennung 
// bis zur Abschaltung gewartet werden soll
int waitBeforeSwitchOff = 3000;

// Wartezeit in ms, die nach dem Abschalten der Beleuchtung 
// gewartet werden soll, bevor die Sensoren wieder aktiviert werden.
int sleepAfterLightsOff = 5000;



// Netzwerk-Einstellungen
// Diese sind nur notwendig wenn sensorValueSendToServerActive aktiviert ist.
//--------------------------------------
// IP-Adresse des SmartHome yourself Servers
byte _piAddress[] =  {192, 168, 1, 98};

byte _mac[]  = {0xDF, 0x8D, 0xCB, 0x37, 0xC4, 0xED  };
byte _ip[]   = { 192, 168, 1, 28 };
byte _dns[]  = { 192, 168, 1, 1  };
byte _gate[] = { 192, 168, 1, 1  };
byte _mask[] = { 255, 255, 255, 0  };

//--------------------------------------
// Configuration End
//--------------------------------------



byte tmpShift;

long unsigned int pirTopHighIn;
long unsigned int pirBottomHighIn;
long unsigned int pirTopLowIn;  
boolean pirTopLockLow = true;
boolean pirTopTakeLowTime; 
boolean pirTopMotionActive = false;
boolean pirTopMotionSignalSend = false;

int switchOnDelayTime = 20;
int switchOffDelayTime = 20;

long unsigned int pirBottomLowIn;  
boolean pirBottomLockLow = true;
boolean pirBottomTakeLowTime; 
boolean pirBottomMotionActive = false;
boolean pirBottomMotionSignalSend = false;

int lightValue = 0;
int lastLightRefreshTime = 0;

Shys_Sensor sensor  = Shys_Sensor(_mac, _ip, _dns, _gate, _mask, _piAddress);

/**
 * Standard Setup Methode
 */
void setup() {
  Serial.begin(9600); 

  pinMode(PIR_TOP_PIN, INPUT);
  pinMode(PIR_BOTTOM_PIN, INPUT);
  pinMode(ALL_ON_SWITCH_PIN, INPUT);
  pinMode(RELAIS_PIN, OUTPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(lightPin, INPUT);
  
  delay(700);

  Serial.println("HomeControl - Treppensteuerung");
  Serial.println();
  
  if(lightSensorDigital){
    lightValueMax = 1;
  }
  
  if(sensorValueSendToServerActive){
    sensor.init();
  }

  Serial.print("calibrating PIRs ");
  for(int i = 0; i < calibrationTime*2; i++){
      Serial.print(".");
      delay(450);
     
      digitalWrite(latchPin, LOW);
      shiftOut(random(0, 255));
      shiftOut(random(0, 255));
      delay(50);
      digitalWrite(latchPin, HIGH);
  }
  
  lightsOnUp();
  lightsOffUp();
  lightsOnDown();
  lightsOffDown();

  delay(1000);
  lightsOnAll();
  delay(1500);
  lightsOffAll();

  switchOnDelayTime = switchOnDelayTimeInit;
  switchOffDelayTime = switchOffDelayTimeInit;

  Serial.println(" done");
  Serial.println("SENSOR ACTIVE");
  delay(50);
}
  

char * int2bin(byte x)
{
 static char buffer[8];
 for (int i=0; i<8; i++) buffer[7-i] = '0' + ((x & (1 << i)) > 0);
 buffer[8] ='\0';
 return buffer;
}


/**
 * Standard Loop Methode
 */
void loop() {
  checkSwitchAllOn();
  
  refreshSensors();
  
  if(isLightvalueOk()){
    if(pirTopMotionActive && !pirTopMotionSignalSend){
      resetPirs();
  
      moveDown();
      
      if(sensorValueSendToServerActive){
       sensor.setSensorValue(pirTopSensorId, pirTopMotionActive?1:0);
      }
          
      resetPirs();
    }
  
    if(pirBottomMotionActive && !pirBottomMotionSignalSend){
      resetPirs();
  
      moveUp();
      
      if(sensorValueSendToServerActive){
        sensor.setSensorValue(pirBottomSensorId, pirBottomMotionActive?1:0);
      }
      
      resetPirs();
    }
  }

  delay(100);
}




/**
 * Dauer-An Schalter
 */
 void checkSwitchAllOn(){
   if(!allOnSwitchActive){
     return;
   }
   if(digitalRead(ALL_ON_SWITCH_PIN)==HIGH){
     lightsOnAll();
   }
   while(digitalRead(ALL_ON_SWITCH_PIN)==HIGH){
     delay(200);
   }
   lightsOffAll();
 }


/* *************************************************
 *  Lichtsensoren
 ************************************************* */
/**
 * Wenn der Lichtsensor aktiviert ist, 
 * gibt die Methode an, ob der Sensorwert 
 * Ã¼ber dem eingestellten Wert liegt.
 * 
 * Bei deaktiviertem Lichtsensor liefert sie immer true.
 */
boolean isLightvalueOk(){
  if(lightSensorActive){
    return lightValue < lightValueMax;
  } else {
    return true;
  }
}


/**
 * aktualisiert den Sensorwert wenn die Eingestellte
 * Wartezeit seit letzter Aktualisierung Ã¼berschritten wurde.
 */
void refreshLightSensor(){
  if(lightSensorActive) {
    int diff = lastLightRefreshTime + lightRefreshTime - millis();
    int lastVal = lightValue;
    delay(10);
    if(diff < 0){
      if(!lightSensorDigital){
        int tmpVal = analogRead(lightPin);
        for(int i=0; i<50; i++){
          int newVal = analogRead(lightPin);
          tmpVal = (tmpVal + newVal) / 2;
          delay(10);
        }
        lightValue = lightSensorSignalInverted?1024-tmpVal:tmpVal;
      } else {
        lightValue = (lightSensorSignalInverted?(!digitalRead(lightPin) == HIGH):(digitalRead(lightPin) == HIGH))?1:0;
      }

      if(lastVal!=lightValue){
        //Serial.print("Lichtsensor neuer Wert: ");
        //Serial.println(lightValue);
      }
      lastLightRefreshTime = millis();
    }
  }
  delay(10);
}

/* *************************************************
 * PIR Sensoren
 ************************************************* */
void resetPirs(){
    pirTopMotionSignalSend=true;
    pirBottomMotionSignalSend=true;
    pirTopMotionActive=false;
    pirBottomMotionActive=false;
}


void refreshSensors(){
  refreshLightSensor();

  if(isLightvalueOk()){
    refreshPIRTopSensorValue();
    refreshPIRBottomSensorValue();
  } else {
    resetPirs();
  }
}


/**
 *  Check PIR Sensor
 */
void refreshPIRTopSensorValue(){
  if(digitalRead(PIR_TOP_PIN) == pirSensorSignalInverted?LOW:HIGH){
    if(pirTopLockLow){  
      pirTopLockLow = false;            
      //Serial.print("Top-motion detected at ");
      //Serial.print(millis()/1000);
      //Serial.println(" sec"); 
      pirTopMotionActive=true;
      pirTopMotionSignalSend=false;
      delay(10);
    }         
    pirTopHighIn = millis();
    pirTopTakeLowTime = true;
  }
  
  if(digitalRead(PIR_TOP_PIN) == pirSensorSignalInverted?HIGH:LOW){       
    if(pirTopTakeLowTime){
      pirTopLowIn = millis();
      pirTopTakeLowTime = false;
    }
    
    if(!pirTopLockLow && millis() - pirTopLowIn > pirPause){  
      pirTopLockLow = true;                        
      //Serial.print("motion ended at ");
      //Serial.print((millis() - pirPause)/1000);
      //Serial.println(" sec");
      pirTopMotionActive=false;
      pirTopMotionSignalSend=false;
      delay(10);
    }
  }  
}


/**
 *  Check PIR Sensor
 */
void refreshPIRBottomSensorValue(){
  if(digitalRead(PIR_BOTTOM_PIN) == pirSensorSignalInverted?LOW:HIGH){
    if(pirBottomLockLow){  
      pirBottomLockLow = false;            
      //Serial.print("Bottom-motion detected at ");
      //Serial.print(millis()/1000);
      //Serial.println(" sec"); 
      pirBottomMotionActive=true;
      pirBottomMotionSignalSend=false;
      delay(10);
    }         
    pirBottomHighIn = millis();
    pirBottomTakeLowTime = true;
  }
  
  if(digitalRead(PIR_BOTTOM_PIN) == pirSensorSignalInverted?HIGH:LOW){       
    if(pirBottomTakeLowTime){
      pirBottomLowIn = millis();
      pirBottomTakeLowTime = false;
    }
    
    if(!pirBottomLockLow && millis() - pirBottomLowIn > pirPause){  
      pirBottomLockLow = true;                        
      //Serial.print("Bottom motion ended at ");      //output
      //Serial.print((millis() - pirPause)/1000);
      //Serial.println(" sec");
      pirBottomMotionActive=false;
      pirBottomMotionSignalSend=false;
      delay(10);
    }
  }  
}





/* *************************************************
 * LED Steuerung 
 ************************************************* */
/**
 * Steuert den Ablauf, jemand die Treppe herauf geht
 */
void moveUp(){
  switchRelais(true);
  
  lightsOnUp();

  pirTopLockLow = true;
  tmpShift = 254;
  int aktRegister = 1;
  
  while ( (pirBottomHighIn + maxTimeLightsOn > millis()) && !pirTopMotionActive){
     refreshPIRTopSensorValue();
   
     delay(10);

     if(digitalRead(PIR_BOTTOM_PIN) == pirSensorSignalInverted?LOW:HIGH){
       pirBottomHighIn = millis();
     }

     if (activeAnimation){
       //Serial.println(int2bin(tmpShift));
       digitalWrite(latchPin, LOW);
       if(aktRegister==1){
         shiftOut(255);
         shiftOut(tmpShift);
       } else {
         shiftOut(tmpShift);    
         shiftOut(255);
       }
       digitalWrite(latchPin, HIGH);
  
       if(tmpShift>127){
         tmpShift = tmpShift<<1;
         tmpShift = tmpShift+1;
       } else {
         tmpShift = 254;
         aktRegister=aktRegister==1?0:1;
       }
       delay(animationDelayTime);
     }

     delay(10);        
  }
  
  long waitTmpms = millis();
  boolean onEnd = tmpShift == 254;
  while ( waitTmpms + waitBeforeSwitchOff >= millis() || !onEnd){
     if (activeAnimation){
       //Serial.println(int2bin(tmpShift));
       digitalWrite(latchPin, LOW);
       if(aktRegister==1){
         shiftOut(255);
         shiftOut(tmpShift);
       } else {
         shiftOut(tmpShift);    
         shiftOut(255);
       }       
       digitalWrite(latchPin, HIGH);
  
       if(tmpShift>127){
         onEnd = false;
         tmpShift = tmpShift<<1;
         tmpShift = tmpShift+1;
       } else {
         onEnd = aktRegister==1;
         tmpShift = 254;
         aktRegister=aktRegister==1?0:1;
       }
       delay(animationDelayTime);
     }
  }
  
  lightsOffUp();  
  pirTopLockLow = true;
  pirBottomLockLow = true;

  switchRelais(false);

  delay(sleepAfterLightsOff);
}


/**
 * Steuert den Ablauf, jemand die Treppe hinunter geht
 */
void moveDown(){
  switchRelais(true);
  
  lightsOnDown();
  
  pirBottomLockLow = true;
  pirTopHighIn = millis();
  tmpShift = 127;
  int aktRegister = 0;

  while ( (pirTopHighIn + maxTimeLightsOn > millis()) && !pirBottomMotionActive){
     refreshPIRBottomSensorValue();

     delay(10);
     
     if(digitalRead(PIR_TOP_PIN) == pirSensorSignalInverted?LOW:HIGH){
       pirTopHighIn = millis();
     }

     if(activeAnimation){
      // Serial.println(int2bin(tmpShift));
       digitalWrite(latchPin, LOW);
       if(aktRegister==1){
         shiftOut(255);
         shiftOut(tmpShift);
       } else {
         shiftOut(tmpShift);    
         shiftOut(255);
       }
       digitalWrite(latchPin, HIGH);
  
       if(tmpShift<254){
         tmpShift = tmpShift>>1;
         tmpShift = tmpShift-128;
       } else {
         tmpShift = 127;
         aktRegister=aktRegister==1?0:1;
       }
       delay(animationDelayTime);
     }
          
     delay(10);
  }

  
  long waitTmpms = millis();
  boolean onEnd = tmpShift == 127;
  while ( waitTmpms + waitBeforeSwitchOff > millis() || !onEnd){
     if (activeAnimation){
       //Serial.println(int2bin(tmpShift));
       digitalWrite(latchPin, LOW);
       if(aktRegister==1){
         shiftOut(255);
         shiftOut(tmpShift);
       } else {
         shiftOut(tmpShift);    
         shiftOut(255);
       }
       digitalWrite(latchPin, HIGH);
  
       if(tmpShift<254){
         onEnd = false;
         tmpShift = tmpShift>>1;
         tmpShift = tmpShift-128;
       } else {
         onEnd = aktRegister==1;
         tmpShift = 127;
         aktRegister=aktRegister==1?0:1;
       }      
     }
     delay(animationDelayTime);
  }
    
  lightsOffDown();  
  pirTopLockLow = true;
  pirBottomLockLow = true;

  switchRelais(false);

  delay(sleepAfterLightsOff);
}


/**
 * Schaltet alle LEDs auf einmal aus
 */
void lightsOffAll(){
  digitalWrite(latchPin, LOW);
  shiftOut(0);
  shiftOut(0);
  delay(100);
  digitalWrite(latchPin, HIGH);

  switchRelais(false);
}


/**
 * Schaltet alle LEDs auf einmal ein
 */
void lightsOnAll(){
  switchRelais(true);

  digitalWrite(latchPin, LOW);
  shiftOut(255);
  shiftOut(255);
  delay(100);
  digitalWrite(latchPin, HIGH);
}


/**
 * Schaltet die LEDs der Reihe nach von unten nach oben ein
 * 
 * data = 1, 3, 7, 15, 31...
 */
void lightsOnUp(){
  Serial.println("UP");
  int data = 1;
  int additional = 2;
  for (int j = 0; j < 8; j++) {
    digitalWrite(latchPin, LOW);
    shiftOut(0);
    shiftOut(data);
    digitalWrite(latchPin, HIGH);

//    Serial.print(int2bin(0));
//    Serial.println(int2bin(data));

    data=data+additional;
    additional = additional*2;
    delay(switchOnDelayTime);
  }
      
  data = 1;
  additional = 2;
  for (int j = 0; j < 8; j++) {
    digitalWrite(latchPin, LOW);
    shiftOut(data);
    shiftOut(255);
    digitalWrite(latchPin, HIGH);

//    Serial.print(int2bin(data));
//    Serial.println(int2bin(255));
    
    data=data+additional;
    additional = additional*2;
    delay(switchOnDelayTime);
  }
}



/**
 * Schaltet die LEDs der Reihe nach von oben nach unten ein
 * 
 * data = 128, 192, 224,...
 */
void lightsOnDown(){
  Serial.println("DOWN");
  int data = 128;
  int additional = 64;
  for (int j = 0; j < 8; j++) {
    digitalWrite(latchPin, LOW);
    shiftOut(data);
    shiftOut(0);
    digitalWrite(latchPin, HIGH);

//    Serial.print(int2bin(data));
//    Serial.println(int2bin(0));
    
    data=data+additional;
    additional = additional/2;
    delay(switchOnDelayTime);
  }
      
  data = 128;
  additional = 64;
  for (int j = 0; j < 8; j++) {
    digitalWrite(latchPin, LOW);
    shiftOut(255);
    shiftOut(data);
    digitalWrite(latchPin, HIGH);

//    Serial.print(int2bin(255));
//    Serial.println(int2bin(data));

    data=data+additional;
    additional = additional/2;
    delay(switchOnDelayTime);
  }
}


/**
 * Schaltet der Reihe nach die LEDs von unten nach oben aus
 * 
 * data = 255, 254, 251, 244, ...
 */
void lightsOffUp(){
  int data = 1;
  int additional = 1;
  for (int j = 0; j < 8; j++) {
    digitalWrite(latchPin, LOW);
    shiftOut(255);
    shiftOut(255-data);
    digitalWrite(latchPin, HIGH);

//    Serial.print(int2bin(255));
//    Serial.println(int2bin(255-data));

    additional=additional*2;
    data=data+additional;
    delay(switchOffDelayTime);
  }
  
  
  data = 1;
  additional = 1;
  for (int j = 0; j < 8; j++) {
    digitalWrite(latchPin, LOW);
    shiftOut(255-data);
    shiftOut(0);
    digitalWrite(latchPin, HIGH);
    
//    Serial.print(int2bin(255-data));
//    Serial.println(int2bin(0));

    additional=additional*2;
    data=data+additional;
    delay(switchOffDelayTime);
  }
}

/**
 * Schaltet der Reihe nach die LEDs von oben nach unten aus
 * 
 * data = 255, 128, 64, 32, ...
 */
void lightsOffDown(){
  int data = 127;
  int additional = 64;
  for (int j = 0; j < 8; j++) {
    digitalWrite(latchPin, LOW);
    shiftOut(data);
    shiftOut(255);
    digitalWrite(latchPin, HIGH);

//    Serial.print(int2bin(data));
//    Serial.println(int2bin(255));
    
    data=data-additional;
    additional = additional/2;
    delay(switchOffDelayTime);
  }
  
  data = 127;
  additional = 64;
  for (int j = 0; j < 8; j++) {
    digitalWrite(latchPin, LOW);
    shiftOut(0);
    shiftOut(data);
    digitalWrite(latchPin, HIGH);

//    Serial.print(int2bin(0));
//    Serial.println(int2bin(data));

    data=data-additional;
    additional = additional/2;
    delay(switchOffDelayTime);
  }
}



void shiftOut(byte dataOut) {
  int i=0;
  int pinState;

  digitalWrite(dataPin, 0);
  digitalWrite(clockPin, 0);
  for (i=7; i>=0; i--)  {
    digitalWrite(clockPin, 0);

    if ( dataOut & (1<<i) ) {
      pinState= 1;
    } else {  
      pinState= 0;
    }

    digitalWrite(dataPin, pinState);
    digitalWrite(clockPin, 1);
    digitalWrite(dataPin, 0);
  }

  digitalWrite(clockPin, 0);
}


/**
 * Schaltet (falls aktiviert) das Relais je nach dem übergebenen Wert für activate  ein bzw. aus.
 */
void switchRelais(boolean activate) {
  if(useRelaisForPowerSupply){
      digitalWrite(RELAIS_PIN, activate?1:0);
      if (activate){
        delay(250);  
      }
  }
}
