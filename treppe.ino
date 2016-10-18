#include <Ethernet.h>
#include <SPI.h>
#include <Shys_Sensor.h>


//--------------------------------------
// Configuration Start
//--------------------------------------

//Bewegungsmelder Einstellungen
//---------------------------------
//PIR-Data-Pins
#define PIR_TOP_PIN 3
#define PIR_BOTTOM_PIN 4

// PIR-Sensor-ID
long pirTopSensorId = 22637;
long pirBottomSensorId = 22638;

//PIR Timings
// Dauer der Initialisierung
int calibrationTime = 10;        

// Wie viele ms nach der letzten erkannten Bewegung 
// soll der Bewegungsmelder wieder auf LOW schalten
long unsigned int pirPause = 5000;  

// Gibt an, ob die Werte an den Server gesendet werden sollen
// (Langsamer und benoetigt Netzwerkzugriff auf den Server)
boolean sensorValueSendToServerActive = false;


//Speed settings
// gibt an, wie lang die Pause zwischen den einzelnen Stufen in ms ist
int switchOnDelayTimeInit = 250;
int switchOffDelayTimeInit = 350;

//Maximale Wartezeit bis zur automatischen 
//Abschaltung der LEDs falls nicht der andere
//Bewegungsmelder aktiviert wird.
int maxTimeLightsOn = 30000;

// Dauer in ms, die nach der Bewegungserkennung zur Abschaltung
// gewartet werden soll
int waitBeforeSwitchOff = 3000;

// Wartezeit in ms, die nach dem Abschalten der Beleuchtung 
// gewartet werden soll, bevor die Sensoren wieder aktiv werden.
int sleepAfterLightsOff = 5000;


// Shiftregister Settings
//---------------------------------
int latchPin = 8;
int dataPin = 11;
int clockPin = 12;


// Hellligkeitssensor Settings  *optional
//-----------------------------------------
int lightPin = 7;

// Gibt ab, ob ein Lichtsensor angeschlossen ist und verwendet werden soll
boolean lightSensorActive = true;
// Gibt an, ob der Lichtsensor bei voller Helligkeit den maximal oder minimalwert angibt
// Wenn der maximalwert (HIGH bei digital, 1023 bei analog) volle Helligkeit bedeutet, 
// muss der Wert auf false stehen. Gibt der Sensor bei voller Helligkeit 0 zurück muuss hier true gesetzt werden.
boolean lightSensorSignalInverted = true;
// Gibt an ob der Sensor digital oder analog betrieben wird
boolean lightSensorDigital = true;

// Der maximale Helligkeitswert unter dem die Beleuchtung automatisch aktiviert wird. 
// bei analog: 0-1023 / bei digitalen immer 1 
int lightValueMax = 300;

// Gibt an, wie viele ms Pause zwischen den Helligkeitsmessungen liegen soll
int lightRefreshTime = 5000;

//--------------------------------------
// Configuration End
//--------------------------------------




long unsigned int pirTopHighIn;
long unsigned int pirBottomHighIn;
long unsigned int pirTopLowIn;  
boolean pirTopLockLow = true;
boolean pirTopTakeLowTime; 
boolean pirTopMotionActive = false;
boolean pirTopMotionSignalSend = false;

int switchOnDelayTime = 50;
int switchOffDelayTime = 50;

long unsigned int pirBottomLowIn;  
boolean pirBottomLockLow = true;
boolean pirBottomTakeLowTime; 
boolean pirBottomMotionActive = false;
boolean pirBottomMotionSignalSend = false;

int lightValue = 0;
int lastLightRefreshTime = 0;

byte _piAddress[] =  {192, 168, 1, 98};

byte _mac[]  = {0xDF, 0x8D, 0xCB, 0x37, 0xC4, 0xED  };
byte _ip[]   = { 192, 168, 1, 28 };
byte _dns[]  = { 192, 168, 1, 1  };
byte _gate[] = { 192, 168, 1, 1  };
byte _mask[] = { 255, 255, 255, 0  };

Shys_Sensor sensor  = Shys_Sensor(_mac, _ip, _dns, _gate, _mask, _piAddress);

/**
 * Standard Setup Methode
 */
void setup() {
  Serial.begin(9600); 
  delay(700);

  Serial.println("HomeControl - Treppensteuerung");
  Serial.println();
  
  if(lightSensorDigital){
    lightValueMax = 1;
  }
  
  if(sensorValueSendToServerActive){
    sensor.init();
  }
    
  pinMode(PIR_TOP_PIN, INPUT);
  pinMode(PIR_BOTTOM_PIN, INPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(lightPin, INPUT);
  
  Serial.print("calibrating PIRs ");
  for(int i = 0; i < calibrationTime*2; i++){
      Serial.print(".");
      delay(450);
     
      digitalWrite(latchPin, 0);
      shiftOut(random(0, 255));
      shiftOut(random(0, 255));
      delay(50);
      digitalWrite(latchPin, 1);
  }
  lightsOnUp();
  lightsOffUp();
  lightsOnDown();
  lightsOffDown();
  delay(1000);
  lightsOnAll();
  delay(1500);
  lightsOffAll();
  
  Serial.println(" done");
  Serial.println("SENSOR ACTIVE");
  delay(50);
}



/**
 * Standard Loop Methode
 */
void loop() {
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




/* *************************************************
 *  Lichtsensoren
 ************************************************* */
/**
 * Wenn der Lichtsensor aktiviert ist, 
 * gibt die Methode an, ob der Sensorwert 
 * über dem eingestellten Wert liegt.
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
 * Wartezeit seit letzter Aktualisierung überschritten wurde.
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
  if(digitalRead(PIR_TOP_PIN) == HIGH){
    if(pirTopLockLow){  
      pirTopLockLow = false;            
      //Serial.print("Top-motion detected at ");
      //Serial.print(millis()/1000);
      //Serial.println(" sec"); 
      pirTopMotionActive=true;
      pirTopMotionSignalSend=false;
      delay(50);
    }         
    pirTopHighIn = millis();
    pirTopTakeLowTime = true;
  }
  
  if(digitalRead(PIR_TOP_PIN) == LOW){       
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
      delay(50);
    }
  }  
}


/**
 *  Check PIR Sensor
 */
void refreshPIRBottomSensorValue(){
  if(digitalRead(PIR_BOTTOM_PIN) == HIGH){
    if(pirBottomLockLow){  
      pirBottomLockLow = false;            
      //Serial.print("Bottom-motion detected at ");
      //Serial.print(millis()/1000);
      //Serial.println(" sec"); 
      pirBottomMotionActive=true;
      pirBottomMotionSignalSend=false;
      delay(50);
    }         
    pirBottomHighIn = millis();
    pirBottomTakeLowTime = true;
  }
  
  if(digitalRead(PIR_BOTTOM_PIN) == LOW){       
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
      delay(50);
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
  lightsOnUp();

  pirTopLockLow = true;
    
  while ( (pirBottomHighIn + maxTimeLightsOn > millis()) && !pirTopMotionActive){
     refreshPIRTopSensorValue();
   
     delay(10);

     if(digitalRead(PIR_BOTTOM_PIN) == HIGH){
       pirBottomHighIn = millis();
     }
     
     delay(10);
  }

  delay(waitBeforeSwitchOff);

  lightsOffUp();  
  pirTopLockLow = true;
  pirBottomLockLow = true;

  delay(sleepAfterLightsOff);
}


/**
 * Steuert den Ablauf, jemand die Treppe hinunter geht
 */
void moveDown(){
  lightsOnDown();
  
  pirBottomLockLow = true;
  pirTopHighIn = millis();

  while ( (pirTopHighIn + maxTimeLightsOn > millis()) && !pirBottomMotionActive){
     refreshPIRBottomSensorValue();
     
     if(digitalRead(PIR_TOP_PIN) == HIGH){
       pirTopHighIn = millis();
     }

     delay(20);
  }

  delay(waitBeforeSwitchOff);
  
  lightsOffDown();  
  pirTopLockLow = true;
  pirBottomLockLow = true;

  delay(sleepAfterLightsOff);
}


/**
 * Schaltet alle LEDs auf einmal aus
 */
void lightsOffAll(){
  digitalWrite(latchPin, 0);
  shiftOut(0);
  shiftOut(0);
  delay(100);
  digitalWrite(latchPin, 1);
}


/**
 * Schaltet alle LEDs auf einmal ein
 */
void lightsOnAll(){
  digitalWrite(latchPin, 0);
  shiftOut(255);
  shiftOut(255);
  delay(100);
  digitalWrite(latchPin, 1);
}


/**
 * Schaltet die LEDs der Reihe nach von unten nach oben ein
 * 
 * data = 1, 3, 7, 15, 31...
 */
void lightsOnUp(){
  int data = 0;
  int additional = 1;
  for (int j = 0; j <= 8; j++) {
    digitalWrite(latchPin, 0);
    shiftOut(0);
    shiftOut(data);
    delay(100);
    digitalWrite(latchPin, 1);
    data=data+additional;
    additional = additional*2;
    delay(switchOnDelayTime);
  }
      
  data = 1;
  additional = 2;
  for (int j = 0; j < 8; j++) {
    digitalWrite(latchPin, 0);
    shiftOut(data);
    shiftOut(255);
    delay(100);
    digitalWrite(latchPin, 1);
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
  int data = 128;
  int additional = 64;
  for (int j = 0; j <= 8; j++) {
    digitalWrite(latchPin, 0);
    shiftOut(data);
    shiftOut(0);
    delay(100);
    digitalWrite(latchPin, 1);
    data=data+additional;
    additional = additional/2;
    delay(switchOnDelayTime);
  }
      
  data = 128;
  additional = 64;
  for (int j = 0; j < 8; j++) {
    digitalWrite(latchPin, 0);
    shiftOut(255);
    shiftOut(data);
    delay(100);
    digitalWrite(latchPin, 1);
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
    digitalWrite(latchPin, 0);
    shiftOut(255);
    shiftOut(255-data);
    delay(100);
    digitalWrite(latchPin, 1);
    additional=additional*2;
    data=data+additional;
    delay(switchOffDelayTime);
  }
  
  
  data = 1;
  additional = 1;
  for (int j = 0; j <= 8; j++) {
    digitalWrite(latchPin, 0);
    shiftOut(255-data);
    shiftOut(0);
    delay(100);
    digitalWrite(latchPin, 1);
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
  int data = 255;
  int additional = 128;
  for (int j = 0; j < 8; j++) {
    digitalWrite(latchPin, 0);
    shiftOut(data);
    shiftOut(255);
    delay(100);
    digitalWrite(latchPin, 1);
    data=data-additional;
    additional = additional/2;
    delay(switchOffDelayTime);
  }
  
  
  data = 255;
  additional = 128;
  for (int j = 0; j <= 8; j++) {
    digitalWrite(latchPin, 0);
    shiftOut(0);
    shiftOut(data);
    delay(100);
    digitalWrite(latchPin, 1);
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




