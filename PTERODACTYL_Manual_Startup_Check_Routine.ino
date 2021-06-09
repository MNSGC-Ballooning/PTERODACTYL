// PTERODACTYL Startup Check Routine
// Written for PTERODACTLY Board Mk II (Designed by Andrew Van Gerpen, UMN)
// Last Updated 10/6/2020
//
// TO USE: Upload Code and follow instructions on OLED Screen.
// Warnings and errors will appear if the system thinks something is off
// Check the sensor results yourself to make sure they look okay


#include <SFE_MicroOLED.h>
#include <Wire.h>
#include <SD.h>
#include <UbloxGPS.h>
#include <TinyGPS++.h>
#include <MS5611.h>

#define PIN_RESET 9
#define DC_JUMPER 1
#define fixLED 26
#define ppodLED 24
#define xbeeLED 27 
#define sdLED 25
#define ppodSwitchPin 30
#define magnetSwitchPin 31
#define commsSwitchPin 32
#define setAltSwitch 28
#define altSwitch 29
#define baroSwitchPin 9
#define id1SwitchPin 20
#define id2SwitchPin 21
#define id3SwitchPin 22
#define id4SwitchPin 23
#define pullBeforeFlightPin 16
#define thermIntPin A16
#define thermExtPin A17
#define ubloxSerial Serial3

MicroOLED oled(PIN_RESET, DC_JUMPER);
bool sdActive = false;
File datalog;
char filename[] = "TEST00.csv";

int analogResolutionBits = 14;
float thermistorInt;
float thermistorExt;
float adcMax = pow(2,analogResolutionBits)-1.0;
float A = 0.001125308852122;
float B = 0.000234711863267;
float C = 0.000000085663516;
float R1 = 10000;
float Tinv;
float adcVal;
float logR;
float T;
float currentTempC;
float currentTempF;

MS5611 baro;
float msPressure = -1.0;
float msTemperature = -1.0;

UbloxGPS ublox(&ubloxSerial);
float altitudeFtGPS;
float latitudeGPS;
float longitudeGPS;
String groundData;
unsigned long int fixTimer = 0;
bool fix = false; // determines if the GPS has a lock

void setup() {
  Serial.begin(9600);

  //OLED
  Wire.begin();
  oled.begin();
  oled.clear(ALL);
  oled.display();
  delay(1000);
  oled.clear(PAGE); 
  randomSeed(analogRead(A0) + analogRead(A1));
  updateOled("Initializing...");
  delay(1000);
  updateOled("OLED Started!");

  //LED
  pinMode(fixLED,OUTPUT); 
  pinMode(xbeeLED,OUTPUT);
  pinMode(sdLED,OUTPUT); 
  pinMode(ppodLED,OUTPUT);
  pinMode(ppodSwitchPin,INPUT_PULLUP);
  pinMode(magnetSwitchPin,INPUT_PULLUP);
  pinMode(commsSwitchPin,INPUT_PULLUP);
  pinMode(setAltSwitch,INPUT_PULLUP);
  pinMode(altSwitch,INPUT_PULLUP);
  pinMode(id1SwitchPin,INPUT_PULLUP);
  pinMode(id2SwitchPin,INPUT_PULLUP);
  pinMode(id3SwitchPin,INPUT_PULLUP);
  pinMode(id4SwitchPin,INPUT_PULLUP);
  pinMode(pullBeforeFlightPin,INPUT_PULLUP);

  while(!baro.begin()){
    updateOled("baro init failed!");
  }
  /*if(!baro.begin()){
    Serial.println("MS5611 Altimeter not active");
    updateOled("digital baro not active");
  }*/
  updateOled("baro init\ncomplete!");
  delay(1000);

  checkLed();
  printoutln("Will now use LED " + String(ppodLED) + " for testing");
  delay(1000);
  checkSwitches();
  delay(1000);
  checkShorts();
  delay(1000);
  checkTherms();
  delay(1000);
  checkPress();
  delay(1000);
  ubloxSetup();
}

void loop() {
  checkGPS();
}

void printout(String to_print)  {
  Serial.print(to_print);
  updateOled(to_print);
  if(sdActive)  {
    datalog = SD.open(filename, FILE_WRITE);
    datalog.println(to_print);
    datalog.close();
  }
}
void printoutln(String to_print)  {
  printout(to_print + "\n");
}
void updateOled(String disp){
  oled.clear(PAGE);
  oled.setFontType(0);
  oled.setCursor(0, 0);
  oled.println(disp);
  oled.display();
}

void ubloxSetup(){
  ubloxSerial.begin(UBLOX_BAUD);
  ublox.init();
  
  byte i = 0;
  while (i<50) {
    i++;
    if (ublox.setAirborne()) {
      Serial.println("Air mode successfully set.");
      break;}
    if (i==50){
      Serial.println("Failed to set to air mode.");
      updateOled("Failed to set GPS Air Mode");
      delay(5000);
    }
  }
  updateOled("GPS init\ncomplete!");
  delay(1000);
}

void checkLed() {
  printoutln("LED 1: PPOD: Test");
  digitalWrite(ppodLED,HIGH);
  delay(2000);
  digitalWrite(ppodLED,LOW);
  printoutln("LED 2: SD: Test");
  digitalWrite(sdLED,HIGH);
  delay(2000);
  digitalWrite(sdLED,LOW);
  printoutln("LED 3: Fix: Test");
  digitalWrite(fixLED,HIGH);
  delay(2000);
  digitalWrite(fixLED,LOW);
  printoutln("LED 4: Xbee: Test");
  digitalWrite(xbeeLED,HIGH);
  delay(2000);
  digitalWrite(xbeeLED,LOW);
}

void checkTherms()  {
  analogReadResolution(analogResolutionBits);
  adcVal = analogRead(thermIntPin);
  logR = log(((adcMax/adcVal)-1)*R1);
  Tinv = A+B*logR+C*logR*logR*logR;
  T = 1/Tinv;
  currentTempC = T-273.15; // converting to celcius
  currentTempF = currentTempC*9/5+32;
  thermistorInt = currentTempF;

  adcVal = analogRead(thermExtPin);
  logR = log(((adcMax/adcVal)-1)*R1);
  Tinv = A+B*logR+C*logR*logR*logR;
  T = 1/Tinv;
  currentTempC = T-273.15; // converting to celcius
  currentTempF = currentTempC*9/5+32;
  thermistorExt = currentTempF;

  printoutln("Temps: " + String(thermistorInt) + " (Int), " + String(thermistorExt) + " (Ext) (Deg F)");
  delay(2000);
  if((thermistorInt < -20) || (thermistorExt < -20)) {
    printoutln("WARNING: Thermistor failure");
    delay(3000);
  }
}

void checkPress() {
  msTemperature = baro.readTemperature();
  msTemperature = msTemperature*(9.0/5.0) + 32.0;
  msPressure = baro.readPressure(); 
  msPressure = msPressure * 0.000145038;

  printoutln("Press: " + String(msPressure) + " (PSI)");
  delay(2000);
  if(msPressure > 15 || msPressure < 12) {
    printoutln("WARNING: Pressure Sensor failure");
    delay(3000);
  }
}

void checkGPS() {
  ublox.update();
  
  altitudeFtGPS = ublox.getAlt_feet();
  latitudeGPS = ublox.getLat();
  longitudeGPS = ublox.getLon();

  groundData = String(ublox.getMonth()) + "/" + String(ublox.getDay()) + "/" + String(ublox.getYear()) + "," +
            String(ublox.getHour()-5) + ":" + String(ublox.getMinute()) + ":" + String(ublox.getSecond()) + ","
           + String(ublox.getLat(), 4) + ", " + String(ublox.getLon(), 4) + ", " + String(altitudeFtGPS, 4);

  printoutln(String(latitudeGPS) + "\n" + String(longitudeGPS) + "\n" + String(altitudeFtGPS,1) + "ft\nInt:");
  if(ublox.getFixAge() > 2000) fix = false;
  else fix = true;
  Serial.println(groundData);
}

void checkSwitches()  {
  int timer = 10;
  unsigned long str = millis();
  while(digitalRead(altSwitch)) {
    printout("Press Altitude Switch... ");
    if(millis()-str >= 1000)  {
      str = millis();
      timer -= 1;
    }
    if(!timer)  {
      printout("ERROR: No input recieved. ");
      delay(1000);
      printoutln("Try again.");
    }
  }
  timer = 10;
  while(!digitalRead(altSwitch)) {
    printout("Release Altitude Switch... ");
    if(millis()-str >= 1000)  {
      str = millis();
      timer -= 1;
    }
    if(!timer)  {
      printout("ERROR: No input recieved. ");
      delay(1000);
      printoutln("Try again.");
      delay(1000);
    }
  }
  timer = 10;
  while(digitalRead(setAltSwitch)) {
    printout("Press Set Altitude Switch... ");
    if(millis()-str >= 1000)  {
      str = millis();
      timer -= 1;
    }
    if(!timer)  {
      printout("ERROR: No input recieved. ");
      delay(1000);
      printoutln("Try again.");
    }
  }
  timer = 10;
  while(!digitalRead(setAltSwitch)) {
    printout("Release Set Altitude Switch... ");
    if(millis()-str >= 1000)  {
      str = millis();
      timer -= 1;
    }
    if(!timer)  {
      printout("ERROR: No input recieved. ");
      delay(1000);
      printoutln("Try again.");
      delay(1000);
    }
  }
  timer = 10;
  while(digitalRead(ppodSwitchPin)) {
    printout("Engage PPOD Switch... ");
    if(millis()-str >= 1000)  {
      str = millis();
      timer -= 1;
    }
    if(!timer)  {
      printout("ERROR: No input recieved. ");
      delay(1000);
      printoutln("Try again.");
    }
  }
  timer = 10;
  while(!digitalRead(ppodSwitchPin)) {
    printout("Release PPOD Switch... ");
    if(millis()-str >= 1000)  {
      str = millis();
      timer -= 1;
    }
    if(!timer)  {
      printout("ERROR: No input recieved. ");
      delay(1000);
      printoutln("Try again.");
      delay(1000);
    }
  }
  timer = 10;
  while(digitalRead(magnetSwitchPin)) {
    printout("Engage Magnet Switch... ");
    if(millis()-str >= 1000)  {
      str = millis();
      timer -= 1;
    }
    if(!timer)  {
      printout("ERROR: No input recieved. ");
      delay(1000);
      printoutln("Try again.");
    }
  }
  timer = 10;
  while(!digitalRead(magnetSwitchPin)) {
    printout("Release Magnet Switch... ");
    if(millis()-str >= 1000)  {
      str = millis();
      timer -= 1;
    }
    if(!timer)  {
      printout("ERROR: No input recieved. ");
      delay(1000);
      printoutln("Try again.");
      delay(1000);
    }
  }
  timer = 10;
  while(digitalRead(commsSwitchPin)) {
    printout("Engage Comms Switch... ");
    if(millis()-str >= 1000)  {
      str = millis();
      timer -= 1;
    }
    if(!timer)  {
      printout("ERROR: No input recieved. ");
      delay(1000);
      printoutln("Try again.");
    }
  }
  timer = 10;
  while(!digitalRead(commsSwitchPin)) {
    printout("Release Comms Switch... ");
    if(millis()-str >= 1000)  {
      str = millis();
      timer -= 1;
    }
    if(!timer)  {
      printout("ERROR: No input recieved. ");
      delay(1000);
      printoutln("Try again.");
      delay(1000);
    }
  }
}

void checkShorts() {
  int timer = 10;
  unsigned long str = millis();
  while(digitalRead(id1SwitchPin)) {
    printout("Short Pin 20... ");
    if(millis()-str >= 1000)  {
      str = millis();
      timer -= 1;
    }
    if(!timer)  {
      printout("ERROR: No input recieved. ");
      delay(1000);
      printoutln("Try again.");
    }
  }
  timer = 10;
  while(!digitalRead(id1SwitchPin)) {
    printout("Release Altitude Switch... ");
    if(millis()-str >= 1000)  {
      str = millis();
      timer -= 1;
    }
    if(!timer)  {
      printout("ERROR: No input recieved. ");
      delay(1000);
      printoutln("Try again.");
      delay(1000);
    }
  }
  timer = 10;
  while(digitalRead(id2SwitchPin)) {
    printout("Short Pin 21... ");
    if(millis()-str >= 1000)  {
      str = millis();
      timer -= 1;
    }
    if(!timer)  {
      printout("ERROR: No input recieved. ");
      delay(1000);
      printoutln("Try again.");
    }
  }
  timer = 10;
  while(!digitalRead(id2SwitchPin)) {
    printout("Release Pin 21... ");
    if(millis()-str >= 1000)  {
      str = millis();
      timer -= 1;
    }
    if(!timer)  {
      printout("ERROR: No input recieved. ");
      delay(1000);
      printoutln("Try again.");
      delay(1000);
    }
  }
  timer = 10;
  while(digitalRead(id3SwitchPin)) {
    printout("Short Pin 22... ");
    if(millis()-str >= 1000)  {
      str = millis();
      timer -= 1;
    }
    if(!timer)  {
      printout("ERROR: No input recieved. ");
      delay(1000);
      printoutln("Try again.");
    }
  }
  timer = 10;
  while(!digitalRead(id3SwitchPin)) {
    printout("Release Pin 22... ");
    if(millis()-str >= 1000)  {
      str = millis();
      timer -= 1;
    }
    if(!timer)  {
      printout("ERROR: No input recieved. ");
      delay(1000);
      printoutln("Try again.");
      delay(1000);
    }
  }
  timer = 10;
  while(digitalRead(id4SwitchPin)) {
    printout("Short Pin 23... ");
    if(millis()-str >= 1000)  {
      str = millis();
      timer -= 1;
    }
    if(!timer)  {
      printout("ERROR: No input recieved. ");
      delay(1000);
      printoutln("Try again.");
    }
  }
  timer = 10;
  while(!digitalRead(id4SwitchPin)) {
    printout("Release Pin 23... ");
    if(millis()-str >= 1000)  {
      str = millis();
      timer -= 1;
    }
    if(!timer)  {
      printout("ERROR: No input recieved. ");
      delay(1000);
      printoutln("Try again.");
      delay(1000);
    }
  }
  timer = 10;
}
