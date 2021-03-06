#include <SD.h>
#define fixLED 26                 // LED pin to indicate GPS fix
#define ppodLED 24                // LED pin to indicate PPOD status
#define xbeeLED 27                // LED pin to indicate xbee communication
#define sdLED 25                  // LED pin to indicate sd status
#define serialBAUD 9600           // when using arduino serial monitor, make sure baud rate is set to this same value

#define ppodSwitchPin 30          
#define satSwitchPin 31
#define commsSwitchPin 32
#define setAltSwitch 28
#define altSwitch 29
#define baroSwitchPin 9
#define id1SwitchPin 20
#define id2SwitchPin 21
#define id3SwitchPin 22
#define id4SwitchPin 23
#define pullBeforeFlightPin 16

#define chipSelect BUILTIN_SDCARD //Should highlight if you have teensy 3.5/3.6/4.0 selected


int rfd900 = 1; // set true if you want this thing to operate with a rfd900 on serialX (declare above)
int ppod = 1; // set true if you want this thing to operate as ppod flight computer
int satCom = 1; // set true if you want to activate flight by waving a magnet over the IMU
int setAltVal = 1;
int altVal = 1;
int id1On = 1;
int id2On = 1;
int id3On = 1;
int id4On = 1;
int pullOn = 1;

float cutTime = 70*60000.0; // x (minutes) * 60000;
float cutAltitude = 10000; // 50,000 ft ASL cut!!
bool smartRelease = false; // Releases the smart unit to deploy cubes
int smartReleasePosition = 170; // the angle in degrees for smart release. Will vary based on smart unit
int smartInitialPosition = 0; // the angle in degrees for smart initial position. Will vary based on smart unit
bool smartReleaseTransmission = true; // This ensures only one message is relayed to the ground after release has occurred.
String commandMessage; // Appends a message stating the radio deployed the cubes if that happens
String proCommand;
String satComPacket;
int lineNumber = 0;

String header = "Year, Month, Day, Hour, Minute, Second, Lat, Lon, Alt(ft), AltEst(ft), intT(F), extT(F), batTemp(F), msTemp(F), analogPress(PSI), msPressure(PSI), time since pin pulled (sec), magnetometer x, magnetometer y, magnetometer z, accelerometer x, accelerometer y, accelerometer z, gyroscope x, gyroscope y, gyroscope z, Heater Status, Siren Status, Recent Radio Traffic";
unsigned long int dataTimer = 0;
unsigned long int dataTimerIMU = 0;
unsigned long int ppodOffset = 0;
int dataRate = 1000; // 1000 millis = 1 second
float pressureBoundary1;
float pressureBoundary2;
float pressureBoundary3;
float msPressure = -1.0;
float msTemperature = -1.0;
float altitudeFt = -1.0;
unsigned long int fixTimer = 0;
bool fix = false; // determines if the GPS has a lock

///////////////////// Sensor Global Variables /////////////////////////////

float dallasOneF;
float thermistorInt;
float thermistorExt;
float magnetometer[3]; // {x, y, z}
float accelerometer[3]; // {x, y, z}
float gyroscope[3]; // {x, y, z}

float altitudeFtGPS;
float latitudeGPS;
float longitudeGPS;
String data;
String groundData;
String IMUdata;

String exclamation = "!"; // this needs to be at the end of every 
bool commandRelease = false;
String groundCommand;
String xbeeID = "UMN0";
String xbeeProID = "AAA";
bool xbeeAlternation = false; // this will allow the xbee to alternate between sending data to MOC SOC and data via MOC SOC to the ground
unsigned long int xbeeTimer = 0;
unsigned long int xbeeRate = 5000; // 10000 millis = 10 seconds
String xbeeMessage; // This saves all xbee transmissions and appends them to the data string

int group0packet[] = {0,0,0,0,0}; // Each group gets 5 integer values (10 bytes)
int group1packet[] = {0,0,0,0,0}; // Each group gets 5 integer values (10 bytes)
byte statusByte;
byte IDByte = 0xbb;
boolean statusArr[] = {0,0,0,0,0,0,0,0};
int statusInt=0;


String heaterStatus;
String sirenStatus;


////////////Global SD varibles////////////

File datalog;
File datalogIMU;
File sender; ///NEWNEWNEW
char filename[] = "SDCARD00.csv";
char senderName[] = "SDCARD00.csv"; /// NEWNEWNEW
bool sdActive = false;

void setup() {
  
  Serial.begin(serialBAUD); //define baud rate in variable decleration above
  Serial.println("Serial online");
  pinMode(fixLED,OUTPUT); 
  pinMode(xbeeLED,OUTPUT);
  pinMode(sdLED,OUTPUT); 
  pinMode(ppodLED,OUTPUT);
  pinMode(13,OUTPUT);
  pinMode(setAltSwitch, INPUT_PULLUP);
  pinMode(altSwitch, INPUT_PULLUP);
  checkSwitches(); // slide and button switch status

  if(id1On==0)xbeeID="UMN1";
  if(id2On==0)xbeeID="UMN2";
  if(id3On==0)xbeeID="UMN3";
  if(id4On==0)xbeeID="UMN4";
  if(rfd900==0)xbeeID = "COMM";
  if(ppod==0)xbeeID = "PPOD";
  if(satCom==0 && rfd900==0) xbeeID = "SATC"; 

  Serial.print("starting OLED setup... ");
  oledSetup();
  Serial.println("OLED setup complete");
  
  Serial.print("starting IMU setup... ");
  imuSetup();
  updateIMU();
  Serial.println("IMU setup complete");

  Serial.print("starting Altimeter setup... ");
  msSetup();
  Serial.println("Altimeter setup complete");

  Serial.print("starting SD setup... ");
  sdSetup();
  Serial.println("SD setup complete");

  Serial.print("starting xbee setup... ");
  xbeeSetup();
  Serial.println("xbee setup complete");

  Serial.print("starting ublox setup... ");
  ubloxSetup();
  Serial.println("ublox setup complete");

  Serial.print("starting heater setup... ");
  heaterSetup();
  Serial.println("heater setup complete");

  if(satCom==1)//If Satcom not connected, using Satcom serial pins (default, can change in code)
  {
    Serial.print("starting siren setup... ");
    sirenSetup();
    Serial.println("siren setup complete");
  }
  
  pressureToAltitudeSetup();
  if(ppod==0) ppodSetup();
  if(rfd900==0) rfd900Setup();
  logData(header);
  if(pullOn==0) pullPin();
  if(satCom==0) satComSetup(); 

  Serial.println("rfd900:"+ String(rfd900) +"," + "satCom:"+ String(satCom));
}

void loop() {
  updateData(); 
}

// Pressure to altitude function setup
void pressureToAltitudeSetup()
{
  float h1 = 36152.0;
  float h2 = 82345.0;
  float T1 = 59-.00356*h1;
  float T2 = -70;
  float T3 = -205.05 + .00164*h2;
  pressureBoundary1 = (2116 * pow(((T1+459.7)/518.6),5.256));
  pressureBoundary2 = (473.1*exp(1.73-.000048*h2)); // does exp function work??
  pressureBoundary3 = (51.97*pow(((T3 + 459.7)/389.98),-11.388));
}

void pressureToAltitude(){
  float pressurePSF = (msPressure*144);
  
  float altFt = -100.0;
  if (pressurePSF > pressureBoundary1){// altitude is less than 36,152 ft ASL
     altFt = (459.7+59-518.6*pow((pressurePSF/2116),(1/5.256)))/.00356;
  }
  else if (pressurePSF <= pressureBoundary1 && pressurePSF > pressureBoundary2){ // altitude is between 36,152 and 82,345 ft ASL
    altFt = (1.73-log(pressurePSF/473.1))/.000048;
  }
  else if (pressurePSF <= pressureBoundary2){// altitude is greater than 82,345 ft ASL
    altFt = (459.7-205.5-389.98*pow((pressurePSF/51.97),(1/-11.388)))/-.00164;
  } 
  altitudeFt = altFt;
}

void updateData(){
  updateUblox();
  updateXbee();
  updateXbeePro();
  sirenUpdate();

  if(millis() - dataTimer > dataRate){
    dataTimer = millis();
    
    if(fix == true){digitalWrite(fixLED,HIGH);}
    
    digitalWrite(sdLED,HIGH);
    delay(30);
    digitalWrite(sdLED,LOW);
    digitalWrite(fixLED,LOW);

    pressureToAltitude();
    updateThermistor();
    updateMS();    
    updateIMU();
    updateSmart();
    updateDataStrings();
    setHeaterState();
    sirenUpdate();
    xbeeMessage="";
  }        
}

// Initialize Pullup/Pulldown pins for switches
void checkSwitches(){
  pinMode(ppodSwitchPin, INPUT_PULLUP);
  pinMode(satSwitchPin, INPUT_PULLUP);
  pinMode(commsSwitchPin, INPUT_PULLUP);
  pinMode(id1SwitchPin, INPUT_PULLUP);
  pinMode(id2SwitchPin, INPUT_PULLUP);
  pinMode(id3SwitchPin, INPUT_PULLUP);
  pinMode(id4SwitchPin, INPUT_PULLUP);
  pinMode(pullBeforeFlightPin, INPUT_PULLUP);

  ppod = digitalRead(ppodSwitchPin);
  rfd900 = digitalRead(commsSwitchPin);
  satCom = digitalRead(satSwitchPin);
  id1On = digitalRead(id1SwitchPin);
  id2On = digitalRead(id2SwitchPin);
  id3On = digitalRead(id3SwitchPin);
  id4On = digitalRead(id4SwitchPin);
  pullOn = digitalRead(pullBeforeFlightPin);
}
