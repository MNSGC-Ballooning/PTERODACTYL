#include <UbloxGPS.h> //https://github.com/MNSGC-Ballooning/FlightGPS/
#include <TinyGPS++.h> // https://github.com/mikalhart/TinyGPSPlus
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h> // https://github.com/sparkfun/SparkFun_LSM9DS1_Arduino_Library
#include <OneWire.h>
#include <MS5611.h> // https://github.com/jarzebski/Arduino-MS5611
#include <SFE_MicroOLED.h>  // https://github.com/sparkfun/SparkFun_Micro_OLED_Arduino_Library

// Values for OLED library
#define PIN_RESET 9  
#define DC_JUMPER 1 

#define thermIntPin A16
#define thermExtPin A17
#define ubloxSerial Serial3       // Serial communication lines for the ublox GPS -- PCB pins: Serial5

MS5611 baro;
MicroOLED oled(PIN_RESET, DC_JUMPER);    // I2C declaration
LSM9DS1 imu;
UbloxGPS ublox(&ubloxSerial);

/////////////// Thermistor constants //////////////////////
int analogResolutionBits = 14;
float adcMax = pow(2,analogResolutionBits)-1.0; // The maximum adc value given to the thermistor
float A = 0.001125308852122; // A, B, and C are constants used for a 10k resistor and 10k thermistor for the steinhart-hart equation
float B = 0.000234711863267;
float C = 0.000000085663516; 
float R1 = 10000; // 10k Ω resistor
float Tinv;
float adcVal;
float logR;
float T; // these three variables are used for the calculation from adc value to temperature
float currentTempC; // The current temperature in Celcius
float currentTempF; // The current temperature in Fahrenheit

// Functions
// void ubloxSetup()
// void imuSetup()
// void updateIMU()
// void oledSetup()
// void updateOled(String disp)
// void msSetup()
// void updateMS()
// void updatePressure()
// void updateThermistor()
// void updateUblox()
// void updateDataStrings()
// void pullPin()

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
      delay(3000);
    }
  }
  updateOled("GPS init\ncomplete!");
  delay(1000);
}

void imuSetup(){
  Wire.begin();
  if(!imu.begin()){
    Serial.println("Failed to communicate with LSM9DS1.");
    updateOled("IMU\nOffline."); // random placement of '\n' in OLED strings is to prevent line breaks
    delay(5000);
  }
  else{
    updateOled("IMU init\ncomplete!");
    delay(1000);
  }
}

// Update Sparkfun 9DOF IMU
void updateIMU(){
  if( imu.gyroAvailable() ) imu.readGyro();
  if( imu.accelAvailable() ) imu.readAccel();
  if( imu.magAvailable() ) imu.readMag();

  magnetometer[0] = imu.calcMag(imu.mx);
  magnetometer[1] = imu.calcMag(imu.my);
  magnetometer[2] = imu.calcMag(imu.mz);
  accelerometer[0] = imu.calcAccel(imu.ax);
  accelerometer[1] = imu.calcAccel(imu.ay);
  accelerometer[2] = imu.calcAccel(imu.az);
  gyroscope[0] = imu.calcGyro(imu.gx);
  gyroscope[1] = imu.calcGyro(imu.gy);
  gyroscope[2] = imu.calcGyro(imu.gz);
}

// Set up MicroOLED display
void oledSetup(){
  Wire.begin();
  oled.begin();    // Initialize the OLED
  oled.clear(ALL); // Clear the display's internal memory
  oled.display();  // Display what's in the buffer (splashscreen)
  oled.clear(PAGE); // Clear the buffer.

  randomSeed(analogRead(A0) + analogRead(A1));

  updateOled("Initializing...");
}

// take in string to display on the MicroOLED
void updateOled(String disp){
  oled.clear(PAGE);
  oled.setFontType(0);
  oled.setCursor(0, 0);
  oled.println(disp);
  oled.display();
}

// setup MS5611 pressure sensor
void msSetup() {
  while(!baro.begin()){
    updateOled("baro init failed!");
  }
  updateOled("baro init\ncomplete!");
  delay(1000); // for display purposes
}

// update MS5611 pressure sensor
void updateMS() {
  msTemperature = baro.readTemperature();
  msTemperature = msTemperature*(9.0/5.0) + 32.0; // Celcius to Fahrenheit
  msPressure = baro.readPressure(); 
  msPressure = msPressure * 0.000145038; //mbar to PSI (i think)
}

void updateThermistor(){
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
}

void updateUblox(){
  ublox.update();
}

void updateStatus(){
  //set your status bits statusArr[0] through statusArr[7] here
  statusArr[0] = true;
  statusArr[1] = true;
  
  for(int i=0; i<8; i++){ // increment through each index of arrayy[] 
    if(statusArr[i]){ // only when the array index holds a 1 will we perform the next operation. This is because the byte default is 00000000, so we only need to bit shift if we have a 1
      statusByte |= (1 << i); // bit shifting - moves 1s into the correct place in the byte. take the value 1 and move it over (7-i) places - ex/ '1' in index 5 (status 5). Add 1 and move it over 7-5=2 bits (00000100) 
    }
  }
  statusInt = (IDByte<<8) | (statusByte);
}

void updateDataStrings(){
  altitudeFtGPS = ublox.getAlt_feet();
  latitudeGPS = ublox.getLat();
  longitudeGPS = ublox.getLon();

 groundData = String(ublox.getYear()) + "," + String(ublox.getMonth()) + "," + String(ublox.getDay()) + "," +
            String(ublox.getHour()-5) + "," + String(ublox.getMinute()) + "," + String(ublox.getSecond()) + ","
           + String(ublox.getLat(), 4) + ", " + String(ublox.getLon(), 4) + ", " + String(altitudeFtGPS, 4)
           +  ", " + String(altitudeFt) + ", " + String(thermistorInt) + ", " + String(thermistorExt) + ", "
           + String(msTemperature) + ", " + String(msPressure) + "," + String(0) + "," + String(0);

 data = groundData + String(magnetometer[0]) + ", " + String(magnetometer[1]) + ", " + String(magnetometer[2]) + ", " +
            String(accelerometer[0]) + ", " + String(accelerometer[1]) + ", " + String(accelerometer[2]) + ", " +
            String(gyroscope[0]) + ", " + String(gyroscope[1]) + ", " + String(gyroscope[2]) + "," +  xbeeMessage;

 if(ppod==0) data = data + ", " + String(smartRelease);
 if(satCom==0 && rfd900==1){
  satComPacket = "S," + String(statusInt) + "," + String(lineNumber) + "," + String(ublox.getYear()) + "," + String(ublox.getMonth()) + "," + String(ublox.getDay()) + "," + String(ublox.getHour()) + ",";
 }
  lineNumber +=1;
 
  updateOled(String(latitudeGPS) + "\n" + String(longitudeGPS) + "\n" + String(altitudeFtGPS,1) + "ft\nInt:" + String(int(thermistorInt)) + " F\nExt:" + String(int(thermistorExt)) + " F\n" + String(msPressure,2) + " PSI");
  if(ublox.getFixAge() > 2000) fix = false;
  else fix = true;
  logData(data);
}

void pullPin(){
  updateOled("Pull Flight Pin to start timer");
  
  while(pullOn==0)
  {
    digitalWrite(fixLED,HIGH);
    digitalWrite(ppodLED,HIGH);
    digitalWrite(xbeeLED,HIGH);
    digitalWrite(sdLED,HIGH);
    pullOn = digitalRead(pullBeforeFlightPin);
  }
  ppodOffset = millis();
  updateOled("Timer Starting");
  digitalWrite(fixLED,LOW);
  digitalWrite(ppodLED,LOW);
  digitalWrite(xbeeLED,LOW);
  digitalWrite(sdLED,LOW);
  delay(2000);
}
