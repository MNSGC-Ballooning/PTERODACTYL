#include <RelayXBee.h>            // Relay Xbee - Download at https://github.com/MNSGC-Ballooning/XBee 

#define xbeeSerial Serial5        // Serial communication lines for the xbee radio -- PCB pins: Serial3
#define rfd900Serial Serial1   //this line should all be commented if you aren't using an RFD900
#define xbeeProSerial Serial2  //SATCOM
#define XBEE_PRO_BAUD 38400
#define RELAY_BAUD 57600

byte xbeeProRequest[] = {0x41, 0x41, 0x41, 0x2E}; // That is, {'A','A','A','.'}
int sentBytes = 0;
byte dataID = 0x00;

RelayXBee xbee = RelayXBee(&xbeeSerial, xbeeID);            // Declare xbee instance
RelayXBee xbeePro = RelayXBee(&xbeeProSerial, xbeeProID);   // This is only relevant when Stratostar Comms is being used

void xbeeSetup(){
  updateOled("Xbee Radio\nInit..."); 
  char xbeeChannel = 'A'; //
  xbeeSerial.begin(9600);   // Xbee Baud rate is 9600
  xbee.init(xbeeChannel);   // Need to make sure xbees on both ends have the same identifier. This sets "AAAA"
  xbee.enterATmode();       // Allows the setting of xbee parameters
  if(rfd900==0){            // If the board has an onboard RFD radio for radio relay, set the xbee to receive all other xbee transmissions
    xbee.atCommand("ATDL1");  
    xbee.atCommand("ATMY0");
  }
  else{                     // If the board does not have an onboard long range radio, transmit so radio relay board can listen
    xbee.atCommand("ATDL0");
    xbee.atCommand("ATMY1");
  }
  xbee.exitATmode();
  Serial.println("Xbee initialized on channel: " + String(xbeeChannel) + "; ID: " + xbeeID);  //Print to serial monitor for debugging
  updateOled("Xbee\nChannel: " + String(xbeeChannel) + "\nID: " + xbeeID);                    //Show init is finished on the OLED
  delay(2000);                                                                                //Delay two seconds so the OLED displays xbee settings
}

// Function required to convert floats into 4 byte hex vals (Only critical for SatCom transmissions)
typedef union
{
  float number;
  uint8_t bytes[4];
}FLOATUNION_t;

// Function required to convert ints into 2 byte hex vals (Only critical for SatCom transmissions)
typedef union
{
  int number;
  uint8_t bytes[2];
}INTUNION_t;

// function that takes in a float value and prints it to the xbeePro serial as 4 bytes. (Only critical for SatCom transmissions)
void sendAddress(){
  byte address[] = {0x41, 0x41, 0x41};
  
  for (int i=0; i<3; i++){
    xbeeProSerial.write(address[i]);
  }
}

void sendFloat(float sendMe){ //(Only critical for SatCom transmissions)
  FLOATUNION_t myFloat;
  myFloat.number = sendMe;
  for (int i=0; i<4; i++){
    xbeeProSerial.write(myFloat.bytes[i]);
  }
  sentBytes += 4;
}

// function that takes in an int value and prints it to the xbeePro serial as 2 bytes.
void sendInt(int sendMe){
  INTUNION_t myInt;
  myInt.number = sendMe;
  for (int i=1; i>=0; i--){
    xbeeProSerial.write(myInt.bytes[i]);
  }
  sentBytes += 2;
}

void sendByte(byte sendMe){
  xbeeProSerial.write(sendMe);
  sentBytes += 1;
}

// rounds out the 21 byte xbee pro transmission with "0" bytes to fill packet
void finishSend(){
  for (int i=0; i<(21-sentBytes); i++){
    xbeeProSerial.write(0);
  }
}

void satComSetup(){
  updateOled("Setting up Xbee Pro on Serial2");
  delay(1000);
  
  xbeeProSerial.begin(XBEE_PRO_BAUD); // RFD900 Baud

  Serial.println("Xbee Pro initialized");
  updateOled("Xbee\nPro\nInitialized");
  delay(2000);
}

void rfd900Setup(){
  digitalWrite(xbeeLED,HIGH);
  Serial.print("Initializing Long Range Radio... ");
  updateOled("Setting up RFD900 on Serial1");
  rfd900Serial.begin(RELAY_BAUD); // RFD900 Baud
  delay(2000);
  
  Serial.println("Radio Initialized");
  digitalWrite(xbeeLED,LOW);
}

void updateXbee(){ // This is disgusting
  
  // Key: (keep this up to date)
  //
  // TIME   reports back time remaining until cut.
  // +##    adds time to cut timer 
  // -##    subtracts time to cut timer
  // ALT    reports remaining altitude until cut.
  // A+##   adds altitude (m) to cut altitude
  // A-##   subtracts altitude (m) to cut altitude
  // CUT    releases cubes from PPOD
  // DATA   sends data string
  // MARCO  Polo! just a ping command
  // FREQ=##  new send rate for xbee
  // SIRENON  turns on siren (if connected) - overrides altitude criteria for siren
  // SIRENOFF turns off siren (if connected) - overrides altitude criteria for siren
  //GIVE    Transmit all SD data for emergency recovery (payload cannot be recovered but is visable) 
  //NAME    gets name of file currently being read for use in reciver 


  //RFD900 Comms Relaying incoming xbee data
  if (xbeeSerial.available() > 10 && rfd900==0) { // RELAY!!
    groundCommand = xbeeSerial.readString();
    xbeeSerial.flush();
    rfd900Serial.println(groundCommand);
    xbeeMessage = "RELAYED: " + groundCommand;
    }

  // RFD Comms Realying or interpreting incoming RFD900 requests
  if (rfd900==0 && rfd900Serial.available() > 0){
    groundCommand = rfd900Serial.readStringUntil("\r\n");
    Serial.println(groundCommand);
    if(groundCommand.startsWith(xbeeID)){
      groundCommand.remove(0,xbeeID.length()+1);
      rfd900Serial.println(xbeeID + ", " + interpretMessage(groundCommand) + "!");
      xbeeMessage = "COMMS RECEIVED: " + groundCommand;
    }
    else{
      xbeeSerial.print(groundCommand);
    }
  }

 // Payload receiving a message via rfd900 Comms unit
 if (rfd900==1 && satCom==1 && xbeeSerial.available() > 0){ //why >10 for serial available
  groundCommand = xbeeSerial.readString();
  if(groundCommand.startsWith(xbeeID))
  {
    groundCommand.remove(0,xbeeID.length()+1);
    if (groundCommand.startsWith("GIVE")){
      interpretMessage(groundCommand);
    }
    else{
    xbeeSerial.println(xbeeID + ", " + interpretMessage(groundCommand) + "!");
    xbeeMessage = xbeeID + " RECEIVED: " + groundCommand + "; SENT: " + interpretMessage(groundCommand);
    }
  }
  else{
    groundCommand.remove(0,xbeeID.length()+1);
    if (groundCommand.startsWith("GIVE")){
      interpretMessage(groundCommand);
    }
    else{ 
    xbeeSerial.println("GLOBAL, " + interpretMessage(groundCommand) + "!");
    xbeeMessage = "GLOBAL: " + groundCommand + "; SENT: " + interpretMessage(groundCommand);
    }
  }
 }
 

 if((millis() - xbeeTimer) > xbeeRate){
      
    xbeeTimer = millis();

    // Sending Comms Payload data
    if(rfd900==0){
      rfd900Serial.println(xbeeID + "," + groundData + "!");
    }
    else {
      xbeeSerial.print(xbeeID + "," + groundData + "!");
      Serial.print(xbeeID + "," + groundData + "!");
    } 
    digitalWrite(xbeeLED,HIGH);
    delay(80);
    digitalWrite(xbeeLED,LOW);
    xbeeMessage = "DATA STRING TRANSMITTED";
  }
}

// Checks for packet request from central unit
void updateXbeePro(){
  if(xbeeProSerial.available()>0){
    digitalWrite(13,LOW);
    int byteCounter = 0;
    byte variable[10];
    byte index = 0;
    
    while(xbeeProSerial.available() > 0){
      byte b = xbeeProSerial.read();
      variable[index] = b;
      if(b==xbeeProRequest[index]){byteCounter++;}
        index++;
    }
      
    if(byteCounter==4){
       updateOled("Data Requested from SatCom");
       sentBytes = 0;
       sendAddress(); // 21 available bytes to send
       updateOled("Sending via SatCom");
       if(altitudeFtGPS >= 60000){
          dataID = 0x01;
          sendByte(dataID);
          sendFloat(altitudeFtGPS);
          sendFloat(altitudeFt);
          sendInt(int(thermistorInt));
          sendInt(int(thermistorExt));
          sendInt(int(msTemperature));
          sendFloat(msPressure);
       }
       else if(altitudeFtGPS <= 10000){
          dataID = 0x02;
          sendByte(dataID);
          sendFloat(latitudeGPS);
          sendFloat(longitudeGPS);
          sendFloat(altitudeFtGPS);
          sendFloat(altitudeFt);
          sendInt(int(thermistorExt));
       }
       else{
          dataID = 0x03;
          sendByte(dataID);
          sendInt(int(thermistorInt));
          sendInt(int(thermistorExt));
          sendInt(int(msTemperature));
          sendFloat(msPressure);
          sendFloat(altitudeFtGPS);
          sendFloat(altitudeFt);
       }
       finishSend(); // finishes 21 byte packet (sends 21-6 = 15 "0x00" bytes)
    }
  }
}

String interpretMessage( String myCommand ){
    
    if(myCommand.startsWith("TIME")){
      xbeeMessage = String(cutTime-millis());
    }
    else if(myCommand.startsWith("+")){
      myCommand.remove(0,1);
      float timeAdded = myCommand.toFloat();
      cutTime = cutTime + timeAdded;
      xbeeMessage = "New cut time: " + String(cutTime);
    }
    else if(myCommand.startsWith("-")){
      myCommand.remove(0,1);
      float timeSubtracted = myCommand.toFloat();
      cutTime = cutTime - timeSubtracted;
      xbeeMessage = "New cut time: " + String(cutTime);
    }
    else if(myCommand.startsWith("ALT")){
      xbeeMessage = "Altitude calculated from pressure: " + String(altitudeFt);
    }
    else if(myCommand.startsWith("A+")){
      myCommand.remove(0,1);
      float altitudeAdded = myCommand.toFloat();
      cutAltitude = cutAltitude + altitudeAdded;
      xbeeMessage = "New cut altitude: " + String(cutAltitude);
    }
    else if(myCommand.startsWith("A-")){
      myCommand.remove(0,1);
      float altitudeSubtracted = myCommand.toFloat();
      cutAltitude = cutAltitude - altitudeSubtracted;
      xbeeMessage = "New cut altitude: " + String(cutAltitude);
    }
    else if(myCommand.startsWith("CUT")){
      if(smartRelease) xbee.send("Cubes deployed prior to command");
      else{ smartRelease = true;
            commandRelease = true;
      }
    }
    else if(myCommand.startsWith("DATA")){
      xbeeMessage = data;
    }
    else if(myCommand.startsWith("MARCO")){
      xbeeMessage = "POLO";
    }
    else if(myCommand.startsWith("FREQ=")){
      myCommand.remove(0,5);
      xbeeMessage = "New Send Rate: " + myCommand;
      xbeeRate = myCommand.toInt();
    }
    else if(myCommand.startsWith("SIRENOFF")){
      if(satCom == 1){
        overrideOn();
        sirenOff();
        xbeeMessage = "Siren Off";
      }
      else{
        xbeeMessage = "Error, no siren connected";
      }
    }
    else if(myCommand.startsWith("SIRENON")){
      if(satCom == 1){
        overrideOn();
        sirenOn();
        xbeeMessage = "Siren On";
      }
      else{
        xbeeMessage = "Error, no siren connected";
      }
    }
    else if(myCommand.startsWith("ID")){
      xbeeMessage = xbeeID;
    }
     else if(myCommand.startsWith("GIVE")){
       myCommand.remove(0,4);
       Serial.println(myCommand);  
       giveData(myCommand); 
       xbeeMessage = "Data sent to reciver unit!";  
    }
    else if(myCommand.startsWith("NAME")){
      giveFileName(); 
      xbeeMessage = "File name: " + String(filename);   
    }
    else{
      xbeeMessage = "Error - command not recognized: " + groundCommand;
    }
    if(xbeeMessage!="") return xbeeMessage;
  }
  
 //Radio Ideas:
 //cutaway, resistor burner(string wrapped around)
 //remote data download from SD
 //communicating between pteros - may be possible, need to test
