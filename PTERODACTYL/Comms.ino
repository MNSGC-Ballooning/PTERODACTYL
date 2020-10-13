// Contains all of the setup information and functions related to XBee communication. 
// Students should not have to use or add any code here unless adding new radio commands

#include <RelayXBee.h>

#define xbeeSerial Serial5        // Serial communication lines for the xbee radio -- PCB pins: Serial3
#define rfd900Serial Serial1   //this line should all be commented if you aren't using an RFD900
#define xbeeProSerial Serial2
#define XBEE_PRO_BAUD 38400

byte xbeeProRequest[] = {0x41, 0x41, 0x41, 0x2E}; // That is, {'A','A','A','â'}

RelayXBee xbee = RelayXBee(&xbeeSerial, xbeeID);
RelayXBee xbeePro = RelayXBee(&xbeeProSerial, xbeeProID);

float testFloat = 128.0;
int testInt = 1280;
int sentBytes = 0;

void xbeeSetup(){
  updateOled("Xbee Radio\nInit...");
  char xbeeChannel = 'A';
  xbeeSerial.begin(XBEE_BAUD);
  xbee.init(xbeeChannel); // Need to make sure xbees on both ends have the same identifier. "AAAA"
  xbee.enterATmode();
  xbee.atCommand("ATDL0");
  xbee.atCommand("ATMY1");
  xbee.exitATmode();
  Serial.println("Xbee initialized on channel: " + String(xbeeChannel) + "; ID: " + xbeeID);
  updateOled("Xbee\nChannel: " + String(xbeeChannel) + "\nID: " + xbeeID);
  delay(2000);
}

// Function required to convert floats into 4 byte hex vals
typedef union
{
  float number;
  uint8_t bytes[4];
}FLOATUNION_t;

// Function required to convert ints into 2 byte hex vals
typedef union
{
  int number;
  uint8_t bytes[2];
}INTUNION_t;

// function that takes in a float value and prints it to the xbeePro serial as 4 bytes.
void sendAddress(){
  byte address[] = {0x41, 0x41, 0x41};
  for (int i=0; i<3; i++)
  {
    xbeeProSerial.write(address[i]);
  }
}

void sendFloat(float sendMe){
  FLOATUNION_t myFloat;
  myFloat.number = sendMe;
  for (int i=0; i<4; i++)
  {
    xbeeProSerial.write(myFloat.bytes[i]);
  }
  sentBytes += 4;
}

// function that takes in an int value and prints it to the xbeePro serial as 2 bytes.
void sendInt(int sendMe){
  INTUNION_t myInt;
  myInt.number = sendMe;
  for (int i=0; i<2; i++)
  {
    xbeeProSerial.write(myInt.bytes[i]);
  }
  sentBytes += 2;
}

// rounds out the 21 byte xbee pro transmission with "0" bytes to fill packet
void finishSend(){
  for (int i=0; i<(21-sentBytes); i++){
    xbeeProSerial.write(0);
    //xbeeProSerial.print(' ');
  }
}

void updateXbee(){ // This is disgusting
  
  // Key: (keep this up to date)
  //
  // T      reports back time remaining until cut.
  // +##    adds time to cut timer 
  // -##    subtracts time to cut timer
  // A      reports remaining altitude until cut.
  // A+##   adds altitude (m) to cut altitude
  // A-##   subtracts altitude (m) to cut altitude
  // C      releases cubes from PPOD
  // D      sends data string
  // M      Polo! just a ping command

  if((millis() - xbeeTimer) > xbeeRate){ 
    
    xbeeTimer = millis();
    
    xbeeSerial.print(xbeeID + "," + groundData + "!");

    digitalWrite(xbeeLED,HIGH);
    delay(80);
    digitalWrite(xbeeLED,LOW);
    xbeeMessage = "DATA STRING TRANSMITTED";
  }

 if (xbeeSerial.available() > 10){
  groundCommand = xbeeSerial.readString();
  if(groundCommand.startsWith(xbeeID))
  {
    groundCommand.remove(0,xbeeID.length()+1);
    xbeeSerial.println(xbeeID + ", " + interpretMessage(groundCommand) + "!");
    xbeeMessage = xbeeID + " RECEIVED: " + groundCommand + "; SENT: " + interpretMessage(groundCommand);
  }
 }
 Serial.println("message: " + xbeeMessage);
}

// Checks for packet request from central unit
void updateXbeePro(){
  if(xbeeProSerial.available()>0){
    int byteCounter = 0;
    byte variable[10];
    byte index = 0;
    while(xbeeProSerial.available() > 0)
   {
     byte b = xbeeProSerial.read();
     variable[index] = b;
     if(b==xbeeProRequest[index]){byteCounter++;}
     index++;
    }  
    if(byteCounter==4){
       updateOled("Data Requested from SatCom");
       Serial.println("In loop");
       digitalWrite(ppodLED,HIGH);
      digitalWrite(xbeeLED,HIGH);
      digitalWrite(fixLED,HIGH);
      digitalWrite(sdLED,HIGH);
       
       sentBytes = 0;
       sendAddress();
       sendFloat(testFloat);
       sendInt(testInt);
       finishSend();

       delay(1000);
       digitalWrite(ppodLED,LOW);
      digitalWrite(xbeeLED,LOW);
      digitalWrite(fixLED,LOW);
      digitalWrite(sdLED,LOW);
    }
  }
}

String interpretMessage( String myCommand ){
    
    if(myCommand.startsWith("ALT"))
    {
      xbeeMessage = "Altitude calculated from pressure: " + String(altitudeFt);
    }
    else if(myCommand.startsWith("DATA"))
    {
      xbeeMessage = data;
    }
    else if(myCommand.startsWith("MARCO"))
    {
      xbeeMessage = "POLO";
    }
    else if(myCommand.startsWith("FREQ="))
    {
      myCommand.remove(0,5);
      xbeeMessage = "New Send Rate: " + myCommand;
      xbeeRate = myCommand.toInt();
    }
    else{
      xbeeMessage = "Error - command not recognized: " + groundCommand;
    }
    if(xbeeMessage!="") return xbeeMessage;
  }
  
