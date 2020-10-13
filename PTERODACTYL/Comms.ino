// Contains all of the setup information and functions related to XBee communication. 
// Students should not have to use or add any code here unless adding new radio commands
// Some functions for converting numbers to the proper binary elements have been kept in case they're needed fro the SatCom system

#include <RelayXBee.h>

#define xbeeSerial Serial5        // Serial communication lines for the xbee radio -- PCB pins: Serial3

RelayXBee xbee = RelayXBee(&xbeeSerial, xbeeID);

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

void sendFloat(float sendMe){
  FLOATUNION_t myFloat;
  myFloat.number = sendMe;
  for (int i=0; i<4; i++)
  {
    // Send converted floats here
  }
  sentBytes += 4;
}

// function that takes in an int value and prints it to the xbeePro serial as 2 bytes.
void sendInt(int sendMe){
  INTUNION_t myInt;
  myInt.number = sendMe;
  for (int i=0; i<2; i++)
  {
    // Send converted ints here
  }
  sentBytes += 2;
}

// rounds out the 21 byte xbee pro transmission with "0" bytes to fill packet
void finishSend(){
  for (int i=0; i<(21-sentBytes); i++){
    // Send completed packet here
  }
}

void updateXbee(){ // This is disgusting

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
  
