#include <RelayXBee.h>            // Relay Xbee - Download at https://github.com/MNSGC-Ballooning/XBee 

#define xbeeSerial Serial5        // Serial communication lines for the xbee radio -- PCB pins: Serial3
#define rfd900Serial Serial1   //this line should all be commented if you aren't using an RFD900
#define xbeeProSerial Serial2
#define XBEE_PRO_BAUD 38400
#define RELAY_BAUD 57600

byte xbeeProRequest[] = {0x41, 0x41, 0x41, 0x2E}; // That is, {'A','A','A','.'}
int sentBytes = 0;
byte protocolID = 0x00;

int CombinedIDA;
int LineNumberA;
int Sensor1A;
int Sensor2A;
int Sensor3A;
int CombinedIDB;
int LineNumberB;
int Sensor1B;
int Sensor2B;
int Sensor3B;
int Sensor1D;
int Sensor2D;

String satDataarray[10];
String satDataarray2[2];

bool readyfordata = true;
bool C1readytosend = false;
bool D1readytosend;

String a0;
String b0;
String C0;
String D0;

String combinedString;

RelayXBee xbee = RelayXBee(&xbeeSerial, xbeeID);            // Declare xbee instance
RelayXBee xbeePro = RelayXBee(&xbeeProSerial, xbeeProID);   //Declare xbee pro instance (This is only relevant when Stratostar Comms is being used)

void xbeeSetup() {
  // updateOled("Xbee Radio\nInit...");
  char xbeeChannel = 'A'; //
  xbeeSerial.begin(9600);   // Xbee Baud rate is 9600
  xbee.init(xbeeChannel);   // Need to make sure xbees on both ends have the same identifier. This sets "AAAA"
  xbee.enterATmode();       // Allows the setting of xbee parameters
  // If the board has an onboard RFD radio for radio relay, set the xbee to receive all other xbee transmissions
  xbee.atCommand("ATDL1");
  xbee.atCommand("ATMY0");
  // If the board does not have an onboard long range radio, transmit so radio relay board can listen
  xbee.atCommand("ATDL 1");
  xbee.atCommand("ATMY0");

  xbee.exitATmode();
  Serial.println("Xbee initialized on channel: " + String(xbeeChannel) + "; ID: " + xbeeID);  //Print to serial monitor for debugging
  // updateOled("Xbee\nChannel: " + String(xbeeChannel) + "\nID: " + xbeeID);                    //Show init is finished on the OLED
  delay(2000); //Delay two seconds so the OLED displays xbee settings
}

// Function required to convert floats into 4 byte hex vals (Only critical for SatCom transmissions)
typedef union
{
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;

// Function required to convert ints into 2 byte hex vals (Only critical for SatCom transmissions)
typedef union
{
  int number;
  uint8_t bytes[2];
} INTUNION_t;

// function that takes in a float value and prints it to the xbeePro serial as 4 bytes. (Only critical for SatCom transmissions)
void sendAddress() {
  byte address[] = {0x41, 0x41, 0x41};

  for (int i = 0; i < 3; i++) {
    xbeeProSerial.write(address[i]);
  }
}

void sendFloat(float sendMe) { //(Only critical for SatCom transmissions)
  FLOATUNION_t myFloat;
  myFloat.number = sendMe;
  for (int i = 0; i < 4; i++) {
    xbeeProSerial.write(myFloat.bytes[i]);
    Serial.println(myFloat.bytes[i]);
  }
  sentBytes += 4;
}

// function that takes in an int value and prints it to the xbeePro serial as 2 bytes.
void sendInt(int   sendMe) {
  INTUNION_t myInt;
  myInt.number = sendMe;
  for (int i = 1; i >= 0; i--) {
    xbeeProSerial.write(myInt.bytes[i]);
  }
  sentBytes += 2;
}

void sendByte(byte sendMe) {
  xbeeProSerial.write(sendMe);
  sentBytes += 1;
}

// rounds out the 21 byte xbee pro transmission with "0" bytes to fill packet
void finishSend() {
  for (int i = 0; i < (21 - sentBytes); i++) {
    xbeeProSerial.write  (0);
  }
}

void satComSetup() {
  updateOled("Setting up Xbee Pro on Serial2");
  delay(1000);
  xbeeProSerial.begin(XBEE_PRO_BAUD); // RFD900 Baud

  Serial.println("Xbee Pro initialized");
  updateOled("Xbee\nPro\nInitialized");
  delay(2000);
}

void rfd900Setup() {
  digitalWrite(xbeeLED, HIGH);
  Serial.print("Initializing Long Range Radio... ");
  updateOled("Setting up RFD900 on Serial1");
  rfd900Serial.begin(RELAY_BAUD); // RFD900 Baud
  delay(2000);

  Serial.println("Radio Initialized");
  digitalWrite(xbeeLED, LOW);
}


void updateXbee() { // This is disgusting

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

  //RFD900 Comms Relaying incoming xbee data

  if (xbeeSerial.available() > 10 && readyfordata == true) { // RELAY!!
    //delay(10);
    // Serial.println(xbeeSerial.available());
    readyfordata = false;
   // Serial.println(xbeeSerial.available());
    groundCommand = xbeeSerial.readString();
    xbeeSerial.flush();
   //Serial.println(groundCommand);
    if (groundCommand.startsWith("UMN1") && groundCommand.indexOf('|') > 0) {
      a0 = groundCommand;  
    }
    if (groundCommand.startsWith("UMN2") && groundCommand.indexOf('|') > 0) {
      b0 = groundCommand;
    }
    if (groundCommand.startsWith("UMN3") && groundCommand.indexOf('|') > 0 ) {
      C0 = groundCommand;
    }
    if (groundCommand.startsWith("UMN4") && groundCommand.indexOf('|') > 0) {
      D0 = groundCommand;
    }
    

    readyfordata = true;
    // rfd900Serial.println(groundCommand);
    // Serial.println(xbeeSerial.available());
    // Serial.println(xbeeSerial.readString());
    xbeeMessage = "RELAYED: " + groundCommand;
  }

  // RFD Comms Realying or interpreting incoming RFD900 requests
  if (rfd900 == 0 && rfd900Serial.available() > 0) {
    groundCommand = rfd900Serial.readStringUntil("\r\n");
    Serial.println(groundCommand);
    if (groundCommand.startsWith(xbeeID)) {
      groundCommand.remove(0, xbeeID.length() + 1);
      rfd900Serial.println(xbeeID + ", " + interpretMessage(groundCommand) + "!");
      xbeeMessage = "COMMS RECEIVED: " + groundCommand;
    }
    else {
      xbeeSerial.print(groundCommand);
    }
  }

  // Payload receiving a message via rfd900 Comms unit
  if (rfd900 == 1 && satCom == 1 && xbeeSerial.available() > 10) {
    groundCommand = xbeeSerial.readString();
    if (groundCommand.startsWith(xbeeID))
    {
      groundCommand.remove(0, xbeeID.length() + 1);
      xbeeSerial.println(xbeeID + ", " + interpretMessage(groundCommand) + "!");
      xbeeMessage = xbeeID + " RECEIVED: " + groundCommand + "; SENT: " + interpretMessage(groundCommand);

    }
  }

  if ((millis() - xbeeTimer) > xbeeRate) {

    xbeeTimer = millis();

    // Sending Comms Payload data
    if (rfd900 == 0) {
      rfd900Serial.println(xbeeID + "," + groundData + "!");


    }
    else {
      xbeeSerial.print(xbeeID + "," + groundData + "!");
      Serial.print(xbeeID + "," + groundData + "!");

    }

    xbeeMessage = "DATA STRING TRANSMITTED";

    xbeeSerial.write("Readyfordata");
    digitalWrite(xbeeLED, HIGH);
    delay(80);
    digitalWrite(xbeeLED, LOW);
    combinedString = a0 + b0 + C0 + D0;
    a0.remove(0);
    b0.remove(0);
    C0.remove(0);
    D0.remove(0);
    logRelay(combinedString);
    Serial.println(combinedString);

    //Serial.println(groundCommand);

    if (combinedString.indexOf('|') > 0) {
      separateData(combinedString);
      //Serial.println(groundCommand);
    }

  }
}
void separateData(String mystring) {
  String datastring = mystring.remove(0, combinedString.indexOf('|') + 1);
  int numofdata = 10;
  int pos;
  String newstring;
  //Serial.println(datastring);
  for (int i = 0; i < numofdata; i++) {
    pos = datastring.indexOf(',');
    newstring = datastring.substring(0, pos);
    satDataarray[i] = newstring;
   // Serial.println(datastring);
    datastring.remove(0, pos + 1);
    if (i == 4) {
      datastring.remove(0, datastring.indexOf('|') + 1);
    }
//    if (i == 5) {
//      datastring.remove(0, datastring.indexOf('|') + 1);
//    }
//    if (i == 8) {
//      datastringring.remove(0, datastring.indexOf('|') + 1);
//    }
  }
  /*
    datastring.remove(0,datastring.indexOf('|')+1);
    for(int j = 0; j<2; j++){
     pos = datastring.indexOf(',');
     newstring = datastring.substring(0,pos);
     satDataarray2[j] = newstring;
     datastring.remove(0,pos+1);
    }
  */
}
// Checks for packet request from sat comm
void updateXbeePro() {
  if (xbeeProSerial.available() > 0) {
    digitalWrite(13, LOW);
    int byteCounter = 0;
    byte variable[10];
    byte index = 0;
    while (xbeeProSerial.available() > 0) {
      byte b = xbeeProSerial.read();
      variable[index] = b;
      if (b == xbeeProRequest[index]) {
        byteCounter++;
      }
      index++;
    }

    if (byteCounter == 4) {
      updateOled("Data Requested from SatCom");
      sentBytes = 0;
      sendAddress(); //   21 available bytes to send
      updateOled("Sending via SatCom");

      protocolID = 0x20;
      CombinedIDA = satDataarray[0].toInt();
      LineNumberA = satDataarray[1].toInt();
      Sensor1A = satDataarray[2].toInt();
      Sensor2A = satDataarray[3].toInt();
      Sensor3A = satDataarray[4].toInt();
      CombinedIDB = satDataarray[5].toInt();
      LineNumberB = satDataarray[6].toInt();
      Sensor1B = satDataarray[7].toInt();
      Sensor2B = satDataarray[8].toInt();
      Sensor3B = satDataarray[9].toInt();
//      Sensor1D = satDataarray[10].toInt();
//      Sensor2D = satDataarray[11].toInt();
      sendByte(protocolID);
      sendByte(CombinedIDA);
      sendInt(int(LineNumberA));
      sendInt(int(Sensor1A));
      sendInt(int(Sensor2A));
      sendInt(int(Sensor3A));
      sendByte(CombinedIDB);
      sendInt(int(LineNumberB));
      sendInt(int(Sensor1B));
      sendInt(int(Sensor2B));
      sendInt(int(Sensor3B));

      finishSend(); // finishes 21 byte packet (sends 21-6 = 15 "0x00" bytes)
    }
    //Serial.println(satDataarray[4]);
  }
}


String interpretMessage( String myCommand ) {

  if (myCommand.startsWith("TIME")) {
    xbeeMessage = String(cutTime - millis());
  }
  else if (myCommand.startsWith("+")) {
    myCommand.remove(0, 1);
    float timeAdded = myCommand.toFloat();
    cutTime = cutTime + timeAdded;
    xbeeMessage = "New cut time: " + String(cutTime);
  }
  else if (myCommand.startsWith("-")) {
    myCommand.remove(0, 1);
    float timeSubtracted = myCommand.toFloat();
    cutTime = cutTime - timeSubtracted;
    xbeeMessage = "New cut time: " + String(cutTime);
  }
  else if (myCommand.startsWith("ALT")) {
    xbeeMessage = "Altitude calculated from pressure: " + String(altitudeFt);
  }
  else if (myCommand.startsWith("A+")) {
    myCommand.remove(0, 1);
    float altitudeAdded = myCommand.toFloat();
    cutAltitude = cutAltitude + altitudeAdded;
    xbeeMessage = "New cut altitude: " + String(cutAltitude);
  }
  else if (myCommand.startsWith("A-")) {
    myCommand.remove(0, 1);
    float altitudeSubtracted = myCommand.toFloat();
    cutAltitude = cutAltitude - altitudeSubtracted;
    xbeeMessage = "New cut altitude: " + String(cutAltitude);
  }
  else if (myCommand.startsWith("CUT")) {
    if (smartRelease) xbee.send("Cubes deployed prior to command");
    else {
      smartRelease = true;
      commandRelease = true;
    }
  }
  else if (myCommand.startsWith("DATA")) {
    xbeeMessage = data;
  }
  else if (myCommand.startsWith("MARCO")) {
    xbeeMessage = "POLO";
  }
  else if (myCommand.startsWith("FREQ=")) {
    myCommand.remove(0, 5);
    xbeeMessage = "New Send Rate: " + myCommand;
    xbeeRate = myCommand.toInt();
  }
  else {
    xbeeMessage = "Error - command not recognized: " + groundCommand;
  }
  if (xbeeMessage != "") return xbeeMessage;
}
