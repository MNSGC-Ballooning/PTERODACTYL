
String halfOne = "";
String Combine = "";
int tracker = 1;

///////// Functions ////////////
// void sdSetup()
// logData(String Data)

void sdSetup() {
  pinMode(chipSelect, OUTPUT);
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    updateOled("Turn off\nand Insert SD card");
    for (int i = 1; i < 20; i++) {
      digitalWrite(13, HIGH);
      digitalWrite(sdLED, LOW);
      delay(100);
      digitalWrite(13, LOW);
      digitalWrite(sdLED, HIGH);
      delay(100);
    }
  }
  else {
    Serial.println("Card initialized.\nCreating File...");
    for (byte i = 0; i < 100; i++) {
      filename[6] = '0' + i / 10;
      filename[7] = '0' + i % 10;
      if (!SD.exists(filename)) {
        datalog = SD.open(filename, FILE_WRITE);
        sdActive = true;
        Serial.println("Logging to: " + String(filename));
        updateOled("Logging:\n\n" + String(filename));
        delay(1000);
        break;
      }
    }
    if (!sdActive) {
      Serial.println("No available file names; clear SD card to enable logging");
      updateOled("Clear SD!");
      delay(5000);
    }
  }
}

void logData(String Data) {
  datalog = SD.open(filename, FILE_WRITE);
  datalog.println(Data);
  datalog.close();
  Serial.println(Data);
}

void giveData(String FileName) {
  senderName[6] = FileName.charAt(6);
  senderName[7] = FileName.charAt(7);
  Serial.println(senderName);

  updateOled("Starting... Send File" + String(senderName));

  sender = SD.open(senderName, FILE_READ); //open file for reading

  delay(1000);

  if (!sender) {
    xbeeSerial.println("FAILX");
    updateOled("Failed to open or   File DNE!");
    delay(2000);
  }
  else if (sender) {  //if opened, do function
    int checkFirst = 0;
    updateOled("Begining Data Transfer...");
    delay(1000);
    xbeeSerial.print("STARTX");

    while (sender.available() > 0) {
      String dataline = sender.readStringUntil('\r');
      updateOled("Sending   Data");
      if ( checkFirst < 3) { //loop to not send header, makes string very messy if sent and is too long for xbee
        String dataline = "";
        checkFirst++;
      }
      else {

        if (tracker == 1) { //Arduino for some reason only likes to pick up ~106 chars at a line per line, this splits the line of data for later transmission
          halfOne = dataline;
          tracker = 2;
        }
        else if (tracker == 2) {
          Combine = (halfOne + dataline); //recombine for xbee transmission
          xbeeSerial.print(Combine);
          xbeeSerial.print('X'); //send terminator for reciver
          xbeeSerial.flush();
          digitalWrite(sdLED, HIGH);
          digitalWrite(xbeeLED, HIGH);
          tracker = 1;
        }
      }
    }

    sender.close();
    digitalWrite(sdLED, LOW);
    digitalWrite(xbeeLED, LOW);
    updateOled("Data successfully sent!");
    delay(1000);
    xbeeSerial.print("ENDX"); //Terminator for command to close file on reciver end
    updateOled("Ending...");
    delay(1000);
    return;
  }
  else {
    xbeeSerial.println("FAILX");
    updateOled("Failed to open or   File DNE!");
    delay(2000);
  }


}
