#include <SD.h>

#define chipSelect BUILTIN_SDCARD //Should highlight if you have teensy 3.5/3.6/4.0 selected

File datalog;
File datalogIMU;
char filename[] = "SDCARD00.csv";
char filename2[] = "CARGOS00.csv";

bool sdActiveF1 = false;
bool sdActiveF2 = false;

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
    Serial.println("Card initialized.\nCreating Files...");
    for (byte i = 0; i < 100; i++) {
      filename[6] = '0' + i / 10;
      filename[7] = '0' + i % 10;
      if (!SD.exists(filename)) {
        datalog = SD.open(filename, FILE_WRITE);
        datalog.close();
        sdActiveF1 = true;
        Serial.println("Logging to: " + String(filename));
        updateOled("Logging:\n\n" + String(filename));
        delay(1000);
        break;
      }
    }
    for (byte i = 0; i < 100; i++) {
      filename2[6] = '0' + i / 10;
      filename2[7] = '0' + i % 10;
      if (!SD.exists(filename2)) {
        datalog = SD.open(filename2, FILE_WRITE);
        datalog.close();
        sdActiveF2 = true;
        Serial.println("Logging to: " + String(filename2));
        updateOled("Logging:\n\n" + String(filename2));
        delay(1000);
        break;
      }
    }
   // if (datalog = SD.open(filename2, FILE_WRITE)) {
   //   Serial.println("yesssssssssss");
  //  }
    if (!sdActiveF1) {
      Serial.println("No available file names; clear SD card to enable logging");
      updateOled("Clear SD!");
      delay(5000);
    }
    if (!sdActiveF2) {
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
  //Serial.println(Data);
}

void logRelay(String Relay) {
  datalog = SD.open(filename2, FILE_WRITE);
  datalog.println(Relay);
  datalog.close();
}
