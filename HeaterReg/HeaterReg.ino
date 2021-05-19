#define heaterSet 14
#define heaterReset 15
#define thermIntPin A22 // With analog pins you must specify using ‘A’ out front
float minimumTemp = 90; // Value is not important for now
float tolerance = 1; // Again, not important for now
float heaterTempValue; // This will be set by some sensor read function. Just pretend this variable always has the temperature
float minimumTemperature = minimumTemp;
float maximumTemperature = (minimumTemp + tolerance); // all of these are redudant, but allows for changing of maximum/min temp as opposed to having max temp be just = to tolerance

int analogResolutionBits = 13;
float adcMax = pow(2,analogResolutionBits)-1.0;
float A = 0.001125308852122; // A, B, and C are constants used for a 10k resistor and 10k thermistor for the steinhart-hart equation
float B = 0.000234711863267;
float C = 0.000000085663516; 
int R1 = 10000; //ohms

void setup(){
  heaterSetup();
  Serial.begin(9600);
}

void loop() {
  setHeaderState();
  delay(1000);
  updateThermistor();   
}

void heaterSetup(){
  pinMode(heaterSet,OUTPUT);
  pinMode(heaterReset,OUTPUT);
}

void setHeaderState(){
 
  if(heaterTempValue <= minimumTemperature){ 
    digitalWrite(heaterSet,HIGH);
    delay(10); 
    digitalWrite(heaterSet,LOW);
  }
  else if(heaterTempValue >= maximumTemperature){ 
    digitalWrite(heaterReset,HIGH);
    delay(10);
    digitalWrite(heaterReset,LOW);
  }
 }

 void updateThermistor(){
  analogReadResolution(analogResolutionBits);
  float adcVal = analogRead(thermIntPin);
  float logR = log(((adcMax/adcVal)-1)*R1);
  float Tinv = A+B*logR+C*logR*logR*logR;
  float T = 1/Tinv;
  float currentTempC = T-273.15; // converting to celcius
  float currentTempF = currentTempC*9/5+32;
  float thermistorInt = currentTempF;
  heaterTempValue = thermistorInt;
  Serial.println("Temp (F): " + String(thermistorInt));
}
