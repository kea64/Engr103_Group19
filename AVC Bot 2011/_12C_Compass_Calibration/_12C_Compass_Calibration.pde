//Nathan Schomer 2011

#include <Wire.h>             //used for communication with compass and gps

int HMC6352Address = 0x42;
int slaveAddress;
byte headingData[2];

void setup(){
  Serial.begin(9600);
  slaveAddress = HMC6352Address >> 1;
  Wire.begin();
}

void loop(){
  
  delay(5000);
  
  Serial.println("Calibration of HMC6352 begining in:");
  Serial.println("3");
  delay(1000);
  Serial.println("2");
  delay(1000);
  Serial.println("1");
  delay(1000);
  
  Wire.beginTransmission(slaveAddress);
  Wire.send("C");              
  Wire.endTransmission();
  
  Serial.println("Calibration Mode Entered.");
  Serial.println("Please place the module on a flat surface and rotate slowly to calibrate.");
  Serial.println("30 Seconds Remaining.");
  delay(15000);
  Serial.println("15 Seconds Remaining.");
  delay(12000);
  Serial.println("Calibration mode ending in:");
  Serial.println("3");
  delay(1000);
  Serial.println("2");
  delay(1000);
  Serial.println("1");
  delay(1000);
  
  Wire.beginTransmission(slaveAddress);
  Wire.send("E");              
  Wire.endTransmission();
  Serial.println("Calibration Mode Exited.");
}
