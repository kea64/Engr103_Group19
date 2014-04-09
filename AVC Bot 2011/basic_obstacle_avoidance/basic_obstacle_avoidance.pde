#include <Servo.h>

Servo turningServo;
Servo esc;

void setup(){
  turningServo.attach(9);
  esc.attach(10);
  esc.write(93);
  pinMode(13, OUTPUT);
}

void loop(){
  
  int sensorValue = analogRead(A0);
  
  if(sensorValue >= 237){
    digitalWrite(13, HIGH);
    esc.write(93);
    delay(500);
    turningServo.write(55);
    esc.write(106);
    delay(1000);
  }
  
  else{
    esc.write(86);
    turningServo.write(87);
    digitalWrite(13, LOW);
  }
}
