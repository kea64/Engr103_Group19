#include <Servo.h>

Servo turningServo;
Servo esc;

void setup(){
  turningServo.attach(9);
  esc.attach(10);
  esc.write(93);
}

void loop(){
        esc.write(86);
        delay(5000);
        esc.write(93);
        delay(1000);
        turningServo.write(55);
        delay(750);
        esc.write(86);
        delay(2000);
        turningServo.write(122);
        esc.write(93);
        delay(60000);
    }

