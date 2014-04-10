#include <Servo.h> 

Servo myServo;
int pin=9;
void setup() 
{ 
  myServo.attach(9);
  delay(1000);
  pinMode(pin,OUTPUT);
  digitalWrite(pin,HIGH);
  delay(12);
  digitalWrite(pin,LOW);
  delay(8.5);

  myServo.write(110);
  delay(2000);
  myServo.write(90);
  delay(1500);
  myServo.write(70);
  delay(1500);
  myServo.write(90);

 
 
//  myServo.write();
//  delay(11);
//  myServo.write(180);
//  delay(11);
//  myServo.write(90);
  
}

void loop() 
{
//analogWrite(pin,127);
//myServo.write(110);
//
//digitalWrite(pin,HIGH);
//delay(1.5);
//digitalWrite(pin,LOW);
//delay(8.5);

} 
