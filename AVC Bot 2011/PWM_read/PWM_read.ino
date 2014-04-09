#include <SPI.h>
#include <Ethernet.h>

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192,168,1, 177);

int steeringPin = 6;
int throttlePin = 7;

EthernetServer server(80);

unsigned long steeringValue;
unsigned long throttleValue;

void setup(){
  Ethernet.begin(mac, ip);
  server.begin();
  Serial.begin(9600);
  pinMode(steeringPin, INPUT);
  pinMode(throttlePin, INPUT);
  pinMode(13, OUTPUT);
}

void loop(){
  steeringValue = pulseIn(steeringPin, HIGH);
  throttleValue = pulseIn(throttlePin, HIGH);
  
  Serial.print("Steering Value = ");
  Serial.println(steeringValue, DEC);
  Serial.println("************************************************************");
  delay(2000);
  
  Serial.print("Throttle Value = ");
  Serial.println(throttleValue, DEC);
  Serial.println("************************************************************");
  delay(2000);
}


