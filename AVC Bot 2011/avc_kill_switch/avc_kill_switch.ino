int steeringPin = 6;
int throttlePin = 7;

unsigned long steeringValue;
unsigned long throttleValue;

void setup(){
  pinMode(steeringPin, INPUT);
  pinMode(throttlePin, INPUT);
  pinMode(13, OUTPUT);
}

void loop(){
  steeringValue = pulseIn(steeringPin, HIGH);
  throttleValue = pulseIn(throttlePin, HIGH);
  
  if(steeringValue >= 2070 && throttleValue <= 1000){
    digitalWrite(13, HIGH);
  }
}


