const int motorPin = 9;      // LED connected to digital pin 9
int val = 0;         // variable to store the read value

void setup()
{
  pinMode(motorPin, OUTPUT);   // sets the pin as output
  Serial.begin(9600);
}

void loop()
{
  if (Serial.available() > 0) {
    val = Serial.read();
    analogWrite(motorPin, val);  //analogWrite values from 0 to 255
    Serial.print("I received: ");
    Serial.println(val, DEC);
  }
}
