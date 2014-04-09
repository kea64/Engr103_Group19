const int baroPin = 1;           //barometer input pin
const int safePin = 5;           //safety deactivated by activating relay on ejection charge battery
const int firePin = 4;           //relay closes circuit to ejection charge            
const int switchPin = 8;         //used to control if statements within "void loop"
const int altButton = 7;         //used to set baseline pressure
const int testButtonPin = 6;    //used for system test

int altButtonState = 0;    //variable for baseline pressure
int testButtonState = 0;   //variable for buttonPin
int switchState = 0;       //variable for switchPin
int x = 0;                 //variable for baseline pressure
int y = 846;               //846 = 200 feet

void setup(){
  pinMode(baroPin, INPUT);        //barometer is an input
  pinMode(testButtonPin, INPUT);  //button input w/ tactile switch
  pinMode(safePin, OUTPUT);       //safety is an output
  pinMode(firePin, OUTPUT);       //firePin is an output
  pinMode(switchPin, INPUT);      //switch to control if statements
  pinMode(altButton, INPUT);      //altButton is 
}

void loop(){  
 altButtonState = digitalRead(altButton);
 if(altButtonState = HIGH){
   x = analogRead(5);
 } 
 
  switchState = digitalRead(switchPin); 
  if(switchState == LOW){goto flight;}
  if(switchState == HIGH){goto test;}
  
    // x = current barometer reading
    // y = the height you want to fly
    // y would be defined using the mathmatical formula
    //if(analogRead(baroPin) > x + Y
    
loop();{    
  flight: 
  if(analogRead(baroPin) >= x + y){   //change value based on P.E.L. (int 0- 1,023)
   digitalWrite(safePin, HIGH);     //safety deactivated
  }
  else{
    digitalWrite(safePin, LOW);     //safety activated all other times
  }
  
  
   if(analogRead(baroPin) <= x + y){   //change value to match desiered P.E.L. (int 0- 1,023)
   digitalWrite(firePin, HIGH);     //fire parachute ejection charge by setting firePin HIGH
  }
  else {
    digitalWrite(firePin, LOW);     //firePin is LOW all other times
    }
  }

  
 /********************************************************************
  ******************************************************************** 
  ********************************************************************
  ********************************************************************
  ********************************************************************/
 
  loop();{
  test:
    testButtonState = digitalRead(testButtonPin);
    if(testButtonState == HIGH){
      digitalWrite(safePin, HIGH);
      digitalWrite(firePin, HIGH);
    }
    else{
      digitalWrite(safePin, LOW);
      digitalWrite(firePin, LOW);
    }
  }
}
