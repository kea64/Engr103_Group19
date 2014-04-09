#include <NewSoftSerial.h>    //allows other pins to be used as RX & TX pins on arduino
#include <TinyGPS.h>          //obviously used for gps
#include <Wire.h>             //used for communication with compass and gps
#include <Servo.h>

TinyGPS gps;
NewSoftSerial nss(2, 3);

void gpsdump(TinyGPS &gps);
bool feedgps();
void printFloat(double f, int digits = 2);

float flat, flon, x2lat, x2lon;
int HMC6352Address = 0x42;
int slaveAddress;
byte headingData[2];
int i, headingValue;
int headingcompass;
int x4=0;
long lat, lon;
float heading;
Servo servo;
Servo esc;

int steeringPin = 6;
int throttlePin = 7;
unsigned long steeringValue;
unsigned long throttleValue;

float flat2 = 40.29664722; 
float flon2 = -79.463925;
float flat3 = 40.2967693747;
float flon3 = -79.463776412;
float flat4 = 40.29683056;
float flon4 = -79.46356111;
float flat5 = 40.29689444;
float flon5 = -79.46335;
float flat6 = 40.29697222;
float flon6 = -79.46301667;

int waycont=1;

int waypoints = 5;

void setup(){
  Serial.begin(115200);      //Serial baud rate of 115200
  nss.begin(4800);           //communication with gps module runs @ 4800 baud
  slaveAddress = HMC6352Address >> 1;
  
  Wire.begin();
  servo.attach(9);
  esc.attach(10);
  esc.write(93);
  
  pinMode(12, OUTPUT);
  pinMode(steeringPin, INPUT);
  pinMode(throttlePin, INPUT);
}

void loop()
{
  bool newdata = false;
  unsigned long start = millis();

  // Every fourth of a second we print an update
  while (millis() - start < 250)
  {
    if (feedgps())
      newdata = true;
  }
  
  if (newdata)
  {
    Serial.println("Acquired Data");
    Serial.println("-------------");
    gpsdump(gps);
    Serial.println("-------------");
    Serial.println();
  }
}

void printFloat(double number, int digits)
{
  
  
  if(analogRead(A2) >= 237){
    avoid();
  }
  
  // Handle negative numbers
  if (number < 0.0)
  {
     Serial.print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Serial.print("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint; 
  } 
  
    steeringValue = pulseIn(steeringPin, HIGH);
  throttleValue = pulseIn(throttlePin, HIGH);
  
  if(steeringValue >= 2070 && throttleValue <= 1000){
    done();
  }
}

void gpsdump(TinyGPS &gps)
{
  
  
  if(analogRead(A2) >= 237){
    avoid();
  }
  
  long lat, lon;
  float flat, flon;
  unsigned long age, date, time, chars;
  unsigned short sentences, failed;

  gps.get_position(&lat, &lon, &age);
  Serial.print("Lat/Long(10^-5 deg): "); Serial.print(lat); Serial.print(", "); Serial.print(lon); 
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");
  
  feedgps(); // If we don't feed the gps during this long routine, we may drop characters and get checksum errors

  gps.f_get_position(&flat, &flon, &age);
  Serial.print("Lat/Long(float): "); printFloat(flat, 5); Serial.print(", "); printFloat(flon, 5);
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");
  distance(flat, flon);

  feedgps();

  gps.stats(&chars, &sentences, &failed);
  Serial.print("Stats: characters: "); Serial.print(chars); Serial.print(" sentences: ");
  Serial.print(sentences); Serial.print(" failed checksum: "); Serial.println(failed);
  //    distance();
  
    steeringValue = pulseIn(steeringPin, HIGH);
  throttleValue = pulseIn(throttlePin, HIGH);
  
  if(steeringValue >= 2070 && throttleValue <= 1000){
    done();
  }
}
  
bool feedgps()
{
  while (nss.available()){
    
    if (gps.encode(nss.read()))
      return true;
  }
  return false;
}

void distance(float flat1, float flon1){
  
  
  if(analogRead(A2) >= 237){
    avoid();
  }
  
  if(waycont==1){

 x2lat = flat2;      // setting x2lat and x2lon equal to our first waypoint

 x2lon = flon2;   

  }

  if(waycont==2){

    x2lat = flat3;

    x2lon = flon3;

  }

  if(waycont==3){

    x2lat = flat4;

    x2lon = flon4;

  }

  if(waycont==4){

  x2lat = flat5;

  x2lon = flon5;

  }

   if(waycont==5){

  x2lat = flat6;

  x2lon = flon6;

  }
   
   //float flat1=flat;     // flat1 = our current latitude. flat is from the gps data. 
   //float flon1=flon;  // flon1 = our current longitude. flon is from the fps data.
   float dist_calc=0;
   float dist_calc2=0;
   float diflat=0;
   float diflon=0;

   Serial.print("flat1/flon1: "); printFloat(flat1, 5); Serial.print(", "); printFloat(flon1, 5); Serial.println("");
   Serial.print("x2lat/x2lon: "); printFloat(x2lat, 5); Serial.print(", "); printFloat(x2lon, 5); Serial.println("");
   
//------------------------------------------ distance formula below. Calculates distance from current location to waypoint
   
   diflat=radians(x2lat-flat1);  //notice it must be done in radians
   flat1=radians(flat1);    //convert current latitude to radians
   x2lat=radians(x2lat);  //convert waypoint latitude to radians
   diflon=radians((x2lon)-(flon1));   //subtract and convert longitudes to radians
   
   dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
   dist_calc2= cos(flat1); 
   dist_calc2*=cos(x2lat);
   dist_calc2*=sin(diflon/2.0);                                       
   dist_calc2*=sin(diflon/2.0);
   dist_calc +=dist_calc2;
   dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));
   dist_calc*=6371000.0; //Converting to meters
   
   Serial.println("distance");
   Serial.println(dist_calc);    //print the distance in meters
   
   flon1 = radians(flon1);
   x2lon = radians(x2lon);
   heading = atan2(sin(x2lon-flon1)*cos(x2lat),cos(flat1)*sin(x2lat)-sin(flat1)*cos(x2lat)*cos(x2lon-flon1)),2*3.1415926535;
   heading = heading*180/3.1415926535;  // convert from radians to degrees

   int head =heading; //make it a integer now

   if(head<0){

      heading+=360;   //if the heading is negative then add 360 to make it positive

  }

   Serial.println("heading:");
   Serial.println(heading);   // print the heading
   Wire.beginTransmission(slaveAddress);        //the wire stuff is for the compass module

   Wire.send("A");              
   Wire.endTransmission();
   delay(10);                  
   Wire.requestFrom(slaveAddress, 2);       
   i = 0;
   
   while(Wire.available() && i < 2){ 
      headingData[i] = Wire.receive();
      i++;
   }

   headingValue = headingData[0]*256 + headingData[1];
   int pracheading = headingValue / 10;      // this is the heading of the compass
   
   if(pracheading>0){
      headingcompass=pracheading;
   }

   Serial.println("current heading:");
   Serial.println(headingcompass);
   x4=headingcompass-heading;   //getting the difference of our current heading to our needed bearing

   int turn;

//-------------------------------------- below tells us which way we need to turn

   if(x4>=-180){

   if(x4<=0){

      turn=8;    //set turn =8 which means "right"         

    }

  }

if(x4<-180){

  turn=5;      //set turn = 5 which means "left"

}

if(x4>=0){

  if(x4<180){

    turn=5;   //set turn = 5 which means "left"

  }

}

if(x4>=180){     //set turn =8 which means "right"

  turn=8;

}

int hd = headingcompass;

 

if(hd==heading){

    turn=3;   //then set turn = 3 meaning go "straight"

}

if(turn==3){
  servo.write(88);
  Serial.println("straight");
}

//-------------------------------------------------------------------------------------turn right

if(turn==8){

rightturn();

}

//------------------------------------------------------------------------------------------turn left

if(turn==5){

leftturn();

}

  steeringValue = pulseIn(steeringPin, HIGH);
  throttleValue = pulseIn(throttlePin, HIGH);
  
  if(steeringValue >= 2070 && throttleValue <= 1000){
    done();
  }

Serial.println("distance"); 

Serial.println(dist_calc);

if(dist_calc<4){             //this goes after the serial print of the distance in the function "distance". refer to my code. 4=radius

if(waycont==waypoints){   //this has to do with switching waypoints

  done();     //go to the function to stop motors

}

else{
waycont+=1;
servo.write(88);
esc.write(93);
digitalWrite(12, HIGH);
delay(2000);
digitalWrite(12, LOW); 
}
}
}

//----------------------------------------------------------------------------------
//---------------------------------------------------------------------------
//------------------------------------------------------right turning
void rightturn(){
if(headingcompass+2>heading){
  if(headingcompass-2<heading){
    servo.write(88);
    delay(60);
    return;
  }
}
 x4=headingcompass-heading;  
if(x4<-180){
return;
}
if(x4>=0){
  if(x4<180){
  return;
  }
}
servo.write(97);
esc.write(86);
  Wire.beginTransmission(slaveAddress);        //the wire stuff is for the compass module
  Wire.send("A");              
  Wire.endTransmission();
  delay(10);                  
  Wire.requestFrom(slaveAddress, 2);       
  i = 0;
  
  while(Wire.available() && i < 2){ 
    headingData[i] = Wire.receive();
    i++;
  }
  
  headingValue = headingData[0]*256 + headingData[1];  
headingcompass = headingValue / 10;      // this is the heading of the compass
 
  steeringValue = pulseIn(steeringPin, HIGH);
  throttleValue = pulseIn(throttlePin, HIGH);
  if(steeringValue >= 2070 && throttleValue <= 1000){
    done();
  }
  
  
  if(analogRead(A2) >= 237){
    avoid();
  }
    
rightturn();
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------
//----------------------------------------------left turning
void leftturn(){
  if(headingcompass+2>heading){
  if(headingcompass-2<heading){
  servo.write(88);
    delay(60);
    return;
  }
}
x4=headingcompass-heading;  
if(x4>=-180){
  if(x4<=0){
     return;         
  }
}

if(x4>=180){     
  return;
}
servo.write(78);  // turn wheels left
esc.write(86);  //our speed

  Wire.beginTransmission(slaveAddress);        //the wire stuff is for the compass module
  Wire.send("A");              
  Wire.endTransmission();
  delay(10);                  
  Wire.requestFrom(slaveAddress, 2);       
  i = 0;
  while(Wire.available() && i < 2)
  { 
    headingData[i] = Wire.receive();
    i++;
  }
headingValue = headingData[0]*256 + headingData[1];  
headingcompass = headingValue / 10;      // this is the heading of the compass

  steeringValue = pulseIn(steeringPin, HIGH);
  throttleValue = pulseIn(throttlePin, HIGH);
  if(steeringValue >= 2070 && throttleValue <= 1000){
    done();
  }
  
  
  if(analogRead(A2) >= 237){
    avoid();
  }
    
leftturn();
}

/*---------------------------DONE-------------------------------------------*/

void done(){
  servo.write(87);
  esc.write(93);
  digitalWrite(12, HIGH);
  delay(2000);
  digitalWrite(12, LOW);  
  done();
}

void avoid(){
  esc.write(93);
  delay(1000);
  
  if(analogRead(A2) >= 237){
    servo.write(78);
    delay(200);
    esc.write(86);
    delay(2500);
    esc.write(93);
  }
  
  
  if(analogRead(A2) >= 237){
    avoid();
  }
  
  return;
}
    
