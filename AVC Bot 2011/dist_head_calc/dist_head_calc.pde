/*
GPS:
VCC connected to V
GND connected to GND
RX connected to digital pin 11
TX connected to digital pin 10

Magnetometer:
GND connected to GND
VCC connectd to 3.3V
SDA connected to digital pin 20
SCl connected to digital pin 21*/

#include <Servo.h>
#include <SoftwareSerial.h>              //allows for serial communication on pins 10 & 11
#include <icrmacros.h>                   //?
#include <TinyGPS.h>                     //GPS library to communicate w gps module
#include <Wire.h>                        //to communicate w 12c interfaces like the magnetometer

TinyGPS gps;
SoftwareSerial mySerial(10, 11);         //GPS connected to pins 10 & 11

/*void gpsdump(TinyGPS &gps);*/         

bool feedgps();
void printFloat(double f, int digits = 2);


int R = 6371000; //meters                //radius of the earth for calculations
float latA = 40.3525;                    //latitude of first waypoint
float lonA = -79.4714;                   //longitude of first waypoint

float flat, flon;
long lat, lon;
unsigned long age, date, time, chars;
int year;
byte month, day, hour, minute, second, hundredths;
unsigned short sentences, failed;

int HMC6352SlaveAddress = 0x42;
int HMC6352ReadAddress = 0x41; //"A" in hex, A command is: 
int headingValue;


int headTol = 10;
float headDiff;

Servo turnServo;                        //Servo for turning
Servo ESC;                              //ESC


void setup(){
  Serial.begin(115200);                  //Serial communication w computer @ 115200 bps
  mySerial.begin(4800);                  //Serial communication w GPS @ 4800 bps
  
  Wire.begin();                          //12C communication w Magnetometer
  HMC6352SlaveAddress = HMC6352SlaveAddress >> 1; //Address of Magnetometer
  
  turnServo.attach(2);
  ESC.attach(3);
  ESC.write(93);  
}

void loop(){                             //main control loop
 bool newdata = false;
  unsigned long start = millis();

  while (millis() - start < 250)         // Every 1/4 second we print an update
  {
    if (feedgps())
      newdata = true;
  }
  
  if (newdata){
    
 /* Serial.println("Acquired Data");
    Serial.println("-------------");
    gpsdump(gps);
    Serial.println("-------------"); */
    
    
    Serial.println();                 
    Serial.println();
    Serial.println("Calculated Data");
    Serial.println("--------------");
    calculations();                      //goto calculations
  }
}



void calculations(){  //Main GPS Distance and heading calculations
  
  feedgps();
  
  gps.f_get_position(&flat, &flon);
  float currLat = flat;
  float currLon = flon;
  
  Serial.print("Current Location: ");
  Serial.print(currLat, 5);
  Serial.print(", ");
  Serial.println(currLon, 5);

  
  float lat0 = radians(currLat);
  float lon0 = radians(currLon);
  float lat1 = radians(latA);
  float lon1 = radians(lonA);
  
  float dLat = (lat1-lat0);
  float dLon = (lon1-lon0);


/********************************GPS CALCULATIONS************************************/  
  
  feedgps();
  
  float a = sin(dLat/2) * sin(dLat/2) + cos(lat1) * //haversine formula for calculating
          cos(lat1) * sin(dLon/2) * sin(dLon/2);      //distances on a sphere (like earth)
          
  float c = 2 * atan2(sqrt(a), sqrt(1-a));          
  float m = R * c;                                   
  float dist = m * 3.2808;                         

  Serial.print("Distance in feet: ");              
  Serial.println(dist);
  Serial.print("Distance in meters: ");
  Serial.println(m);
  
  
/*********************************HEADING CALCULATIONS**********************************/

  feedgps();
  
  float y = sin(dLon) * cos(lat1);                    //formula for determining the initial
                                                      //heading of current location
  float x = cos(lat0)*sin(lat1) -                     //to the next waypoint
        sin(lat0)*cos(lat1)*cos(dLon);
  
  float brng = degrees(atan2(y, x));
  
  brng = brng*(180/3.14159265358979);
  
  brng = brng + 360;
  
  
  
  Serial.println();     
  Serial.print("Heading to next coordinate: ");
  Serial.println(brng);
  
  
/************************************Turning Calculations*****************************/

  //"Get Data. Compensate and Calculate New Heading"
  Wire.beginTransmission(HMC6352SlaveAddress);
  Wire.send(HMC6352ReadAddress);              // The "Get Data" command
  Wire.endTransmission();

  //time delays required by HMC6352 upon receipt of the command
  //Get Data. Compensate and Calculate New Heading : 6ms
  delay(6);

  Wire.requestFrom(HMC6352SlaveAddress, 2); //get the two data bytes, MSB and LSB

  //"The heading output data will be the value in tenths of degrees
  //from zero to 3599 and provided in binary format over the two bytes."
  byte MSB = Wire.receive();
  byte LSB = Wire.receive();

  float headingSum = (MSB << 8) + LSB; //(MSB / LSB sum)
  float currHead = headingSum / 10; 
  
  if (currHead < 0){
    currHead = currHead + 360;
  }

  Serial.print("current heading: ");
  Serial.print(currHead);
  Serial.println(" degrees");
  
  headDiff = currHead - brng;      //calculating the difference between the current
  
  Serial.print("Heading Difference: ");
  Serial.println(headDiff);

  delay(100);
  
  /***************now to determine which way we need to turn****************/ 
  int turn;

  if(headDiff > -360 + headTol){                  //if the differnce in current heading and the bearing to the next coordinate is greater than
    if(headDiff < 0 - headTol){                      // or equal to -360 plus the tolerance and less than or equal to zero minus the tolerance then 
      rightturn;                                     //then turn right
    }
  }
  
  if(headDiff < 360 - headTol){
    if(headDiff > 0 + headTol){
      leftturn;                                  //turn left  
    }
  }
    
  if(headDiff <= headTol){
    if(headDiff >= -1 * headTol){
      Serial.println("Turning Status: STRAIGHT");
      //servo.write(88);                           //go straight;
    }
  }
}



void rightturn(){
  Serial.println("Turning Status: RIGHT");
  turnServo.write(88);
  delay(60);
  turnServo.write(122);
  
  if(headDiff > -360 + headTol){                  //if the differnce in current heading and the bearing to the next coordinate is greater than
    if(headDiff < 0 - headTol){                      // or equal to -360 plus the tolerance and less than or equal to zero minus the tolerance then 
      rightturn;                                     //then turn right
    }
    else{
      calculations;
    }
  }
  else{
    calculations;
  }
}


void leftturn(){
  Serial.println("Turning Status: LEFT");
  turnServo.write(88);
  delay(60);
  turnServo.write(78);
  
  if(headDiff < 360 - headTol){
    if(headDiff > 0 + headTol){
      leftturn;                                  //turn left  
    }
    else{
      calculations;
    }
  }
  else{
    calculations;
  }
}


/*void gpsdump(TinyGPS &gps){  //Gathers and prints all GPS data available
  
  gps.get_position(&lat, &lon, &age);
  Serial.print("Lat/Long(10^-5 deg): "); Serial.print(lat); Serial.print(", "); Serial.print(lon); 
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");
  
  feedgps(); // If we don't feed the gps during this long routine, we may drop characters and get checksum errors

  gps.f_get_position(&flat, &flon, &age);
  Serial.print("Lat/Long(float): "); printFloat(flat, 5); Serial.print(", "); printFloat(flon, 5);
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");

  feedgps();

  gps.get_datetime(&date, &time, &age);
  Serial.print("Date(ddmmyy): "); Serial.print(date); Serial.print(" Time(hhmmsscc): "); Serial.print(time);
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");

  feedgps();

  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  Serial.print("Date: "); Serial.print(static_cast<int>(month)); Serial.print("/"); Serial.print(static_cast<int>(day)); Serial.print("/"); Serial.print(year);
  Serial.print("  Time: "); Serial.print(static_cast<int>(hour)); Serial.print(":"); Serial.print(static_cast<int>(minute)); Serial.print(":"); Serial.print(static_cast<int>(second)); Serial.print("."); Serial.print(static_cast<int>(hundredths));
  Serial.print("  Fix age: ");  Serial.print(age); Serial.println("ms.");
  
  feedgps();

  Serial.print("Alt(cm): "); Serial.print(gps.altitude()); Serial.print(" Course(10^-2 deg): "); Serial.print(gps.course()); Serial.print(" Speed(10^-2 knots): "); Serial.println(gps.speed());
  Serial.print("Alt(float): "); printFloat(gps.f_altitude()); Serial.print(" Course(float): "); printFloat(gps.f_course()); Serial.println();
  Serial.print("Speed(knots): "); printFloat(gps.f_speed_knots()); Serial.print(" (mph): ");  printFloat(gps.f_speed_mph());
  Serial.print(" (mps): "); printFloat(gps.f_speed_mps()); Serial.print(" (kmph): "); printFloat(gps.f_speed_kmph()); Serial.println();

  feedgps();

  gps.stats(&chars, &sentences, &failed);
  Serial.print("Stats: characters: "); Serial.print(chars); Serial.print(" sentences: "); Serial.print(sentences); Serial.print(" failed checksum: "); Serial.println(failed);
}
*/





bool feedgps()
{
  while (mySerial.available())
  {
    if (gps.encode(mySerial.read()))
      return true;
  }
  return false;
}




void printFloat(double number, int digits) //used for rounding numbers and handling ngatives
{
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
}
