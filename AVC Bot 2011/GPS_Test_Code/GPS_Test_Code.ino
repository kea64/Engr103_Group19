#include <SoftwareSerial.h>
#include <TinyGPS.h>

TinyGPS gps;
SoftwareSerial ss(2, 3);
float flat, flon, x2lat, x2lon;
void gpsdump(TinyGPS &gps);
bool feedgps();
void printFloat(double f, int digits = 2);

void setup(){
  Serial.begin(115200);
  ss.begin(4800); 
}

void loop(){
  bool newdata = false;
  unsigned long start = millis();

  // Every fourth of a second we print an update
  while (millis() - start < 250){
    if (feedgps())
      newdata = true;
  }
  
  if (newdata){
    Serial.println("Acquired Data");
    Serial.println("-------------");
    gpsdump(gps);
    Serial.println("-------------");
    Serial.println();
  }
}

void printFloat(double number, int digits){
  // Handle negative numbers
  if (number < 0.0){
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
  while (digits-- > 0){
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint; 
  } 
}

void gpsdump(TinyGPS &gps){
  long lat, lon;

  unsigned long age, date, time, chars;
  unsigned short sentences, failed;

  gps.get_position(&lat, &lon, &age);
  Serial.print("Lat/Long(10^-5 deg): "); Serial.print(lat); Serial.print(", "); Serial.print(lon); 
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");
  
  feedgps(); // If we don't feed the gps during this long routine, we may drop characters and get checksum errors

  gps.f_get_position(&flat, &flon, &age);
  Serial.print("Lat/Long(float): "); printFloat(flat, 5); Serial.print(", "); printFloat(flon, 5);
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");

  feedgps();

  gps.stats(&chars, &sentences, &failed);
  Serial.print("Stats: characters: "); Serial.print(chars); Serial.print(" sentences: "); Serial.print(sentences); Serial.print(" failed checksum: "); Serial.println(failed);
  distance();
}
  
bool feedgps(){
  while (ss.available()){
    if (gps.encode(ss.read()))
      return true;
  }
  return false;
}

void distance(){
float flat1=flat;     // flat1 = our current latitude. flat is from the gps data. 
float flon1=flon;  // flon1 = our current longitude. flon is from the fps data.
float dist_calc=0;
float dist_calc2=0;
float diflat=0;
float diflon=0;
x2lat=    25  ;  //enter a latitude point here   this is going to be your waypoint
x2lon= 25;
//---------------------------------- distance formula below. Calculates distance from current location to waypoint
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
}

