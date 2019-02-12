#include <NewPing.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include "Adafruit_FONA.h"

#define FONA_RX 10
#define FONA_TX 11
#define FONA_RST 4

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
char replybuffer[255];
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;

char Lat[15],Lon[15];
const char *MapLink       = " http://maps.google.com/maps?q=";
const char *SMSMsg[]      = {"High CO Content @", "Your Vehicle Got Accident @", "Your Vehicle is Located @", " Automated SMS By: SV"};
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define TRIGGER_PIN  7  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     6  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
SoftwareSerial mySerial(3, 2);

float currentLat,
      currentLong;
     

Adafruit_GPS GPS(&mySerial);
 
#define GPSECHO  true


boolean usingInterrupt = false;
void useInterrupt(boolean); 


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
float sona_dist;



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {

/////////////////////////////////////////////******************************************************************/////////////////////////////////////////////////
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
   GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
   GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
   GPS.sendCommand(PGCMD_ANTENNA);
   useInterrupt(true);
   delay(1000);
   mySerial.println(PMTK_Q_RELEASE);
//////////////////////////////////////////////*********************************************************************//////////////////////////////////////////////////////
  while (!Serial);
  Serial.println(F("FONA basic test"));
  Serial.println(F("Initializing....(May take 3 seconds)"));

  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    //while (1);
  }
  type = fona.type();
  Serial.println(F("FONA is OK"));
  Serial.print(F("Found "));
  switch (type) {
    case FONA800L:
      Serial.println(F("FONA 800L")); break;
    case FONA800H:
      Serial.println(F("FONA 800H")); break;
    case FONA808_V1:
      Serial.println(F("FONA 808 (v1)")); break;
    case FONA808_V2:
      Serial.println(F("FONA 808 (v2)")); break;
    case FONA3G_A:
      Serial.println(F("FONA 3G (American)")); break;
    case FONA3G_E:
      Serial.println(F("FONA 3G (European)")); break;
    default: 
      Serial.println(F("???")); break;
  }
  
  // Print module IMEI number.
  char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("Module IMEI: "); Serial.println(imei);
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  delay(50);                     
  Serial.print("Ping: ");
  Serial.print(sonar.ping_cm()); 
  Serial.println("cm");
  sona_dist = sonar.ping_cm();
          processGPS();
          main_control();


  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Function for gps parsing
void processGPS(void)
{
  if (! usingInterrupt) {
    char c = GPS.read();
    
    if (GPSECHO)
      if (c) Serial.print(c);
  } 
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   
      return;  
  } 
  if (timer > millis())  timer = millis();
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Location (in degrees, works with Google Maps): ");
      Serial.print(GPS.latitudeDegrees, 4);
      Serial.print(", "); 
      Serial.println(GPS.longitudeDegrees, 4);
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      currentLat = GPS.latitudeDegrees;
      currentLong = GPS.longitudeDegrees;
             
     if (GPS.lat == 'S')            // make them signed
        currentLat = -currentLat;
     if (GPS.lon == 'W')  
        currentLong = -currentLong;              

       Serial.print(" \n Current Latitude   "); 
       Serial.print(currentLat ); 
       Serial.print(", ");  Serial.print(", \n Current Longtitude    ");
       Serial.print(currentLong ); 
       Serial.print(", \n");

            ftoa(currentLat, Lat, 4);
            ftoa(currentLong, Lon, 4); 
    }
  } 
}

// Converts a floating point number to string.
void ftoa(double n, char *res, int afterpoint){
  // Extract integer part
  int ipart = (int)n;
  // Extract floating part
  double fpart = n - (float)ipart;
  // convert integer part to string
  int i = intToStr(ipart, res, 0);
  // check for display option after point
  if (afterpoint != 0)  {
    res[i] = '.';  // add dot
    // Get the value of fraction part upto given no.
    // of points after dot. The third parameter is needed
    // to handle cases like 233.007
    fpart = fpart * pow(10, afterpoint);
    intToStr((int)fpart, res + i + 1, afterpoint);
  }
}
// Converts a given integer x to string str[].  d is the number
// of digits required in output. If d is more than the number
// of digits in x, then 0s are added at the beginning.
int intToStr(int x, char str[], int d){
  int i = 0;
  while (x) {
    str[i++] = (x%10) + '0';
    x = x/10;
  }
  // If number of digits required is more, then
  // add 0s at the beginning
  while (i < d)
  str[i++] = '0';
  
  rever(str, i);
  str[i] = '\0';
  return i;
}
void rever(char *str, int len){
  int i=0, j=len-1, temp;
  while (i<j) {
    temp = str[i];
    str[i] = str[j];
    str[j] = temp;
    i++; j--;
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void fonas(){
       while (! Serial.available() ) {
    if (fona.available()) {
      Serial.write(fona.read());
    }
  }

      char i, gsmmsg[150],message[180];
  gsmmsg[0] = '\0';
  
  strcat(gsmmsg, MapLink);
    strcat(gsmmsg,Lat);
    strcat(gsmmsg,",");
    strcat(gsmmsg,Lon); 
  strcat(gsmmsg, SMSMsg[3]);
  strcat(message,gsmmsg);
  
 // send an SMS!
        char sendto[21]={"+919155355533"};
        flushSerial();
        Serial.print(F("Send to #+919155355533"));
        //readline(sendto, 20);
        Serial.println(sendto);
        //Serial.print(F("Type out one-line message (140 char): "));
        //readline(message, 140);
        Serial.println(message);
        if (!fona.sendSMS(sendto, message)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("Sent!"));
        }
}
void flushSerial() {
  while (Serial.available())
    Serial.read();
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void main_control()
{
     /* if (sona_dist < 24){
        fonas();
        
      }
  
  */
}

















