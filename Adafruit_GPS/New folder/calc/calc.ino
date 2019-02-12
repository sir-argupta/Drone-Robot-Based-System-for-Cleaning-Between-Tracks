
#include <math.h>


float currentLat = 24.0384,
      currentLong = 84.0588,
      targetLat = 24.038073,
      targetLong = 84.059076;
int distanceToTarget,            // current distance to target (current waypoint)
    originalDistanceToTarget;    // distance to original waypoing when we started navigating to it


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");

}

void loop() {
  // put your main code here, to run repeatedly:

   distanceToWaypoint();
       courseToWaypoint(); 

}


void nextWaypoint(void)
{
  
  
  if ((targetLat == 0 && targetLong == 0) )    // last waypoint reached? 
    {
      /*driveMotor->run(RELEASE);    // make sure we stop
      turnMotor->run(RELEASE);  
      lcd.clear();
      lcd.println(F("* LAST WAYPOINT *"));
      loopForever();*/
    }

     Serial.print(" \n Target Latitude   "); 
       Serial.print(targetLat ); 
       Serial.print(", \n Target Longitude   "); 
       Serial.print(targetLong ); 
      Serial.print(", \n");
    
   //processGPS();
   distanceToTarget = originalDistanceToTarget = distanceToWaypoint();
   courseToWaypoint();
   
}  // nextWaypoint()





// returns distance in meters between two positions, both specified 
// as signed decimal-degrees latitude and longitude. Uses great-circle 
// distance computation for hypothetical sphere of radius 6372795 meters.
// Because Earth is no exact sphere, rounding errors may be up to 0.5%.
// copied from TinyGPS library
int distanceToWaypoint() 
{

  Serial.print(" \n Target Latitude   "); 
       Serial.print(targetLat ); 
       Serial.print(", \n Target Longitude   "); 
       Serial.print(targetLong ); 
      Serial.print(", \n");
  
  float delta = radians(currentLong - targetLong);
  float sdlong = sin(delta);
  float cdlong = cos(delta);
  float lat1 = radians(currentLat);
  float lat2 = radians(targetLat);
  float slat1 = sin(lat1);
  float clat1 = cos(lat1);
  float slat2 = sin(lat2);
  float clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong); 
  delta = sq(delta); 
  delta += sq(clat2 * sdlong); 
  delta = sqrt(delta); 
  float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong); 
  delta = atan2(delta, denom); 
  distanceToTarget =  delta * 6372795; 
   
  // check to see if we have reached the current waypoint
//  if (distanceToTarget <= WAYPOINT_DIST_TOLERANE)
   // nextWaypoint();

    Serial.print(", \n Printing From DistanceToWayPt.  ");
    Serial.print(distanceToTarget);
    
  return distanceToTarget;
}  // distanceToWaypoint()




// returns course in degrees (North=0, West=270) from position 1 to position 2,
// both specified as signed decimal-degrees latitude and longitude.
// Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
// copied from TinyGPS library
int courseToWaypoint() 
{
  float dlon = radians(targetLong-currentLong);
  float cLat = radians(currentLat);
  float tLat = radians(targetLat);
  float a1 = sin(dlon) * cos(tLat);
  float a2 = sin(cLat) * cos(tLat) * cos(dlon);
  a2 = cos(cLat) * sin(tLat) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }

   Serial.print(", \n Printing From CourseToWayPt.  ");
    Serial.print(degrees(a2));
  
 // targetHeading = degrees(a2);
 // return targetHeading;
 
}   // courseToWaypoint()





