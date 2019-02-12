// ---------------------------------------------------------------------------
// Example NewPing library sketch that does a ping about 20 times per second.
// ---------------------------------------------------------------------------

#include <NewPing.h>
#include <AFMotor.h>

AF_DCMotor motor(2);
AF_DCMotor motork(4);


#define TRIGGER_PIN  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

// Object avoidance distances (in inches)
#define SAFE_DISTANCE 70
#define TURN_DISTANCE 25
#define STOP_DISTANCE 12

#define TURN_LEFT BACKWARD
#define TURN_RIGHT FORWARD
#define TURN_STRAIGHT RELEASE

// Speeds (range: 0 - 255)
#define FAST_SPEED 150
#define NORMAL_SPEED 125
#define TURN_SPEED 100
#define SLOW_SPEED 75
int speed = NORMAL_SPEED;



// Steering/turning 
enum directions {left = TURN_LEFT, right = TURN_RIGHT, straight = TURN_STRAIGHT} ;
directions turnDirection = straight;


 int sonarDistance = 0;

void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
   // turn on motor
//motor.setSpeed(200);
 
   Serial.println("Motor test!");
  motor.run(RELEASE);
  motork.run(RELEASE);
}

void loop() {
  
  delay(50);                     // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  Serial.print("Ping: ");
  Serial.print(sonar.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.println("cm");


  
   sonarDistance = sonar.ping_cm();
   moveAndAvoid(); 
   
}


void moveAndAvoid(void)
{
     
    if (sonarDistance >= SAFE_DISTANCE)       // no close objects in front of car
        {
           if (turnDirection == straight)
             speed = FAST_SPEED;
           else
             speed = TURN_SPEED;
           motor.setSpeed(speed);
           motor.run(FORWARD);       
          // motork.run(turnDirection);
           Serial.print("\nMoving Forward");
           return;
        }

if (sonarDistance > TURN_DISTANCE && sonarDistance < SAFE_DISTANCE)    // not yet time to turn, but slow down
       {
         if (turnDirection == straight)
           speed = NORMAL_SPEED;
         else
           {
              speed = TURN_SPEED;
           //   motork.run(turnDirection);      // alraedy turning to navigate
            }
         motor.setSpeed(speed);
         motor.run(FORWARD);
         Serial.print("\nSlowing Down & Moving Forward");       
         return;
       }
     
  /*   if (sonarDistance <  TURN_DISTANCE && sonarDistance > STOP_DISTANCE)  // getting close, time to turn to avoid object        
        {
          speed = SLOW_SPEED;
          motor.setSpeed(speed);      // slow down
          motor.run(FORWARD); 
          Serial.print("\nSlowing Down & Turning Right");
          turnDirection = right;
           motork.run(turnDirection);
           motor.setSpeed(speed);
          motor.run(FORWARD);
           motork.run(RELEASE); 
                if (sonarDistance <  STOP_DISTANCE){
                  turnDirection = left;
                            Serial.print("\nSlowing Down & Turning Left");
                   motor.run(RELEASE);            // stop 
                   motork.run(RELEASE); 
                    motor.setSpeed(NORMAL_SPEED);  // go back at higher speet
                    motor.run(BACKWARD);
                 motork.run(turnDirection);  // turn in the new direction
                motor.setSpeed(speed);
          motor.run(FORWARD);
          motork.run(RELEASE); 
              }
         return;
        }

*/

  
    if (sonarDistance <  STOP_DISTANCE)          // too close, stop and back up
       {
       //  motor.run(RELEASE);            // stop 
     //    motork.run(RELEASE);             // straighten up
         turnDirection = straight;
         motor.setSpeed(NORMAL_SPEED);  // go back at higher speet
         motor.run(BACKWARD);           
               
                // stop backing up
         Serial.print("\nBreaking & Moving Backward");
         return;
        } // end of IF TOO CLOSE
     
}   // moveAndAvoid()





