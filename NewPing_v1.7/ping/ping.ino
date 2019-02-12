/*
 HC-SR04 Ping distance sensor]
 VCC to arduino 5v GND to arduino GND
 Echo to Arduino pin 13 Trig to Arduino pin 12
 Red POS to Arduino pin 11
 Green POS to Arduino pin 10
 560 ohm resistor to both LED NEG and GRD power rail
 More info at: http://goo.gl/kJ8Gl
 Original code improvements to the Ping sketch sourced from Trollmaker.com
 Some code and wiring inspired by http://en.wikiversity.org/wiki/User:Dstaub/robotcar
 */


#include <AFMotor.h>

AF_DCMotor motor(2);
AF_DCMotor motork(1);


#define trigPin 12
#define echoPin 11



// Object avoidance distances (in inches)
#define SAFE_DISTANCE 70
#define TURN_DISTANCE 40
#define STOP_DISTANCE 12

#define TURN_LEFT 1
#define TURN_RIGHT 2
#define TURN_STRAIGHT 99

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
  Serial.begin (9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);


//turn on motor
  motor.setSpeed(200);
 
   Serial.println("Motor test!");
  motor.run(RELEASE);
  motork.run(RELEASE);
 
}

void loop() {
  long duration, distance;
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
//  delayMicroseconds(1000); - Removed this line
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;

   Serial.print(distance);
    Serial.println(" cm");
   sonarDistance = distance;
    
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
           motork.run(turnDirection);
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
          //    motork.run(turnDirection);      // alraedy turning to navigate
            }
         motor.setSpeed(speed);
         motor.run(FORWARD);
         Serial.print("\nSlowing Down & Moving Forward");       
         return;
       }
     
  /*  if (sonarDistance <  TURN_DISTANCE && sonarDistance > STOP_DISTANCE)  // getting close, time to turn to avoid object        
        {
          speed = SLOW_SPEED;
          DMotor.setSpeed(speed);      // slow down
          DMotor.run(FORWARD); 
          switch (turnDirection)
          {
            case straight:                  // going straight currently, so start new turn
              {
                if (headingError <= 0)
                  turnDirection = left;
                else
                  turnDirection = right;
                TMotor.run(turnDirection);  // turn in the new direction
                break;
              }
            case left:                         // if already turning left, try right
              {
                TMotor.run(TURN_RIGHT);    
                break;  
              }
            case right:                       // if already turning right, try left
              {
                TMotor.run(TURN_LEFT);
                break;
              }
          } // end SWITCH
          
         return;
        }*/


   if (sonarDistance <  STOP_DISTANCE)          // too close, stop and back up
       {
         motor.run(RELEASE);            // stop 
      //   motork.run(RELEASE);             // straighten up
         turnDirection = straight;
         motor.setSpeed(NORMAL_SPEED);  // go back at higher speet
         motor.run(BACKWARD);           
         
                // stop backing up
         Serial.print("\nBreaking & Moving Backward");
         return;
        } // end of IF TOO CLOSE
     
}
 
// moveAndAvoid()

