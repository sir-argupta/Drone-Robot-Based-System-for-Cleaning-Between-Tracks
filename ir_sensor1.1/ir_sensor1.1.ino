const int analogInPin = A0;  // Analog input pin that the receiver is attached to
int sensorValue = 0;        // value read from the receiver

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  //initialize the indicator LED:
  pinMode(13, OUTPUT);
}

void loop() {
  // read the analog in value:
  sensorValue = analogRead(analogInPin);

  // print the results to the serial monitor:
  Serial.print("\nsensor = ");
  Serial.print(sensorValue);
  //the threshold found fron analog In Out program was when object is detected, the sensor value is below 100
  //set the threshold whihc you get
  //the threshold varies for different sets of emitter-receiver pairs
  if(sensorValue < 100){ //checks if object is there or not
    digitalWrite(13, HIGH);
    Serial.print("\nObject Detected");
    }
  else{
    digitalWrite(13, LOW);
    Serial.print("\nNo object in Front");
    }
  delay(500);
}


