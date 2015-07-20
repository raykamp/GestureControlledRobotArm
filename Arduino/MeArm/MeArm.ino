// Written by Ray Kampmeier

#include <Servo.h> 
 
Servo servo[4];  // create servo objects to control a servo 
// Change these pins based on how you connect your arm. 
// upper arm, base, lower arm, claw
static const int servoPin[4] = {2,6,7,8};
// Change the following values to calibrate your arm
static const int minValues[4] = {100,45,60,140};
static const int maxValues[4] = {170,135,130,180};
 
void setup() 
{ 
  Serial.begin(9600);
  for(int i=0; i<4; i++){ 
    servo[i].attach(servoPin[i]); 
  }
  servo[0].write(160); // upper arm
  servo[1].write(90);  // base
  servo[2].write(110); // lower arm
  servo[3].write(160); // claw
} 
 
void loop() 
{ 
  // Servos are controlled by sending 5 ASCII characters over Serial
  // servo index 0 -> 3, servo position 000 -> 179, new line '\n' 
  if(Serial.available() >= 5) //check if there is charecter in the serial buffer
  {
    char bytes[5];
    int bytesRead = Serial.readBytesUntil('\n', bytes, 5);
    
    if(bytesRead != 4){
      return;
    }
    
    // Parse integer value
    int value = 0;
    for(int i=0; i<3; i++){
      int charValue = ((int)bytes[i+1] - '0');
      for(int j=2; j>i; j--){ charValue *= 10; }
      value += charValue;
    }
     
    uint8_t servoNum = bytes[0] - '0'; 
    value = map(value, 0, 179, minValues[servoNum], maxValues[servoNum]);     // scale it to use it with the servo (value between 0 and 180) 
    servo[servoNum].write(value);           // sets the servo position according to the scaled value 
    // waits for the servo to get there
     
  }
  delay(15);
} 
