/**
 * 5 finger control of a robotic arm 
 * by Ray Kampmeier
 *
 * Based on "Read Contacts"
 * by Aaron Zarraga - Sensel, Inc
 * 
 * This sketch opens a serial communication with the Sensel pad 
 * as well as a servo-controlled robot arm (MeArm).
 * The sketch allows you to controll all aspects of the arm with 5 finger gestures on the sensel pad
 */
import processing.serial.*;
Serial robotArm;

static final int SERVO_MAX_ANGLE = 179;
static final int SERVO_MIN_ANGLE = 0;
int savedTime;
int totalTime = 100;
boolean sensel_sensor_opened = false;

int previousThumbID = -1;
int thumbRemovalTimeout = 10;

int WINDOW_WIDTH_PX = 1150;
// We will scale the height such that we get the same aspect ratio as the sensor
int WINDOW_HEIGHT_PX;
SenselDevice sensel;

AverageValue forceVector, rotationVector, yLocation, clawAngle;

void setup() 
{ 
  forceVector = new AverageValue();
  rotationVector = new AverageValue();
  yLocation = new AverageValue();
  clawAngle = new AverageValue();
  
  println(Serial.list());
  robotArm = new Serial(this, "/dev/tty.usbmodem1421", 9600);
  DisposeHandler dh = new DisposeHandler(this);
  savedTime = millis();
  sensel = new SenselDevice(this);
  sensel_sensor_opened = sensel.openConnection();
  
  if(!sensel_sensor_opened){
    println("Unable to open Sensel sensor!");
    exit();
    return; 
  }
  
  //Init window height so that display window aspect ratio matches sensor.
  //NOTE: This must be done AFTER senselInit() is called, because senselInit() initializes
  //  the sensor height/width fields. This dependency needs to be fixed in later revisions 
  WINDOW_HEIGHT_PX = (int) (sensel.getSensorHeightMM() / sensel.getSensorWidthMM() * WINDOW_WIDTH_PX);
  
  size(WINDOW_WIDTH_PX, WINDOW_HEIGHT_PX);
  
  //Enable contact sending
  sensel.setFrameContentControl(SenselDevice.SENSEL_FRAME_CONTACTS_FLAG);
  
  //Enable scanning
  sensel.startScanning();
}

void draw() 
{
  boolean foundPreviousThumb = false; 
  int averageFingerLocation_Y = 0;
  
  if(!sensel_sensor_opened) return;
  background(0);
 
  SenselContact[] c = sensel.readContacts();
  SenselContact thumb;
   
  // Invalid number of contacts. Must be 5 fingers
  if(c == null
  || c.length != 5){
    if(thumbRemovalTimeout <= 0){
      previousThumbID = -1;
    }else{
      thumbRemovalTimeout--;
    }
    return;
  }
  println("****");
  thumbRemovalTimeout = 10;
  
  // Find preexisting thumb contact
  thumb = c[0]; // to start, thumb is the first touch
  if(previousThumbID != -1){
    for(int i = 0; i < 5; i++)
    {
      if(previousThumbID == c[i].id){
        foundPreviousThumb = true;
         thumb = c[i];
         for(int j = i+1 ; j < 5; j++)
          {
            c[j-1] = c[j];
          }
         c[4] = thumb;
      }
    }
  }
  
  // If no preexisting thumb contact is found, 
  // find the lowest contact and set it as the thumb
  if(foundPreviousThumb == false){
    //Find thumb vs fingers
    thumb = c[0]; // to start, thumb is the first touch
    for(int i = 1; i < 5; i++)
    {
      if(c[i].y_pos_mm > thumb.y_pos_mm){
        c[i-1] = thumb;
        thumb = c[i];
      }else{
        c[i-1] = c[i];
      }
    }
    c[4] = thumb; // thumb is now c[4]
    previousThumbID = thumb.id;
  }
  
  // Find average y location of all contacts
  for(int i = 1; i < 5; i++){
    averageFingerLocation_Y += c[i].y_pos_mm;
  }
  averageFingerLocation_Y /= 5;
  println("average y: "+averageFingerLocation_Y); // 20 to 75
  
  // Average finger vectors
  PVector averageFingerVect = new PVector(0,0);
  int averageFingerForce = 0;
  for(int i = 0; i < 4; i++)
  {
    PVector fingerVect = new PVector(c[i].x_pos_mm - thumb.x_pos_mm, c[i].y_pos_mm - thumb.y_pos_mm);
    averageFingerVect.add(fingerVect);
    averageFingerForce += c[i].total_force;
  }
  averageFingerVect.div(4.0);
  averageFingerForce /= 4.0;
  PVector averageFingerVect_screen = sensorVectorMappedToScreen(averageFingerVect); 
  
  // Thumb vector
  PVector thumbVect = new PVector(thumb.x_pos_mm, thumb.y_pos_mm);
  PVector thumbVect_screen = sensorVectorMappedToScreen(thumbVect);
  
  // Rotation
  float angle;
  if (averageFingerVect_screen.x == 0){
    angle = PI / 2;
  }else{
    angle = atan(averageFingerVect_screen.y / averageFingerVect_screen.x);
  }
  if(angle <= 0){ angle += PI;}
  
  clawAngle.addValue((int)clampedMap(averageFingerVect.mag(), 50, sensel.getSensorHeightMM()/1.5, SERVO_MAX_ANGLE, SERVO_MIN_ANGLE));
  rotationVector.addValue((int)clampedMap(angle, 1, PI-1, SERVO_MAX_ANGLE, SERVO_MIN_ANGLE));
  forceVector.addValue((int)clampedMap(averageFingerForce, 500, 3000, SERVO_MAX_ANGLE, SERVO_MIN_ANGLE));
  yLocation.addValue((int)clampedMap(averageFingerLocation_Y, 20, 75, SERVO_MAX_ANGLE, SERVO_MIN_ANGLE));
  
  // This code runs 10 times a second
  int passedTime = millis() - savedTime;
  if (passedTime > totalTime) { // do periodic task
    savedTime = millis(); // Save the current time to restart the timer
 
    setServoAngle(3,clawAngle.getAverageAndReset()); // claw
    setServoAngle(1,rotationVector.getAverageAndReset()); // base
    setServoAngle(0,forceVector.getAverageAndReset()); // upper arm
    setServoAngle(2,yLocation.getAverageAndReset()); // lower arm
  }
 
  // Print visualization
  stroke(255,0,0); 
  strokeWeight(4);
  line(thumbVect_screen.x, thumbVect_screen.y, 
  thumbVect_screen.x + averageFingerVect_screen.x,
  thumbVect_screen.y + averageFingerVect_screen.y);
  
  for(int i = 0; i < 5; i++){
    int screen_x = (int) (c[i].x_pos_mm / sensel.getSensorWidthMM()  * WINDOW_WIDTH_PX);
    int screen_y = (int) (c[i].y_pos_mm / sensel.getSensorHeightMM() * WINDOW_HEIGHT_PX);
    
    // Draw circles
    int size = c[i].total_force / 100;
    if(size < 20) size = 20;
    
    if(i == 4){
      fill(255,0,0); 
    }else{
      fill(255,255,255); 
    }
    strokeWeight(0);
    ellipse(screen_x, screen_y, size, size);
  }
}

PVector sensorVectorMappedToScreen(PVector vector){
  PVector screenVector = new PVector(
    vector.x / sensel.getSensorWidthMM()  * WINDOW_WIDTH_PX ,
    vector.y / sensel.getSensorHeightMM() * WINDOW_HEIGHT_PX);
  return screenVector;
}

// This is like the built-in map function, but "clampes" the returned value between start2 and stop2
float clampedMap(float value, float start1, float stop1, float start2, float stop2){
  float newValue = map(value,start1,stop1,start2,stop2);
  if(stop2 > start2){
    newValue = newValue>stop2 ? stop2 : newValue;
    newValue = newValue<start2 ? start2 : newValue;
  }else{
    newValue = newValue<stop2 ? stop2 : newValue;
    newValue = newValue>start2 ? start2 : newValue;
  }
  return newValue;
}

// Writes a serial command to set a particular servo angle
// Angle should be from 0 -> 179 
void setServoAngle(int servo, int angle){
  robotArm.write('0' + servo);
  robotArm.write('0' + angle/100);
  robotArm.write('0' + (angle%100)/10);
  robotArm.write('0' + angle%10);
  robotArm.write('\n');
}

public class DisposeHandler 
{   
  DisposeHandler(PApplet pa)
  {
    pa.registerMethod("dispose", this);
  }  
  public void dispose()
  {      
    println("Closing sketch");
    if(sensel_sensor_opened)
    {
      sensel.stopScanning();
      sensel.closeConnection();
    }
  }
}
