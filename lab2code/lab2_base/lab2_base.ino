#include <sparki.h>
#include <math.h>
#define CYCLE_TIME .100  // seconds

// Program States
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_DISTANCE_MEASURE 2

// Hak
#define PI 3.1415926535897932384626433832795

#define MOVE_LEFT 1
#define MOVE_RIGHT 2
#define MOVE_FORWARD 3
#define MOVE_FORWARD_LAP 4

int current_state = CONTROLLER_FOLLOW_LINE; // Change this variable to determine which controller to run
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;

float pose_x = 0., pose_y = 0., pose_theta = 0.;

unsigned long startTime;
unsigned long endTime;
float velocity = 30.0 / 11500.0;
float angularVelocity = (2 * PI) / 9761.0;

void setup() {
  sparki.clearLCD();
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;
  Serial.begin(9600);
}

void readSensors() {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
}

void updateOdometry(int movement) {
  if (movement == MOVE_LEFT){
    pose_theta = pose_theta + (-angularVelocity * 100.0);
  }
  else if (movement == MOVE_RIGHT){
    pose_theta = pose_theta + (angularVelocity * 100.0);
  }
  else if (movement == MOVE_FORWARD){
    pose_x += 100.0 * velocity * cos(pose_theta);
    pose_y += 100.0 * velocity * sin(pose_theta);
  }
  else if (movement == MOVE_FORWARD_LAP){
    pose_x = 0.;
    pose_y = 0.;
  }

  
}

void displayOdometry() {
  sparki.clearLCD(); // erase the LCD
  sparki.print("x: "); 
  sparki.println(pose_x);
  sparki.print("y: "); 
  sparki.println(pose_y);
  sparki.print("theta: "); 
  sparki.println(pose_theta);
  sparki.updateLCD(); // put what has been drawn onto the screen
}

int followLine() {
  
  int threshold = 800;
  int moved = -1;
  
  int lineLeft   = sparki.lineLeft();   // measure the left IR sensor
  int lineCenter = sparki.lineCenter(); // measure the center IR sensor
  int lineRight  = sparki.lineRight();  // measure the right IR sensor
 
  
 if (!(lineLeft < threshold && lineCenter < threshold && lineRight < threshold)) {
   // if all sensors are seeing black then it has hit start

   if ( lineLeft < threshold ) {
     sparki.moveLeft(); // turn left
     moved = MOVE_LEFT;
   }

   if ( lineRight < threshold ) {
     sparki.moveRight(); // turn right
     moved = MOVE_RIGHT;
   }

   if ( (lineCenter < threshold) && (lineLeft > threshold) && (lineRight > threshold) ) {
     sparki.moveForward(); // move forward
     moved = MOVE_FORWARD;
   }
   return moved;
 }
 else {
   
   return MOVE_FORWARD_LAP;
 }
 
  // if the center line sensor is the only one reading a line

  
  // all three are showing


  return moved; // Return which movement was made
}

void loop() {
  int direction;
  unsigned long timing;
  // TODO: Insert loop timing/initialization code here
  unsigned long startTime;
  startTime = millis();
  unsigned long currTime;
  
  if (!startTime) {
    startTime = millis();
  }
  direction = followLine(); //follow the line and return the direction of movement
  //Odometry code?
  updateOdometry(direction);
  displayOdometry();
  if (direction == MOVE_FORWARD_LAP){
  Serial.print("Timing: ");
  Serial.println(timing);
  Serial.print("Movement: ");
  Serial.println(direction);
  Serial.print("Theta: ");
  Serial.println(pose_theta);
  Serial.print("X: ");
  Serial.println(pose_x);
  Serial.print("Y: ");
  Serial.println(pose_y);
  }
  currTime = millis();
  timing = currTime - startTime;
  delay(100 - timing);
  /*
  sparki.movestop;
  Serial.print("Timing: ");
  Serial.println(timing);
  Serial.print("Movement: ");
  Serial.println(direction);
  delay(500);
  */

}