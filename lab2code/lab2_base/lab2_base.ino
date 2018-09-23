#include <sparki.h>

#define CYCLE_TIME .100  // seconds

// Program States
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_DISTANCE_MEASURE 2


int current_state = CONTROLLER_FOLLOW_LINE; // Change this variable to determine which controller to run
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;

float pose_x = 0., pose_y = 0., pose_theta = 0.;

void setup() {
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;
}

bool followLine() {
 int threshold = 700;
 int lineLeft   = sparki.lineLeft();
 int lineCenter = sparki.lineCenter();
 int lineRight  = sparki.lineRight();

 if (!(lineLeft < threshold && lineCenter < threshold && lineRight < threshold)) {
   // if all sensors are seeing black then it has hit start

   if ( (lineCenter < threshold) && (lineLeft > threshold) && (lineRight > threshold) ) {
     sparki.moveForward(); // move forward
     Serial.println("Moving forward");
     return false;
   }

 }
 else {
   sparki.moveStop();
   return true;

 }
}



void measureRotation(){
  unsigned int rotStart;

  sparki.moveRight(360);
  rotStart = millis();
  Serial.println(rotStart);
}


//11315
//11332
//11323
//2.6494 cm/s
//360 degrees in 9761ms

void readSensors() {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
  // distance = sparki.ping();
}

void measure_30cm_speed() {
  bool stopped = false;
  unsigned long start;
  start = millis();

  while (!stopped){
    stopped = followLine();
  }
  
  Serial.println(millis() - start);

  
  /*
   *100ms implementation
  while (!stopped){
    unsigned long start;
    unsigned long endT;
    start = millis();
    followLine();
    delay(88);
    endT = millis();
    Serial.println(endT - start);
  }

  */
}


void updateOdometry() {
  // TODO
}

void displayOdometry() {
  // TODO
}

void loop() {

  // TODO: Insert loop timing/initialization code here
  
  measureRotation();
}
