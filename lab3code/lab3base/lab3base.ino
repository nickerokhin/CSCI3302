//#define M_PI 3.14159
#define ROBOT_SPEED 2.75  // centimeters/second
#define CYCLE_TIME .050 // Default 50ms cycle time
#define AXLE_DIAMETER 8.57 // centimeters
#define WHEEL_RADIUS 3.0 // centimeters
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_GOTO_POSITION_PART2 2
#define CONTROLLER_GOTO_POSITION_PART3 3

#define FWD 1
#define NONE 0
#define BCK -1

#define MOVE_LEFT 1
#define MOVE_RIGHT 2
#define MOVE_FORWARD 3

// Line following configuration variables
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;

// Controller and dTheta update rule settings
int current_state = CONTROLLER_GOTO_POSITION_PART2;

// Odometry bookkeeping
float orig_dist_to_goal = 0.0;
float pose_x = 0., pose_y = 0., pose_theta = 0.;
float dest_pose_x = 40., dest_pose_y = 15., dest_pose_theta = 1.;
float d_err = 0., b_err = 0., h_err = 0.; // Distance error (m), bearing error (rad), heading error (rad)
float phi_l = 0., phi_r = 0.; // Wheel rotation (radians)

// Wheel rotation vars
float left_speed_pct = 0.;
float right_speed_pct = 0.;
int left_dir = DIR_CCW;
int right_dir = DIR_CW;
int left_wheel_rotating = NONE;
int right_wheel_rotating = NONE;

//
bool complete = false;

// X and Theta Updates (global for debug output purposes)
// and their respective feedback controller gains
const float distance_gain = 1.;
const float theta_gain = 1.;
const float heading_gain = 1.;
float dX  = 0., dTheta = 0.;

float to_radians(double deg) {
  return  deg * 3.1415/180.;
}

float to_degrees(double rad) {
  return  rad * 180 / 3.1415;
}

float angularVelocity = (2 * M_PI) / 9761.0;

void setup() {
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;
  left_wheel_rotating = NONE;
  right_wheel_rotating = NONE;

  // Set test cases here!
  set_pose_destination(0.15,0.05, to_radians(135));  // Goal_X_Meters, Goal_Y_Meters, Goal_Theta_Radians
}

// Sets target robot pose to (x,y,t) in units of meters (x,y) and radians (t)
void set_pose_destination(float x, float y, float t) {
  dest_pose_x = x;
  dest_pose_y = y;
  dest_pose_theta = t;
  if (dest_pose_theta > M_PI) dest_pose_theta -= 2*M_PI;
  if (dest_pose_theta < -M_PI) dest_pose_theta += 2*M_PI;
  orig_dist_to_goal = 0; // TODO
}

void readSensors(int movement) {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
}

void displayOdometry() {
  sparki.print("X: ");
  sparki.print(pose_x);
  sparki.print(" Xg: ");
  sparki.println(dest_pose_x);
  sparki.print("Y: ");
  sparki.print(pose_y);
  sparki.print(" Yg: ");
  sparki.println(dest_pose_y); 
  sparki.print("T: ");
  sparki.print(to_degrees(pose_theta));
  sparki.print(" Tg: ");
  sparki.println(to_degrees(dest_pose_theta));

  sparki.print("dX : ");
  sparki.print(dX );
  sparki.print("   dT: ");
  sparki.println(dTheta);
  sparki.print("phl: "); sparki.print(phi_l); sparki.print(" phr: "); sparki.println(phi_r);
  sparki.print("p: "); sparki.print(d_err); sparki.print(" a: "); sparki.println(to_degrees(b_err));
  sparki.print("h: "); sparki.println(to_degrees(h_err));  
}

void partTwoController(float desired_pose_x, float desired_pose_y, float desired_theta){
  //Assume starting at 0,0
  // Turn towards the point
  float dist = sqrt(pow((desired_pose_x - pose_x),2) + (pow(desired_pose_y - pose_y, 2)));
  float bearing = atan2((desired_pose_y - pose_y),(desired_pose_x - pose_x)) - pose_theta;
  float heading = desired_theta - bearing;
  Serial.println(bearing);
  Serial.println(dist);
  
  // Make the initial rotation 
  int timeToRotate = (int) (bearing/angularVelocity);
  Serial.println(timeToRotate);
  sparki.moveRight();
  delay(timeToRotate);
  sparki.moveStop();
  updateOdometry2(MOVE_RIGHT, timeToRotate);
  
  // Move to the point
  int timeToMoveForward = (dist/100)/ROBOT_SPEED * 1000; 
  Serial.println(timeToMoveForward);
  sparki.moveForward();
  delay(timeToMoveForward);
  sparki.moveStop();
  updateOdometry2(MOVE_FORWARD, timeToMoveForward);
  
  // Rotate towards the heading
  timeToRotate = (int) (heading/angularVelocity);
  sparki.moveRight();
  Serial.println("Time to rotate heading: ");
  Serial.print(timeToRotate);
  delay(timeToRotate);
  sparki.moveStop();
  updateOdometry2(MOVE_RIGHT, timeToRotate);

}

void updateOdometry2(int movement, float time_to_move) {
  if (movement == MOVE_LEFT){
    pose_theta = pose_theta + (-angularVelocity * time_to_move);
    Serial.println(pose_theta);
  }
  else if (movement == MOVE_RIGHT){
    pose_theta = pose_theta + (angularVelocity * time_to_move);
  }
  else if (movement == MOVE_FORWARD){
    pose_x += time_to_move * velocity * cos(pose_theta);
    pose_y += time_to_move * velocity * sin(pose_theta);
  }
  Serial.print("Theta: ");
  Serial.println(pose_theta);
  Serial.print("X: ");
  Serial.println(pose_x);
  Serial.print("Y: ");
  Serial.println(pose_y);
}

void updateOdometry3(int percent_l, int percent_r) {
  
  int phi_l = ((ROBOT_SPEED * percent_l) * AXLE_DIAMETER) / WHEEL_RADIUS;
  int phi_r = ((ROBOT_SPEED * percent_r) * AXLE_DIAMETER) / WHEEL_RADIUS; 
  
  // Recalculate pose theta
  int d_theta = (phi_r * WHEEL_RADIUS / AXLE_DIAMETER) - (phi_l * WHEEL_RADIUS / AXLE_DIAMETER);
  pose_theta += d_theta;
  
  // Calculate X_R
  int X_R = (WHEEL_RADIUS * phi_l / 2) + (WHEEL_RADIUS * phi_r / 2);
  
  // Recalculate pose_x and pose_y
  pose_x += X_R * cos(pose_theta);
  pose_y += X_R * sin(pose_theta);
  
  // Leave for debugging
  Serial.print("Theta: ");
  Serial.println(pose_theta);
  Serial.print("X: ");
  Serial.println(pose_x);
  Serial.print("Y: ");
  Serial.println(pose_y); 
}

void gains(){
  
  float p1 - 0.1;
  float p2 = 0.1;
  float p3 = 1/d_err;
  if p3 > 1{
    p3 = 1
  }
  
  
  
}

void partThreeController(float desired_pose_x, float desired_pose_y, float desired_theta){
  //Assume starting at 0,0
	unsigned long timing;
  unsigned long startTime;
  startTime = millis();
  unsigned long currTime;

  // Turn towards the point
  d_err = sqrt(pow((desired_pose_x - pose_x),2) + (pow(desired_pose_y - pose_y, 2)));
 	b_err = bearing = atan2((desired_pose_y - pose_y),(desired_pose_x - pose_x)) - pose_theta;
  if (!startTime) {
    startTime = millis();
  }
  updateOdometry3(left_speed_pct, righ_speed_pct);
  displayOdometry();
  gains()
        
  sparki.motorRotate(MOTOR_LEFT, left_dir, 100);
  sparki.motorRotate(MOTOR_RIGHT, right_dir, 100);
  currTime = millis();
	timing = currTime - startTime;
  delay(100 - timing);
  
  // Drive the distance to the point
  
  // Turn to desired theta
}


void loop() {
  unsigned long begin_time = millis();
  unsigned long end_time = 0;
  unsigned long delay_time = 0;
  
  int moved = -1;
   
  switch (current_state) {
    case CONTROLLER_FOLLOW_LINE:
      // Useful for testing odometry updates
      //readSensors();
      if (line_center < threshold) {
        // TODO: Fill in odometry code
        moved = MOVE_LEFT;
        sparki.moveForward();
      } else if (line_left < threshold) {
        // TODO: Fill in odometry code
        moved = MOVE_LEFT;
        sparki.moveLeft();
      } else if (line_right < threshold) {
        // TODO: Fill in odometry code
        sparki.moveRight();
        moved = MOVE_RIGHT;
      } else {
        sparki.moveStop();
      }

      // Check for start line, use as loop closure
      if (line_left < threshold && line_right < threshold && line_center < threshold) {
        pose_x = 0.;
        pose_y = 0.;
        pose_theta = 0.;
      } 
      break;
    case CONTROLLER_GOTO_POSITION_PART2:
      partTwoController(dest_pose_x,dest_pose_y,dest_pose_theta);
      
      // TODO: Implement solution using moveLeft, moveForward, moveRight functions
      // This case should arrest control of the program's control flow (taking as long as it needs to, ignoring the 100ms loop time)
      // and move the robot to its final destination
        
      break;      
    case CONTROLLER_GOTO_POSITION_PART3:
      
      //partThreeController(dest_pose_x, dest_pose_y, dest_pose_theta);
      
      //updateOdometry();
      // TODO: Implement solution using motorRotate and proportional feedback controller.
      // sparki.motorRotate function calls for reference:
      //      sparki.motorRotate(MOTOR_LEFT, left_dir, int(left_speed_pct*100));
      //      sparki.motorRotate(MOTOR_RIGHT, right_dir, int(right_speed_pct*100));

      break;
  }

  sparki.clearLCD();
  displayOdometry();
  sparki.updateLCD();

  end_time = millis();
  delay_time = end_time - begin_time;
  delay(100000);
  /*
  if (delay_time < 1000*CYCLE_TIME)
    delay(1000*CYCLE_TIME - delay_time); // each loop takes CYCLE_TIME ms
  else
    delay(10);
  */
}