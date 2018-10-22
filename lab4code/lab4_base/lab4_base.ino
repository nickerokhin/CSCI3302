
// UPDATED 10/10
#include <sparki.h>


#define ROBOT_SPEED 0.0278
#define MIN_CYCLE_TIME .100
#define AXLE_DIAMETER 0.0865
#define FWD 1
#define NONE 0
#define BCK -1

#define MAX 2**15 - 1

// Screen size
#define SCREEN_X_RES 128.
#define SCREEN_Y_RES 64.

// Map size
#define NUM_X_CELLS 4
#define NUM_Y_CELLS 4

// Start line is 18", 2" from bottom left corner
#define START_LINE_X .4572 
#define START_LINE_Y .0508 

#define SERVO_POS_DEG 45

int current_state = 1;
const int threshold = 800;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;
float distance = 0.;
unsigned long last_cycle_time = 0;

float pose_x = 0., pose_y = 0., pose_theta = 0., pose_servo = 0.;
int left_wheel_rotating = 0, right_wheel_rotating = 0;

// TODO: Define world_map multi-dimensional array
short world_map[4][6];

// TODO: Figure out how many meters of space are in each grid cell
const float CELL_RESOLUTION_X = 0;  // Line following map is ~60cm x ~42cm
const float CELL_RESOLUTION_Y = 0; // Line following map is ~60cm x ~42cm

// Define object position
float obj_x;
float obj_y;
float *object_x = &obj_x;
float *object_y = &obj_y;

bool objectFound = false;

void setup() {
  pose_x = START_LINE_X;
  pose_y = START_LINE_Y;
  pose_theta = 0.;
  left_wheel_rotating = NONE;
  right_wheel_rotating = NONE;
  pose_servo = to_radians(SERVO_POS_DEG);
  
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 4; j++) {
      world_map[j][i] = 0;
    }
  }

  sparki.servo(-to_degrees(pose_servo)); // Servo axis points down instead of up, so we need to negate it to be consistent with the robot's rotation axis

  // TODO: Initialize world_map

  sparki.clearLCD();
  displayMap();
  delay(1000);  
  last_cycle_time = millis();
}

float to_radians(float deg) {
  return deg * 3.14159 / 180.;
}

float to_degrees(float rad) {
  return rad * 180. / 3.14159;
}

int checkForObject(){
  int ping_cm = sparki.ping();
  float cm = ((float) ping_cm)/100.;
  //Serial.print("m: ");
  //Serial.println(cm);

    transform_us_to_robot_coords(cm, pose_servo, object_x, object_y);
    float temp_x = *object_x;
    float temp_y = *object_y;
  
    //Serial.print("temp_x: ");
    //Serial.println(temp_x);
    //Serial.print("temp_y: ");
    //Serial.println(temp_y);

    transform_robot_to_world_coords(temp_x, temp_y, object_x, object_y);


    if (*object_x < 0.60 && *object_y < 0.42 && *object_x > 0 && *object_y > 0){
    Serial.print("\nObject x: ");
    Serial.print(*object_x);
    Serial.print("\nObject y: ");
    Serial.println(*object_y);
    int obj_y_idx = (int) (*object_y * 100)/10.5;
    int obj_x_idx = (int) (*object_x * 100)/10;
  
    Serial.print("x_idx: ");
    Serial.print(obj_x_idx);
    Serial.print("y_idx: ");
    Serial.print(obj_y_idx);
    
    //obj_y_idx -= 1;
    //obj_x_idx -= 1;
    //if (obj_y_idx > 0 || obj_y_idx > 0){
      
    
    world_map[obj_y_idx][obj_x_idx] += 1;
    //}
    }
    //Serial.print("Map: \n");
    //Serial.print(world_map);
}

// Ultrasonic Sensor Readings -> Robot coordinates
void transform_us_to_robot_coords(float dist, float theta, float *rx, float *ry) {
  *rx = dist * cos(theta);
  *ry = dist * sin(theta);
  //Serial.println(*rx);
  //Serial.println(*ry);
}

// Robot coordinates -> World frame coordinates
void transform_robot_to_world_coords(float x, float y, float *gx, float *gy) {
  /*
  rotation_matrix = [[cos(pose_theta), -sin(pose_theta), 0],
                    [sin(pose_theta), cos(pose_theta), 0],
                    [0, 0, 1]];
  object_vector = [rx, 
                  ry];
  sparki_vector = [pose_x,
                  pose_y];
  
  for(i=0; i<2; i++){
    for(j=0; j<2; j++){
      
    }
  }
  
  world_vector = rotation_matrix * sparki_vector + object_vector
  */
  
  *gx = (x * cos(pose_theta) - y * sin(pose_theta)) + pose_x;
  *gy = (y * cos(pose_theta) + x * sin(pose_theta)) + pose_y;
}

bool transform_xy_to_grid_coords(float x, float y, int *i, int *j) {
  
  // TODO: Set *i and *j to their corresponding grid coords  
  *i = x * 10;
  *j = y * 10.5;

  // TODO: Return 0 if the X,Y coordinates were out of bounds
  if (*i > 60) {
    return 0;
  } 
  if (*j > 42) {
    return 0; 
  }

  return 1;
}

// Turns grid coordinates into world coordinates (grid centers)
bool transform_grid_coords_to_xy(float i, float j, float *x, float *y) {
  
  // TODO: Return 0 if the I,J coordinates were out of bounds
  if (i > 60) {
    return 0;
  }
  if (j > 42) {
    return 0;
  }

  // TODO: Set *x and *y
  *x = i / 10;
  *y = j / 10.5;
  
  return 1;
}

void readSensors() {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
  distance = float(sparki.ping()) / 100.;
}

void moveRight() {
  left_wheel_rotating = FWD;
  right_wheel_rotating = BCK;
  sparki.moveRight();
}

void moveLeft() {
  left_wheel_rotating = BCK;
  right_wheel_rotating = FWD;
  sparki.moveLeft();
}

void moveForward() {
  left_wheel_rotating = FWD;
  right_wheel_rotating = FWD;
  sparki.moveForward();
}

void moveBackward() {
  left_wheel_rotating = BCK;
  right_wheel_rotating = BCK;
  sparki.moveBackward();
}

void moveStop() {
  left_wheel_rotating = NONE;
  right_wheel_rotating = NONE;
  sparki.moveStop();
}

void updateOdometry(float cycle_time) {
  pose_x += cos(pose_theta) * cycle_time * ROBOT_SPEED * (left_wheel_rotating + right_wheel_rotating)/2.;
  pose_y += sin(pose_theta) * cycle_time * ROBOT_SPEED * (left_wheel_rotating + right_wheel_rotating)/2.;
  pose_theta += (right_wheel_rotating - left_wheel_rotating) * cycle_time * ROBOT_SPEED / AXLE_DIAMETER;
}

void displayMap() {
  // TODO: Measure how many pixels will be taken by each grid cell
  const int PIXELS_PER_X_CELL = 0;
  const int PIXELS_PER_Y_CELL = 0; 
  int cur_cell_x=-1, cur_cell_y=-1;
  //Loop for grid rows

  float new_coords_x = (pose_x * 100) * (128/60);
  float new_coords_y = 64. - (pose_y * 100. * (64./42.));
  sparki.clearLCD();
  /*
  sparki.print("new_coords_x: ");
  sparki.println(new_coords_x);
  sparki.print("new_coords_y: ");
  sparki.println(new_coords_y);
  sparki.updateLCD();
  */
  
  /*
  while (y < 64){
    while (x < 128){

      if (abs(x - new_coords_x) < 10.53 && abs(y - new_coords_y) < 8){
        sparki.drawCircleFilled(x,y,8);
        
      }
      else{
      sparki.drawCircle(x, y, 8);
      }

      
    }
    
  }
  */
  int y = 8;
  int x = 10;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 6; j++) {
      
      if (world_map[i][j] >= 7){
        
        sparki.drawCircleFilled(x, y, 8);
        //matrix_2_map_x = (4/14) * i;
        //matrix_2_map_y = (6/20) * j;
      }
      else{
        sparki.drawCircle(x,y,8);
      }
      x+=21;
    }
    x = 10;
    y += 16;
  }

  /*
  for(y = 10; y < 64; y += 15){
    for(int x = 10; x < 128; x += 20){
      Serial.print("x: ");
      Serial.println(x);
      Serial.print("y: ");
      Serial.println(y);
      sparki.drawCircle(x, y, 8);
    }
  }
  */

  // TODO: Make sure that if the robot is "off-grid", e.g., at a negative grid position or somewhere outside your grid's max x or y position that you don't try to plot the robot's position!
  
  // TODO: Draw Map
}

void serialPrintOdometry() {
  Serial.print("\n\n\nPose: ");
  Serial.print("\nX: ");
  Serial.print(pose_x);
  Serial.print("\nY: ");
  Serial.print(pose_y);
  Serial.print("\nT: ");
  Serial.print(pose_theta * 180. / 3.14159);
  Serial.print("\n");
}

void displayOdometry() {
  sparki.println("Pose: ");
  sparki.print("X: ");
  sparki.println(pose_x);
  sparki.print("Y: ");
  sparki.println(pose_y);
  sparki.print("T: ");
  sparki.println(pose_theta * 180. / 3.14159);
}

void loop() {
  unsigned long begin_time = millis();
  unsigned long begin_movement_time = 0;
  unsigned long end_time = 0;
  unsigned long delay_time = 0;
  float elapsed_time = 0.;
  bool found_object = 0;
  readSensors();
  
  sparki.clearLCD();
  last_cycle_time = (millis() - last_cycle_time);
  elapsed_time = last_cycle_time / 1000.;
  updateOdometry(elapsed_time);
  last_cycle_time = millis(); // Start timer for last motor command to determine cycle time
  serialPrintOdometry();

  // Mapping Code
  sparki.servo(-to_degrees(pose_servo));

  // TODO: Check if sensors found an object
  checkForObject();
  

  // TODO: Adjust Map to accommodate new object


  if (line_center < threshold) {
    moveForward();
  } else if (line_left < threshold) {
    moveLeft();
  } else if (line_right < threshold) {
    moveRight();
  } else {
    moveStop();
  }

  
  // Check for start line, use as loop closure
  // NOTE: Assumes robot is moving counter-clockwise around the map (by setting pose_theta = 0)!
  //       If your robot is moving clockwise, set pose_theta to pi radians (i.e., pointing left).
  if (line_left < threshold && line_right < threshold && line_center < threshold) {
    pose_x = START_LINE_X;
    pose_y = START_LINE_Y;
    pose_theta = 0.;
  } 
  displayMap();
  sparki.updateLCD();
  end_time = millis();
  delay_time = end_time - begin_time;
  
  if (delay_time < 1000*MIN_CYCLE_TIME)
    delay(10000*MIN_CYCLE_TIME - delay_time); // make sure each loop takes at least MIN_CYCLE_TIME ms
  else
    delay(10);
    
    //delay(3000);
}





/*
 * Part 3.2 Functions
 */
int cell_to_int(int i, int j) {
  // j will never be greater than 60
  return i * 60 + j;
}

struct Point {
    int i;
    int j;
};

struct Point int_to_cell(int cell) {
  int j = cell % 60;
  int i = (cell-j)/60;
  Point p = {i,j};
  return p;
}

int dist(int start, int dest) {
  struct Point start_p = int_to_cell(start);
  struct Point dest_p = int_to_cell(dest);
  return abs(start_p.i - dest_p.i) + abs(start_p.j - dest_p.j);
}
