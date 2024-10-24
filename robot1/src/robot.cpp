/**
 * 
 * has all of the functionality 
 * to make Beanas find and send AprilTag information
 * includes: checkers, handlers, state machine,
 * starters, stoppers, and other functions
 * 
 **/

#include <robot.h>
#include <Chassis.h>
#include <HC-SR04.h>
#include <ir_codes.h>
#include <LSM6.h>
#include <openmv.h>
#include <apriltagdatum.h>
#include <headings.h>

HC_SR04 hc_sr04(17, 4);
void ISR_HC_SR04(void)
{
    hc_sr04.ISR_echo();
}

/* variables */

Chassis chassis;
OpenMV camera;
HeadingsDatum robotHeadings; // romi's i, j, direction

ROBOT_STATE robotState; // states of the romi
int turnState; // turn states : RIGHT, LEFT, or UTURN

float objectDistance;

/* basic functions */

String keyCodeString; // his may come in handy later
void handleKeyCode(int16_t keyCode)
{ 
  if(keyCode == ENTER_SAVE) idle();

  switch(keyCode)
  {
    case PLAY_PAUSE: // start Beanas Ramp Navigation
      turnState = RIGHT;

      startTurn(turnState);
      startTimer();
      robotState = ROBOT_TURNING;
      break;
  }
}

void idle(void) // stops romi
{
    Serial.println("Idling!");
    chassis.setMotorEfforts(0, 0);
    robotState = ROBOT_IDLE;
}

void initialize(void) // initializes in main.cpp
{
    chassis.init();

      Serial1.begin(115200);

    hc_sr04.init(ISR_HC_SR04);
    hc_sr04.getDistance(objectDistance);

    pinMode(A3, INPUT);
    pinMode(A4, INPUT);
}

void sendMessage(const String& topic, const String& message) // formats messages to MQTT
{
    Serial1.println(topic + String(':') + message);
}

/* start functions */

unsigned long timerStarts = 0;

void startMotor() { // starts motors forward for centering
  chassis.setWheelTargetSpeeds(7, 7);
}

void startTimer() { // sets initial time for checkTimerExpires
  timerStarts = millis();
}

void startTurn(int turnType) { // starts motors for turning, either RIGHT, LEFT, or UTURN
  int baseSpeed = 5;
  int turnSpeed = baseSpeed - (2 * baseSpeed * (turnType % 2));
    chassis.setWheelTargetSpeeds(turnSpeed, -turnSpeed);
}

/* stop functions */

void stopMotors() { // turns off motors
  chassis.setMotorEfforts(0,0);
}

/* check functions */

bool prevCheckObject;
bool objectInFront;
bool exploredSecondColumn = false;
bool exploreSecondColumn = true;
bool wallInSecondColumn = false;
bool foundAprilTag = false;

bool checkAprilTag(AprilTagDatum& tag) { // checks for new AprilTag reading from camera
  uint8_t tagCount = camera.getTagCount();
  if(tagCount) { // if there is an AprilTag found
    return camera.readTag(tag);
  }
  return false;
}

bool checkIntersection() { // checks if both line sensors are reading less than white line threshold
  return (analogRead(A3) < 500) && (analogRead(A4) < 500);
}

bool checkObject(float tooCloseDistance) { // checks if the distance reading from ultrasonic sensor is less than inputted value
  if (hc_sr04.getDistance(objectDistance)) 
  {
    prevCheckObject = objectDistance < tooCloseDistance;
    return objectDistance < tooCloseDistance;
  }
  return prevCheckObject;
}

bool checkPitchAtBottom() { // check that romi is going up the ramp
  return chassis.updatePitch() < -8;
}

bool checkPitchAtTop() { // check that romi is at the top of the ramp and the platform flips
  return chassis.updatePitch() > -1;
}

bool checkSecondColumnWall() { // check if the romi has explored the second / i=1 column
  return !exploredSecondColumn && robotHeadings.i == 1;
}

bool checkTimerExpires(unsigned long timerDelay) { // checks if millis() time is larger than initial time + inputted value
  return (timerStarts + timerDelay) < millis();
}

/* handle functions */

void handleAprilTag(AprilTagDatum tag) { // handles what to do with the information from the AprilTag
  sendToMQTT(tag);
  startTimer();
  foundAprilTag = true;
  robotState = ROBOT_READ_APRILTAG;
}

void handleIntersection() { // starts motors and timer after seeing an intersection
  startMotor();
  startTimer();
  robotState = ROBOT_CENTERING;
}

void handleObjectInFront(bool objectInFront) { // handles what to do in centering based on if there is a wall in front or not
  updatePosition();
  if (objectInFront) { // object too close
    if (checkSecondColumnWall()) { // if there is a wall and they are in the second column
      turnState = UTURN;
      exploredSecondColumn = true;
    }
    startTurn(turnState);
    robotState = ROBOT_TURNING;
  }
  else { // no object
    robotState = ROBOT_DRIVING;
  }
}

void handleReachedRamp() { // handles what to do when romi gets on the ramp
  startTimer();
  robotState = ROBOT_UP_RAMP;
}

void handleReachedTop() { // handles what to do when the romi gets to the top of the ramp
  stopMotors();
  robotState = ROBOT_IDLE;
}

void handleTimerExpires() { // handles when timer expires after reading AprilTag
  robotState = ROBOT_IDLE;
}

void handleTurnDone() { // handles logic after turn finishes for updating direction
  robotState = ROBOT_DRIVING;
  switch (turnState) {
    case RIGHT: // after a right turn, go left
      turnState = LEFT;
      break;
    case LEFT: // logic when in left turns
      if (wallInSecondColumn) // if there is a wall blocking off second column, turn right to go to third column
      {
        turnState = RIGHT;
        exploredSecondColumn = false;
      }
      break;
    case UTURN: // after a uturn, go back to turning left only
      turnState = LEFT;
      break;
  }
  updateDirection();
  startTimer();
}

/* update functions */

int deltaX = 0;
int deltaY = 1;

void updateDirection() { // updates the romis heading and prints to MQTT
  robotHeadings.direction = (turnState == RIGHT) ? robotHeadings.direction += 1 : robotHeadings.direction; // sets new heading after RIGHT turn
  robotHeadings.direction = (turnState == LEFT) ? robotHeadings.direction -= 1 : robotHeadings.direction; // sets new heading after LEFT turn
  robotHeadings.direction = (turnState == UTURN) ? robotHeadings.direction += 2 : robotHeadings.direction; // sets new heading after UTURN turn
  robotHeadings.direction = (robotHeadings.direction + 4) % 4;
  
  switch (robotHeadings.direction) { // sets the change in X and Y for updating i and j
    case NORTH:
      deltaX = 0;
      deltaY = 1;
      break;
    case EAST:
      deltaX = 1;
      deltaY = 0;
      break;
    case SOUTH:
      deltaX = 0;
      deltaY = -1;
      break;
    case WEST:
      deltaX = -1;
      deltaY = 0;
      break;                 
  }

  Serial1.println("Beanas/Heading:" + String(robotHeadings.direction));
}

void updatePosition() { // updates the I and J values of romi
  robotHeadings.i += deltaX;
  robotHeadings.j += deltaY;
  sendMessage("Beanas/Location", "(" + String(robotHeadings.i) + ", " + String(robotHeadings.j) + ")");
}

/* other functions */

void lineFollow() { // makes romi line follow using proportional control
  float baseSpeed = 10;
  float lineFollowKp = 0.025;
  float error = lineFollowKp * (analogRead(A4) - analogRead(A3));
  chassis.setWheelTargetSpeeds(baseSpeed + error, baseSpeed - error);
}

void sendToMQTT(AprilTagDatum tag) { // sends AprilTag information to MQTT
  Serial1.println("AprilTag/Info:" + String(tag.id) + "," + String(tag.rot));
}


/* state machines */

AprilTagDatum tag;
unsigned long printTimer = 0;
String stateIn = "IDLE";

void beanas() { // main state machine
  // just a printer for romi state, on a clock so it doesn't overfill MQTT
  if (millis() > printTimer + 1000) {
    Serial1.println("Beanas/State:" + stateIn);
    printTimer = millis();
  }

  switch (robotState) {
    case ROBOT_IDLE: // idle state
      stateIn = "IDLE";

      if (checkAprilTag(tag)) {
        handleAprilTag(tag); // what happens after camera sees an AprilTag
      }
      break;
    case ROBOT_DRIVING: // driving/linefollowing state
      stateIn = "DRIVING";

      lineFollow();
      if (checkIntersection()) {
        handleIntersection(); // what happens when both line sensors see the white line
      }
      if (checkPitchAtBottom() && checkTimerExpires(1000)) {
        handleReachedRamp(); // what happens after romi starts going up the ramp
      }
      break;
    case ROBOT_TURNING: // turning state
      stateIn = "TURNING";

      // timer based turning
      switch(turnState) {
        case RIGHT: // what to do during RIGHT turn
          if (checkTimerExpires(2450)) { 
            handleTurnDone(); // handles what to do after the turn
          }
          break;
        case LEFT: // what to do during LEFT turn
          if (exploreSecondColumn) {
            objectInFront = checkObject(20);
          }
        
          if (checkTimerExpires(2450)) {
            if (exploreSecondColumn) { // if needs to explore the second column
              if (objectInFront) { // if there is a wall immediately after turning
                startTimer();
                startTurn(RIGHT);
              }
              exploreSecondColumn = false;
              prevCheckObject = false;
              objectInFront = false;
            } 
            handleTurnDone(); // handles what to do after the turn
          }
          break;
        case UTURN: // what to do during UTURN turn
          if (checkTimerExpires(4900)) {
            handleTurnDone(); // handles what to do after the turn
          }
          break;
      }
      break;
    case ROBOT_CENTERING: // centering state
      stateIn = "CENTERING";

      objectInFront = checkObject(20);
      if (checkTimerExpires(1100)) { // timer based centering
        handleObjectInFront(objectInFront); // normal maze navigation when there is wall in front
        if (exploreSecondColumn) { // if needs to explore the second column for ramp
          startTurn(LEFT);
          robotState = ROBOT_TURNING;
          exploreSecondColumn = false;
        }
        startTimer();
        prevCheckObject = false;
        objectInFront = false;
      }
      
      break;
    case ROBOT_UP_RAMP: // going up the ramp state
      stateIn = "UP_RAMP";

      lineFollow();
      if (checkPitchAtTop() && checkTimerExpires(1000)) {
        handleReachedTop(); // handle what to do after reaching the top of the ramp
      }
      break;
    case ROBOT_READ_APRILTAG: // reading AprilTag state
      stateIn = "TAG_READING";

      if (checkTimerExpires(2000)) {
        handleTimerExpires(); // go back to IDLE after certain amount of time
      }
      break;
    default:
      break;
  }

}
