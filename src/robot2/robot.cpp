/**
 * 
 * has all of the functionality 
 * to make Pallas escape
 * includes: checkers, handlers, state machine,
 * starters, stoppers, and other functions
 * 
 **/

#include <robot.h>
#include <Chassis.h>
#include <HC-SR04.h>
#include <ir_codes.h>
#include <headings.h>
#include <LSM6.h>

HC_SR04 hc_sr04(17, 4);
void ISR_HC_SR04(void)
{
    hc_sr04.ISR_echo();
}

/* variables */

Chassis chassis;
HeadingsDatum robotHeadings; // romi's i, j, direction
HeadingsDatum doorHeadings; // doors's i, j, direction

ROBOT_STATE robotState; // states of the romi
int turnState; // turn states : RIGHT, LEFT, or UTURN
bool clockwise; // which way romi needs to move to escape, either clockwise or counterclockwise
bool ESCAPEDFUCKYEAH;

float objectDistance;
int cycle; // amount of times ir had to emit to open door

/* basic functions */

String keyCodeString; // his may come in handy later
void handleKeyCode(int16_t keyCode)
{ 
  if(keyCode == ENTER_SAVE) idle();

  switch(keyCode)
  {
    case PLAY_PAUSE: // start Pallas Door Navigation
      turnState = RIGHT;
      clockwise = doorHeadings.direction % 3; // calculate to move clockwise or counterclockwise

      if (clockwise) { // if clockwise
        robotState = ROBOT_DRIVING;
      } else { // if counterclockwise
        startTimer();
        startTurn(turnState);
        robotState = ROBOT_TURNING;
      }
      break;
  }
}

void idle(void) // stops romi
{
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
    pinMode(11, OUTPUT);
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

String SerialOneReading; // string from MQTT
String AprilTagSearchString; // string to parse for AprilTag
String CodeSearchString; // string to parse for Code for Door
bool prevCheckObject; // if seen object in loop, don't replace with false readings
bool foundDoor; // flag that romi is at door
bool foundDoorI; // flag that romi is at the door I coordinate
bool foundDoorJ; // flag that romi is at the door J coordinate

bool checkAprilTagFound() { // checks if incoming MQTT message is AprilTag info
  bool foundAprilTag = AprilTagSearchString.startsWith("AprilTag/Info:");
  if (!foundAprilTag) {SerialOneReading = "";};
  return foundAprilTag;
}

bool checkCode() { // checks if incoming MQTT message is the code for door
  bool foundCode = CodeSearchString.startsWith("Code/CodeBase:");
  if (!foundCode) {SerialOneReading = "";}
  return foundCode;
}

bool checkCoordinateI() { // checks if romi I coordinate is at door I coordinate
  return robotHeadings.i == doorHeadings.i && !foundDoor && !foundDoorI;
}

bool checkCoordinateJ() { // checks if romiJ coordinate is at door J coordinate
  return robotHeadings.j == doorHeadings.j && !foundDoor && !foundDoorJ;
}

bool checkIntersection() { // checks if both line sensors are reading less than white line threshold
  return (analogRead(A3) < 200) && (analogRead(A4) < 200);
}

bool checkObject(float tooCloseDistance) { // checks if the distance reading from ultrasonic sensor is less than inputted value
  if (hc_sr04.getDistance(objectDistance)) 
  {
    prevCheckObject = objectDistance < tooCloseDistance;
    return objectDistance < tooCloseDistance;
  }
  return prevCheckObject;
}

bool checkSerialOne() { // checks if there is a new MQTT message
  if(Serial1.available())
  {
      char c = Serial1.read();
      if(c == '\n') return true;
      else if (c != '\r') SerialOneReading += c;
  }
  AprilTagSearchString = SerialOneReading; // string for AprilTag parsing
  CodeSearchString = SerialOneReading; // string for Code for Door parsing
  return false;
}

bool checkTimerExpires(unsigned long timerDelay) { // checks if millis() time is larger than initial time + inputted value
  return (timerStarts + timerDelay) < millis();
}

/* handle functions */

uint32_t doorCode = 0;
bool objectInFront;

void handleAprilTagFound() { // parses AprilTag information from MQTT string
  //example of MQTT message: "AprilTag/Info:14,180";
  int commaAt = AprilTagSearchString.indexOf(",");

  // parsing for the values of the AprilTag info
  String idString = AprilTagSearchString.substring(14, commaAt);
  String rotString = AprilTagSearchString.substring(commaAt + 1, AprilTagSearchString.length());

  // turning the strings to ints
  int id = idString.toInt(); 
  int rot = rotString.toInt();

  doorHeadings.i = id / 10; // setting the value for the door I coordinate
  doorHeadings.j = id % 10; // setting the value for the door J coordinate
  doorHeadings.direction = round(rot / 90.0) % 4; // setting the value for the door direction

  // clear MQTT string reading
  AprilTagSearchString = "";
  SerialOneReading = "";
}

void handleCheckDoor() { // handles if door has opened or not
  if (objectInFront) { // if door is closed
    startTimer();
    robotState = ROBOT_EMIT_IR;
    cycle++;
  } else { // if door is open
    ESCAPEDFUCKYEAH = true;
    robotState = ROBOT_DRIVING;
    Serial1.println("AmountOfTimesChecked:" + String(cycle));
  }
}

void handleCode() { // parses Code for Door information from MQTT string
  //example of MQTT message: "Code/CodeBase:242"
  String codeString = CodeSearchString.substring(CodeSearchString.length() - 3, CodeSearchString.length()); // parse Code for Door
  int codeBase = codeString.toInt(); // cast to int
  uint32_t code = 0;

  // calculate the code
  uint8_t codePartOne = 48 + (codeBase / 100); // num 1 calculation
  code = codePartOne;
  uint8_t codePartTwo = 48 + ((codeBase % 100) / 10); // num 2 calculation
  code = (code << 8) | codePartTwo;
  uint8_t codePartThree = 48 + (codeBase % 10); // num 3 calculation
  code = (code << 8) | codePartThree;
  uint8_t codePartFour = codePartOne ^ codePartTwo ^ codePartThree; // num 4 calculation
  code = (code << 8) | codePartFour;

  // set doorCode in romi to calculated code
  doorCode = code;
  Serial1.println("Code/Code:" + String(code));

  // clear MQTT string reading
  SerialOneReading = "";
  CodeSearchString = "";
}

void handleCoordinateI() { // handles what happens when romi reaches door I
  if (clockwise) { // if driving clockwise, finding I coordinate is the second step, right before finding door
    findDoor();
  } else { // else, turn since finding I coordinate happens first in counterclockwise navigation
    startTimer();
    startTurn(turnState);
    robotState = ROBOT_TURNING;
    foundDoorI = true;
  }
}

void handleCoordinateJ() { // handles what happens when romi reaches door J
  if (clockwise) { // else, turn since finding J coordinate happens first in clockwise navigation
    startTimer();
    startTurn(turnState);
    robotState = ROBOT_TURNING;
    foundDoorJ = true;
  } else { // if driving counterclockwise, finding J coordinate is the second step, right before finding door
    findDoor();
  }
}

void handleEmitIR() { // emits Code for Door and goes to check door state
  emitIR();
  startTimer();
  robotState = ROBOT_CHECK_DOOR;
}

void handleEscapedFuckYeah() { // makes romi stop inside of excape box and sends MQTT message that it finished
  stopMotors();
  sendMessage("Pallas/Escaped","1");
  robotState = ROBOT_IDLE;
}

void handleGoToEMIT() { // makes romi stop in front of door and start emitting code
  stopMotors();
  startTimer();
  robotState = ROBOT_EMIT_IR;
}

void handleIntersection() { // starts motors and timer after seeing an intersection
  startMotor();
  startTimer();
  robotState = ROBOT_CENTERING;
}

void handleObjectInFront() { // handles what to do in centering based on if there is a wall in front or not
  startTimer();
  if (objectInFront) { // object too close
    startTurn(turnState);
    robotState = ROBOT_TURNING;
  }
  else { // no object
    robotState = ROBOT_DRIVING;
  }
  prevCheckObject = false;
  objectInFront = false;
}

void handleTurnDone() { // handles logic after turn finishes for updating direction
  robotState = ROBOT_DRIVING;
  updateDirection();
  if (turnState == RIGHT) { // after turning right, set the turn to left 
    turnState = LEFT;       // (doesnt affect clockwise navigation and works for counterclockwise navigation)
  }

  if (foundDoor) { // handles what to do if it finds door in front after I and J navigation
    stopMotors();
    chassis.setWheelTargetSpeeds(5, 5);
    robotState = ROBOT_GO_TO_DOOR;
  }
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

  Serial1.println("Pallas/Heading:" + String(robotHeadings.direction));
}

void updatePosition() { // updates the I and J values of romi
  robotHeadings.i += deltaX;
  robotHeadings.j += deltaY;
  sendMessage("Pallas/Location", "(" + String(robotHeadings.i) + ", " + String(robotHeadings.j) + ")");
}

/* other functions */

void emitIR() { // emits the Code for Door
  uint32_t codeToProcess = doorCode;

  // initializer
  TCCR1A |= 0b00001000;
  delayMicroseconds(9000);
  TCCR1A &= 0b11110111;
  delayMicroseconds(4500);

  // 32 bits of code
  for (int i = 0; i < 32; i++) {
    TCCR1A |= 0b00001000;
    delayMicroseconds(563);
    TCCR1A &= 0b11110111;
    if (codeToProcess % 2 == 0) 
    {
      delayMicroseconds(562); // delay for "0"
    } 
    else 
    {
      delayMicroseconds(1687); // delay for "1"
    }
    codeToProcess = codeToProcess >> 1;
  }

  // ender
  TCCR1A |= 0b00001000;
  delayMicroseconds(563);
  TCCR1A &= 0b11110111;

}

void findDoor() { // makes romi face the right way when its at the right I and J coordinates of the door
  // calculation for door heading
  int toTurn = robotHeadings.direction - doorHeadings.direction; 
  toTurn -= 4 * (toTurn / 3); /*returns -1, 0, or 1*/

  if (toTurn == 0) { // if romi is facing door already
    chassis.setWheelTargetSpeeds(5, 5);
    robotState = ROBOT_GO_TO_DOOR;
  } else { // if romi needs to turn to reach door
    turnState = (toTurn + 1) / 2; /*makes -1 -> 0 and 1 -> 1, this is because our turnState has 0 = RIGHT and 1 = LEFT*/
    startTimer();
    startTurn(turnState);
    robotState = ROBOT_TURNING;
  }
  foundDoor = true;
}

void lineFollow() { // makes romi line follow using proportional control
  float baseSpeed = 10;
  float lineFollowKp = 0.025;
  float error = lineFollowKp * (analogRead(A4) - analogRead(A3));
  chassis.setWheelTargetSpeeds(baseSpeed + error, baseSpeed - error);
}

/* state machines */

unsigned long printTimer = 0;
String stateIn = "IDLE";

void pallas() { // main state machine
  // just a printer for romi state, on a clock so it doesn't overfill MQTT
  if (millis() > printTimer + 1000) {
    Serial1.println("Pallas/State:" + stateIn);
    printTimer = millis();
  }

  switch (robotState) {
    case ROBOT_IDLE: // idle state
      stateIn = "IDLE";

      if (checkSerialOne()) { 
        if (checkAprilTagFound()) {
          handleAprilTagFound(); // what happens after AprilTag id and rot is sent over MQTT
        }
        if (checkCode()) {
          handleCode(); // what happens after button pusher romi sends i,j, and headings over MQTT
        }
      }
      break;
    case ROBOT_DRIVING: // driving/linefollowing state
      stateIn = "DRIVING";

      lineFollow();
      if (checkIntersection()) { 
        handleIntersection(); // what happens when both line sensors see the white line
      }
      break;
    case ROBOT_TURNING: // turning state
      stateIn = "TURNING";

      // timer based turning
      if (checkTimerExpires(2450 + 2450 * (turnState / 2))) { //calculates how long to wait for turnStates: RIGHT, LEFT, or UTURN
            handleTurnDone();
      }
      break;
    case ROBOT_CENTERING: // centering state
      stateIn = "CENTERING";

      objectInFront = checkObject(20);

      if (checkTimerExpires(785)) { // timer based centering      
        updatePosition();

        if (ESCAPEDFUCKYEAH) {
          handleEscapedFuckYeah(); // what happens after it escapes
        } else {
          if (checkCoordinateI() || checkCoordinateJ()) {
            if (checkCoordinateI()) { 
              handleCoordinateI(); // what happens after romi reaches correct I coordinate
            }
            if (checkCoordinateJ()) {
              handleCoordinateJ(); // what happens after romi reaches correct J coordinate
            }
          } else {
            handleObjectInFront(); // normal maze navigation when there is wall in front
          }
        }
      }
      break;
    case ROBOT_GO_TO_DOOR: // getting closer to door state
      stateIn = "GO TO DOOR";

      objectInFront = checkObject(4);
      if(objectInFront) {
        handleGoToEMIT(); // handles emitting the Code for Door
      }
      break;
    case ROBOT_EMIT_IR: // emiting ir state
      stateIn = "EMITTING IR";
      
      if (checkTimerExpires(100)) {
        handleEmitIR(); // emits the IR code
      }
      break;
    case ROBOT_CHECK_DOOR: // checking if door is open state
      objectInFront = checkObject(20);
      if (checkTimerExpires(100)) {
        handleCheckDoor(); // handles logic for door open vs. closed
      }
      break;
    default:
      break;
  }
}
