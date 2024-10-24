#include <robot.h>
#include <Chassis.h>
#include <HC-SR04.h>
#include <ir_codes.h>
#include <LSM6.h>
#include <openmv.h>
#include <apriltagdatum.h>

#include <IRDirectionFinder.h>
IRDirectionFinder irFinder;

HC_SR04 hc_sr04(17, 4);
void ISR_HC_SR04(void)
{
    hc_sr04.ISR_echo();
}

/* variables */

ROBOT_STATE robotState = ROBOT_IDLE;
int direction = NORTH;
int turnState = RIGHT;

Chassis chassis;
OpenMV camera;

float objectDistance = 0;
float tooCloseDistance = 30;
unsigned long timerStarts = 0;

bool prevCheckObject;
bool objectInFront;
bool headingToZero = false;
int doorDirection = NORTH;
bool clockwise = false;

int deltaX = 0;
int deltaY = 1;
int robotI = 0;
int robotJ = 0;

int doorI = 0;
int doorJ = 0;

bool foundDoor = false;
bool foundDoorI = false;
bool foundDoorJ = false;
uint32_t doorCode = 0;
bool didTheUTurn = false;
bool reachedOrigin = false;

bool exploredSecondRow = false;
bool exploreSecondRow = true;
bool wallInSecondRow = false;
bool foundAprilTag = false;
bool WALLIEESCAPEDFUCKYEAH = false;

int count = 0;
bool hasEscaped;
int cycle = 0;

float baseSpeed = 5;
float distance = 10;

String PallasEscapedString;
String SerialOneReading;
String AprilTagSearchString;
String CodeSearchString;

/* basic functions */

String keyCodeString; // his may come in handy later
void handleKeyCode(int16_t keyCode)
{ 
  Serial.println(keyCode);

  if(keyCode == ENTER_SAVE) idle();

  switch(keyCode)
  {
    case UP_ARROW:
      chassis.setMotorTargetSpeeds(10, 10);
      robotState = ROBOT_UP;
      break;
    case RIGHT_ARROW:
      chassis.setMotorTargetSpeeds(10, -10);
      robotState = ROBOT_RIGHT;
      break;
    case DOWN_ARROW:
      chassis.setMotorTargetSpeeds(-10, -10);
      robotState = ROBOT_DOWN;
      break;
    case LEFT_ARROW:
      chassis.setMotorTargetSpeeds(-10, 10);
      robotState = ROBOT_LEFT;
      break;
    case PLAY_PAUSE:
      turnState = RIGHT;
      robotState = ROBOT_DRIVING;
      startTimer();
      break;
    case NUM_9:
    Serial.println(analogRead(A3));
    Serial.println(analogRead(A4));
  }
}

void idle(void)
{
  Serial.println("Idling!");
  chassis.setMotorEfforts(0, 0);
  // if (hasEscaped == 1) {
  //   robotState = HOME_IDLE;
  // } else robotState = ROBOT_IDLE;
}

void initialize(void)
{
    chassis.init();

    Serial1.begin(115200);

    hc_sr04.init(ISR_HC_SR04);
    hc_sr04.getDistance(objectDistance);

    pinMode(A3, INPUT);
    pinMode(A4, INPUT);

    irFinder.begin();
}

void sendMessage(const String& topic, const String& message)
{
    Serial1.println(topic + String(':') + message);
}

/* start functions */

void startMotor() {
  chassis.setWheelTargetSpeeds(5, 5);
}

bool checkAprilTagFound() {
  Serial.println(AprilTagSearchString);
  bool foundAprilTag = AprilTagSearchString.startsWith("AprilTag/Info:");
  if (!foundAprilTag) {SerialOneReading = "";};
  return foundAprilTag;
}

void startTimer() {
  timerStarts = millis();
}

void startTurn(int turnType) {
  int baseSpeed = 5;
  int turnSpeed = baseSpeed - (2 * baseSpeed * (turnType % 2));
  if (hasEscaped == 1) {
    chassis.setWheelTargetSpeeds(-turnSpeed, turnSpeed);
  } else chassis.setWheelTargetSpeeds(turnSpeed, -turnSpeed);
}

/* stop functions */

void stopMotors() {
  chassis.setMotorEfforts(0,0);
}

/* check functions */

bool checkIntersection() {
  return (analogRead(A3) < 200) && (analogRead(A4) < 200);
}

bool checkCoordinateI() {
  return robotI == doorI && !foundDoor && !foundDoorI;
}

bool checkCoordinateJ() {
  return robotJ == doorJ && !foundDoor && !foundDoorJ;
}

bool checkObject(float tooCloseDistance) {
  if (hc_sr04.getDistance(objectDistance)) 
  {
    prevCheckObject = objectDistance < tooCloseDistance;
    return objectDistance < tooCloseDistance;
  }
  //Serial.println("check no work");
  return prevCheckObject;
}

bool checkPallasEscaped() {
  //change topic and shit when we finalize mqtt
  bool foundPallasEscaped = PallasEscapedString.startsWith("Pallas/Escaped:");
  if (!foundPallasEscaped) {SerialOneReading = "";};
  return foundPallasEscaped;
}

bool checkPitchAtBottom() {
  return chassis.updatePitch() < -8;
}

bool checkPitchAtTop() {
  return chassis.updatePitch() > -1;
}

bool checkCode() {
  bool foundCode = CodeSearchString.startsWith("Code/CodeBase:");
  if (!foundCode) {SerialOneReading = "";}
  return foundCode;
}

/* handle functions */

void handleAprilTagFound() {
  //"AprilTag/Info:14,180";
  int commaAt = AprilTagSearchString.indexOf(",");

  String idString = AprilTagSearchString.substring(14, commaAt);
  String rotString = AprilTagSearchString.substring(commaAt + 1, AprilTagSearchString.length());

  int id = idString.toInt();
  int rot = rotString.toInt();

  doorI = id / 10;
  doorJ = id % 10;
  doorDirection = round(rot / 90.0) % 4; //could be wrong

  AprilTagSearchString = false;
  SerialOneReading = "";
}

void handleCoordinateI() {
  Serial.println("Found Door i");
  if (clockwise) {
    findDoor();
  } else {
    startTimer();
    startTurn(turnState);
    robotState = ROBOT_TURNING;
    foundDoorI = true;
  }
}

void handleCoordinateJ() {
  Serial.println("Found Door j");
  if (clockwise) {
    startTimer();
    startTurn(turnState);
    robotState = ROBOT_TURNING;
    foundDoorJ = true;
  } else {
    findDoor();
  }
}

bool checkSerialOne() {
  if(Serial1.available()) {
      char c = Serial1.read();
      if(c == '\n') return true;
      else if (c != '\r') SerialOneReading += c;
  }
  PallasEscapedString = SerialOneReading;
  return false;
}

bool checkSecondColumnWall() {
  return !exploredSecondRow && robotI == 1;
}

bool checkTimerExpires(unsigned long timerDelay) {
  return (timerStarts + timerDelay) < millis();
}

/* handle functions */

void handleAprilTag(AprilTagDatum tag) {
  sendToMQTT(tag);
  foundAprilTag = true;
  turnState = UTURN;
  robotState = ROBOT_TURNING;
}

void handleIntersection() {
  robotState = ROBOT_CENTERING;
}

void handleObjectInFront(bool objectInFront) {
  updatePosition();
  if (objectInFront) {
    startTurn(turnState);
    robotState = ROBOT_TURNING;
  }
  else {
    robotState = ROBOT_DRIVING;
  }
}

void handleCode() {
  //"Code/CodeBase:242"
  String codeString = CodeSearchString.substring(CodeSearchString.length() - 3, CodeSearchString.length());
  int codeBase = codeString.toInt();
  uint32_t code = 0;
  uint8_t codePartOne = 48 + (codeBase / 100);
  code = codePartOne;
  uint8_t codePartTwo = 48 + ((codeBase % 100) / 10);
  code = (code << 8) | codePartTwo;
  uint8_t codePartThree = 48 + (codeBase % 10);
  code = (code << 8) | codePartThree;
  uint8_t codePartFour = codePartOne ^ codePartTwo ^ codePartThree;
  code = (code << 8) | codePartFour;

  SerialOneReading = "";
  Serial1.println("Code/Code:" + String(code));

  doorCode = code;
  CodeSearchString = "";
}

void handlePallasEscaped() {
  hasEscaped = (PallasEscapedString.substring(14, 15)).toInt();
  Serial.println("Ready to go!");
  PallasEscapedString = false;
  SerialOneReading = "";
  turnState = UTURN;
  startTurn(turnState);
  startTimer();
  robotState = ROBOT_TURNING;
  headingToZero = true;
}

void handleTurnDone() {
    if(headingToZero) {
      updateDirection();
      turnState = LEFT;
    } else {
      updateDirection();
    }
    robotState = ROBOT_DRIVING;
    startTimer();
}

/* update functions */

void updateDirection() {
  switch (direction) {
    case NORTH:
      if(turnState == RIGHT) {
        direction = EAST;
        deltaX = 1;
        deltaY = 0;
      } 
      else if(turnState == UTURN) {
        direction = SOUTH;
        deltaX = 0;
        deltaY = -1;
      }
      else {
        direction = WEST;
        deltaX = -1;
        deltaY = 0;
      }
      break;
    case EAST:
      if(turnState == RIGHT) {
        direction = SOUTH;
        deltaX = 0;
        deltaY = -1;
      } 
      else if(turnState == UTURN) {
        direction = WEST;
        deltaX = -1;
        deltaY = 0;
      }else {
        direction = NORTH;
        deltaX = 0;
        deltaY = 1;
      }
      break;
    case SOUTH:
      if(turnState == RIGHT) {
        direction = WEST;
        deltaX = -1;
        deltaY = 0;
      }
      else if(turnState == UTURN) {
        direction = NORTH;
        deltaX = 0;
        deltaY = 1;
      } else {
        direction = EAST;
        deltaX = 1;
        deltaY = 0;
      }
      break;
    case WEST:
      if(turnState == RIGHT) {
        direction = NORTH;
        deltaX = 0;
        deltaY = 1;
      }
      else if(turnState == UTURN) {
        Serial.print("Went east");
        direction = EAST;
        deltaX = 1;
        deltaY = 0;
        didTheUTurn = true;
      } else {
        direction = SOUTH;
        deltaX = 0;
        deltaY = -1;
      }
      break;
  }
  Serial1.println("Wallie/Heading:" + String(direction));
}

void updatePosition() {
  if(didTheUTurn == false) {
    robotI += deltaX;
    robotJ += deltaY;
  }
  else {
    didTheUTurn = false;
  }
  sendMessage("Wallie/Location", "(" + String(robotI) + "," + String(robotJ) + ")");
}

void emitIR() {
  //emit code every 250ms
  uint32_t codeToProcess = doorCode;
  TCCR1A |= 0b00001000;
  delayMicroseconds(9000);
  TCCR1A &= 0b11110111;
  delayMicroseconds(4500);

  for (int i = 0; i < 32; i++) {
    TCCR1A |= 0b00001000;
    delayMicroseconds(563);
    TCCR1A &= 0b11110111;
    if (codeToProcess % 2 == 0) 
    {
      delayMicroseconds(562);
    } 
    else 
    {
      delayMicroseconds(1687);
    }
    codeToProcess = codeToProcess >> 1;
  }

  TCCR1A |= 0b00001000;
  delayMicroseconds(563);
  TCCR1A &= 0b11110111;

}

void findDoor() {
  Serial1.println("Got To FindDoor/gtfd");
  int toTurn = direction - doorDirection;
  toTurn -= 4 * (toTurn / 3);

  if (toTurn == 0) {
    chassis.setWheelTargetSpeeds(5, 5);
    robotState = ROBOT_GO_TO_DOOR;
    stopMotors();
  } else {
    turnState = (toTurn + 1) / 2;
    startTimer();
    startTurn(turnState);
    robotState = ROBOT_TURNING;
  }
  foundDoor = true;
}

/* other functions */

void lineFollow() {
  float lineFollowKp = 0.020;
  float error = lineFollowKp * (analogRead(A4) - analogRead(A3));

  chassis.setWheelTargetSpeeds(10 + error, 10 - error);
}

void lineFollow(float baseSpeed) {
  float lineFollowKp = 0.02;
  float error = lineFollowKp * (analogRead(A4) - analogRead(A3));
  chassis.setWheelTargetSpeeds(baseSpeed + error, baseSpeed - error);
}

void sendToMQTT(AprilTagDatum tag) {
  Serial1.println("AprilTag/Info:" + String(tag.id) + "," + String(tag.rot));
}


void irStuff() {
  irFinder.requestPosition();
  Point point = irFinder.ReadPoint(0);
  if (robotState == ROBOT_CENTERING) {
    sendMessage("IRFinder/X", String(point.x));
    sendMessage("IRFinder/Y", String(point.y));
  }

  if (irFinder.available()) {
    for (int i=0; i<4; i++) 
    {
      Point point1 = irFinder.ReadPoint(i);
      Serial.print(point1.x);
      Serial.print(",");
      
      Serial.print(point1.y);
      Serial.print(";");
    }
    Serial.print('\n');
  }

Serial.println(robotState);
  if ((point.x != 1023 && point.y != 1023) && robotState == ROBOT_CENTERING && (!headingToZero)) {
    updatePosition();
    startTurn(turnState);
    robotState = ROBOT_IR_READ_TURN;
    Serial.println("Wallie was here");
  }

}

void sendCode() {
  Serial1.println("Code/CodeBase:" + String(robotI) + String(robotJ) + String(direction));
}

void waitForBeanas() {
  if(checkTimerExpires(300000)) {
    startTurn(turnState);
    robotState = ROBOT_TURNING;
  }
}

/* state machines */

AprilTagDatum tag;
unsigned long printTimer = 0;
String stateIn = "IDLE";

void wallie() {
    if (millis() > printTimer + 1000) {
      Serial1.println("Wallie/State:" + stateIn);
      printTimer = millis();
    }

  switch (robotState) {
    case ROBOT_IDLE:
      stateIn = "IDLE";

      if (checkSerialOne()) {
        if (checkPallasEscaped()) {
          handlePallasEscaped(); //->turnState = LEFT; robotState = HOME_DRIVING;
        }
      }
      
      break;
    case ROBOT_DRIVING:
      stateIn = "DRIVING";

        lineFollow();
        if (checkTimerExpires(1000)) {
          if(!headingToZero) {
            irStuff();
          }
          startTimer();
        }

        if (checkIntersection()) {
          handleIntersection();
          startMotor();
          startTimer();
        }
      break;
    case ROBOT_TURNING:
      stateIn = "TURNING";

      switch(turnState) {
        case RIGHT:
          if (checkTimerExpires(2450)) {
            handleTurnDone();
          }
          break;
        case LEFT:
          if (checkTimerExpires(2450)) {
            handleTurnDone();
          } 
          break;
        case UTURN:
          if (checkTimerExpires(4900)) {
            handleTurnDone();
          }
          break;
      }
      break;
    case ROBOT_CENTERING:
      stateIn = "CENTERING";
      objectInFront = checkObject(30);
      if(headingToZero && reachedOrigin == false) {
        if(checkAtOrigin()) {
        handleAtOrigin();
        }
      }
      if (checkTimerExpires(1100)) {
        if(!headingToZero) {
          irStuff(); 
        }
        if (robotState != ROBOT_IR_READ_TURN) {
          handleObjectInFront(objectInFront);
          prevCheckObject = false;
          objectInFront = false;
        }
        startTimer();
      }
      break;
    case ROBOT_IR_READ_TURN:
      stateIn = "IR TURN";
      if (checkTimerExpires(2450)) {
        updateDirection();
        startTimer();
        robotState = ROBOT_IR_READ_DRIVE;
      }
      break;
    case ROBOT_IR_READ_DRIVE:
      stateIn = "IR DRIVE";
      if (hc_sr04.getDistance(distance)) {
        float error = distance - 5.2;
        float errorSum =+ error;
        baseSpeed = error*0.6 + errorSum*0.08;
      }
      lineFollow(baseSpeed);
      if(baseSpeed < 0.8 || checkTimerExpires(7000)) {
        robotState = ROBOT_IDLE;
        chassis.setMotorEfforts(0,0);
        sendCode();
      }

      if (checkIntersection() && checkTimerExpires(1000)) {
        updatePosition();
        startTimer();
      }
      break;
    default:
      break;
  }

}

bool checkAtOrigin() {
  return robotI == 0 && robotJ == 1;
}

void handleAtOrigin() {
  reachedOrigin = true;
  robotState = ROBOT_TURNING;
  if(clockwise) {
    turnState = UTURN;
    startTurn(turnState);
    startTimer();
    robotState = ROBOT_DRIVING;
    turnState = LEFT;
    pallas();
  }
  else {
    turnState = LEFT;
    startTurn(turnState);
    startTimer();
    robotState = ROBOT_DRIVING;
    turnState = LEFT;
    pallas();
  }
  //stopMotors
  //UTURN
  //go to Pallas State Machine
}

void pallas() {
  if (millis() > printTimer + 750) {
    Serial1.println("Pallas/State:" + stateIn);
    printTimer = millis();
    cycle = (cycle + 1) % 3;
  }

  switch (robotState) {
    case ROBOT_IDLE:
      stateIn = "IDLE";

      if (checkSerialOne()) {
        if (checkAprilTagFound()) {
          handleAprilTagFound();
        }
        if (checkCode()) {
          handleCode();
        }
      }
      break;
    case ROBOT_DRIVING:
      stateIn = "DRIVING";

      lineFollow();
      if (checkIntersection()) {
        handleIntersection();
      }
      break;
    case ROBOT_TURNING:
      stateIn = "TURNING";

      // if (checkTimerExpires(2450 + 2450 * (turnState / 2))) {
      //       handleTurnDone();
      // }

      switch(turnState) {
        case RIGHT:
          if (checkTimerExpires(2450)) {
            handleTurnDone();
          }
          break;
        case LEFT:  
          if (checkTimerExpires(2450)) {
            handleTurnDone();
          }
          break;
        case UTURN:
          if (checkTimerExpires(4900)) {
            handleTurnDone();
          }
          break;
      }
      break;
    case ROBOT_CENTERING:
      stateIn = "CENTERING";

      objectInFront = checkObject(20);

      if (checkTimerExpires(785)) {
        if(WALLIEESCAPEDFUCKYEAH) {
          delay(1500);
          stopMotors();
          sendMessage("Pallas/Escaped","1");
          robotState = ROBOT_IDLE;
        }
        
        updatePosition();
        if (checkCoordinateI() || checkCoordinateJ()) {
          if (checkCoordinateI()) {
            handleCoordinateI();
          }
          if (checkCoordinateJ()) {
            handleCoordinateJ();
          }
        } else if (!WALLIEESCAPEDFUCKYEAH) {
            handleObjectInFront(objectInFront);
          }
        }

      break;
    case ROBOT_EMIT_IR:
    stateIn = "EMITTING IR";
      Serial.println("starting up!!!");
      Serial.println("we stop");

      for (int i = 0; i < 4; i++) {
        delay(250);
        emitIR();
        //Serial.println("blink");
      }
      // objectInFront = checkObject();
      // delay(100);
      // emitIR();
      // delay(100);
      // objectInFront = checkObject();
      // delay(100);

      // if (!objectInFront) {
      WALLIEESCAPEDFUCKYEAH = true;
      robotState = ROBOT_DRIVING;
      // }
    break;
    case ROBOT_GO_TO_DOOR:
    stateIn = "GO TO DOOR";
    objectInFront = checkObject(5);
    if(objectInFront) {
      chassis.setMotorEfforts(0, 0);
      robotState = ROBOT_EMIT_IR;
    }

    break;
    case ROBOT_CHECK_DOOR:
      objectInFront = checkObject(10);
      if (checkTimerExpires(200)) {
        if (objectInFront) {
          startTimer();
          robotState = ROBOT_EMIT_IR;
          cycle++;
      } else {
          WALLIEESCAPEDFUCKYEAH = true;
          robotState = ROBOT_DRIVING;
          Serial1.println("AmountOfTimesChecked:" + String(cycle));
      }
      }
      break;
    default:
    break;
  }
}