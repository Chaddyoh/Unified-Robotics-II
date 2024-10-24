#pragma once

#include <Arduino.h>
#include <Chassis.h>
#include <apriltagdatum.h>

enum ROBOT_STATE
{   
    ROBOT_IDLE, 

    ROBOT_LEFT,
    ROBOT_RIGHT,
    ROBOT_UP,
    ROBOT_DOWN,

    ROBOT_DRIVING,
    ROBOT_CENTERING,
    ROBOT_TURNING,

    ROBOT_IR_READ_TURN,
    ROBOT_IR_READ_DRIVE,

    ROBOT_GO_TO_DOOR,
    ROBOT_EMIT_IR,
    ROBOT_CHECK_DOOR
};

enum DIRECTION
{
    NORTH,
    EAST,
    SOUTH,
    WEST
};

enum TURNSTATE
{
    RIGHT,
    LEFT,
    UTURN
};

void initialize(void);
void idle(void);
void handleKeyCode(int16_t keyCode);

void wallie();

void startMotor();
void startTimer();
void startTurn(int turnType);

void stopMotors();

bool checkAprilTag(AprilTagDatum& tag);
bool checkIntersection();
bool checkObject(float tooCloseDistance);
bool checkPallasEscaped();
bool checkPitchAtBottom();
bool checkPitchAtTop();
bool checkSecondColumnWall();
bool checkSerialONE();
bool checkTimerExpires(unsigned long timerDelay);

void handleAprilTag(AprilTagDatum tag);
void handleIntersection();
void handleObjectInFront(bool objectInFront);
void handlePallasEscaped();
void handleReachedRamp();
void handleReachedTop();
void handleTimerExpires();
void handleTurnDone();

void updateDirection();
void updatePosition();

void lineFollow();
void sendToMQTT(AprilTagDatum tag);
void irStuff();
bool checkCoordinateI();
bool checkCoordinateJ();
bool checkAprilTagFound();
void emitIR();
void findDoor();
void pallas();
bool checkAtOrigin();
void handleAtOrigin();

