/**
 * 
 * contains all of the enums for our states and
 * all of the headers of our functions used in robot.cpp
 * 
 **/

#pragma once

#include <Arduino.h>
#include <Chassis.h>

enum ROBOT_STATE
{   
    ROBOT_IDLE, 

    ROBOT_DRIVING,
    ROBOT_CENTERING,
    ROBOT_TURNING,
    ROBOT_EMIT_IR,
    ROBOT_CHECK_DOOR,
    ROBOT_GO_TO_DOOR
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

// basic functions
void initialize(void);
void idle(void);
void handleKeyCode(int16_t keyCode);
void sendMessage(const String& topic, const String& message);

// start functions
void startMotor();
void startTimer();
void startTurn(int turnType);

// stop functions
void stopMotors();

// check functions
bool checkAprilTagFound();
bool checkCode();
bool checkCoordinateI();
bool checkCoordinateJ();
bool checkIntersection();
bool checkObject(float tooCloseDistance);
bool checkSerialOne();
bool checkTimerExpires(unsigned long timerDelay);

// handle functions
void handleAprilTagFound();
void handleCheckDoor();
void handleCode();
void handleCoordinateI();
void handleCoordinateJ();
void handleEmitIR();
void handleEscapedFuckYeah();
void handleGoToEMIT();
void handleIntersection();
void handleObjectInFront();
void handleTurnDone();

// update functions
void updateDirection();
void updatePosition();

// other functions
void emitIR();
void findDoor();
void lineFollow();

// state machines
void pallas();


