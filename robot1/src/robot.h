/**
 * 
 * contains all of the enums for our states and
 * all of the headers of our functions used in robot.cpp
 * 
 **/

#pragma once

#include <Arduino.h>
#include <Chassis.h>
#include <apriltagdatum.h>

enum ROBOT_STATE
{   
    ROBOT_IDLE, 

    ROBOT_DRIVING,
    ROBOT_CENTERING,
    ROBOT_READ_APRILTAG,
    ROBOT_UP_RAMP,
    ROBOT_TURNING,
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

// start functions
void startMotor();
void startTimer();
void startTurn(int turnType);

// stop functions
void stopMotors();

// check functions
bool checkAprilTag(AprilTagDatum& tag);
bool checkIntersection();
bool checkObject(float tooCloseDistance);
bool checkPitchAtBottom();
bool checkPitchAtTop();
bool checkSecondColumnWall();
bool checkTimerExpires(unsigned long timerDelay);

// handle functions
void handleAprilTag(AprilTagDatum tag);
void handleIntersection();
void handleObjectInFront(bool objectInFront);
void handleReachedRamp();
void handleReachedTop();
void handleTimerExpires();
void handleTurnDone();

// update functions
void updateDirection();
void updatePosition();

// random functions
void lineFollow();
void sendToMQTT(AprilTagDatum tag);

// state machines
void beanas();