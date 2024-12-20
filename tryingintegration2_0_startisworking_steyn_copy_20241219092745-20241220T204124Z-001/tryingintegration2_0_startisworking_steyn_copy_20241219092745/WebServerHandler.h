#ifndef WEBSERVERHANDLER_H
#define WEBSERVERHANDLER_H

#include <WebServer.h>
#include <ArduinoJson.h>
#include "ViveSensors.h"

// Declare external variables
extern float targetX;
extern float targetY;
extern bool newTargetReceived;
extern int mode;


// Declare server object
extern WebServer server;

void handleRoot();
void handleViveData();
void initWebServer();
void handleWebRequests();
void handleWallFollowing();
void handleWallFollowingAttack();
void handleServoSwing();

void driveBackward();
void driveForward();
void turnLeft();
void turnRight();
void driveForwardAttack();
void turnLeftAttack();
void turnRightAttack();
void stopMotors();

#endif
