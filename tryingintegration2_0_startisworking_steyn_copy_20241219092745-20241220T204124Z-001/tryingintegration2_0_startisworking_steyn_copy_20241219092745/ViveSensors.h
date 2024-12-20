// #ifndef VIVESENSORS_H
// #define VIVESENSORS_H

// #include "vive510.h"

// // Pin definitions for both Vive sensors
// #define SIGNALPIN1 9
// #define SIGNALPIN2 10  // Define a second signal pin for the second sensor
// #define LED_BUILTIN 2

// // Vive sensor instances
// extern Vive510 vive1;
// extern Vive510 vive2;

// // Function prototypes
// void initVive();
// bool readVive1Position(float &x, float &y);
// bool readVive2Position(float &x, float &y);


// #endif
// ViveSensors.h

#ifndef VIVESENSORS_H
#define VIVESENSORS_H

#include "vive510.h"

// Function prototypes
void initVive();
bool readVive1Position(float &x, float &y);
bool readVive2Position(float &x, float &y);
bool isWithinBounds(float x, float y);

// Declare external VIVE510 objects
extern Vive510 vive1;
extern Vive510 vive2;



#endif
