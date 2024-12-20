//this code is created with the aid of ChatGPT and W3Schools, we also modified part of the html lines from lab 4.1.3b
//team member: Stan Han, Steyn Knollema, Matt Rabin
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include "WebServerHandler.h"
#include "ViveSensors.h"
#include <PID_v1_bc.h>
#include <Arduino.h>
// --------------------- Constants and Pin Definitions ---------------------

//



// Define pin assignments
#define SDA_PIN 37
#define SCL_PIN 38
#define XSHUT1_PIN 41
#define XSHUT2_PIN 40
#define XSHUT3_PIN 39

// LEDC Configuration
#define LEDC_RESOLUTION_BITS 7
#define LEDC_RESOLUTION ((1 << LEDC_RESOLUTION_BITS) - 1)
#define LEDC_RESOLUTION_BITS_2 14
#define LEDC_RESOLUTION_2 ((1 << LEDC_RESOLUTION_BITS_2) - 1)
#define LEDC_CHANNEL_1 0  // Channel for Motor 1 PWM
#define LEDC_CHANNEL_2 1  // Channel for Motor 2 PWM
#define LEDC_CHANNEL_3 2  // Channel for Servo
#define LEDC_PIN1 6       // Motor 1 PWM pin
#define LEDC_PIN2 16      // Motor 2 PWM pin
#define LEDC_PIN3 11      // Servo PWM pin
#define I2C_SLAVE_ADDR 0x28

enum Modes { WALL_FOLLOWING = 0, WALL_FOLLOWING_ATTACK=1, ATTACK=2 };
int mode = WALL_FOLLOWING;

int f_dis;
int l_dis;
int r_dis;

// Orientation Angle
float orientationAngle = 0.0f;     // Current orientation angle
float orientationFiltered = 0.0f;  // Smoothed orientation angle
const float alpha = 0.1f;          // Smoothing factor

// Instantiate two Adafruit VL53L0X objects
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();

// wall following parameters
int max_dis = 300;  //max distance
int min_dis = 200;  //min distance
int front_dis;      //front distance
int right_dis;      //right distance


// Motor Control Pins
const int motorPin1_1 = 4;  // Motor 1 H-Bridge pin 1 (Direction)
const int motorPin2_1 = 5;  // Motor 1 H-Bridge pin 2 (Direction)


// Define 'newTargetReceived' once
bool newTargetReceived = false;

const int motorPin1_2 = 7;   // Motor 2 H-Bridge pin 1 (Direction)
const int motorPin2_2 = 15;  // Motor 2 H-Bridge pin 2 (Direction)

// Define a structure for targets
struct Target {
  float x;
  float y;
  bool reached;  // Indicates if the target has been reached
};

// top hat
int hp_decrement;
int hp_remaining;
int tophat_error;
//

// Declare three specific targets
const int numTargets = 3;
Target targets[numTargets] = {
    {61.34, 36.23, false},  // Target 1 AT THE BACK FULL FORCE IN

  {48.00, 47.50, false},  // Target 2 IN THE MIDDLE (FIGURE OUT HOW TO DO)
  {40.50, 24.86, false}   // Target 3 ON THE HILL 90 DEGREE TURN AND SEND


};

// Stopping state variables
bool isStopping = false;          // Indicates if the robot is currently stopping
unsigned long stopStartTime = 0;  // Timestamp when stopping began
const unsigned long stopDuration = 5000; // Duration to stop (milliseconds)

float curX1 = 0;
float curX2 = 0;
float curY1 = 0;
float curY2 = 0;

float targetX=0.0f;
float targetY=0.0f;
// Encoder Pins
const int encoderPin_1 = 3;  // Encoder Signal A for Motor 1
const int encoderPin_2 = 8;  // Encoder Signal A for Motor 2

// Wi-Fi Credentials
const char* ssid = "Steyn";          // SSID for Access Point
const char* password = "123456789";  // Password for Access Point

// Create Web Server on Port 80


// --------------------- Global Variables ---------------------

// PWM Values
int pwmValue_1 = 0;
int pwmValue_2 = 0;

// Proportional Control Adjustments
float u_1 = 0.0;
float u_2 = 0.0;

// Encoder Pulse Counters
volatile unsigned long encoderTicks_1 = 0;  // Motor 1
volatile unsigned long encoderTicks_2 = 0;  // Motor 2

// Timing Variables
unsigned long previousMillis = 0;  // Last time speed was calculated
unsigned long previousMillis_2 = 0;
unsigned long currentMillis = 0;
const unsigned long timeInterval = 1000;  // Interval for speed calculation (ms) possible 250 if this doesn't work
const float n = 1000.0 / timeInterval;   // Scaling factor

// Proportional Control Parameters
const float Kp_1 = 2.0;  // Proportional Gain for Motor 1
const float Kp_2 = 2.0;  // Proportional Gain for Motor 2

// Error Variables
float error_1 = 0.0;
float error_2 = 0.0;

// Motor Speed Variables
float motorSpeed_1 = 0.0;  // Motor 1 Speed (counts/sec)
float motorSpeed_2 = 0.0;  // Motor 2 Speed (counts/sec)

// Desired Speed Variables
float desiredSpeed_1 = 0.0;  // Desired Speed for Motor 1
float desiredSpeed_2 = 0.0;  // Desired Speed for Motor 2

// --------------------- Function Definitions ---------------------

// Handle Root Path

// ISR for Encoder 1
void IRAM_ATTR encoderISR_1() {
  encoderTicks_1++;  // Increment encoder tick count on each pulse for Motor 1
}

// ISR for Encoder 2
void IRAM_ATTR encoderISR_2() {
  encoderTicks_2++;  // Increment encoder tick count on each pulse for Motor 2
}

// LEDC Analog Write for Motor 1
void ledcAnalogWrite1(uint8_t channel, uint32_t value, uint32_t valueMax) {
  uint32_t duty = LEDC_RESOLUTION * min((int)value, (int)valueMax) / valueMax;
  ledcWrite(LEDC_PIN1, duty);  // Write duty cycle to LEDC channel
}

// LEDC Analog Write for Motor 2
void ledcAnalogWrite2(uint8_t channel, uint32_t value, uint32_t valueMax) {
  uint32_t duty = LEDC_RESOLUTION * min((int)value, (int)valueMax) / valueMax;
  ledcWrite(LEDC_PIN2, duty);  // Write duty cycle to LEDC channel
}

// LEDC Analog Write for Servo
void ledcAnalogWrite3(uint8_t channel, uint32_t value, uint32_t valueMax) {
  uint32_t duty = LEDC_RESOLUTION * min((int)value, (int)valueMax) / valueMax;
  ledcWrite(LEDC_PIN3, duty);  // Write duty cycle to LEDC channel
}
void Servomode(){
     ledcAnalogWrite3(LEDC_CHANNEL_3, 10, 100);
    delay(500);
    ledcAnalogWrite3(LEDC_CHANNEL_3, 60, 100);
    delay(500);
    Serial.println("attack");
}
bool isAtTarget(float curX, float curY, float targetX, float targetY, float threshold);
void stopMotors();

void handleWallFollowing() {
    // Set the robot to Wall Following mode
    mode = 0;
    Serial.println("Wall Following mode activated.");
    server.send(200, "text/plain", "Wall Following mode activated");
    delay(5000);
}

void handleWallFollowingAttack() {
    // Set the robot to Wall Following + Attack mode
    mode = 1;
    Serial.println("Wall Following + Attack mode activated.");
    server.send(200, "text/plain", "Wall Following + Attack mode activated");
    delay(5000);
}

void handleServoSwing() {
    // Activate Servo Arm Swing
    mode = 2;
    Serial.println("Swinging Servo Arm activated");
        server.send(200, "text/plain", "Swinging servo arm activated");
    delay(5000);
}

void printCurrentMode() {
    Serial.print("Current mode: ");
    switch (mode) {
        case 0:
            Serial.println("WALL_FOLLOWING");
            delay(100);
            break;
        case 1:
            Serial.println("WALL_FOLLOWING_ATTACK");
            delay(100);
            break;
        case 2:
            Serial.println("ServoMode");
            delay(100);
            break;
        default:
            Serial.println("UNKNOWN");
            break;
    }
}

void Servofunction(){
      ledcAnalogWrite3(LEDC_CHANNEL_3, 10, 100);
    delay(500);
    ledcAnalogWrite3(LEDC_CHANNEL_3, 60, 100);
    delay(500);
}


void send_I2C_byte(uint8_t data) {
  // Send data to slave
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write(data);  // Send some test data
  uint8_t error = Wire.endTransmission();
  tophat_error=error;
  if (error == 0) {
    Serial.println("Data sent successfully");
    //rgbLedWrite(2, 0, 20, 0);  // green
  } else {
    Serial.printf("Error sending data: %d\n", error);
    //rgbLedWrite(2, 20, 0, 0);  // red
  }
}

uint8_t receive_I2C_byte() {  // data should have space declared from caller
  // Request data from slave
  uint8_t bytesReceived = Wire.requestFrom(I2C_SLAVE_ADDR, 1);
  uint8_t byteIn = 0;

  if (bytesReceived > 0) {
    //Serial.print("Received from slave: ");
    while (Wire.available()) {
      byteIn = Wire.read();
      //Serial.printf("0x%02X ", byteIn);
    }
    //Serial.println();
  } else return 0;
  return byteIn;  // return number of bytes read
}

// --------------------- Setup Function ---------------------

void setup() {
  Serial.begin(115200);

  // Initialize Vive sensors
  // Set up Wi-Fi
  WiFi.softAP(ssid, password);
  Serial.println("Access Point started: " + String(ssid));

   Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());

    Wire.begin(SDA_PIN, SCL_PIN, 40000);
  Serial.println("ESP32-C3 I2C Master initialized");
  Serial.printf("SDA: %d, SCL: %d\n", SDA_PIN, SCL_PIN);

  delay(1000);

  initWebServer();  // Initialize the web server

  delay(1000);
  initVive();
  // Initialize I2C master

  delay(5000);

  // Initialize Motor Control Pins
  pinMode(motorPin1_1, OUTPUT);
  pinMode(motorPin2_1, OUTPUT);
  pinMode(LEDC_PIN1, OUTPUT);

  pinMode(motorPin1_2, OUTPUT);
  pinMode(motorPin2_2, OUTPUT);
  pinMode(LEDC_PIN2, OUTPUT);

  // Initialize Encoder Pins
  pinMode(encoderPin_1, INPUT_PULLUP);
  pinMode(encoderPin_2, INPUT_PULLUP);

  // Initialize Servo Pin
  //pinMode(LEDC_PIN3, OUTPUT);

  // Attach Interrupts to Encoder Pins
  attachInterrupt(digitalPinToInterrupt(encoderPin_1), encoderISR_1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPin_2), encoderISR_2, RISING);

  delay(5000);



  delay(500);
  // Start the Web Server
  server.begin();
  Serial.println("HTTP server started");

  ledcAttachChannel(LEDC_PIN1, 1000, LEDC_RESOLUTION_BITS, LEDC_CHANNEL_1);
  ledcAttachChannel(LEDC_PIN2, 1000, LEDC_RESOLUTION_BITS, LEDC_CHANNEL_2);
  ledcAttachChannel(LEDC_PIN3, 250, 14, LEDC_CHANNEL_3);

  delay(1000);

  while (!Serial) {
    delay(10);  // Wait for Serial Monitor to open
  }
  Serial.println("Dual VL53L0X Distance Measurement with ESP32-S2");

  // Initialize XSHUT pins
  pinMode(XSHUT1_PIN, OUTPUT);
  pinMode(XSHUT2_PIN, OUTPUT);
  pinMode(XSHUT3_PIN, OUTPUT);

  // Set both sensors to reset
  digitalWrite(XSHUT1_PIN, LOW);
  digitalWrite(XSHUT2_PIN, LOW);
  digitalWrite(XSHUT3_PIN, LOW);
  delay(10);

  // Start Sensor 1
  digitalWrite(XSHUT1_PIN, HIGH);
  delay(10);
  Wire.begin(SDA_PIN, SCL_PIN);  // Initialize I2C bus

  if (!lox1.begin(0x30)) {  // Assign new I2C address to Sensor 1
    Serial.println("Failed to boot VL53L0X sensor 1!");
    while (1)
      ;
  }
  Serial.println("Sensor 1 initialized.");
 
  // Start Sensor 2
  digitalWrite(XSHUT2_PIN, HIGH);
  delay(10);

  if (!lox2.begin(0x31)) {  // Assign new I2C address to Sensor 2
    Serial.println("Failed to boot VL53L0X sensor 2!");
    while (1);
  }
  Serial.println("Sensor 2 initialized.");


  // Start Sensor 3
  digitalWrite(XSHUT3_PIN, HIGH);
  delay(10);

  if (!lox3.begin(0x32)) {  // Assign new I2C address to Sensor 3
    Serial.println("Failed to boot VL53L0X sensor 3!");
    while (1)
      ;
  }
  Serial.println("Sensor 3 initialized.");
}

// --------------------- Loop Function ---------------------

void loop() {
  // webpage control
  handleWebRequests();  // Handle incoming web requests



  server.handleClient();
  currentMillis = millis();

  // Perform Speed Calculation and Proportional Control at Defined Intervals
  if (currentMillis - previousMillis >= timeInterval) {
    previousMillis = currentMillis;

    //vive sensor data functions below
    float curX1, curY1;
    float curX2, curY2;
    bool valid1 = readVive1Position(curX1, curY1);
    bool valid2 = readVive2Position(curX2, curY2);

    printCurrentMode();

  if (mode == 0) {
        // Implement Wall Following Logic
        wallFollowingBehavior();
        Serial.print("Wall following ON");
        } 
        
  else if (mode == 1) {
        // Implement Wall Following + Attack Logic
        wallFollowingAttackBehavior();
        Serial.print("Wallfollowingattack behavior ON");
   
    }
  else if (mode == 2){
      Serial.print("Servomode");
      Servofunction();
    }

  // unsure if following is needed, if not uncomment
  else if(mode == 3){
    Serial.print("manual control");
  }

ledcAnalogWrite3(LEDC_CHANNEL_3,99,100);// if servo is not used -> turn off
    // Check if the robot is currently stopping
if (isStopping) {
  if (millis() - stopStartTime >= stopDuration) {
    // Resume wall following after stopping
    isStopping = false;
    Serial.println("Resuming wall following.");
  }
} else {

 

    if (valid1 && valid2) {  // Both sensors valid
      // Calculate orientation angle
      float deltaX = curX2 - curX1;
      float deltaY = curY2 - curY1;
      float theta = atan2(deltaY, deltaX) * (180.0 / PI);  // Convert to degrees

      // Apply smoothing
      orientationFiltered = alpha * theta + (1.0f - alpha) * orientationFiltered;

      // Update angle
      orientationAngle = orientationFiltered;

      // Debugging
      Serial.printf("Orientation Angle: %.2f degrees\n", orientationAngle);


    } else {
      Serial.println("One or both VIVE sensors invalid. Cannot calculate orientation.");
      // Optionally handle invalid sensor data
    }
  }
  }

  // Safely Read and Reset Encoder Ticks
  noInterrupts();
  unsigned long ticks1 = encoderTicks_1;
  unsigned long ticks2 = encoderTicks_2;
  encoderTicks_1 = 0;
  encoderTicks_2 = 0;
  interrupts();

  // Calculate Motor Speeds (counts per second)
  motorSpeed_1 = ticks1 * n / 22;
  motorSpeed_2 = ticks2 * n / 22;

  error_1 = desiredSpeed_1 - motorSpeed_1;     // Compute Error
  u_1 = Kp_1 * error_1;                        // Apply Proportional Gain
  pwmValue_1 += u_1;                           // Adjust PWM Value
  pwmValue_1 = constrain(pwmValue_1, 0, 127);  // Ensure PWM is within 7-bit range

  // Proportional Control for Motor 2
  error_2 = desiredSpeed_2 - motorSpeed_2;     // Compute Error
  u_2 = Kp_2 * error_2;                        // Apply Proportional Gain
  pwmValue_2 += u_2;                           // Adjust PWM Value
  pwmValue_2 = constrain(pwmValue_2, 0, 127);  // Ensure PWM is within 7-bit range

  // Proportional Control for Motor 1 new try, delete if breaking code



  // Apply Adjusted PWM Values to Motors
  ledcAnalogWrite1(LEDC_CHANNEL_1, pwmValue_1, LEDC_RESOLUTION);
  ledcAnalogWrite2(LEDC_CHANNEL_2, pwmValue_2, LEDC_RESOLUTION);

  VL53L0X_RangingMeasurementData_t measure;

  // Perform a ranging measurement for Sensor 1
  lox1.rangingTest(&measure, false);
  //Serial.print("left - ");
  if (measure.RangeStatus != 4) {  // 4 means out of range
    //Serial.print("Distance: ");
    l_dis = measure.RangeMilliMeter;
    //Serial.print(l_dis);
    //Serial.println(" mm");
  } else {
    l_dis = 8000;
    //Serial.println("Out of range");
  }

  delay(10);

  // Perform a ranging measurement for Sensor 2
  lox2.rangingTest(&measure, false);
  //Serial.print("front - ");
  if (measure.RangeStatus != 4) {  // 4 means out of range
    //Serial.print("Distance: ");
    f_dis = measure.RangeMilliMeter;
    //Serial.print(f_dis);
    //Serial.println(" mm");
  } else {
    f_dis = 8000;
    //Serial.println("Out of range");
  }

  delay(10);  // Delay between measurements

  // Perform a ranging measurement for Sensor 3
  lox3.rangingTest(&measure, false);
  //Serial.print("right - ");
  if (measure.RangeStatus != 4) {  // 4 means out of range
    //Serial.print("Distance: ");
    r_dis = measure.RangeMilliMeter;
    //Serial.print(r_dis);
    //Serial.println(" mm");
  } else {
    r_dis = 8000;
    //Serial.println("Out of range");
  }

  if (currentMillis - previousMillis_2 >= 500) {
    previousMillis_2 = currentMillis;
    Serial.print("left - ");
    Serial.print("Distance: ");
    Serial.print(l_dis);
    Serial.print(" mm");
    Serial.print(" right - ");
    Serial.print("Distance: ");
    Serial.print(r_dis);
    Serial.print(" mm");
    Serial.print(" front - ");
    Serial.print("Distance: ");
    Serial.print(f_dis);
    Serial.println(" mm");
    Serial.println(mode);
  }

    // read the value (HP) from top hat
  if(hp_remaining==0 && tophat_error==0){// when top hat is connected and HP is 0
    digitalWrite(motorPin1_1, LOW);
    digitalWrite(motorPin2_1, LOW);
    digitalWrite(motorPin1_2, LOW);
    digitalWrite(motorPin2_2, LOW);
  }

  delay(10);  // Delay between measurements
}






bool isAtTarget(float curX, float curY, float targetX, float targetY, float threshold) {
  bool atTarget = (fabs(targetX - curX) < threshold) && (fabs(targetY - curY) < threshold);
  if (atTarget) {
    Serial.print("Reached target at (");
    Serial.print(targetX);
    Serial.print(", ");
    Serial.print(targetY);
    Serial.println(")");
  }
  return atTarget;
}

// Function to operate motors
void stopMotors() {
  // Stop left motor
  digitalWrite(motorPin1_1, LOW);
  digitalWrite(motorPin2_1, LOW);
  ledcAnalogWrite1(LEDC_PIN1, 0, 100);  // Apply PWM to left motor
  // Stop right motor
  digitalWrite(motorPin1_2, LOW);
  digitalWrite(motorPin2_2, LOW);
  ledcAnalogWrite2(LEDC_PIN2, 0, 100);  // Apply PWM to left motor

  Serial.println("Motors stopped.");
}


// Function to drive forward for a specified time (ms)
void driveForward() {
      if (mode != 3) { // Define 3 as manual mode
        mode = 3;
    }
    // Set Motor Directions for Forward Movement
    digitalWrite(motorPin1_1, HIGH);
    digitalWrite(motorPin2_1, LOW);
    digitalWrite(motorPin1_2, HIGH);
    digitalWrite(motorPin2_2, LOW);
    server.send(200, "text/plain", "Moving Forward");
    Serial.println("Moving Forward");
    desiredSpeed_1 = 60;
    desiredSpeed_2 = 60;
    // mode = 1;
}

void driveBackward(){
      if (mode != 3) { // Define 3 as manual mode
        mode = 3;
    }
      // Set Motor Directions for Backward Movement
    digitalWrite(motorPin1_1, LOW);
    digitalWrite(motorPin2_1, HIGH);
    digitalWrite(motorPin1_2, LOW);
    digitalWrite(motorPin2_2, HIGH);
    server.send(200, "text/plain", "Moving Backward");
    Serial.println("Moving Backward");
    desiredSpeed_1 = 60;
    desiredSpeed_2 = 60;
    // mode = 1;
}

void turnRight(){
      if (mode != 3) { // Define 3 as manual mode
        mode = 3;
    }
      // Set Motor Directions for Right Turn
    digitalWrite(motorPin1_1, HIGH);
    digitalWrite(motorPin2_1, LOW);
    digitalWrite(motorPin1_2, LOW);
    digitalWrite(motorPin2_2, HIGH);
    server.send(200, "text/plain", "Turning Right");
    Serial.println("Turning Right");
    desiredSpeed_1 = 50;
    desiredSpeed_2 = 50;
    // mode = 1;
}

void turnLeft(){
      if (mode != 3) { // Define 3 as manual mode
        mode = 3;
    }
     // Set Motor Directions for Left Turn
    digitalWrite(motorPin1_1, LOW);
    digitalWrite(motorPin2_1, HIGH);
    digitalWrite(motorPin1_2, HIGH);
    digitalWrite(motorPin2_2, LOW);
    server.send(200, "text/plain", "Turning Left");
    Serial.println("Turning Left");
    desiredSpeed_1 = 50;
    desiredSpeed_2 = 50;
    // mode = 1;
}


//////////attacking functions///////////////
void driveForwardAttack(int duration) {
  digitalWrite(motorPin1_1, HIGH);
  digitalWrite(motorPin2_1, LOW);
  digitalWrite(motorPin1_2, HIGH);
  digitalWrite(motorPin2_2, LOW);
  desiredSpeed_1 = 60;
  desiredSpeed_2 = 60;
  delay(duration);//adjust based on if it hits target


}

// Function to turn left for a specified duration (ms)
void turnLeftAttack(int duration) {
  digitalWrite(motorPin1_1, LOW);
  digitalWrite(motorPin2_1, HIGH);
  digitalWrite(motorPin1_2, HIGH);
  digitalWrite(motorPin2_2, LOW);
    desiredSpeed_1 = 50;
  desiredSpeed_2 = 50;
  delay(duration); //add duration to make sure it hits the target

  stopMotors();
}

// Function to turn right for a specified duration (ms)
void turnRightAttack(int duration) {
  digitalWrite(motorPin1_1, HIGH);
  digitalWrite(motorPin2_1, LOW);
  digitalWrite(motorPin1_2, LOW);
  digitalWrite(motorPin2_2, HIGH);
      desiredSpeed_1 = 50;
  desiredSpeed_2 = 50;
  delay(duration);

}

// Function to turn the robot by a specific angle (degrees)
void turnByAngle(float angle, int turnSpeed) {
  int duration = abs(angle) * 10; // Adjust the multiplier based on testing/calibration
  if (angle > 0) {
    turnRightAttack(duration);
  } else {
    turnLeftAttack(duration);
  }
}

void wallFollowingBehavior() {
    // Wall-following logic
    if (f_dis < 100 && f_dis  >= 40) {
      Serial.println("Front too close. Turning Left");
      digitalWrite(motorPin1_1, LOW);
      digitalWrite(motorPin2_1, HIGH);
      digitalWrite(motorPin1_2, HIGH);
      digitalWrite(motorPin2_2, LOW);
      desiredSpeed_1 = 0;
      desiredSpeed_2 = 25;
    } else if (f_dis < 60 || r_dis < 60) {
      Serial.println("Obstacle detected. Moving Backward");
      digitalWrite(motorPin1_1, LOW);
      digitalWrite(motorPin2_1, HIGH);
      digitalWrite(motorPin1_2, LOW);
      digitalWrite(motorPin2_2, HIGH);
      desiredSpeed_1 = 40;
      desiredSpeed_2 = 40;
      delay(500);
    } else if (r_dis < min_dis) {
      Serial.println("Right too close. Turning Left");
      digitalWrite(motorPin1_1, LOW);
      digitalWrite(motorPin2_1, HIGH);
      digitalWrite(motorPin1_2, HIGH);
      digitalWrite(motorPin2_2, LOW);
      desiredSpeed_1 = 0;
      desiredSpeed_2 = 25;
    } else if (r_dis > max_dis) {
      Serial.println("Right too far. Turning Right");
      digitalWrite(motorPin1_1, HIGH);
      digitalWrite(motorPin2_1, LOW);
      digitalWrite(motorPin1_2, LOW);
      digitalWrite(motorPin2_2, HIGH);
      desiredSpeed_1 = 25;
      desiredSpeed_2 = 0;
    } else {
      Serial.println("All good. Moving Forward");
      digitalWrite(motorPin1_1, HIGH);
      digitalWrite(motorPin2_1, LOW);
      digitalWrite(motorPin1_2, HIGH);
      digitalWrite(motorPin2_2, LOW);
      desiredSpeed_1 = 50;
      desiredSpeed_2 = 50;
    }
    ///////////// replace this with wall following behaviour
}

void wallFollowingAttackBehavior() {
    // Wall-following logic (replace with attack behavior part)
    wallFollowingBehavior();

    // Add attack logic
    Serial.println("Wall Follow Attacking");
    ledcAnalogWrite3(LEDC_CHANNEL_3, 10, 100);

    ledcAnalogWrite3(LEDC_CHANNEL_3, 60, 100);

     for (int i = 0; i < numTargets; i++) {
  if (!targets[i].reached && isAtTarget(curX1, curY1, targets[i].x, targets[i].y, 3)) {
    Serial.print("Target ");
    Serial.print(i + 1);
    Serial.println(" reached. Performing target-specific actions.");

    if (i == 0) {
      // Target 1: Move forward for 3 seconds
      driveForwardAttack(5000);
    } else if (i == 1) {
      // Target 2: Turn left 45 degrees, then forward for 2 seconds
      turnByAngle(-45, 50); // Negative for left turn
      driveForwardAttack(2000);
    } else if (i == 2) {
      // Target 3: Turn right 90 degrees, then forward
      turnByAngle(90, 50); // Positive for right turn
      driveForwardAttack(3000);
    }

    targets[i].reached = true;  // Mark the target as reached
    isStopping = true;          // Prevent immediate re-check
    stopStartTime = millis();
    break; // Stop checking after the first target is handled
  }
}



}


