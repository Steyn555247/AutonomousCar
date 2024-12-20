//this code is created with the aid of ChatGPT and W3Schools, we also modified part of the html lines from lab 4.1.3b
//team member: Stan Han, Steyn Knollema, Matt Rabin
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// --------------------- Constants and Pin Definitions ---------------------

// top hat
int hp_decrement;
int hp_remaining;
int tophat_error;
//

int mode=0;//0 is auto, 1 is manual
int attack_mode=0;

// Define pin assignments
#define SDA_PIN 37    
#define SCL_PIN 38  
#define XSHUT1_PIN 41 
#define XSHUT2_PIN 40 
#define XSHUT3_PIN 39 
#define I2C_SLAVE_ADDR 0x28

int f_dis;
int l_dis;
int r_dis;

// Instantiate two Adafruit VL53L0X objects
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();

// wall following parameters
int max_dis = 300;//max distance
int min_dis = 200;//min distance
int front_dis;//front distance
int right_dis;//right distance

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
#define LEDC_PIN3 11       // Servo PWM pin

// Motor Control Pins
const int motorPin1_1 = 4;   // Motor 1 H-Bridge pin 1 (Direction)
const int motorPin2_1 = 5;   // Motor 1 H-Bridge pin 2 (Direction)


const int motorPin1_2 = 7;   // Motor 2 H-Bridge pin 1 (Direction)
const int motorPin2_2 = 15;   // Motor 2 H-Bridge pin 2 (Direction)


// Encoder Pins
const int encoderPin_1 = 3;  // Encoder Signal A for Motor 1
const int encoderPin_2 = 8;  // Encoder Signal A for Motor 2

// Wi-Fi Credentials
const char* ssid = "Stan";              // SSID for Access Point
const char* password = "123456789";     // Password for Access Point

// Create Web Server on Port 80
WebServer server(80);

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
unsigned long previousMillis = 0; // Last time speed was calculated
unsigned long previousMillis_2 = 0;
unsigned long previousMillis_3 = 0;
unsigned long currentMillis = 0;
const unsigned long timeInterval = 1000; // Interval for speed calculation (ms)
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
float desiredSpeed_1 = 0.0; // Desired Speed for Motor 1
float desiredSpeed_2 = 0.0; // Desired Speed for Motor 2

// --------------------- HTML Content ---------------------

const char* htmlContent = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Stan Han_Lab 4.2_Motors Control</title>
    <script>
        // WASD Keyboard Controls
        document.addEventListener("DOMContentLoaded", function() {
            const keysPressed = {};

            function sendCommand(command) {
                fetch("/" + command)
                    .then(response => response.text())
                    .then(text => {
                        document.getElementById("currentAction").innerText = text;
                    })
                    .catch(err => console.error('Error sending command:', err));
            }

            window.addEventListener("keydown", function(e) {
                if (['w', 'a', 's', 'd'].includes(e.key.toLowerCase()) && !keysPressed[e.key.toLowerCase()]) {
                    keysPressed[e.key.toLowerCase()] = true;
                    switch(e.key.toLowerCase()) {
                        case 'w':
                            sendCommand('forward');
                            break;
                        case 's':
                            sendCommand('backward');
                            break;
                        case 'a':
                            sendCommand('left');
                            break;
                        case 'd':
                            sendCommand('right');
                            break;
                    }
                }
            });

            window.addEventListener("keyup", function(e) {
                if (['w', 'a', 's', 'd'].includes(e.key.toLowerCase()) && keysPressed[e.key.toLowerCase()]) {
                    keysPressed[e.key.toLowerCase()] = false;
                    sendCommand('stop');
                }
            });
        });
    </script>
</head>
<body>
    <h1>Group22_Lab 4.2_Motors Control</h1>
    
    <!-- WASD Controls -->
    <h2>WASD Controls</h2>
    <p>Use W, A, S, D keys to control the motors.</p>
    <p>W: Forward | S: Backward | A: Left | D: Right</p>
    <p>Current Action: <span id="currentAction">Stopped</span></p>
    <button onclick="fetch('/atk/on')">Attack</button>
</body>
</html>
)rawliteral";

// --------------------- Function Definitions ---------------------
// top hat

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

// Handle Root Path
void handleRoot() {
  server.send(200, "text/html", htmlContent);
}

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
  ledcWrite(LEDC_PIN1, duty); // Write duty cycle to LEDC channel
}

// LEDC Analog Write for Motor 2
void ledcAnalogWrite2(uint8_t channel, uint32_t value, uint32_t valueMax) {
  uint32_t duty = LEDC_RESOLUTION * min((int)value, (int)valueMax) / valueMax;
  ledcWrite(LEDC_PIN2, duty); // Write duty cycle to LEDC channel
}

// LEDC Analog Write for Servo
void ledcAnalogWrite3(uint8_t channel, uint32_t value, uint32_t valueMax) {
  uint32_t duty = LEDC_RESOLUTION_2 * min((int)value, (int)valueMax) / valueMax;
  ledcWrite(LEDC_PIN3, duty); // Write duty cycle to LEDC channel
}

// --------------------- Setup Function ---------------------

void setup() {

  Serial.begin(115200);
  // Initialize I2C master
  Wire.begin(SDA_PIN, SCL_PIN, 40000);
  Serial.println("ESP32-C3 I2C Master initialized");
  Serial.printf("SDA: %d, SCL: %d\n", SDA_PIN, SCL_PIN);

  delay(1000);

  Serial.begin(115200);
  
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

  
  // Attach Interrupts to Encoder Pins
  attachInterrupt(digitalPinToInterrupt(encoderPin_1), encoderISR_1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPin_2), encoderISR_2, RISING);
  
  // Initialize Wi-Fi in Access Point Mode
  WiFi.softAP(ssid, password);
  Serial.println("Access Point started: " + String(ssid));
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());

  // Define Server Routes
  server.on("/", handleRoot);
  
  // Define WASD Control Endpoints
  server.on("/forward", []() {
    // Set Motor Directions for Forward Movement
    digitalWrite(motorPin1_1, HIGH);
    digitalWrite(motorPin2_1, LOW);
    digitalWrite(motorPin1_2, HIGH);
    digitalWrite(motorPin2_2, LOW);
    server.send(200, "text/plain", "Moving Forward");
    Serial.println("Moving Forward");
    desiredSpeed_1 = 60;
    desiredSpeed_2 = 60;
    //attack_mode=1;
    mode=1;// manual
    hp_decrement+=1;
  });

  server.on("/backward", []() {
    // Set Motor Directions for Backward Movement
    digitalWrite(motorPin1_1, LOW);
    digitalWrite(motorPin2_1, HIGH);
    digitalWrite(motorPin1_2, LOW);
    digitalWrite(motorPin2_2, HIGH);
    server.send(200, "text/plain", "Moving Backward");
    Serial.println("Moving Backward");
    desiredSpeed_1 = 60;
    desiredSpeed_2 = 60;
    mode=1;// manual
    hp_decrement+=1;
  });

  server.on("/left", []() {
    // Set Motor Directions for Left Turn
    digitalWrite(motorPin1_1, LOW);
    digitalWrite(motorPin2_1, HIGH);
    digitalWrite(motorPin1_2, HIGH);
    digitalWrite(motorPin2_2, LOW);
    server.send(200, "text/plain", "Turning Left");
    Serial.println("Turning Left");
    desiredSpeed_1 = 50;
    desiredSpeed_2 = 50;
    mode=1;// manual
    hp_decrement+=1;
  });

  server.on("/right", []() {
    // Set Motor Directions for Right Turn
    digitalWrite(motorPin1_1, HIGH);
    digitalWrite(motorPin2_1, LOW);
    digitalWrite(motorPin1_2, LOW);
    digitalWrite(motorPin2_2, HIGH);
    server.send(200, "text/plain", "Turning Right");
    Serial.println("Turning Right");
    desiredSpeed_1 = 50;
    desiredSpeed_2 = 50;
    mode=1;// manual
    hp_decrement+=1;
  });

  server.on("/stop", []() {
    // Stop All Motors
    digitalWrite(motorPin1_1, LOW);
    digitalWrite(motorPin2_1, LOW);
    digitalWrite(motorPin1_2, LOW);
    digitalWrite(motorPin2_2, LOW);
    server.send(200, "text/plain", "Stopped");
    Serial.println("Stopped");
    desiredSpeed_1 = 0;
    desiredSpeed_2 = 0;
    mode=0;// goes back to auto
    attack_mode=0;
    hp_decrement+=1;
  });

  //launch attack
  server.on("/atk/on", []() {
    while(1){
      ledcAnalogWrite3(LEDC_CHANNEL_3, 10, 100);
      delay(500);
      ledcAnalogWrite3(LEDC_CHANNEL_3, 60, 100);
      delay(500);
    }
    Serial.println("attack");
    
  });

  // Start the Web Server
  server.begin();
  Serial.println("HTTP server started");

  ledcAttachChannel(LEDC_PIN1, 1000, LEDC_RESOLUTION_BITS, LEDC_CHANNEL_1);
  ledcAttachChannel(LEDC_PIN2, 1000, LEDC_RESOLUTION_BITS, LEDC_CHANNEL_2);
  ledcAttachChannel(LEDC_PIN3, 250, LEDC_RESOLUTION_BITS_2 , LEDC_CHANNEL_3);

  delay(1000);

  Serial.begin(115200);
  while (!Serial) {
    delay(10); // Wait for Serial Monitor to open
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
  Wire.begin(SDA_PIN, SCL_PIN); // Initialize I2C bus

  if (!lox1.begin(0x60)) { // Assign new I2C address to Sensor 1
    Serial.println("Failed to boot VL53L0X sensor 1!");
    while (1);
  }
  Serial.println("Sensor 1 initialized.");

  // Start Sensor 2
  digitalWrite(XSHUT2_PIN, HIGH);
  delay(10);

  if (!lox2.begin(0x61)) { // Assign new I2C address to Sensor 2
    Serial.println("Failed to boot VL53L0X sensor 2!");
    while (1);
  }
  Serial.println("Sensor 2 initialized.");


  // Start Sensor 3
  digitalWrite(XSHUT3_PIN, HIGH);
  delay(10);

  if (!lox3.begin(0x62)) { // Assign new I2C address to Sensor 3
    Serial.println("Failed to boot VL53L0X sensor 3!");
    while (1);
  }
  Serial.println("Sensor 3 initialized.");


  

}

// --------------------- Loop Function ---------------------

void loop() {
  currentMillis = millis();

  // webpage control
    server.handleClient();

  // Perform Speed Calculation and Proportional Control at Defined Intervals
  if (currentMillis - previousMillis >= timeInterval) {
    previousMillis = currentMillis;

    // Safely Read and Reset Encoder Ticks
    noInterrupts();
    unsigned long ticks1 = encoderTicks_1;
    unsigned long ticks2 = encoderTicks_2;
    encoderTicks_1 = 0;
    encoderTicks_2 = 0;
    interrupts();

    // Calculate Motor Speeds (counts per second)
    motorSpeed_1 = ticks1 * n/22;
    motorSpeed_2 = ticks2 * n/22;

    error_1 = desiredSpeed_1 - motorSpeed_1;  // Compute Error
    u_1 = Kp_1 * error_1;                     // Apply Proportional Gain
    pwmValue_1 += u_1;                         // Adjust PWM Value
    pwmValue_1 = constrain(pwmValue_1, 0, 127); // Ensure PWM is within 7-bit range

    // Proportional Control for Motor 2
    error_2 = desiredSpeed_2 - motorSpeed_2;  // Compute Error
    u_2 = Kp_2 * error_2;                     // Apply Proportional Gain
    pwmValue_2 += u_2;                         // Adjust PWM Value
    pwmValue_2 = constrain(pwmValue_2, 0, 127); // Ensure PWM is within 7-bit range

    // Proportional Control for Motor 1 new try, delete if breaking code



    // Apply Adjusted PWM Values to Motors
    ledcAnalogWrite1(LEDC_CHANNEL_1, pwmValue_1, LEDC_RESOLUTION);
    ledcAnalogWrite2(LEDC_CHANNEL_2, pwmValue_2, LEDC_RESOLUTION);

    // Debugging Information
    /*Serial.print("Motor 1 Speed: ");
    Serial.print(motorSpeed_1);
    Serial.print(" counts/sec, PWM: ");
    Serial.println(pwmValue_1);

    Serial.print("Motor 2 Speed: ");
    Serial.print(motorSpeed_2);
    Serial.print(" counts/sec, PWM: ");
    Serial.println(pwmValue_2);*/
  }


  VL53L0X_RangingMeasurementData_t measure;

  // Perform a ranging measurement for Sensor 1
  lox1.rangingTest(&measure, false);
  //Serial.print("left - ");
  if (measure.RangeStatus != 4) {  // 4 means out of range
    //Serial.print("Distance: ");
    l_dis=measure.RangeMilliMeter;
    //Serial.print(l_dis);
    //Serial.println(" mm");
  } else {
    l_dis=8000;
    //Serial.println("Out of range");
  }

  delay(10);

  // Perform a ranging measurement for Sensor 2
  lox2.rangingTest(&measure, false);
  //Serial.print("front - ");
  if (measure.RangeStatus != 4) {  // 4 means out of range
    //Serial.print("Distance: ");
    f_dis=measure.RangeMilliMeter;
    //Serial.print(f_dis);
    //Serial.println(" mm");
  } else {
    f_dis=8000;
    //Serial.println("Out of range");
  }

  delay(10); // Delay between measurements

    // Perform a ranging measurement for Sensor 3
  lox3.rangingTest(&measure, false);
  //Serial.print("right - ");
  if (measure.RangeStatus != 4) {  // 4 means out of range
    //Serial.print("Distance: ");
    r_dis=measure.RangeMilliMeter;
    //Serial.print(r_dis);
    //Serial.println(" mm");
  } else {
    r_dis=8000;
    //Serial.println("Out of range");
  }

  if (currentMillis - previousMillis_2 >= 500){
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
    /*ledcAnalogWrite3(LEDC_CHANNEL_3, 10, 100);
    delay(100);
    ledcAnalogWrite3(LEDC_CHANNEL_3, 60, 100);
    delay(100);*/

  }

  if (currentMillis - previousMillis_3 >= 500){
    previousMillis_3 = currentMillis;
    Serial.print("HP decrement is: ");
    Serial.println(hp_decrement);
    send_I2C_byte(hp_decrement);
    hp_remaining=receive_I2C_byte();       // loop back
    Serial.print("HP remaining");
    Serial.println(hp_remaining);
    hp_decrement=0;
    Serial.print("top hat error: ");
    Serial.println(tophat_error);
  }

  if(attack_mode==1){
    ledcAnalogWrite3(LEDC_CHANNEL_3, 10, 100);
    delay(500);
    ledcAnalogWrite3(LEDC_CHANNEL_3, 60, 100);
    delay(500);
  }

  else{
    ledcAnalogWrite3(LEDC_CHANNEL_3, 99, 100);
  }

  // read the value (HP) from top hat
  if(hp_remaining==0 && tophat_error==0){// when top hat is connected and HP is 0
    digitalWrite(motorPin1_1, LOW);
    digitalWrite(motorPin2_1, LOW);
    digitalWrite(motorPin1_2, LOW);
    digitalWrite(motorPin2_2, LOW);
  }

  else{
    if(mode==0)
    {
    if(f_dis<200 && f_dis>=100){// front close, turn left
      //Serial.println("front close Turning Left");
      digitalWrite(motorPin1_1, LOW);
      digitalWrite(motorPin2_1, HIGH);
      digitalWrite(motorPin1_2, HIGH);
      digitalWrite(motorPin2_2, LOW);
      desiredSpeed_1 = 30;
      desiredSpeed_2 = 30;
      delay(10);
    }   
    else if(f_dis<100 || r_dis<=60){
      //Serial.println("all too close backward");
      digitalWrite(motorPin1_1, LOW);
      digitalWrite(motorPin2_1, HIGH);
      digitalWrite(motorPin1_2, LOW);
      digitalWrite(motorPin2_2, HIGH);
      desiredSpeed_1 = 40;
      desiredSpeed_2 = 40;
      delay(500);
      digitalWrite(motorPin1_1, LOW);
      digitalWrite(motorPin2_1, HIGH);
      digitalWrite(motorPin1_2, HIGH);
      digitalWrite(motorPin2_2, LOW);
      desiredSpeed_1 = 25;
      desiredSpeed_2 = 25;
      delay(50);
      digitalWrite(motorPin1_1, HIGH);
      digitalWrite(motorPin2_1, LOW);
      digitalWrite(motorPin1_2, HIGH);
      digitalWrite(motorPin2_2, LOW);
      desiredSpeed_1 = 50;
      desiredSpeed_2 = 50;
      delay(500);
    }
    else if(r_dis<100 && r_dis>=60){// right close, turn left
      //Serial.println("right close Turning Left");
      digitalWrite(motorPin1_1, LOW);
      digitalWrite(motorPin2_1, HIGH);
      digitalWrite(motorPin1_2, HIGH);
      digitalWrite(motorPin2_2, LOW);
      desiredSpeed_1 = 25;
      desiredSpeed_2 = 25;
      delay(10);
    }
    else if(r_dis>200){// right far, turn right
      //Serial.println("right far Turning Right");
      digitalWrite(motorPin1_1, HIGH);
      digitalWrite(motorPin2_1, LOW);
      digitalWrite(motorPin1_2, LOW);
      digitalWrite(motorPin2_2, HIGH);
      desiredSpeed_1 = 25;
      desiredSpeed_2 = 25;
      delay(10);
    }
    else{// go straight 
      //Serial.println("all good Moving Forward");
      digitalWrite(motorPin1_1, HIGH);
      digitalWrite(motorPin2_1, LOW);
      digitalWrite(motorPin1_2, HIGH);
      digitalWrite(motorPin2_2, LOW);
      desiredSpeed_1 = 50;
      desiredSpeed_2 = 50;
      delay(10);
    }
  }

  }



  delay(10); // Delay between measurements

    /*ledcAnalogWrite3(LEDC_CHANNEL_3, 10, 100);
    delay(100);
    ledcAnalogWrite3(LEDC_CHANNEL_3, 60, 100);
    delay(100);*/




}
