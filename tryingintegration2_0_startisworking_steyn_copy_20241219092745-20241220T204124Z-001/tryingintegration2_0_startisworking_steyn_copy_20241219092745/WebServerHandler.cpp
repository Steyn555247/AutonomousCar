#include "WebServerHandler.h"

// Initialize the web server on port 80
WebServer server(80);

// Define the HTML content before its usage
const char* index_html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Robot Control</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            text-align: center;
            background-color: #f0f0f0;
            margin: 0;
            padding: 20px;
        }

        h1 {
            color: #333;
        }

        button {
            font-size: 1.2em;
            padding: 10px 20px;
            margin: 10px;
            border: none;
            border-radius: 5px;
            cursor: pointer;
        }

        #wallFollowing {
            background-color: #4CAF50; /* Green */
            color: white;
        }

        #wallFollowingAttack {
            background-color: #FF5733; /* Red */
            color: white;
        }

        #servoSwing {
            background-color: #007BFF; /* Blue */
            color: white;
        }

        button:hover {
            opacity: 0.9;
        }
    </style>
</head>
<body>
    <h1>Robot Control Panel</h1>
    <button id="wallFollowing">Wall Following</button>
    <button id="wallFollowingAttack">Wall Following + Attack</button>
    <button id="servoSwing">Swing Servo Arm</button>

      <h1>Group22_Lab 4.2_Motors Control</h1>
    
    <!-- WASD Controls -->
    <h2>WASD Controls</h2>
    <p>Use W, A, S, D keys to control the motors.</p>
    <p>W: Forward | S: Backward | A: Left | D: Right</p>
    <p>Current Action: <span id="currentAction">Stopped</span></p>
    <button onclick="fetch('/atk/on')">Attack</button>

        <!-- Vive container -->
   <div id="viveContainer">
    <h2>Vive Coordinates</h2>
    <p id="vive1Coords">Vive1: X: --, Y: --</p>
    <p id="vive2Coords">Vive2: X: --, Y: --</p>
</div>

    <script>
        // Event listeners for buttons
        document.getElementById('wallFollowing').addEventListener('click', function() {
            fetch('/wall_following').then(response => response.text()).then(data => {
                console.log(data);
            });
        });

        document.getElementById('wallFollowingAttack').addEventListener('click', function() {
            fetch('/wall_following_attack').then(response => response.text()).then(data => {
                console.log(data);
            });
        });

        document.getElementById('servoSwing').addEventListener('click', function() {
            fetch('/servo_swing').then(response => response.text()).then(data => {
                console.log(data);
            });
        });

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
                if (['w', 'a', 's', 'd','f'].includes(e.key.toLowerCase()) && !keysPressed[e.key.toLowerCase()]) {
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

                 // Function to fetch Vive data
    function fetchViveData() {
        fetch('/vive')
            .then(response => response.json())
            .then(data => {
                const vive1Coords = document.getElementById('vive1Coords');
                const vive2Coords = document.getElementById('vive2Coords');
                
                if (data.vive1.x >= 0 && data.vive1.y >= 0) {
                    vive1Coords.textContent = `Vive1: X: ${data.vive1.x.toFixed(2)}, Y: ${data.vive1.y.toFixed(2)}`;
                } else {
                    vive1Coords.textContent = `Vive1: Invalid Data`;
                }
                
                if (data.vive2.x >= 0 && data.vive2.y >= 0) {
                    vive2Coords.textContent = `Vive2: X: ${data.vive2.x.toFixed(2)}, Y: ${data.vive2.y.toFixed(2)}`;
                } else {
                    vive2Coords.textContent = `Vive2: Invalid Data`;
                }
            })
            .catch(error => console.error('Error fetching Vive data:', error));
    }

    // Fetch Vive data every second
    setInterval(fetchViveData, 1000);
    </script>
</body>
</html>
)rawliteral";

// Function to handle the root URL
void handleRoot() {
    server.send(200, "text/html", index_html);
}



// Function to handle Vive data
// Function to handle Vive data
// Function to handle Vive data
void handleViveData() {
    float x1, y1, x2, y2;
    bool success1 = readVive1Position(x1, y1); // Attempt to read from Vive1
    bool success2 = readVive2Position(x2, y2); // Attempt to read from Vive2

    // Debugging: Log the success status and coordinates
    Serial.print("Vive1 Read Success: ");
    Serial.println(success1);
    if (success1) {
        Serial.printf("Vive1 Coordinates: X=%.2f, Y=%.2f\n", x1, y1);
    } else {
        Serial.println("Vive1: Failed to read valid data.");
    }

    Serial.print("Vive2 Read Success: ");
    Serial.println(success2);
    if (success2) {
        Serial.printf("Vive2 Coordinates: X=%.2f, Y=%.2f\n", x2, y2);
    } else {
        Serial.println("Vive2: Failed to read valid data.");
    }

    // Create JSON document to hold sensor data
    StaticJsonDocument<200> jsonDoc;

    // Populate JSON for Vive1
    if (success1) {
        jsonDoc["vive1"]["x"] = x1;
        jsonDoc["vive1"]["y"] = y1;
    } else {
        jsonDoc["vive1"]["x"] = -1;
        jsonDoc["vive1"]["y"] = -1;
    }

    // Populate JSON for Vive2
    if (success2) {
        jsonDoc["vive2"]["x"] = x2;
        jsonDoc["vive2"]["y"] = y2;
    } else {
        jsonDoc["vive2"]["x"] = -1;
        jsonDoc["vive2"]["y"] = -1;
    }

    // Serialize JSON to a string
    String response;
    serializeJson(jsonDoc, response);

    // Debugging: Log the JSON response being sent
    Serial.println("Sending Vive Data JSON:");
    Serial.println(response);

    // Send JSON response to the client with CORS header
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", response);
}




// Function to initialize the web server
void initWebServer() {
    // Define the routes
    server.on("/", HTTP_GET, handleRoot);
    // server.on("/data", HTTP_POST, handleData);
    server.on("/vive", HTTP_GET, handleViveData);
    server.on("/wall_following", []() {
    handleWallFollowing();
    server.send(200, "text/plain", "Wall Following mode activated.");
    Serial.println("Wall Following mode activated.");
});

server.on("/wall_following_attack", []() {
    handleWallFollowingAttack();
    server.send(200, "text/plain", "Wall Following Attack mode activated.");
    Serial.println("Wall Following Attack mode activated.");
});

server.on("/servo_swing", []() {
    handleServoSwing();
    server.send(200, "text/plain", "ServoSwing activated.");
    Serial.println("ServoSwing");
    delay(500);
});


  // Define WASD Control Endpoints
  server.on("/forward", []() {
    driveForward();
  });

  server.on("/backward", []() {
driveBackward();
  });

  server.on("/left", []() {
 turnLeft();
  });

  server.on("/right", []() {
turnRight();
  });

  server.on("/stop", []() {
    stopMotors();
  });

    // Start the server
    server.begin();
    Serial.println("Web server started");
}

// Function to handle web server client
void handleWebRequests() {
    server.handleClient();
}
