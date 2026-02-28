#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
#include <PubSubClient.h>
#include "RadarSensor.h"
#include "secrets.h"

// WiFi credentials (defined in secrets.h)
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// MQTT broker settings (defined in secrets.h)
const char* mqtt_broker = MQTT_BROKER;
const char* mqtt_user = MQTT_USER;
const char* mqtt_password = MQTT_PASSWORD;
const int mqtt_port = MQTT_PORT;

// MQTT Topics
const char* mqtt_topic_distance = "home/radar/distance";
const char* mqtt_topic_angle = "home/radar/angle";
const char* mqtt_topic_speed = "home/radar/speed";
const char* mqtt_topic_detected = "home/radar/detected";
unsigned long lastMQTTPublish = 0;
const unsigned long MQTT_PUBLISH_INTERVAL = 2000;  // Publish every 2 seconds

// Hardware pins
const int buzzerPin = 7;

// Radar sensor
RadarSensor radar(Serial1);

// Web server
WebServer server(80);

// WiFi and MQTT clients
WiFiClient espClient;
PubSubClient client(espClient);

// System state variables
struct SystemConfig {
  bool armed = true;
  int detectionDistance = 1000;  // mm (1 meter default)
  int alarmDuration = 10000;     // ms (10 seconds default)
  bool alarmActive = false;
  unsigned long alarmStartTime = 0;
  bool systemEnabled = true;
};

SystemConfig config;
RadarTarget currentTarget;
bool targetDetected = false;
unsigned long lastBlinkTime = 0;
bool blinkState = false;

// =====================
// NeoPixel (Built-in RGB)
// =====================
#define RGB_PIN 8        // Built-in LED pin on ESP32-C6-Zero
#define NUM_PIXELS 1

Adafruit_NeoPixel pixel(NUM_PIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);

#define LED_BRIGHTNESS 50   // 0–255 (keep low, it's very bright!)

// Function prototypes
void setupWebServer();
void updateLEDs();
void updateBuzzer();
void handleAlarm();
String getMainHTML();
void handleGetStatus();
void handleSetConfig();
void handleGetConfig();
void handleArmDisarm();
void handleStopAlarm();
void handleGetRadarData();
void connectToMQTT();
void publishDiscovery();
void publishRadarData();

void setup() {
  Serial.begin(115200);
  Serial1.begin(256000, SERIAL_8N1, 17, 16);
  
  // Initialize radar
  radar.begin();
  Serial.println("Radar Sensor Started");
  
  // Initialize pins
  pinMode(buzzerPin, OUTPUT);

  // Initialize NeoPixel
  pixel.begin();
  pixel.setBrightness(LED_BRIGHTNESS);
  pixel.clear();
  pixel.show();
  
  // Initialize WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());
  
  // Setup MQTT
  client.setServer(mqtt_broker, mqtt_port);
  connectToMQTT();

  // Publish discovery messages
  publishDiscovery();

  // Setup web server routes
  setupWebServer();
  server.begin();
  
  // Initial LED state
  updateLEDs();
}

void loop() {
  server.handleClient();

  if (!client.connected()) {
    connectToMQTT();
  }
  
  // Handle radar updates
  if (radar.update()) {
    currentTarget = radar.getTarget();
    targetDetected = (currentTarget.distance > 0 && 
                     currentTarget.distance <= config.detectionDistance);
    
    Serial.print("Distance: "); Serial.print(currentTarget.distance);
    Serial.print("mm, Angle: "); Serial.print(currentTarget.angle);
    Serial.print("°, Target: "); Serial.println(targetDetected ? "YES" : "NO");
  

    // Publish to MQTT only if connected
    if (client.connected()&& (millis() - lastMQTTPublish >= MQTT_PUBLISH_INTERVAL)) {
      Serial.println("Publishing MQTT data");
      publishRadarData();
      Serial.println("MQTT published");
      lastMQTTPublish = millis();
    }
  }
  
  // Handle alarm logic
  handleAlarm();
  
  // Update LEDs and buzzer
  updateLEDs();
  updateBuzzer();
  
  delay(50);
}
void connectToMQTT() {
  int attempts = 0;
  while (!client.connected() && attempts < 10) {
    Serial.print("Attempting MQTT connection...");
    
    if (client.connect("ESP32-Radar", mqtt_user, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
      attempts++;
    }
  }
}

void publishDiscovery() {
  DynamicJsonDocument doc(1024);
  char buffer[1024];
  
  // Device info
  doc["device"]["identifiers"][0] = "esp32_radar";
  doc["device"]["name"] = "Radar Sensor";
  doc["device"]["manufacturer"] = "ESP32-C6";
  
  // Distance Sensor
  doc.clear();
  doc["name"] = "Radar Distance";
  doc["unique_id"] = "radar_distance";
  doc["state_topic"] = "home/radar/distance";
  doc["unit_of_measurement"] = "mm";
  doc["icon"] = "mdi:ruler";
  doc["device"]["identifiers"][0] = "esp32_radar";
  serializeJson(doc, buffer);
  client.publish("homeassistant/sensor/radar/distance/config", buffer, true);
  
  // Angle Sensor
  doc.clear();
  doc["name"] = "Radar Angle";
  doc["unique_id"] = "radar_angle";
  doc["state_topic"] = "home/radar/angle";
  doc["unit_of_measurement"] = "°";
  doc["icon"] = "mdi:compass";
  doc["device"]["identifiers"][0] = "esp32_radar";
  serializeJson(doc, buffer);
  client.publish("homeassistant/sensor/radar/angle/config", buffer, true);
  
  // Speed Sensor
  doc.clear();
  doc["name"] = "Radar Speed";
  doc["unique_id"] = "radar_speed";
  doc["state_topic"] = "home/radar/speed";
  doc["unit_of_measurement"] = "cm/s";
  doc["icon"] = "mdi:speedometer";
  doc["device"]["identifiers"][0] = "esp32_radar";
  serializeJson(doc, buffer);
  client.publish("homeassistant/sensor/radar/speed/config", buffer, true);
  
  // Motion Detection Binary Sensor
  doc.clear();
  doc["name"] = "Radar Motion Detected";
  doc["unique_id"] = "radar_detected";
  doc["state_topic"] = "home/radar/detected";
  doc["payload_on"] = "on";
  doc["payload_off"] = "off";
  doc["device_class"] = "motion";
  doc["device"]["identifiers"][0] = "esp32_radar";
  serializeJson(doc, buffer);
  client.publish("homeassistant/binary_sensor/radar/detected/config", buffer, true);
  
  Serial.println("Discovery messages published");
}

/*void publishRadarData() {
  char buffer[50];
  
  // Publish distance
  snprintf(buffer, sizeof(buffer), "%d", currentTarget.distance);
  client.publish(mqtt_topic_distance, buffer);
  
  // Publish angle
  snprintf(buffer, sizeof(buffer), "%d", currentTarget.angle);
  client.publish(mqtt_topic_angle, buffer);

    // Publish speed
  snprintf(buffer, sizeof(buffer), "%d", currentTarget.speed);
  client.publish(mqtt_topic_speed, buffer);
  
  // Publish detection status
  client.publish(mqtt_topic_detected, targetDetected ? "true" : "false");
  
  Serial.printf("MQTT Published - Distance: %d, Angle: %d, Speed: %d, Detected: %s\n", 
                currentTarget.distance, currentTarget.angle, currentTarget.speed, 
                targetDetected ? "true" : "false");
}
 */

 void publishRadarData() {
  // Create simple string buffers
  char distStr[20];
  char angStr[20];
  char spdStr[20];
  char detStr[10];
  
  // Convert to strings safely
  int dist = (int)currentTarget.distance;
  int ang = (int)currentTarget.angle;
  int spd = (int)currentTarget.speed;
  
  itoa(dist, distStr, 10);
  itoa(ang, angStr, 10);
  itoa(spd, spdStr, 10);
  strcpy(detStr, targetDetected ? "on" : "off");
  
  // Publish with error checking
  Serial.print("Publishing distance: ");
  Serial.println(distStr);
  client.publish("home/radar/distance", distStr);
  delay(10);
  
  Serial.print("Publishing angle: ");
  Serial.println(angStr);
  client.publish("home/radar/angle", angStr);
  delay(10);
  
  Serial.print("Publishing speed: ");
  Serial.println(spdStr);
  client.publish("home/radar/speed", spdStr);
  delay(10);
  
  Serial.print("Publishing detected: ");
  Serial.println(detStr);
  client.publish("home/radar/detected", detStr);
  delay(10);
  
  Serial.println("All MQTT data published");
}

/// @brief //////////////////
void handleAlarm() {
  if (!config.systemEnabled || !config.armed) {
    config.alarmActive = false;
    return;
  }
  
  if (targetDetected && !config.alarmActive) {
    // Start alarm
    config.alarmActive = true;
    config.alarmStartTime = millis();
    Serial.println("ALARM TRIGGERED!");
  }
  
  if (config.alarmActive) {
    // Check if alarm duration exceeded
    if (millis() - config.alarmStartTime >= config.alarmDuration) {
      config.alarmActive = false;
      Serial.println("Alarm timeout - stopping");
    }
  }
}

void setColor(uint8_t r, uint8_t g, uint8_t b) {
  pixel.setPixelColor(0, pixel.Color(r, g, b));
  pixel.show();
}

void updateLEDs() {

  if (!config.systemEnabled) {
    setColor(0, 0, 0);   // OFF
    return;
  }

  if (!config.armed) {
    setColor(0, 0, 255); // Blue (Disarmed)
    return;
  }

  if (config.alarmActive) {
    unsigned long currentTime = millis();
    if (currentTime - lastBlinkTime >= 250) {
      blinkState = !blinkState;
      lastBlinkTime = currentTime;

      if (blinkState)
        setColor(0, 255, 0);   // Red blink
      else
        setColor(0, 0, 0);
    }
  }
  else if (targetDetected) {
    setColor(0, 255, 0); // Solid red (target)
  }
  else {
    setColor(255, 0, 0); // Green (armed & clear)
  }
}

void updateBuzzer() {
  if (config.alarmActive && config.systemEnabled) {
    // Buzzer blinks at same rate as red LED
    digitalWrite(buzzerPin, blinkState ? HIGH : LOW);
  } else {
    digitalWrite(buzzerPin, LOW);
  }
}

void setupWebServer() {
  // Serve main HTML page
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", getMainHTML());
  });
  
  // API endpoints
  server.on("/api/status", HTTP_GET, handleGetStatus);
  server.on("/api/config", HTTP_POST, handleSetConfig);
  server.on("/api/config", HTTP_GET, handleGetConfig);
  server.on("/api/arm", HTTP_POST, handleArmDisarm);
  server.on("/api/stop-alarm", HTTP_POST, handleStopAlarm);
  server.on("/api/radar-data", HTTP_GET, handleGetRadarData);
  
  // Enable CORS
  server.enableCORS(true);
}

void handleGetStatus() {
  StaticJsonDocument<300> doc;
  doc["armed"] = config.armed;
  doc["alarmActive"] = config.alarmActive;
  doc["targetDetected"] = targetDetected;
  doc["systemEnabled"] = config.systemEnabled;
  doc["detectionDistance"] = config.detectionDistance;
  doc["alarmDuration"] = config.alarmDuration;
  doc["uptime"] = millis();
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleGetConfig() {
  StaticJsonDocument<200> doc;
  doc["detectionDistance"] = config.detectionDistance;
  doc["alarmDuration"] = config.alarmDuration;
  doc["systemEnabled"] = config.systemEnabled;
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleSetConfig() {
  if (server.hasArg("plain")) {
    StaticJsonDocument<200> doc;
    deserializeJson(doc, server.arg("plain"));
    
    if (doc.containsKey("detectionDistance")) {
      config.detectionDistance = doc["detectionDistance"];
    }
    if (doc.containsKey("alarmDuration")) {
      config.alarmDuration = doc["alarmDuration"];
    }
    if (doc.containsKey("systemEnabled")) {
      config.systemEnabled = doc["systemEnabled"];
    }
    
    server.send(200, "application/json", "{\"success\":true}");
  } else {
    server.send(400, "application/json", "{\"error\":\"Invalid request\"}");
  }
}

void handleArmDisarm() {
  if (server.hasArg("plain")) {
    StaticJsonDocument<100> doc;
    deserializeJson(doc, server.arg("plain"));
    
    if (doc.containsKey("armed")) {
      config.armed = doc["armed"];
      config.alarmActive = false; // Stop any active alarm
      server.send(200, "application/json", "{\"success\":true}");
    } else {
      server.send(400, "application/json", "{\"error\":\"Missing armed parameter\"}");
    }
  } else {
    server.send(400, "application/json", "{\"error\":\"Invalid request\"}");
  }
}

void handleStopAlarm() {
  config.alarmActive = false;
  server.send(200, "application/json", "{\"success\":true}");
}

void handleGetRadarData() {
  StaticJsonDocument<200> doc;
  doc["distance"] = currentTarget.distance;
  doc["angle"] = currentTarget.angle;
  doc["x"] = currentTarget.x;
  doc["y"] = currentTarget.y;
  doc["speed"] = currentTarget.speed;
  doc["detected"] = targetDetected;
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

String getMainHTML() {
  return R"rawstring(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Radar Security System</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { 
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #1e3c72, #2a5298);
            color: white;
            min-height: 100vh;
            padding: 20px;
        }
        .container { 
            max-width: 1200px; 
            margin: 0 auto;
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 20px;
            height: calc(100vh - 40px);
        }
        .panel {
            background: rgba(255,255,255,0.1);
            backdrop-filter: blur(10px);
            border-radius: 15px;
            padding: 20px;
            border: 1px solid rgba(255,255,255,0.2);
        }
        .radar-panel {
            display: flex;
            flex-direction: column;
            align-items: center;
        }
        .status { 
            text-align: center; 
            margin-bottom: 20px;
        }
        .status h1 { 
            font-size: 2.5em; 
            margin-bottom: 10px;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
        }
        .status-indicator {
            display: inline-block;
            width: 20px;
            height: 20px;
            border-radius: 50%;
            margin-left: 10px;
            animation: pulse 2s infinite;
        }
        .armed { background: #4CAF50; }
        .disarmed { background: #FF9800; }
        .alarm { background: #F44336; animation: blink 0.5s infinite; }
        @keyframes pulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.5; } }
        @keyframes blink { 0%, 100% { opacity: 1; } 50% { opacity: 0; } }
        
        .radar-display {
            width: 400px;
            height: 400px;
            position: relative;
            margin: 20px 0;
        }
        .radar-svg {
            width: 100%;
            height: 100%;
            background: radial-gradient(circle, rgba(0,255,0,0.1) 0%, rgba(0,100,0,0.05) 100%);
            border-radius: 50%;
            border: 2px solid #00ff00;
        }
        .controls {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 15px;
            margin-top: 20px;
        }
        .control-group {
            background: rgba(255,255,255,0.05);
            padding: 15px;
            border-radius: 10px;
        }
        .control-group label {
            display: block;
            margin-bottom: 5px;
            font-weight: bold;
        }
        .control-group input, .control-group select {
            width: 100%;
            padding: 8px;
            border: none;
            border-radius: 5px;
            background: rgba(255,255,255,0.1);
            color: white;
            margin-bottom: 10px;
        }
        .control-group input::placeholder {
            color: rgba(255,255,255,0.7);
        }
        .btn {
            padding: 12px 24px;
            border: none;
            border-radius: 8px;
            cursor: pointer;
            font-weight: bold;
            text-transform: uppercase;
            transition: all 0.3s ease;
            margin: 5px;
        }
        .btn-primary { background: #2196F3; color: white; }
        .btn-success { background: #4CAF50; color: white; }
        .btn-danger { background: #F44336; color: white; }
        .btn-warning { background: #FF9800; color: white; }
        .btn:hover { transform: translateY(-2px); box-shadow: 0 4px 12px rgba(0,0,0,0.3); }
        .info-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 10px;
            margin-top: 20px;
        }
        .info-item {
            background: rgba(255,255,255,0.05);
            padding: 10px;
            border-radius: 8px;
            text-align: center;
        }
        .info-item .label { font-size: 0.9em; opacity: 0.8; }
        .info-item .value { font-size: 1.2em; font-weight: bold; margin-top: 5px; }
        @media (max-width: 768px) {
            .container { grid-template-columns: 1fr; }
            .radar-display { width: 300px; height: 300px; }
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="panel">
            <div class="status">
                <h1>Radar Security <span id="statusIndicator" class="status-indicator armed"></span></h1>
                <p id="statusText">System Armed</p>
            </div>
            
            <div class="controls">
                <div class="control-group">
                    <label>Detection Distance (mm)</label>
                    <input type="number" id="detectionDistance" value="1000" min="100" max="8000">
                    <button class="btn btn-primary" onclick="updateConfig()">Update</button>
                </div>
                
                <div class="control-group">
                    <label>Alarm Duration (seconds)</label>
                    <input type="number" id="alarmDuration" value="10" min="1" max="300">
                    <button class="btn btn-primary" onclick="updateConfig()">Update</button>
                </div>
                
                <div class="control-group">
                    <label>System Control</label>
                    <button class="btn btn-success" id="armBtn" onclick="armSystem()">ARM</button>
                    <button class="btn btn-warning" id="disarmBtn" onclick="disarmSystem()">DISARM</button>
                </div>
                
                <div class="control-group">
                    <label>Alarm Control</label>
                    <button class="btn btn-danger" onclick="stopAlarm()">STOP ALARM</button>
                    <button class="btn btn-primary" onclick="toggleSystem()">ENABLE/DISABLE</button>
                </div>
            </div>
            
            <div class="info-grid">
                <div class="info-item">
                    <div class="label">Distance</div>
                    <div class="value" id="targetDistance">-- mm</div>
                </div>
                <div class="info-item">
                    <div class="label">Angle</div>
                    <div class="value" id="targetAngle">--deg</div>
                </div>
                <div class="info-item">
                    <div class="label">Speed</div>
                    <div class="value" id="targetSpeed">-- cm/s</div>
                </div>
                <div class="info-item">
                    <div class="label">Status</div>
                    <div class="value" id="detectionStatus">Clear</div>
                </div>
            </div>
        </div>
        
        <div class="panel radar-panel">
            <h2>Radar Display</h2>
            <div class="radar-display">
                <svg class="radar-svg" viewBox="0 0 400 400">
                    <!-- Radar grid -->
                    <defs>
                        <pattern id="radarGrid" width="40" height="40" patternUnits="userSpaceOnUse">
                            <path d="M 40 0 L 0 0 0 40" fill="none" stroke="rgba(0,255,0,0.2)" stroke-width="1"/>
                        </pattern>
                    </defs>
                    <rect width="400" height="400" fill="url(#radarGrid)"/>
                    
                    <!-- Distance circles -->
                    <circle cx="200" cy="400" r="50" fill="none" stroke="rgba(0,255,0,0.3)" stroke-width="1"/>
                    <circle cx="200" cy="400" r="100" fill="none" stroke="rgba(0,255,0,0.3)" stroke-width="1"/>
                    <circle cx="200" cy="400" r="150" fill="none" stroke="rgba(0,255,0,0.3)" stroke-width="1"/>
                    <circle cx="200" cy="400" r="200" fill="none" stroke="rgba(0,255,0,0.3)" stroke-width="1"/>
                    
                    <!-- 120-degree arc -->
                    <path d="M 27 273 A 200 200 0 0 1 373 273" fill="none" stroke="#00ff00" stroke-width="2"/>
                    
                    <!-- Center lines -->
                    <line x1="200" y1="400" x2="200" y2="200" stroke="rgba(0,255,0,0.5)" stroke-width="1"/>
                    <line x1="200" y1="400" x2="27" y2="273" stroke="rgba(0,255,0,0.5)" stroke-width="1"/>
                    <line x1="200" y1="400" x2="373" y2="273" stroke="rgba(0,255,0,0.5)" stroke-width="1"/>
                    
                    <!-- Target dot -->
                    <circle id="targetDot" cx="200" cy="400" r="0" fill="#ff0000" stroke="#ffffff" stroke-width="2" opacity="0">
                        <animate attributeName="r" values="5;8;5" dur="1s" repeatCount="indefinite"/>
                    </circle>
                    
                    <!-- Radar sweep -->
                    <line id="radarSweep" x1="200" y1="400" x2="200" y2="200" stroke="#00ff00" stroke-width="2" opacity="0.7">
                        <animateTransform attributeName="transform" attributeType="XML" type="rotate" 
                                                        values="-55 200 400;55 200 400;-55 200 400" dur="3s" repeatCount="indefinite"/>
                    </line>
                </svg>
            </div>
            
            <div style="text-align: center; margin-top: 10px;">
                <small>Detection Range: 8m | Field of View: 120deg </small>
            </div>
        </div>
    </div>

    <script>
        let systemConfig = {
            armed: true,
            systemEnabled: true,
            detectionDistance: 1000,
            alarmDuration: 10000
        };
        
        function updateStatus() {
            fetch('/api/status')
                .then(response => response.json())
                .then(data => {
                    const indicator = document.getElementById('statusIndicator');
                    const statusText = document.getElementById('statusText');
                    
                    if (!data.systemEnabled) {
                        indicator.className = 'status-indicator disarmed';
                        statusText.textContent = 'System Disabled';
                    } else if (data.alarmActive) {
                        indicator.className = 'status-indicator alarm';
                        statusText.textContent = 'ALARM ACTIVE!';
                    } else if (data.armed) {
                        indicator.className = 'status-indicator armed';
                        statusText.textContent = 'System Armed';
                    } else {
                        indicator.className = 'status-indicator disarmed';
                        statusText.textContent = 'System Disarmed';
                    }
                    
                    systemConfig = data;
                });
        }
        
        function updateRadarData() {
            fetch('/api/radar-data')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('targetDistance').textContent = data.distance + ' mm';
                    document.getElementById('targetAngle').textContent = data.angle + '°';
                    document.getElementById('targetSpeed').textContent = data.speed + ' cm/s';
                    document.getElementById('detectionStatus').textContent = data.detected ? 'TARGET' : 'Clear';
                    
                    updateRadarDisplay(data);
                });
        }
        
        function updateRadarDisplay(data) {
            const targetDot = document.getElementById('targetDot');
            
            if (data.detected && data.distance > 0 && data.distance <= 8000) {
                // Convert distance to radar display coordinates
                const maxDistance = 8000; // 8 meters in mm
                const radarRadius = 200; // pixels
                const scale = radarRadius / maxDistance;
                
                // Radar center is at (200, 400), pointing upward (negative Y)
                // Convert angle (0 = straight ahead/up, -60 = left, +60 = right)
                const angleRad = (data.angle || 0) * Math.PI / 180;
                const distance = Math.min(data.distance, maxDistance);
                
                // Calculate position with correct orientation
                const x = 200 + (distance * scale * Math.sin(angleRad));
                const y = 400 - (distance * scale * Math.cos(angleRad));
                
                console.log(`Distance: ${distance}mm, Angle: ${data.angle}°, Position: (${x}, ${y})`);
                
                targetDot.setAttribute('cx', x);
                targetDot.setAttribute('cy', y);
                targetDot.style.opacity = '1';
            } else {
                targetDot.style.opacity = '0';
            }
        }
        
        function updateConfig() {
            const distance = document.getElementById('detectionDistance').value;
            const duration = document.getElementById('alarmDuration').value * 1000; // Convert to ms
            
            fetch('/api/config', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    detectionDistance: parseInt(distance),
                    alarmDuration: parseInt(duration)
                })
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    alert('Configuration updated successfully!');
                }
            });
        }
        
        function armSystem() {
            fetch('/api/arm', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ armed: true })
            });
        }
        
        function disarmSystem() {
            fetch('/api/arm', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ armed: false })
            });
        }
        
        function stopAlarm() {
            fetch('/api/stop-alarm', { method: 'POST' });
        }
        
        function toggleSystem() {
            fetch('/api/config', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ systemEnabled: !systemConfig.systemEnabled })
            });
        }
        
        // Update data every 500ms
        setInterval(() => {
            updateStatus();
            updateRadarData();
        }, 100);
        
        // Initial load
        updateStatus();
        updateRadarData();
    </script>
</body>
</html>
)rawstring";
}