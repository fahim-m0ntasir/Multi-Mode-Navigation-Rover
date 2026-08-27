#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <MPU6050_light.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// --- Core assignments ---
#define PRO_CPU 0
#define APP_CPU 1

// --- WiFi Credentials ---
const char* ssid = "Radioactive Zone";
const char* password = "Chern0by1";

// --- Pin Definitions ---
// Motors
#define IN1 25
#define IN2 26
#define ENA 23
#define IN3 14
#define IN4 27
#define ENB 17

// Line-Follow IR
#define IR_LF_L 5
#define IR_LF_C 15
#define IR_LF_R 4

// Object-Follow IR
#define IR_OF_L 33
#define IR_OF_R 32

// Ultrasonic
#define TRIG_PIN 18
#define ECHO_PIN 19

// Radar Servo
#define SERVO_PIN 16

// Battery
#define BAT_PIN 34

// --- Shared State Variables (Protected by Mutex) ---
SemaphoreHandle_t stateMutex;

struct RobotState {
    int mode = 0; // 0=Idle, 1=Manual, 2=Obs Avoid, 3=Line Follow, 4=Object Follow
    bool running = false;
    bool safeStop = false;
    int speed = 200; // General speed
    
    // Commands
    String manualCmd = "STOP_MOVE";
    bool radarOn = false;
    bool manualRadar = false;   // true = manual servo angle control
    int  manualRadarAngle = 90; // target angle 30-150
    int  manualDist = -1;       // distance reading at manual angle

    // Telemetry data
    float tilt = 0.0;
    float accMag = 0.0;
    float temp = 0.0;
    float yaw = 0.0;
    float gx = 0.0;
    float gy = 0.0;
    float gz = 0.0;
    float battery = 0.0;
    
    // Radar distances
    int distL = -1;
    int distC = -1;
    int distR = -1;
    int distCurr = -1; 
    
    // Motor state (for UI direction indicator)
    int motL = 0;  // -255 to 255
    int motR = 0;  // -255 to 255
    
    // Sensor states
    uint8_t irLfL = 0;
    uint8_t irLfC = 0;
    uint8_t irLfR = 0;
    uint8_t irOfL = 0;
    uint8_t irOfR = 0;
} sharedState;

// --- WebServer & WebSocket ---
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// --- Hardware Objects ---
Servo radarServo;
MPU6050 mpu(Wire);

// --- Task Handles ---
TaskHandle_t TaskNetwork;
TaskHandle_t TaskControl;

// --- Forward Declarations ---
void controlLoop(void *pvParameters);
void networkLoop(void *pvParameters);
void setupHardware();
void motorDrive(int speedL, int speedR);
void motorStop();
void processWebSocketMessage(void *arg, uint8_t *data, size_t len);

void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // disable brownout
    Serial.begin(115200);
    
    stateMutex = xSemaphoreCreateMutex();
    if (stateMutex == NULL) {
        Serial.println("Mutex creation failed");
        while(1);
    }
    
    setupHardware();
    
    // Pin tasks
    xTaskCreatePinnedToCore(networkLoop, "TaskNetwork", 8192, NULL, 1, &TaskNetwork, PRO_CPU);
    xTaskCreatePinnedToCore(controlLoop, "TaskControl", 8192, NULL, 1, &TaskControl, APP_CPU);
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}

void setupHardware() {
    // Motor Pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    
    // ledc for ENA, ENB
    ledcSetup(5, 1000, 8); // Ch 5, 1000Hz, 8-bit
    ledcAttachPin(ENA, 5);
    ledcSetup(6, 1000, 8); // Ch 6, 1000Hz, 8-bit
    ledcAttachPin(ENB, 6);
    
    // IR Pins
    pinMode(IR_LF_L, INPUT);
    pinMode(IR_LF_C, INPUT);
    pinMode(IR_LF_R, INPUT);
    pinMode(IR_OF_L, INPUT_PULLUP);
    pinMode(IR_OF_R, INPUT_PULLUP);
    
    // Ultrasonic
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    
    // Battery
    pinMode(BAT_PIN, INPUT);
    
    // MPU6050
    Wire.begin();
    byte status = mpu.begin();
    if(status != 0){
        Serial.println("MPU6050 init failed");
    } else {
        mpu.calcOffsets(); // takes a few seconds
    }
}

// ================= CORE 0: Network Loop =================
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_DATA) {
        processWebSocketMessage(arg, data, len);
    }
}

void networkLoop(void *pvParameters) {
    // Setup WiFi
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP("Bot_WiFi", "12345678"); // Fallback AP
    WiFi.begin(ssid, password);
    
    // Wait for connection initially
    int retries = 0;
    while(WiFi.status() != WL_CONNECTED && retries < 15) {
        vTaskDelay(pdMS_TO_TICKS(500));
        Serial.print(".");
        retries++;
    }
    Serial.println();
    
    if(WiFi.status() == WL_CONNECTED) {
        Serial.printf("[WIFI] Connected to %s! IP: %s\n", ssid, WiFi.localIP().toString().c_str());
    } else {
        Serial.printf("[WIFI] Failed to connect to %s. Fallback AP 'Bot_WiFi' active. IP: %s\n", ssid, WiFi.softAPIP().toString().c_str());
    }
    
    ws.onEvent(onEvent);
    server.addHandler(&ws);
    server.begin();
    
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t frequency = 200 / portTICK_PERIOD_MS; // 200ms telemetry
    
    unsigned long lastWifiCheck = 0;
    unsigned long wifiLostAt = 0;
    bool wifiWasLost = false;

    while(1) {
        unsigned long now = millis();
        
        // WiFi watchdog
        if (now - lastWifiCheck >= 2000) {
            lastWifiCheck = now;
            if (WiFi.status() != WL_CONNECTED) {
                if (!wifiWasLost) {
                    wifiWasLost = true;
                    wifiLostAt = now;
                    if(xSemaphoreTake(stateMutex, 0) == pdTRUE) {
                        sharedState.running = false;
                        xSemaphoreGive(stateMutex);
                    }
                    motorStop();
                    WiFi.disconnect();
                    WiFi.begin(ssid, password);
                    Serial.println("[WIFI] Lost! Reconnecting...");
                } else if (now - wifiLostAt > 10000) {
                    wifiLostAt = now;
                    WiFi.disconnect();
                    WiFi.begin(ssid, password);
                    Serial.println("[WIFI] Retry reconnect...");
                }
            } else if (wifiWasLost) {
                wifiWasLost = false;
                Serial.printf("[WIFI] Reconnected! IP: %s\n", WiFi.localIP().toString().c_str());
            }
        }

        ws.cleanupClients();
        
        // Broadcast telemetry
        if(xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            JsonDocument doc;
            doc["mode"] = sharedState.mode;
            doc["running"] = sharedState.running ? 1 : 0;
            doc["safe_stop"] = sharedState.safeStop ? 1 : 0;
            doc["tilt"] = sharedState.tilt;
            doc["temp"] = sharedState.temp;
            doc["acc"] = sharedState.accMag;
            doc["yaw"] = sharedState.yaw;
            doc["gx"] = sharedState.gx;
            doc["gy"] = sharedState.gy;
            doc["gz"] = sharedState.gz;
            doc["bat"] = sharedState.battery;
            doc["L"] = sharedState.distL;
            doc["C"] = sharedState.distC;
            doc["R"] = sharedState.distR;
            doc["irL"] = sharedState.irLfL;
            doc["irC"] = sharedState.irLfC;
            doc["irR"] = sharedState.irLfR;
            doc["irObjL"] = sharedState.irOfL;
            doc["irObjR"] = sharedState.irOfR;
            doc["motL"] = sharedState.motL;
            doc["motR"] = sharedState.motR;
            doc["gSpeed"] = sharedState.speed;
            doc["manDist"] = sharedState.manualDist;
            doc["manAngle"] = sharedState.manualRadarAngle;
            
            String json;
            serializeJson(doc, json);
            xSemaphoreGive(stateMutex);
            
            ws.textAll(json);
        }
        
        vTaskDelayUntil(&lastWakeTime, frequency);
    }
}

void processWebSocketMessage(void *arg, uint8_t *data, size_t len) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        data[len] = 0;
        String msg = (char*)data;
        
        if(msg == "PING") return; // Ignore PING
        
        if(xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
            if(msg.startsWith("MODE:")) {
                sharedState.mode = msg.substring(5).toInt();
            } else if(msg.startsWith("RADAR:")) {
                sharedState.radarOn = (msg.substring(6) == "1");
            } else if(msg.startsWith("SERVO_MAN:")) {
                sharedState.manualRadar = (msg.substring(10) == "1");
            } else if(msg.startsWith("SERVO_ANGLE:")) {
                int ang = msg.substring(12).toInt();
                sharedState.manualRadarAngle = constrain(ang, 30, 150);
            } else if(msg.startsWith("SPEED:")) {
                sharedState.speed = msg.substring(6).toInt();
            } else if(msg.startsWith("RUN:")) {
                sharedState.running = (msg.substring(4) == "1");
            } else if(msg.startsWith("SAFE:")) {
                sharedState.safeStop = (msg.substring(5) == "1");
            } else if(msg == "STOP_MOVE" || msg == "W" || msg == "A" || msg == "S" || msg == "D" || 
                      msg == "WA" || msg == "WD" || msg == "SA" || msg == "SD") {
                sharedState.manualCmd = msg;
            }
            xSemaphoreGive(stateMutex);
        }
    }
}

// ================= CORE 1: Control Loop =================

// Helper for ultrasonic distance
int getUltrasonicDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout ~ 500cm
    if(duration == 0) return 999;
    return duration * 0.034 / 2;
}

// Low-pass filter variables
float prevTilt = 0, prevAcc = 0, prevTemp = 0, prevYaw = 0;
const float ALPHA = 0.15;

// Variables for modes
unsigned long mode2StateTime = 0;
int mode2State = 0; // 0=forward, 1=scan right, 2=scan left, 3=backup, 4=turn
float yawIntegrated = 0;
unsigned long stuckStartTime = 0;
bool stuckTimerActive = false;
bool turnLeft = true;
unsigned long lineLostTime = 0;
bool lineLost = false;

// Radar variables
unsigned long lastRadarMoveMs = 0;
int radarState = 0; // 0=detached, 1=moving/settling, 2=settled (ready to detach)
int radarTargetAngle = 90;
int radarCurrentAngle = 90;
int radarCurrentSweep = 0; // 0=Left, 1=Center, 2=Right

void setRadarAngle(int angle) {
    if(radarTargetAngle != angle) {
        radarTargetAngle = angle;
        if (!radarServo.attached()) {
            radarServo.attach(SERVO_PIN, 500, 2400);
        }
        radarServo.write(radarTargetAngle);
        radarCurrentAngle = radarTargetAngle;
        lastRadarMoveMs = millis();
        radarState = 1; // settling
    }
}

void controlLoop(void *pvParameters) {
    while(1) {
        unsigned long currentMs = millis();
        
        // 1. Read Sensors
        mpu.update();
        float currentTilt = mpu.getAngleY(); // or X based on mount
        float accX = mpu.getAccX();
        float accY = mpu.getAccY();
        float accZ = mpu.getAccZ();
        float currentAccMag = sqrt(accX*accX + accY*accY + accZ*accZ) - accZ;
        float currentTemp = mpu.getTemp();
        float currentYaw = mpu.getGyroZ(); // Yaw rate
        
        // LPF
        prevTilt = (ALPHA * currentTilt) + ((1 - ALPHA) * prevTilt);
        prevAcc = (ALPHA * currentAccMag) + ((1 - ALPHA) * prevAcc);
        prevTemp = (ALPHA * currentTemp) + ((1 - ALPHA) * prevTemp);
        prevYaw = (ALPHA * currentYaw) + ((1 - ALPHA) * prevYaw);
        
        int uDist = getUltrasonicDistance();
        
        uint8_t lfL = digitalRead(IR_LF_L);
        uint8_t lfC = digitalRead(IR_LF_C);
        uint8_t lfR = digitalRead(IR_LF_R);
        
        uint8_t ofL = digitalRead(IR_OF_L);
        uint8_t ofR = digitalRead(IR_OF_R);
        
        int batRaw = analogRead(BAT_PIN);
        float batVolt = (batRaw / 4095.0) * 3.3 * 2.0;

        // 2. Fetch Shared State safely
        RobotState localState;
        if(xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            sharedState.tilt = prevTilt;
            sharedState.accMag = prevAcc;
            sharedState.temp = prevTemp;
            sharedState.yaw = prevYaw;
            sharedState.gx = mpu.getGyroX();
            sharedState.gy = mpu.getGyroY();
            sharedState.gz = currentYaw;
            sharedState.battery = batVolt;
            sharedState.distCurr = uDist;
            sharedState.irLfL = lfL;
            sharedState.irLfC = lfC;
            sharedState.irLfR = lfR;
            sharedState.irOfL = ofL;
            sharedState.irOfR = ofR;
            
            // Map ultrasonic directly to Center distance for UI in Mode 4
            if (sharedState.mode == 4) {
                sharedState.distC = uDist;
            }
            
            localState = sharedState;
            xSemaphoreGive(stateMutex);
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        // 3. Servo Radar State Machine
        bool idle = !localState.running || localState.safeStop || localState.mode == 0;
        bool wantsToSweep = localState.radarOn && !localState.manualRadar && localState.running && !localState.safeStop && (localState.mode == 1 || localState.mode == 3);
        bool shouldCenter = idle || localState.mode == 4 || ((localState.mode == 1 || localState.mode == 3) && !localState.radarOn && !localState.manualRadar);

        if (localState.manualRadar && (localState.mode == 1 || localState.mode == 3)) {
            // --- MANUAL MODE: bypass state machine, keep servo attached ---
            if (!radarServo.attached()) radarServo.attach(SERVO_PIN, 500, 2400);
            if (radarCurrentAngle != localState.manualRadarAngle) {
                radarServo.write(localState.manualRadarAngle);
                radarCurrentAngle = localState.manualRadarAngle;
                radarTargetAngle  = localState.manualRadarAngle;
            }
            // Keep radarState == 1 so auto-detach timer never fires
            radarState = 1;
            lastRadarMoveMs = currentMs;
            // Store reading using uDist already read this loop (no extra call needed)
            if(xSemaphoreTake(stateMutex, 0) == pdTRUE) {
                sharedState.manualDist = uDist;
                xSemaphoreGive(stateMutex);
            }
        } else {
            // --- AUTO MODE: normal state machine ---
            // Auto detach after 300ms settling
            if(radarState == 1 && (currentMs - lastRadarMoveMs > 300)) {
                radarServo.detach();
                radarState = 0;
            }

            if (shouldCenter) {
                if(radarCurrentAngle != 90 && radarState == 0) setRadarAngle(90);
            } else if (wantsToSweep && radarState == 0) {
                if(xSemaphoreTake(stateMutex, 0) == pdTRUE) {
                    if(radarCurrentAngle == 150) sharedState.distL = uDist;
                    else if(radarCurrentAngle == 90) sharedState.distC = uDist;
                    else if(radarCurrentAngle == 30) sharedState.distR = uDist;
                    xSemaphoreGive(stateMutex);
                }
                radarCurrentSweep = (radarCurrentSweep + 1) % 3;
                if(radarCurrentSweep == 0)      setRadarAngle(150);
                else if(radarCurrentSweep == 1) setRadarAngle(90);
                else                            setRadarAngle(30);
            }
        }
        
        // 4. Executing Mode Logic
        if (idle) {
            motorStop();
            mode2State = 0;
            lineLost = false;
        } else {
            int speed = localState.speed;
            
            if (localState.mode == 1) { // Manual
                if(localState.manualCmd == "W") motorDrive(speed, speed);
                else if(localState.manualCmd == "S") motorDrive(-speed, -speed);
                else if(localState.manualCmd == "A") motorDrive(-speed, speed);
                else if(localState.manualCmd == "D") motorDrive(speed, -speed);
                else if(localState.manualCmd == "WA") motorDrive(speed, speed/2);
                else if(localState.manualCmd == "WD") motorDrive(speed/2, speed);
                else if(localState.manualCmd == "SA") motorDrive(-speed/2, -speed);
                else if(localState.manualCmd == "SD") motorDrive(-speed, -speed/2);
                else motorStop();
                
            } else if (localState.mode == 2) { // Obstacle Avoidance
                const long SAFE_DIST  = 45;
                const long CLOSE_DIST = 25;
                int OA_SPEED = constrain(speed, 120, 220);

                const unsigned long STOP_MS   = 120;
                const unsigned long SERVO_MS  = 300;
                const unsigned long BACK_MS   = 300;
                const unsigned long RESUME_MS = 100;
                const float TURN_TARGET_DEG = 10.00f;
                const float STUCK_ACCEL_THRESHOLD = 0.2f;
                const unsigned long STUCK_DURATION = 800;

                static long scanR = -1;
                static long scanL = -1;

                if(mode2State == 0) { // OA_FORWARD
                    if(radarCurrentAngle != 90 && radarState == 0) setRadarAngle(90);
                    if(radarCurrentAngle == 90) {
                        if(xSemaphoreTake(stateMutex, 0) == pdTRUE) {
                            sharedState.distC = uDist;
                            // Keep distL/distR from last scan — do NOT clear them here
                            xSemaphoreGive(stateMutex);
                        }
                    }

                    // Stuck detection
                    if (uDist <= 0 || uDist > CLOSE_DIST) {
                        if(currentAccMag < STUCK_ACCEL_THRESHOLD) {
                            if(!stuckTimerActive) {
                                stuckTimerActive = true;
                                stuckStartTime = currentMs;
                            } else if(currentMs - stuckStartTime >= STUCK_DURATION) {
                                motorStop();
                                mode2State = 1; // OA_STOP
                                mode2StateTime = currentMs;
                                stuckTimerActive = false;
                                continue;
                            }
                        } else {
                            stuckTimerActive = false;
                        }
                    } else {
                        stuckTimerActive = false;
                    }

                    if (uDist > 0 && uDist <= CLOSE_DIST) {
                        motorStop();
                        mode2State = 1; // OA_STOP
                        mode2StateTime = currentMs;
                        stuckTimerActive = false;
                    } else if (uDist > 0 && uDist <= SAFE_DIST) {
                        motorDrive(130, 130);
                    } else {
                        motorDrive(OA_SPEED, OA_SPEED);
                    }
                } else if(mode2State == 1) { // OA_STOP
                    motorStop();
                    if(currentMs - mode2StateTime >= STOP_MS) {
                        setRadarAngle(30); // scan right
                        mode2StateTime = currentMs;
                        mode2State = 2; // OA_SERVO_R
                    }
                } else if(mode2State == 2) { // OA_SERVO_R
                    if(currentMs - mode2StateTime >= SERVO_MS) {
                        scanR = getUltrasonicDistance();
                        if(xSemaphoreTake(stateMutex, 0) == pdTRUE) { sharedState.distR = scanR; xSemaphoreGive(stateMutex); }
                        mode2StateTime = currentMs;
                        mode2State = 3; // OA_READ_R
                    }
                } else if(mode2State == 3) { // OA_READ_R
                    if(currentMs - mode2StateTime >= 50) {
                        setRadarAngle(150); // scan left
                        mode2StateTime = currentMs;
                        mode2State = 4; // OA_SERVO_L
                    }
                } else if(mode2State == 4) { // OA_SERVO_L
                    if(currentMs - mode2StateTime >= SERVO_MS) {
                        scanL = getUltrasonicDistance();
                        if(xSemaphoreTake(stateMutex, 0) == pdTRUE) { sharedState.distL = scanL; xSemaphoreGive(stateMutex); }
                        mode2StateTime = currentMs;
                        mode2State = 5; // OA_READ_L
                    }
                } else if(mode2State == 5) { // OA_READ_L
                    if(currentMs - mode2StateTime >= 50) {
                        setRadarAngle(90); // Center
                        mode2StateTime = currentMs;
                        mode2State = 6; // OA_SERVO_C
                    }
                } else if(mode2State == 6) { // OA_SERVO_C
                    if(currentMs - mode2StateTime >= SERVO_MS) {
                        if (scanL > 0 && scanR > 0) turnLeft = (scanL >= scanR);
                        else if (scanL > 0) turnLeft = true;
                        else if (scanR > 0) turnLeft = false;
                        else turnLeft = true;
                        
                        radarServo.detach();
                        radarState = 0;

                        motorDrive(-OA_SPEED, -OA_SPEED); // Backup
                        mode2StateTime = currentMs;
                        mode2State = 7; // OA_BACK
                    }
                } else if(mode2State == 7) { // OA_BACK
                    if(currentMs - mode2StateTime >= BACK_MS) {
                        motorStop();
                        yawIntegrated = 0;
                        if(turnLeft) motorDrive(-OA_SPEED, OA_SPEED);
                        else motorDrive(OA_SPEED, -OA_SPEED);
                        mode2StateTime = currentMs;
                        mode2State = 8; // OA_TURN
                    }
                } else if(mode2State == 8) { // OA_TURN
                    yawIntegrated += abs(prevYaw) * 0.015f; // loop time approx
                    if(yawIntegrated >= TURN_TARGET_DEG || currentMs - mode2StateTime >= 800) {
                        motorStop();
                        mode2StateTime = currentMs;
                        mode2State = 9; // OA_RESUME
                    }
                } else if(mode2State == 9) { // OA_RESUME
                    if(currentMs - mode2StateTime >= RESUME_MS) {
                        stuckTimerActive = false;
                        if(xSemaphoreTake(stateMutex, 0) == pdTRUE) { sharedState.distC = -1; xSemaphoreGive(stateMutex); }
                        mode2State = 0; // OA_FORWARD
                    }
                }
                
            } else if (localState.mode == 3) { // Line Following
                bool msFound = (lfC == HIGH);
                bool lsFound = (lfL == HIGH);
                bool rsFound = (lfR == HIGH);

                static int lastDir = 0; // 0=straight, 1=left, -1=right

                if (msFound && !lsFound && !rsFound) {
                    // Perfectly centered
                    motorDrive(speed, speed);
                    lastDir = 0;
                } else if (msFound && lsFound && !rsFound) {
                    // Slight right drift (Center + Left)
                    motorDrive(0, speed); // Gentle Left
                    lastDir = 1;
                } else if (!msFound && lsFound && !rsFound) {
                    // Far right drift (Left only)
                    motorDrive(-speed, speed); // Sharp Left
                    lastDir = 1;
                } else if (msFound && !lsFound && rsFound) {
                    // Slight left drift (Center + Right)
                    motorDrive(speed, 0); // Gentle Right
                    lastDir = -1;
                } else if (!msFound && !lsFound && rsFound) {
                    // Far left drift (Right only)
                    motorDrive(speed, -speed); // Sharp Right
                    lastDir = -1;
                } else if (msFound && lsFound && rsFound) {
                    // Intersection (All 3 on line)
                    motorDrive(speed, speed);
                } else {
                    // Lost line (All off) - use last known direction
                    if (lastDir == 1) {
                        motorDrive(-speed, speed); // Keep turning left
                    } else if (lastDir == -1) {
                        motorDrive(speed, -speed); // Keep turning right
                    } else {
                        motorStop();
                    }
                }
                
            } else if (localState.mode == 4) { // Object Following
                if (uDist > 0 && uDist < 10) {
                    motorDrive(-speed, -speed);
                } else if (!ofL && !ofR) {
                    // Both detect
                    motorDrive(speed, speed);
                } else if (!ofL && ofR) {
                    // Left detects
                    motorDrive(speed, speed/2);
                } else if (ofL && !ofR) {
                    // Right detects
                    motorDrive(speed/2, speed);
                } else {
                    // None detect
                    motorStop();
                }
            }
        }
        
        // Serial monitor telemetry every 2000ms
        static unsigned long lastSerialDump = 0;
        if(currentMs - lastSerialDump >= 2000) {
            lastSerialDump = currentMs;
            const char* modeNames[] = {"IDLE","MANUAL","OA","LINE","OBJFOLLOW"};
            Serial.printf("[STATUS] IP=%s | mode=%s run=%d safe=%d | tilt=%.1f yaw=%.1f acc=%.2f temp=%.1f | bat=%.2fV | IR=%d%d%d objIR=%d%d | dist L=%d C=%d R=%d\n",
                    WiFi.localIP().toString().c_str(),
                    (localState.mode >= 0 && localState.mode <= 4) ? modeNames[localState.mode] : "?",
                    localState.running, localState.safeStop, localState.tilt, localState.yaw, localState.accMag, localState.temp, localState.battery,
                    localState.irLfL, localState.irLfC, localState.irLfR, localState.irOfL, localState.irOfR,
                    localState.distL, localState.distC, localState.distR);
        }

        vTaskDelay(pdMS_TO_TICKS(15));
    }
}

// Motor Control Helper
void motorDrive(int speedL, int speedR) {
    if(speedL > 255) speedL = 255;
    if(speedL < -255) speedL = -255;
    if(speedR > 255) speedR = 255;
    if(speedR < -255) speedR = -255;
    
    // Track motor states for UI
    if(xSemaphoreTake(stateMutex, 0) == pdTRUE) {
        sharedState.motL = speedL;
        sharedState.motR = speedR;
        xSemaphoreGive(stateMutex);
    }
    
    // Left Motor (IN3, IN4, PWM CH 6) -> Forward: IN4=HIGH, IN3=LOW
    if(speedL >= 0) {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        ledcWrite(6, speedL); 
    } else {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        ledcWrite(6, -speedL);
    }
    
    // Right Motor (IN1, IN2, PWM CH 5) -> Forward: IN2=HIGH, IN1=LOW
    if(speedR >= 0) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        ledcWrite(5, speedR); 
    } else {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        ledcWrite(5, -speedR);
    }
}

void motorStop() {
    if(xSemaphoreTake(stateMutex, 0) == pdTRUE) {
        sharedState.motL = 0;
        sharedState.motR = 0;
        xSemaphoreGive(stateMutex);
    }
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    ledcWrite(5, 0);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    ledcWrite(6, 0);
}