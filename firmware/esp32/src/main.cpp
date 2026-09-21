// AgriHexa - ESP32 Control MCU
//
// Responsibilities:
//   - IMU (BNO055) stability PID -> body pitch/roll correction
//   - PS4 controller teleop input
//   - Wi-Fi TCP server accepting autonomous move commands from the
//     ground-station A* planner (see /navigation)
//   - Packs the unified RobotState and streams it over UART to the
//     Servo2040 (see /kinematics), which solves IK and drives the servos
//
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <PS4Controller.h>
#include <WiFi.h>
#include "secrets.h"

#define RELAY_PIN    27
#define I2C_SDA      18
#define I2C_SCL      21
#define SERVO_TX     16
#define SERVO_RX     23

// ==========================================
// 1. HARDWARE SENSORS & PID
// ==========================================
float filtered_pitch = 0.0f;
float filtered_roll  = 0.0f;
const float ALPHA    = 0.1f;
const float DEADBAND = 2.0f;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

struct PID {
    float kp, ki, kd;
    float prev_error;
    float integral;
};

PID pitch_pid = {0.01f, 0.0f, 0.0008f, 0.0f, 0.0f};
PID roll_pid  = {0.01f, 0.0f, 0.0008f, 0.0f, 0.0f};

uint32_t zero_streak = 0;
const uint32_t ZERO_LIMIT = 50;
bool bno_initialized = false;
unsigned long last_time = 0;

float compute_pid(PID &pid, float error, float dt) {
    pid.integral += error * dt;
    pid.integral = constrain(pid.integral, -0.5f, 0.5f);
    float derivative = (error - pid.prev_error) / dt;
    derivative = constrain(derivative, -50.0f, 50.0f);
    pid.prev_error = error;
    return (pid.kp * error) + (pid.ki * pid.integral) + (pid.kd * derivative);
}

void attempt_recovery() {
    Serial.println("[BNO055] Resetting...");
    bno_initialized = false;
    Wire.end(); delay(500);
    Wire.begin(I2C_SDA, I2C_SCL); Wire.setClock(100000); delay(100);
    if (bno.begin()) {
        bno.setExtCrystalUse(true);
        bno_initialized = true;
        zero_streak = 0;
        Serial.println("[BNO055] Recovery successful.");
    }
}

// ==========================================
// 2. KINEMATICS DATA STRUCTURE
// ==========================================
struct __attribute__((packed)) RobotState {
    uint8_t header;
    float body_pitch;
    float body_roll;
    float body_yaw;
    float stride;
    float strafe;
    float height;
    uint8_t walking;
    uint8_t footer;
};

RobotState state = {0xAA, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -12.0f, 0, 0x55};

// ==========================================
// 3. NETWORK & NON-BLOCKING VARIABLES
// ==========================================
const char* ssid     = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// Static IP for the robot on the field network
IPAddress local_IP(192, 168, 0, 51);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);

WiFiServer server(80);
WiFiClient active_client; // Persistent TCP client

// Non-blocking timer variables
const unsigned long STRIDE_DELAY_MS = 1500;
unsigned long moveStartTime = 0;
bool isAutoMoving = false; // Tracks if a Wi-Fi command is currently executing
float auto_stride = 0.0f;
float auto_strafe = 0.0f;

// ==========================================
// 4. PS4 CONTROLLER
// ==========================================
float ps4_stride = 0.0f;
float ps4_strafe = 0.0f;
float height_cmd = -12.0f;
bool  ps4_walking = false;

const float DEAD_ZONE     = 10.0f;
const float STRIDE_SCALE  = 0.007f;
const float STRAFE_SCALE  = -0.007f;
const float HEIGHT_SCALE  = 0.04f;
const float HEIGHT_MIN    = -18.0f;
const float HEIGHT_MAX    = -6.0f;

void read_ps4() {
    if (!PS4.isConnected()) return;
    float lx = PS4.LStickX();
    float ly = PS4.LStickY();
    float ry = PS4.RStickY();

    if (fabsf(ly) > DEAD_ZONE) ps4_stride = constrain(-ly * STRIDE_SCALE, -1.0f, 1.0f);
    else ps4_stride = 0.0f;

    if (fabsf(lx) > DEAD_ZONE) ps4_strafe = constrain(lx * STRAFE_SCALE, -1.0f, 1.0f);
    else ps4_strafe = 0.0f;

    ps4_walking = (fabsf(ly) > DEAD_ZONE) || (fabsf(lx) > DEAD_ZONE);

    if (fabsf(ry) > DEAD_ZONE) {
        height_cmd = constrain(height_cmd + (-ry) * HEIGHT_SCALE * 0.01f, HEIGHT_MIN, HEIGHT_MAX);
    }
}

// ==========================================
// SETUP ROUTINE
// ==========================================
void setup() {
    Serial.begin(115200);

    // Servo UART bridge to the Servo2040: SERVO_RX is where the ESP32
    // receives, SERVO_TX is where the ESP32 transmits.
    Serial2.begin(115200, SERIAL_8N1, SERVO_RX, SERVO_TX);

    // Initialize Hardware Peripherals
    Wire.begin(I2C_SDA, I2C_SCL);
    if (bno.begin()) {
        bno.setExtCrystalUse(true);
        bno_initialized = true;
    }

    // Initialize PS4 with a dummy MAC to prevent crashing
    PS4.begin("03:03:03:03:03:03");

    // Network Setup
    if (!WiFi.config(local_IP, gateway, subnet)) {
        Serial.println("STA Failed to configure static IP");
    }
    WiFi.begin(ssid, password);
    Serial.print("Connecting to Wi-Fi...");
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }

    Serial.println("\nWiFi Connected! IP: ");
    Serial.println(WiFi.localIP());
    server.begin();
    Serial.println("Awaiting autonomous Python commands on Port 80...");
}

// ==========================================
// MAIN LOOP: WiFi + PS4 + PID
// ==========================================
void loop() {
    // 1. Read PS4 Controller First
    read_ps4();

    // 2. Wi-Fi Control (Non-Blocking)
    // Only accept incoming commands if PS4 is idle
    if (ps4_stride == 0.0f && ps4_strafe == 0.0f) {

        // Grab incoming connection if none is active
        if (!active_client || !active_client.connected()) {
            active_client = server.available();
        }

        // Listen for new commands ONLY if the robot is not currently taking a step
        if (active_client && active_client.connected() && !isAutoMoving) {
            if (active_client.available() > 0) {
                String input = active_client.readStringUntil('\n');
                input.trim();

                if (input.length() > 0) {
                    Serial.println("[Python Command]: " + input);

                    // Map Python commands to temporary kinematics variables
                    if (input == "F") { auto_stride = 1.0f; auto_strafe = 0.0f; isAutoMoving = true; }
                    else if (input == "B") { auto_stride = -1.0f; auto_strafe = 0.0f; isAutoMoving = true; }
                    else if (input == "L") { auto_stride = 0.0f; auto_strafe = -1.0f; isAutoMoving = true; }
                    else if (input == "R") { auto_stride = 0.0f; auto_strafe = 1.0f; isAutoMoving = true; }
                    else if (input == "FR") { auto_stride = 0.707f; auto_strafe = 0.707f; isAutoMoving = true; }
                    else if (input == "FL") { auto_stride = 0.707f; auto_strafe = -0.707f; isAutoMoving = true; }
                    else if (input == "BR") { auto_stride = -0.707f; auto_strafe = 0.707f; isAutoMoving = true; }
                    else if (input == "BL") { auto_stride = -0.707f; auto_strafe = -0.707f; isAutoMoving = true; }
                    else if (input == "S") { auto_stride = 0.0f; auto_strafe = 0.0f; isAutoMoving = false; }

                    if (isAutoMoving) {
                        moveStartTime = millis(); // Start the non-blocking timer
                    } else {
                        active_client.println("ACK"); // Send immediate ACK if just commanded to stop
                    }
                }
            }
        }
    } else {
        // If the user touches the PS4 controller, immediately cancel autonomous movement
        isAutoMoving = false;
        auto_stride = 0.0f;
        auto_strafe = 0.0f;
    }

    // 3. Check if the non-blocking stride has finished
    if (isAutoMoving) {
        if (millis() - moveStartTime >= STRIDE_DELAY_MS) {
            // Step complete! Send ACK back to the laptop
            if (active_client.connected()) {
                active_client.println("ACK");
            }
            // Halt autonomous movement
            isAutoMoving = false;
            auto_stride = 0.0f;
            auto_strafe = 0.0f;
        }
    }

    // 4. Process BNO055 Stability PID
    if (!bno_initialized) { attempt_recovery(); delay(100); return; }

    unsigned long now = millis();
    float dt = (now - last_time) / 1000.0f;

    // Only compute PID if enough time has passed
    if (dt >= 0.02f) {
        last_time = now;

        sensors_event_t event;
        bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);

        filtered_pitch = ALPHA * event.orientation.z + (1.0f - ALPHA) * filtered_pitch;
        filtered_roll  = ALPHA * event.orientation.y + (1.0f - ALPHA) * filtered_roll;

        float p_out = compute_pid(pitch_pid, (fabs(filtered_pitch) > DEADBAND ? -filtered_pitch : 0.0f), dt);
        float r_out = compute_pid(roll_pid,  (fabs(filtered_roll) > DEADBAND ? -filtered_roll : 0.0f), dt);

        state.body_pitch = constrain(p_out, -0.3f, 0.3f);
        state.body_roll  = constrain(r_out, -0.3f, 0.3f);
    }

    // 5. Update Unified State (Auto overrides PS4, unless PS4 is actively used)
    state.stride  = isAutoMoving ? auto_stride : ps4_stride;
    state.strafe  = isAutoMoving ? auto_strafe : ps4_strafe;
    state.height  = height_cmd;
    state.walking = (isAutoMoving || ps4_walking) ? 1 : 0;

    // Transmit constantly to Servo2040 so the PID remains responsive
    Serial2.write((uint8_t*)&state, sizeof(RobotState));
}
