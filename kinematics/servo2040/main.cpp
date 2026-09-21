// AgriHexa - Servo2040 Inverse Kinematics MCU
//
// Receives the fused RobotState from the ESP32 over UART, solves 3-DOF
// inverse kinematics per leg (coxa/femur/tibia), generates the tripod
// gait swing trajectory (quintic Bezier), and drives all 18 DS3235
// servos via the Pimoroni ServoCluster.
//
#include <stdio.h>
#include <math.h>
#include <cstring>
#include <vector>
#include <algorithm>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "drivers/servo/servo_cluster.hpp"

using namespace servo;

#define UART_ID    uart1
#define UART_BAUD  115200
#define PIN_TX     20
#define PIN_RX     21

const float L1 = 4.0, L2 = 8.0, L3 = 12.0;

struct LegConfig {
    const char* name;
    uint8_t pins[3];
    float xyz_offset[3]; // Cartesian trim: X, Y, Z — applied BEFORE solve_ik
    int side_sign;
};

struct Angles {
    float h, t, s;
    bool valid;
};

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

float body_height  = -12.0f;
float body_roll    =   0.0f;
float body_pitch   =   0.0f;
float stride_scale =   0.0f;
float strafe_scale =   0.0f;
bool  walking      =   false;

bool first_packet_received = false;

// Cartesian per-leg trim — adjust these to physically calibrate each leg's resting position.
// X = forward(+)/back(-), Y = outward(+)/inward(-), Z = up(+)/down(-)
LegConfig leg_data[6] = {
    {"LF", {0,  1,  2},  {0.0f, 4.0f, -2.5f}, -1},
    {"LB", {3,  4,  5},  {0.0f, 4.0f, 0.0f}, -1},
    {"LM", {6,  7,  8},  {0.0f, 4.0f, -1.7f}, -1},
    {"RM", {9,  10, 11}, {0.0f, 0.0f, 0.0f}, -1},
    {"RF", {12, 13, 14}, {0.0f, 0.0f, 0.0f}, -1},
    {"RB", {15, 16, 17}, {0.0f, 0.0f, 0.0f}, -1}
};

const float LEG_SIDE[6] = { 1.0f,  1.0f,  1.0f, -1.0f, -1.0f, -1.0f};
const float PITCH_GAIN  = 2.0f;
const float ROLL_GAIN   = 2.0f;
const float Y_OFFSET    = 8.0f;
const float FREQ        = 1.0f;
const float STEP_H      = 8.0f;
const float LEG_X_POS[6] = { 5.0f, -5.0f,  0.0f,  0.0f,  5.0f, -5.0f};
const float LEG_Y_POS[6] = { 8.0f,  8.0f,  8.0f,  8.0f,  8.0f,  8.0f};
float gait_offsets[6] = {0.0, 0.0, 0.5, 0.0, 0.5, 0.5};

void uart_init_comms() {
    uart_init(UART_ID, UART_BAUD);
    gpio_set_function(PIN_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_RX, GPIO_FUNC_UART);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, true);
}

void read_uart_packets() {
    static uint8_t buf[sizeof(RobotState)];
    static int idx = 0;
    while (uart_is_readable(UART_ID)) {
        uint8_t byte = uart_getc(UART_ID);
        if (byte == 0xAA && idx != 0) idx = 0;
        if (idx == 0 && byte != 0xAA) continue;
        buf[idx++] = byte;
        if (idx == sizeof(RobotState)) {
            idx = 0;
            RobotState* rs = (RobotState*)buf;
            if (rs->header == 0xAA && rs->footer == 0x55) {
                body_pitch = rs->body_pitch; body_roll = rs->body_roll;
                stride_scale = rs->stride; strafe_scale = rs->strafe;
                body_height = rs->height; walking = (rs->walking == 1);
                first_packet_received = true;
            }
        }
    }
}

Angles solve_ik(float x, float y, float z, int side) {
    float actual_y = y * (float)side;
    float theta1 = atan2(actual_y, x);
    float r = sqrt(x*x + actual_y*actual_y) - L1;
    float d = sqrt(r*r + z*z);
    if (d > (L2 + L3) || d < fabs(L2 - L3)) return {0, 0, 0, false};
    float cos_t3 = (d*d - L2*L2 - L3*L3) / (2 * L2 * L3);
    float theta3 = acos(std::max(-1.0f, std::min(1.0f, cos_t3)));
    float a1 = atan2(z, r);
    float a2 = acos(std::max(-1.0f, std::min(1.0f, (L2*L2 + d*d - L3*L3) / (2 * L2 * d))));
    return {(float)(theta1 * 180.0 / M_PI), (float)((a1 + a2) * 180.0 / M_PI), (float)(90.0 - (theta3 * 180.0 / M_PI)), true};
}

float bezier5(float t, float p0, float p1, float p2, float p3, float p4, float p5) {
    float mt = 1.0f - t;
    return (pow(mt,5)*p0 + 5*pow(mt,4)*t*p1 + 10*pow(mt,3)*pow(t,2)*p2 + 10*pow(mt,2)*pow(t,3)*p3 + 5*mt*pow(t,4)*p4 + pow(t,5)*p5);
}

void set_leg(ServoCluster& cluster, LegConfig& leg, int i, Angles res) {
    if (!res.valid) return;
    float h = res.h; float t = res.t; float s = res.s; // no more post-IK degree trims
    if (i <= 2) { cluster.value(leg.pins[0], h); cluster.value(leg.pins[1], t); cluster.value(leg.pins[2], -s); }
    else { cluster.value(leg.pins[0], h); cluster.value(leg.pins[1], -t); cluster.value(leg.pins[2], s); }
}

void stand_still(ServoCluster& cluster) {
    for (int i = 0; i < 6; i++) {
        float roll_z  = LEG_SIDE[i] * Y_OFFSET * ROLL_GAIN * sinf(body_roll);
        float roll_y  = LEG_Y_POS[i] * (cosf(body_roll) - 1.0f);
        float pitch_z = LEG_X_POS[i] * PITCH_GAIN * sinf(body_pitch);
        float pitch_x = LEG_X_POS[i] * (cosf(body_pitch) - 1.0f);

        float final_x = pitch_x;
        float final_y = Y_OFFSET + roll_y;
        float final_z = body_height + roll_z + pitch_z;

        if (i < 3) final_x = -final_x;

        // Apply Cartesian trim BEFORE solving IK
        final_x += leg_data[i].xyz_offset[0];
        final_y += leg_data[i].xyz_offset[1];
        final_z += leg_data[i].xyz_offset[2];

        Angles res = solve_ik(final_x, final_y, final_z, leg_data[i].side_sign);
        set_leg(cluster, leg_data[i], i, res);
    }
}

void walk_step(ServoCluster& cluster, float elapsed) {
    float STRIDE_X = 10.0f * stride_scale;
    float STRAFE_Y = 10.0f * strafe_scale;

    for (int i = 0; i < 6; i++) {
        float phase = fmod((elapsed * FREQ + gait_offsets[i]), 1.0f);

        float tx, ty, tz;

        if (phase < 0.5f) {
            float s = phase / 0.5f;
            tx = (0.5f * STRIDE_X) - (STRIDE_X * s);
            ty = LEG_SIDE[i] * ((0.5f * STRAFE_Y) - (STRAFE_Y * s));
            tz = body_height;
        } else {
            float q = (phase - 0.5f) / 0.5f;
            tx = bezier5(q,
                -0.5f*STRIDE_X, -0.5f*STRIDE_X,
                -0.2f*STRIDE_X,  0.2f*STRIDE_X,
                 0.5f*STRIDE_X,  0.5f*STRIDE_X);
            ty = LEG_SIDE[i] * bezier5(q,
                -0.5f*STRAFE_Y, -0.5f*STRAFE_Y,
                -0.2f*STRAFE_Y,  0.2f*STRAFE_Y,
                 0.5f*STRAFE_Y,  0.5f*STRAFE_Y);
            tz = bezier5(q,
                body_height, body_height,
                body_height + STEP_H, body_height + STEP_H,
                body_height, body_height);
        }

        float roll_z  = LEG_SIDE[i] * Y_OFFSET * ROLL_GAIN * sinf(body_roll);
        float roll_y  = LEG_Y_POS[i] * (cosf(body_roll) - 1.0f);
        float pitch_z = LEG_X_POS[i] * PITCH_GAIN * sinf(body_pitch);
        float pitch_x = LEG_X_POS[i] * (cosf(body_pitch) - 1.0f);

        float final_x = tx + pitch_x;
        float final_y = Y_OFFSET + ty + roll_y;
        float final_z = tz + roll_z + pitch_z;

        if (i < 3) final_x = -final_x;

        // Apply Cartesian trim BEFORE solving IK
        final_x += leg_data[i].xyz_offset[0];
        final_y += leg_data[i].xyz_offset[1];
        final_z += leg_data[i].xyz_offset[2];

        Angles res = solve_ik(final_x, final_y, final_z, leg_data[i].side_sign);
        set_leg(cluster, leg_data[i], i, res);
    }
}

void startup_sequence(ServoCluster& cluster) {
    const int STEPS   = 200;
    const int STEP_MS = 15;

    float target_x[6], target_y[6], target_z[6];

    for (int i = 0; i < 6; i++) {
        float pitch_z = LEG_X_POS[i] * sinf(body_pitch);
        float pitch_x = LEG_X_POS[i] * (cosf(body_pitch) - 1.0f);

        target_x[i] = (i < 3) ? -pitch_x : pitch_x;
        target_y[i] = Y_OFFSET;
        target_z[i] = body_height + pitch_z;

        // Apply Cartesian trim
        target_x[i] += leg_data[i].xyz_offset[0];
        target_y[i] += leg_data[i].xyz_offset[1];
        target_z[i] += leg_data[i].xyz_offset[2];
    }

    float start_z = -5.0f;

    for (int step = 0; step <= STEPS; step++) {
        float t = (float)step / (float)STEPS;

        for (int i = 0; i < 6; i++) {
            float iz = start_z + (target_z[i] - start_z) * t;
            float ix = target_x[i] * t;
            float iy = target_y[i];

            Angles res = solve_ik(ix, iy, iz, leg_data[i].side_sign);
            set_leg(cluster, leg_data[i], i, res);
        }
        sleep_ms(STEP_MS);
    }

    printf("Startup complete\n");
}

int main() {
    stdio_init_all();
    sleep_ms(2500);

    printf("--- Servo2040 Booting Up ---\n");
    uart_init_comms();

    ServoCluster cluster(pio0, 0,
        {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17});
    cluster.init();

    sleep_ms(500);

    printf("Waiting for first UART packet from ESP32...\n");
    while (!first_packet_received) {
        read_uart_packets();
        sleep_ms(10);
    }
    printf("First packet received. Starting Servo2040 operations.\n");

    startup_sequence(cluster);
    sleep_ms(500);
    printf("Servo 2040 ready\n");

    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    uint32_t last_print = 0;

    while (true) {
        read_uart_packets();

        uint32_t now = to_ms_since_boot(get_absolute_time());

        if (now - last_print >= 200) {
            printf("P:%.3f R:%.3f Stride:%.3f Strafe:%.3f H:%.3f W:%d\n",
                body_pitch, body_roll, stride_scale, strafe_scale, body_height, walking);
            last_print = now;
        }

        float elapsed = (now - start_time) / 1000.0f;

        if (walking) {
            walk_step(cluster, elapsed);
        } else {
            stand_still(cluster);
        }
        read_uart_packets();
        sleep_ms(10);
    }
}
