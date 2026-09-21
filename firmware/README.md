# Firmware — ESP32 Control MCU

The ESP32 is the "brain" of AgriHexa's dual-MCU control stack. It owns
everything that isn't leg math:

- **Stability**: reads a BNO055 IMU and runs two independent PID loops
  (pitch, roll) to compute body-leveling corrections in real time.
- **Teleop**: pairs with a PS4 controller for manual driving (stride,
  strafe, ride height).
- **Autonomy bridge**: runs a TCP server on port 80 that accepts move
  commands (`F`, `B`, `L`, `R`, `FR`, `FL`, `BR`, `BL`, `S`) from the
  ground-station A* planner in [`/navigation`](../navigation), and
  ACKs each completed step back over the socket.
- **Fusion**: packs pitch/roll/stride/strafe/height/walking into a single
  `RobotState` struct and streams it over UART to the Servo2040
  (see [`/kinematics`](../kinematics)), which solves inverse kinematics
  and drives the 18 servos.

PS4 input always takes priority — touching the controller instantly
cancels any in-progress autonomous move.

## Hardware

| Signal          | ESP32 Pin |
|-----------------|-----------|
| I2C SDA (BNO055)| 18        |
| I2C SCL (BNO055)| 21        |
| UART TX -> Servo2040 | 16   |
| UART RX <- Servo2040 | 23   |
| Relay           | 27        |

## Building

This is a PlatformIO project.

```bash
cd firmware/esp32
cp include/secrets.h.example include/secrets.h   # fill in your own Wi-Fi creds
pio run -t upload
pio device monitor
```

`secrets.h` is gitignored on purpose — never commit real Wi-Fi
credentials. Only `secrets.h.example` is tracked.

## Protocol

`RobotState` (packed, sent every loop iteration over UART2 @ 115200):

```c
struct __attribute__((packed)) RobotState {
    uint8_t header;      // 0xAA
    float body_pitch;
    float body_roll;
    float body_yaw;
    float stride;
    float strafe;
    float height;
    uint8_t walking;
    uint8_t footer;       // 0x55
};
```
