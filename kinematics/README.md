# Kinematics — Servo2040 Inverse Kinematics MCU

The Servo2040 (RP2040, Pimoroni) is dedicated entirely to leg math and
servo output. It receives a fused `RobotState` packet from the ESP32
over UART every control cycle and turns it into 18 servo angles.

## Leg model

Each leg is a 3-DOF serial chain: coxa (`L1`), femur (`L2`), tibia
(`L3`), solved with closed-form inverse kinematics
(`solve_ik(x, y, z, side)` in `servo2040/main.cpp`):

```
L1 = 4.0   // coxa
L2 = 8.0   // femur
L3 = 12.0  // tibia
```

Given a target foot position in the leg's local frame, `solve_ik`
returns hip/thigh/shin (`h`, `t`, `s`) angles, or marks the target
unreachable (`valid = false`) if it falls outside the leg's workspace
(`d > L2+L3` or `d < |L2-L3|`).

Six legs (`LF`, `LB`, `LM`, `RM`, `RF`, `RB`), three servos each = the
project's 18x DS3235. Per-leg Cartesian trims (`xyz_offset`) let each
leg be calibrated independently to absorb small assembly tolerances,
applied before the IK solve rather than as a post-hoc angle offset.

## Gait

`walk_step()` drives a tripod gait: legs are split into two phase
groups (`gait_offsets`), alternating stance (linear ground contact,
carrying the body) and swing (a quintic Bezier arc lifting the foot by
`STEP_H` and carrying it forward). Body pitch/roll from the ESP32's
IMU PID are folded into every leg's target position every cycle so the
body stays level over uneven ground while walking or standing still
(`stand_still()`).

`startup_sequence()` eases every leg from a folded rest position to
its standing pose over 200 interpolated steps, avoiding a snap-to-pose
power-on jolt.

## UART protocol

Listens on `uart1` (pins 20/21, 115200 baud) for the same packed
`RobotState` struct the ESP32 sends (see
[`/firmware`](../firmware/README.md#protocol)), framed by a `0xAA`
header / `0x55` footer byte pair.

## Building

Requires the [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk)
and the [Pimoroni Pico libraries](https://github.com/pimoroni/pimoroni-pico)
(for `ServoCluster`).

```bash
cd kinematics/servo2040
mkdir build && cd build
cmake -DPICO_SDK_PATH=/path/to/pico-sdk \
      -DPIMORONI_PICO_PATH=/path/to/pimoroni-pico ..
make -j4
# flash Hexapod_OS.uf2 to the Servo2040 in BOOTSEL mode
```

Built and tested against Pico SDK 1.5.1.

Alternatively set `PICO_SDK_FETCH_FROM_GIT=ON` /
`PIMORONI_PICO_FETCH_FROM_GIT=ON` to have CMake pull both SDKs
automatically.

## Credit

The IK/gait approach on the Servo2040 builds on ideas from the
open-source [Make Your Pet](https://github.com/makeyourpet) hexapod
platform, substantially reworked for AgriHexa's leg geometry, dual-MCU
split, and Cartesian trim/stability integration.
