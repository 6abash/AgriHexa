# Design differences from Make Your Pet

AgriHexa uses the open-source [Make Your Pet](https://github.com/makeyourpet)
hexapod as a starting platform — credit to them for open-sourcing the
design. Everything below is what changed to turn it from a
phone-controlled hexapod into a standalone system built for precision
farming.

## The platform is a means to an end

Make Your Pet's design is a hexapod platform; AgriHexa uses that
platform to solve a specific problem — precision farming (targeted
planting, terrain-aware autonomous navigation). The mechanical base
carried over, but almost everything around it changed to support that.

## Removing the phone dependency

The original Make Your Pet build relies entirely on a smartphone: the
Chica app supplies the sensor data (including the IMU) the robot needs
to operate, and the phone also powers the Servo2040. That works for a
demo platform, but it isn't something you can actually deploy as a
farming system — there's no phone in the loop.

To make it a standalone system, AgriHexa adds:

- **An ESP32 as a dedicated control MCU** — replacing the phone as the
  thing that talks Wi-Fi, runs the stability control loop, and accepts
  teleop/autonomous commands.
- **A standalone BNO055 IMU** — replacing the phone's IMU as the
  stability sensor, feeding the ESP32's pitch/roll PID loops directly.
- **A buck converter (7.4V -> 5V)** — since there's no longer a phone
  to charge the Servo2040, and the system now carries more electronics
  (ESP32, IMU) than the original design, a buck converter steps the
  main 7.4V battery rail down to the 5V logic supply those components
  need.

Everything else in the electrical/control architecture — the Servo2040
handling inverse kinematics and driving the servos — follows the same
approach as Make Your Pet.

## Mechanical changes

- **Steel servo horns instead of plastic** — the stock plastic horns
  failed under the stall torque of the DS3235 servos. Steel horns were
  a necessary swap, not a preference.
- **Revised mounting holes** — hole sizing was reworked to 3mm in
  SolidWorks for a more reliable fastener fit.
- **CF-PETG as the print material** — arrived at by trial and error as
  the best-performing material for the structural/printed parts.
