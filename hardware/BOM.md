# Bill of Materials

Quantities below are derived directly from the design (6 legs x 3
servos, etc.). Unit/total costs are left blank for you to fill in from
actual receipts — I'm not going to guess prices and pass them off as
real numbers.

| # | Part | Spec / Notes | Qty | Unit Cost | Total |
|---|------|---------------|-----|-----------|-------|
| 1 | Servo | DS3235 digital servo | 18 (3 x 6 legs) | | |
| 2 | Control MCU | ESP32 dev board (Wi-Fi, PS4 teleop, BNO055 stability PID) | 1 | | |
| 3 | Kinematics MCU | Pimoroni Servo2040 (RP2040, 18-channel servo driver) | 1 | | |
| 4 | IMU | Adafruit BNO055 breakout | 1 | | |
| 5 | Buck converter | 7.4V -> 5V, powers ESP32 + BNO055 + logic rail | 1 | | |
| 6 | Battery | 2S LiPo, 7.4V nominal | 1 | | |
| 7 | Relay | Switches planting mechanism / auxiliary load off ESP32 GPIO27 | 1 | | |
| 8 | Servo horn | Steel (replaces stock plastic — failed under DS3235 stall torque) | 18 | | |
| 9 | Leg tube/link | Carbon fiber | 6 legs x 3 links = 18 | | |
| 10 | Structural/printed parts | CF-PETG (chosen over plain PETG/PLA by trial and error) | as needed | | |
| 11 | Controller | PS4 DualShock 4 (teleop) | 1 | | |
| 12 | Planting mechanism | Servo-driven dispenser for 3–5mm radius plants | 1 | | |
| 13 | Fasteners | M3 hardware (3mm mounting holes, SolidWorks-revised) | as needed | | |
| 14 | Wiring | Servo extension leads, JST/XT30 connectors, heat shrink | as needed | | |

Add rows as the hardware doc / CAD get finalized (mounting hardware,
connectors, standoffs, etc. tend to only become exact once the CAD is
locked).

## Wiring diagram

Not built yet. Recommended tool: **Fritzing** — it has parts for ESP32
dev boards and hobby servos, and produces a breadboard-style diagram
that's far more readable in a README than a raw schematic. Export as
SVG/PNG and drop it here as `hardware/wiring-diagram.svg`, referenced
from the root README.

If a proper schematic (net labels, real symbols for the buck converter,
BNO055, relay) is wanted later, KiCad is the step up from Fritzing.
