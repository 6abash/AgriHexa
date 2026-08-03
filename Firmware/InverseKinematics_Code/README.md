Overview & Architecture
The firmware is structured to receive high-level body pose and gait commands via UART from an external master controller (such as an ESP32), process them through real-time mathematical models, and output precise PWM signals to an 18-servo cluster (3 motors per leg across 6 legs).
Key Components:
UART Telemetry & State Parsing (read_uart_packets):

Continuously polls uart1 for packed binary structures (RobotState) containing target body pitch, roll, yaw, stride scaling, strafe scaling, body height, and walking states framed by header 0xAA and footer 0x55.
