| Component | Pin | Connection | Notes |
| :--- | :-- | :--- | :--- |
| RP2040 | GP8 (UART1 TX) | -> 74HC126 Pin 2 (1A) | Microcontroller's Transmit pin |
| | GP9 (UART1 RX) | <- LX-16A Serial Pin | Microcontroller's Receive pin (always listening) |
| | GP10 (Digital Out) | -> 74HC126 Pin 1 (1OE) | Direction control pin (HIGH for TX, LOW for RX) |
| | VCC (3.3V) | -> 74HC126 Pin 14 (VCC) | Power for the 74HC126 |
| | GND | -> 74HC126 Pin 7 (GND) | Common Ground |
| 74HC126 | Pin 1 (1OE) | <- RP2040 GP10 | Output Enable for buffer 1 (Active HIGH) |
| | Pin 2 (1A) | <- RP2040 GP8 (UART1 TX) | Input to buffer 1 |
| | Pin 3 (1Y) | -> LX-16A Serial Pin | Output from buffer 1 to servo data line |
| | Pin 7 (GND) | <- RP2040 GND | Ground connection |
| | Pin 14 (VCC) | <- RP2040 VCC (3.3V) | Power connection |
| LX-16A Servo | Serial Pin | <-> 74HC126 Pin 3 & RP2040 GP9 (UART1 RX) | Single data line for half-duplex communication |
| | Power (Center) | <- External Power Supply (6V-8.4V) | Dedicated power for servo |
| | GND | <- Common Ground | Ground connection |
