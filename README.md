# VitaMotionEngine

Robot dog or something.

# NOTES

MUST CONNECT SERVOS TO SERIAL2

Buslinker library notes:
- Requires logic stepup for ESP32s and RP2040s
- Essential for proper readings from servos powered at 7-12V
- Working on angle calibration for stand up program


madhephaestus/lx16a-servo notes:
- Requires logic stepup for ESP32s and RP2040s
- Compatible with 74HC126 circuit and buslinker
- One-pin setup incompatible with logic stepup
- Movement range: 0-240000
- Use serial2 (pin 8) for TX, pin 9 or other for trigger
