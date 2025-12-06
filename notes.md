# my notes

2025-09-25
So it seems that if the ESP32 is powered by the small 3.3v (?) flat phone (?) rechargable battery, it's not enough to keep the ESP32, display, and logic signals powered. Need to check if using a 5v portable power bank is okay if directly wired to that battery port. 

Updated. The parser now expects messages in the format:
<id>,<type>,<value>

Summary of accepted IDs, types, and values
- walk_forward
  - Type: button
  - Values: press
  - Example: walk_forward,button,press
- walk_backward
  - Type: button
  - Values: press
  - Example: walk_backward,button,press
- turn_left
  - Type: button
  - Values: press
  - Example: turn_left,button,press
- turn_right
  - Type: button
  - Values: press
  - Example: turn_right,button,press
- demo
  - Type: button
  - Values: press
  - Example: demo,button,press
- stop
  - Type: button
  - Values: press
  - Example: stop,button,press
- enable
  - Type: button
  - Values: press
  - Example: enable,button,press
- disable
  - Type: button
  - Values: press
  - Example: disable,button,press
- speed_up
  - Type: button
  - Values: press
  - Example: speed_up,button,press
- speed_down
  - Type: button
  - Values: press
  - Example: speed_down,button,press
- sit
  - Type: button
  - Values: press
  - Example: sit,button,press
- stand
  - Type: button
  - Values: press
  - Example: stand,button,press

D-pad
- up, down, left, right, center
  - Type: dpad
  - Values: press, release
  - Examples:
    - up,dpad,press
    - up,dpad,release
    - center,dpad,press

Slider
- speed
  - Type: slider
  - Values: 0–100 (integer)
  - Example: speed,slider,75

Notes
- Only canonical IDs are accepted; all aliases have been removed.
- Buttons act on press only.
- D-pad uses both press and release (hold-to-act; release returns to idle).