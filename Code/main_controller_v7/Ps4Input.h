#pragma once

#include <stdint.h>

struct Ps4InputState {
  bool connected;

  int8_t lxRaw;
  int8_t lyRaw;
  int8_t rxRaw;
  int8_t ryRaw;

  uint8_t l2Raw;
  uint8_t r2Raw;

  float walkX;
  float walkY;

  float heightAxis;

  bool dpadUp;
  bool dpadDown;
  bool dpadLeft;
  bool dpadRight;

  bool cross;
  bool circle;
  bool square;
  bool triangle;
  bool l1;
  bool r1;
  bool share;
  bool options;
  bool ps;
  bool touchpad;
  bool l3;
  bool r3;
};

extern Ps4InputState g_ps4Input;
void setControllerFeedback(uint8_t r, uint8_t g, uint8_t b, uint8_t rumbleSmall, uint8_t rumbleLarge);
