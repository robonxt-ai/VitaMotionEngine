#include <Arduino.h>
#include <string.h>
#include <PS4Controller.h>
#include "Ps4Input.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"

Ps4InputState g_ps4Input;

static float normalizeAxis(int8_t v) {
  const float inv = 1.0f / 128.0f;
  float f = (float)v * inv;
  if (f > 1.0f) f = 1.0f;
  if (f < -1.0f) f = -1.0f;
  return f;
}

static void ps4EventCallback();
static void ps4ConnectCallback();
static void ps4DisconnectCallback();

void ps4Setup() {
  memset(&g_ps4Input, 0, sizeof(g_ps4Input));

  PS4.attach(ps4EventCallback);
  PS4.attachOnConnect(ps4ConnectCallback);
  PS4.attachOnDisconnect(ps4DisconnectCallback);

  // Uses the default Bluetooth address. If your existing example calls
  // PS4.begin("xx:xx:xx:xx:xx:xx"), you can replace this line accordingly.
  PS4.begin();
  removePairedDevices();  // This helps to solve connection issues
  Serial.print("This device MAC is: ");
  printDeviceAddress();
  Serial.println("");
}

void ps4Loop() {
  // Reserved for future smoothing or edge-detection logic.
}

void removePairedDevices() {
  uint8_t pairedDeviceBtAddr[20][6];
  int count = esp_bt_gap_get_bond_device_num();
  esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
  for (int i = 0; i < count; i++) {
    esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
  }
}

void printDeviceAddress() {
  const uint8_t* point = esp_bt_dev_get_address();
  for (int i = 0; i < 6; i++) {
    char str[3];
    sprintf(str, "%02x", (int)point[i]);
    Serial.print(str);
    if (i < 5) {
      Serial.print(":");
    }
  }
}

static void ps4EventCallback() {
  if (!PS4.isConnected()) {
    return;
  }

  g_ps4Input.lxRaw = PS4.LStickX();
  g_ps4Input.lyRaw = PS4.LStickY();
  g_ps4Input.rxRaw = PS4.RStickX();
  g_ps4Input.ryRaw = PS4.RStickY();

  g_ps4Input.l2Raw = PS4.L2Value();
  g_ps4Input.r2Raw = PS4.R2Value();

  g_ps4Input.walkX = normalizeAxis(g_ps4Input.lxRaw);
  g_ps4Input.walkY = normalizeAxis(g_ps4Input.lyRaw);
  g_ps4Input.heightAxis = normalizeAxis(g_ps4Input.ryRaw);

  g_ps4Input.dpadUp = PS4.Up();
  g_ps4Input.dpadDown = PS4.Down();
  g_ps4Input.dpadLeft = PS4.Left();
  g_ps4Input.dpadRight = PS4.Right();

  g_ps4Input.cross = PS4.Cross();
  g_ps4Input.circle = PS4.Circle();
  g_ps4Input.square = PS4.Square();
  g_ps4Input.triangle = PS4.Triangle();

  g_ps4Input.l1 = PS4.L1();
  g_ps4Input.r1 = PS4.R1();

  g_ps4Input.share = PS4.Share();
  g_ps4Input.options = PS4.Options();
  g_ps4Input.ps = PS4.PSButton();
  g_ps4Input.touchpad = PS4.Touchpad();

  g_ps4Input.l3 = PS4.L3();
  g_ps4Input.r3 = PS4.R3();
}

static void ps4ConnectCallback() {
  g_ps4Input.connected = true;
}

static void ps4DisconnectCallback() {
  g_ps4Input.connected = false;

  g_ps4Input.walkX = 0.0f;
  g_ps4Input.walkY = 0.0f;
  g_ps4Input.heightAxis = 0.0f;

  g_ps4Input.lxRaw = 0;
  g_ps4Input.lyRaw = 0;
  g_ps4Input.rxRaw = 0;
  g_ps4Input.ryRaw = 0;
  g_ps4Input.l2Raw = 0;
  g_ps4Input.r2Raw = 0;

  g_ps4Input.dpadUp = false;
  g_ps4Input.dpadDown = false;
  g_ps4Input.dpadLeft = false;
  g_ps4Input.dpadRight = false;

  g_ps4Input.cross = false;
  g_ps4Input.circle = false;
  g_ps4Input.square = false;
  g_ps4Input.triangle = false;
  g_ps4Input.l1 = false;
  g_ps4Input.r1 = false;
  g_ps4Input.share = false;
  g_ps4Input.options = false;
  g_ps4Input.ps = false;
  g_ps4Input.touchpad = false;
}

void setControllerFeedback(uint8_t r, uint8_t g, uint8_t b, uint8_t rumbleSmall, uint8_t rumbleLarge) {
  if (!PS4.isConnected()) return;
  PS4.setLed(r, g, b);
  PS4.setRumble(rumbleSmall, rumbleLarge);
  PS4.sendToController();
}
