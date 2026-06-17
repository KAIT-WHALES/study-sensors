#include <iostream>
#include <algorithm>
#include <cmath>
#include <array>

#include "kernel_cfg.h"
#include "app.h"
#include <stdio.h>
#include <stdlib.h>

#include <libcpp/spike/IMU.h>
#include <libcpp/spike/Display.h>
#include <libcpp/spike/ForceSensor.h>
#include <libcpp/spike/ColorSensor.h>
#include <libcpp/spike/Button.h>
#include <libcpp/spike/Clock.h>
#include <libcpp/spike/Motor.h>
#include <serial/serial.h>
#include <serial/newlib.h>
#include <syssvc/serial.h>

#include <libcpp/spike/Port.h>

extern "C" {
  void __attribute__((weak)) _fini() {}
  void* __dso_handle __attribute__((weak)) = (void*)-1;
}

enum class Sensor { ColorSensor, IMU, UltraSonic };

namespace {
  using namespace spikeapi;
  Display g_display;
  ForceSensor g_forceSensor(EPort::PORT_D);
  ColorSensor g_colorSensor(EPort::PORT_E);
  Button g_button;
  FILE *fp;
  Sensor _sensor(Sensor::ColorSensor);
}

std::string ESensorToString(Sensor sensor) {
  switch (sensor) {
    case Sensor::ColorSensor: return "カラーセンサ";
    case Sensor::IMU: return "ジャイロセンサ";
    case Sensor::UltraSonic: return "超音波センサ";
  }
}

char ESensorToChar(Sensor sensor) {
  switch (sensor) {
    case Sensor::ColorSensor: return 'C';
    case Sensor::IMU: return 'I';
    case Sensor::UltraSonic: return 'U';
  }
}

void SerialCalibration() {
    // Bluetooth接続が確立されるまで待機
    bool connected = false;
    ER err;
    while (!connected) {
        err = serial_opn_por(SIO_BLUETOOTH_PORTID);
        fp = serial_open_newlib_file(SIO_BLUETOOTH_PORTID);
        if (err == E_OK && fp != nullptr) connected = true;
    }
}

void HardwareCalibration() {
  fprintf(fp, "使いたいセンサを選択してください(左右ボタンで切り替え、中央ボタンで決定)\n");
  fprintf(fp, "C:カラーセンサ, I:ジャイロセンサ, U:超音波センサ\n");
  int currentIdx = 0;
  std::string currentSensor = ESensorToString(_sensor);
  while (1) {
    if (g_button.isRightPressed()) {
      while (g_button.isRightPressed()) {}
      ++currentIdx;
    }
    if (g_button.isLeftPressed() && currentIdx > 0) {
      while (g_button.isLeftPressed()) {}
      --currentIdx;
    }
    g_display.showChar(ESensorToChar(static_cast<Sensor>(currentIdx%3)));
    if (g_button.isCenterPressed()) {
      while (g_button.isCenterPressed()) {}
      _sensor = static_cast<Sensor>(currentIdx%3);
      fprintf(fp, "「%s」が選択されました。\n", ESensorToString(_sensor).c_str());
      break;
    }
  }
}

/* メインタスク(起動時にのみ関数コールされる) */
void main_task(intptr_t unused) {
  g_display.showChar('S'); // Serial
  SerialCalibration();
  g_display.showChar('E'); // End

  HardwareCalibration();

  // フォースセンサのボタンが押されるまで待機
  fprintf(fp, "フォースセンサを押してください\n");
  while(1) {
    if (g_forceSensor.isPressed(0.5f)) {
      while (g_forceSensor.isTouched()) { }
      break;
    }
  } 
  fprintf(fp, "フォースセンサが押されました。\n");
  sta_cyc(TRACER_TASK_CYC); 
  ext_tsk();
}

void tracer_task(intptr_t unused) {
  if (g_forceSensor.isPressed(0.5f)) {
    stp_cyc(TRACER_TASK_CYC);
  }

  ext_tsk();
}
