#include "MotionController.h"
#include <math.h>
#include <algorithm>

using namespace Watch::Controllers;

void MotionController::Update(int16_t x, int16_t  y, int16_t  z, int16_t acc) {

  this->x = y/4096.0f;
  this->y = x/4096.0f;
  this->z = z/4096.0f;
  this->acc = std::sqrt(x*x+y*y+z*z)/4096.0f;
}

bool MotionController::ShouldWakeUp(bool isSleeping) {
  if ((x + 335) <= 670 && z < 0) {
    if (not isSleeping) {
      if (y <= 0) {
        return false;
      } else {
        lastYForWakeUp = 0;
        return false;
      }
    }

    if (y >= 0) {
      lastYForWakeUp = 0;
      return false;
    }
    if (y + 230 < lastYForWakeUp) {
      lastYForWakeUp = y;
      return true;
    }
  }
  return false;
}
void MotionController::IsSensorOk(bool isOk) {
  isSensorOk = isOk;
}
/*
void MotionController::Init(Pinetime::Drivers::Bma421::DeviceTypes types) {
  switch(types){
    case Drivers::Bma421::DeviceTypes::BMA421: this->deviceType = DeviceTypes::BMA421; break;
    case Drivers::Bma421::DeviceTypes::BMA425: this->deviceType = DeviceTypes::BMA425; break;
    default: this->deviceType = DeviceTypes::Unknown; break;
  }
}
*/