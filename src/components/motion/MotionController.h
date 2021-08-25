#pragma once

#include <cstdint>

namespace Watch {
  namespace Controllers {
    class MotionController {
    public:
      enum class DeviceTypes{
        Unknown,
        BMA421,
        BMA425,
      };

      void Update(int16_t x, int16_t y, int16_t z, int16_t acc);

      float X() const {
        return x;
      }
      float Y() const {
        return y;
      }
      float Z() const {
        return z;
      }
      float ACC() const {
        return acc;
      }
      bool ShouldWakeUp(bool isSleeping);

      void IsSensorOk(bool isOk);
      bool IsSensorOk() const {
        return isSensorOk;
      }

      DeviceTypes DeviceType() const {
        return deviceType;
      }

     // void Init(Watch::Drivers::Bma421::DeviceTypes types);

    private:
      float acc;
      float x;
      float y;
      float z;
      int16_t lastYForWakeUp = 0;
      bool isSensorOk = false;
      DeviceTypes deviceType = DeviceTypes::Unknown;
    };
  }
}