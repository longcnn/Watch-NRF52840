#pragma once

#include "Screen.h"
#include <lvgl/src/lv_core/lv_obj.h>
#include "FreeRTOS.h"

namespace Watch {
  namespace Controllers {
    class Ble;
  }
  namespace Applications {
    namespace Screens {

      class FirmwareUpdate : public Screen {
      public:
        FirmwareUpdate(DisplayApp* app, Watch::Controllers::Ble& bleController);
        ~FirmwareUpdate() override;

        bool Refresh() override;

      private:
        enum class States { Idle, Running, Validated, Error };
        Watch::Controllers::Ble& bleController;
        lv_obj_t* bar1;
        lv_obj_t* percentLabel;
        lv_obj_t* titleLabel;
        mutable char percentStr[10];

        States state = States::Idle;

        void DisplayProgression() const;

        bool OnButtonPushed() override;

        void UpdateValidated();

        void UpdateError();

        TickType_t startTime;
      };
    }
  }
}
