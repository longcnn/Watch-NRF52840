#pragma once

#include <cstdint>
#include <chrono>
#include <components/gfx/Gfx.h>
#include "Screen.h"
#include <bits/unique_ptr.h>
#include <libs/lvgl/src/lv_core/lv_style.h>
#include <libs/lvgl/src/lv_core/lv_obj.h>
#include <components/battery/BatteryController.h>
#include <components/ble/BleController.h>
#include <components/motion/MotionController.h>

namespace Watch {
  namespace Applications {
    namespace Screens {

      class Motion : public Screen{
        public:
          Motion(DisplayApp* app, Controllers::MotionController& motionController);
          ~Motion() override;

          bool Refresh() override;
          bool OnButtonPushed() override;

        private:
          Controllers::MotionController& motionController;
          lv_obj_t * chart;
          lv_chart_series_t * ser1;
          lv_chart_series_t * ser2;
          lv_chart_series_t * ser3;

          lv_obj_t* labelStep;
          lv_obj_t* labelStepValue;
          static constexpr uint8_t nbStepsBufferSize = 9;
          char nbStepsBuffer[nbStepsBufferSize+1];
          bool running = true;

      };
    }
  }
}
