#pragma once

#include <lvgl/src/lv_core/lv_obj.h>
#include <chrono>
#include <cstdint>
#include <memory>
#include "Screen.h"
#include "components/datetime/DateTimeController.h"

namespace Watch {
  namespace Controllers {
    class Battery;
    class HeartRateController;
    class Ble;
    class NotificationManager;
  }

  namespace Applications {
    namespace Screens {

      template <class T>
      class DirtyValueTes {
        public:
          explicit DirtyValueTes(T v) { value = v; }
          explicit DirtyValueTes(T& v) { value = v; }
          bool IsUpdated() const { return isUpdated; }
          T& Get() { this->isUpdated = false; return value; }

          DirtyValueTes& operator=(const T& other) {
            if (this->value != other) {
              this->value = other;
              this->isUpdated = true;
            }
            return *this;
          }
        private:
          T value;
          bool isUpdated = true;
      };
      class Test : public Screen {
        public:
          Test(DisplayApp* app,
                  Controllers::DateTime& dateTimeController,
                  Controllers::Battery& batteryController,
                  Controllers::Ble& bleController);
          ~Test() override;

          bool Refresh() override;
          bool OnButtonPushed() override;
          void OnObjectEvent(lv_obj_t *pObj, lv_event_t i);
        private:

          //bool OnTouchEvent(TouchEvents event);

          static const char* MonthToString(Watch::Controllers::DateTime::Months month);
          static const char* DayOfWeekToString(Watch::Controllers::DateTime::Days dayOfWeek);
          static char const *DaysString[];
          static char const *MonthsString[];
         
          uint8_t currentHour = 0;
          uint8_t currentMinute = 0;

          DirtyValueTes<float> batteryPercentRemaining  {0};
          DirtyValueTes<bool> bleState {false};
          DirtyValueTes<std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>> currentDateTime;
          //DirtyValueTes<bool> notificationState {false};

          lv_obj_t* label_time;
          //lv_obj_t* label_date;
          //lv_obj_t* backgroundLabel;
          lv_obj_t* batteryIcon;
          lv_obj_t* bleIcon;
          lv_obj_t* batteryPlug;
          //lv_obj_t* labeleft;
          //lv_obj_t* laberight;
          lv_obj_t* buttonTest;
          lv_obj_t*labelbuttonTest;          
          //lv_obj_t* labelright;  
           //lv_obj_t* labelpoint;        
         // lv_style_t* labelRelStyle;
          //lv_style_t style;

          Controllers::DateTime& dateTimeController;
          Controllers::Battery& batteryController;
          Controllers::Ble& bleController;

          bool running = true;
          uint16_t checkButton=0;

      };
    }
  }
}
