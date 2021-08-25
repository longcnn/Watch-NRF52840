#pragma once


#include <chrono>
#include <cstdint>
#include <memory>
#include "nrf_delay.h"
#include "Screen.h"
#include "components/datetime/DateTimeController.h"
#include "drivers/Acnt101.h"
#include <drivers/Gh301.h>
#include <drivers/VchR02.h>
#define LV_WATCH_BLACK      lv_color_hex(000000)
#define LV_WATCH_WHITE      lv_color_hex(0xffffff)

extern "C" {
    #include <lvgl/src/lv_core/lv_obj.h>
}

namespace Watch {
  namespace Controllers {
    class Battery;
    class Ble;
  }

  namespace Applications {
    namespace Screens {

      class ScreenDisplay : public Screen {
        public:
        enum class ClockType : uint8_t { H24, H12 };
        enum class Modes {Clock, Test, CheckIn, Impact, Fall, Sensor,PairDis, Charging, LowBattery,Oxi,Temp};
          ScreenDisplay(DisplayApp* app,
                  Controllers::DateTime& dateTimeController,
                  Controllers::Battery& batteryController,
                  Controllers::Ble& bleController,
                  Watch::Drivers::Acnt101& tempSensor,
                  Modes mode
                  );
          ~ScreenDisplay() override;
          bool Refresh() override;
          bool OnButtonPushed() override;
          void OnObjectEvent(lv_obj_t *pObj, lv_event_t i);
          bool OnTouchEvent(TouchEvents event) override;
          bool OnTouchEvent(uint16_t x, uint16_t y) override;
          void buttonEven(lv_obj_t *obj, Modes mode);

        private:
                    
          static const char* MonthToString(Watch::Controllers::DateTime::Months month);
          static const char* DayOfWeekToString(Watch::Controllers::DateTime::Days dayOfWeek);
          static char const *DaysString[];
          static char const *MonthsString[];

          lv_obj_t* logo; 
          lv_obj_t* backgroundLabel;  
          lv_obj_t* label_time;
          lv_obj_t* label_date;
          lv_obj_t* batteryIcon;
          lv_obj_t* bleIcon;
          lv_obj_t* batteryPlug;
          lv_obj_t* buttonBig;     
          lv_obj_t* buttonSmall;
          lv_obj_t* heartbeatValue;
          lv_obj_t* tempCValue;
          lv_obj_t* tempFValue;
          lv_obj_t* timeoutLine;
          lv_obj_t* imgSensor; 
          lv_obj_t* labelleft;;
          lv_obj_t* labelright;  
          lv_obj_t* labelpoint;  
          lv_obj_t* checkButton;
          lv_obj_t* imgpair; 
          lv_obj_t* lablebpm;
          lv_obj_t* lablepercent;
          lv_obj_t* lableftemp;
          lv_obj_t* lablectemp;
          lv_obj_t* tempChar;
          lv_obj_t* tempWire;
          /*  
          lv_obj_t* lableleftCircle;
          lv_obj_t* labletopCircle;
          lv_obj_t* lablerightCircle;
          
          lv_obj_t* lableleftCircleSmall;
          lv_obj_t* labletopCircleSmall;
          lv_obj_t* lablerightCircleSmall;
          lv_obj_t* lablebottomCircleSmall;
         
          lv_obj_t* Xmax; 
          lv_obj_t* Ymax;
          lv_obj_t* Zmax;
          lv_obj_t* x;
          lv_obj_t* y;
          lv_obj_t* z;
          lv_obj_t* gwatch;
          */
          lv_obj_t* textline1;
          lv_obj_t* textline2;
          lv_obj_t * label;
          lv_obj_t * arcCharging;
          lv_obj_t * loader;

          lv_obj_t * buttonSmallCircle;
          lv_obj_t * whiteCircle;

          Controllers::DateTime& dateTimeController;
          Controllers::Battery& batteryController;
          Controllers::Ble& bleController;
          Watch::Drivers::VchR02 heartRateSensor;          
          Watch::Drivers::Acnt101& tempSensor;
          uint16_t flagstick = 0;
          uint16_t flag = 0;
          Modes mode = Modes::Clock;
          ClockType clockType = ClockType::H12;
          bool running = true;
          uint32_t timeoutCountStart;
          uint32_t timeoutCountStartL;
          uint32_t timeoutCountStartH;
          uint32_t timeout;
          uint32_t timeoutCountStartS;
          bool checkVibrate= true;          
          bool checkcharging = false;
          uint16_t touchx,touchy;
          uint16_t k=0;
          bool checkTest =false;
          bool checkimpactFall = false;
          bool checkPolling = true;
          uint8_t heartrate =0;
          uint8_t spO2 ;
          uint8_t t ;
          uint8_t mod =0;
          uint8_t modPolling =0;
          uint8_t modt =0;
          uint16_t modoxi=0;
          uint8_t tempTouch=0;
          uint8_t tempTouchSmall=0;
          uint8_t flagbattery=31;
          uint8_t batteryPercent;

          bool checkTouchBigButton =false;
          bool checkTouchSmallButton = false;
          bool enableTouchBigButton = false;
          bool enableTouchSmallButton = false;
          float rawTemp =0.0;
      };
    }
  }
}