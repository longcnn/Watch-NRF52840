#pragma once
#include <FreeRTOS.h>
#include <task.h>
#include <drivers/St7789.h>
#include <drivers/SpiMaster.h>
#include <bits/unique_ptr.h>
#include <queue.h>
#include "components/gfx/Gfx.h"
#include "components/battery/BatteryController.h"
#include "components/ble/BleController.h"
#include "components/datetime/DateTimeController.h"
#include "components/ble/NotificationManager.h"
#include "components/firmwarevalidator/FirmwareValidator.h"
#include "drivers/Cst816s.h"
#include "drivers/Acnt101.h"
#include "LittleVgl.h"
#include <date/date.h>
#include "displayapp/screens/Modal.h"
#include <drivers/Watchdog.h>
#include "TouchEvents.h"
#include "Apps.h"
#include "timers.h"
#include "systemtask/Messages.h"


namespace Watch {
  namespace System {
    class SystemTask;
  };
  namespace Applications {
    class DisplayApp {
      public:
        enum class States {Idle, Running};
        enum class Messages : uint8_t {GoToSleep,GoToRunning,TouchEvent, UpdateBleConnection,Charging,ButtonPushed, BleFirmwareUpdateStarted,Impact,Fall,CheckIn,Clock,Lowbattery};
        enum class TouchModes { Gestures, Polling };
         enum class FullRefreshDirections { None, Up, Down, Left, Right, LeftAnim, RightAnim };

        DisplayApp(Drivers::St7789 &lcd, Components::LittleVgl &lvgl, Drivers::Cst816S &,
                   Controllers::Battery &batteryController, Controllers::Ble &bleController,
                   Controllers::DateTime &dateTimeController, Drivers::WatchdogView &watchdog,               
                   Watch::Drivers::Acnt101& tempSensor);
        void Start();
        void PushMessage(Messages msg);
        void SwitchApp(uint8_t app) ;
        void StartApp(uint8_t app, DisplayApp::FullRefreshDirections direction);
        void SetTouchMode(TouchModes mode);
        void SetFullRefresh(FullRefreshDirections direction);
        void Register(Watch::System::SystemTask* systemTask);
        void TouchPolling();

     private:
        TaskHandle_t taskHandle;
        TimerHandle_t idleTimerTouch;

        Watch::Drivers::St7789& lcd;
        Watch::Components::LittleVgl& lvgl;   
        Watch::Controllers::Battery &batteryController;
        Watch::Controllers::Ble &bleController;
        Watch::Controllers::DateTime& dateTimeController;
        Watch::Drivers::WatchdogView& watchdog;
        Watch::Drivers::Cst816S& touchPanel;
        Watch::System::SystemTask* systemTask = nullptr;
        Watch::Drivers::Acnt101& tempSensor;
        Watch::Controllers::FirmwareValidator validator;      

        
        std::unique_ptr<Screens::Screen> currentScreen;
        Apps currentApp = Apps::None;
        Apps returnToApp = Apps::None;
        FullRefreshDirections returnDirection = FullRefreshDirections::None;
        TouchEvents returnTouchEvent = TouchEvents::None;
        TouchModes touchMode = TouchModes::Polling;
        States state = States::Running;
        QueueHandle_t msgQueue;
        static constexpr uint8_t queueSize = 10;
        static constexpr uint8_t itemSize = 1;
        void RunningState();
        void IdleState();
        static void Process(void* instance);
        void InitHw();
        void Refresh();
        void ReturnApp(Apps app, DisplayApp::FullRefreshDirections direction, TouchEvents touchEvent);
        void LoadApp(Apps app, DisplayApp::FullRefreshDirections direction);
        void PushMessageToSystemTask(Watch::System::Messages message);

        TimerHandle_t idleTimerPolling;

        bool isClock = true;
        uint8_t appIndex=0;       
        Apps nextApp = Apps::None;
        bool onClockApp = true; 
        bool checkupdate = false;
        bool checkFall = false;
        bool checkImpact = false ;
        bool checkCheckin =false;       
    };
  }
}

