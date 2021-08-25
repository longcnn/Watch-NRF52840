#pragma once

#include <memory>

#include <FreeRTOS.h>
#include <task.h>
#include <drivers/SpiMaster.h>
#include <drivers/St7789.h>
#include <algorithm>
#include <vector>
#include "components/battery/BatteryController.h"
#include "displayapp/DisplayApp.h"
#include <drivers/Watchdog.h>
#include <drivers/SpiNorFlash.h>
#include "SystemMonitor.h"
#include "components/ble/NimbleController.h"
#include "timers.h"
#include <drivers/Kx022.h>
#include <drivers/Kx126.h>
#include <drivers/Acnt101.h>
#include <drivers/Acnt101Adc.h>
#include <drivers/Gh301.h>
#include <drivers/VchR02.h>
#include <components/heartrate/HeartRateController.h>
#include <heartratetask/HeartRateTask.h>
#include <components/motion/MotionController.h>
#include "Messages.h"

namespace Watch {
  namespace System {
    class SystemTask {
      public:
        enum class States {One, Two, Three, Four, Five, Six};
  
       SystemTask(Drivers::SpiMaster &spi, Drivers::St7789 &lcd,
                   Watch::Drivers::SpiNorFlash& spiNorFlash,
                   Drivers::TwiMaster& twiMaster, Drivers::Cst816S &touchPanel,
                  //Drivers::Kx022& motionSensor,
                    Drivers::Kx126& motionSensor,
                  // Drivers::Gh301& heartRateSensor,
                   Watch::Controllers::MotionController& motionController,
                   Components::LittleVgl &lvgl,
                   Controllers::Battery &batteryController, Controllers::Ble &bleController,
                   Controllers::DateTime &dateTimeController,
                   Watch::Controllers::NotificationManager& manager,
                   Watch::Drivers::Acnt101& tempSensor,
                   Watch::Applications::DisplayApp& displayApp);

        void Start();
        void PushMessage(Messages msg);

        void OnButtonPushed();
        void OnTouchEvent();
        void OnIdle();
        void ReadTempSensor();
        void ResetSensor();
        void CheckACC();
        void CheckCommon();
        void CheckTracking();
        void CheckHeartbeat();
        void CheckSendLowBattery();
        void UpdateTimeOut(uint32_t timeout);

        Watch::Controllers::NimbleController& nimble() {return nimbleController;};

        bool IsSleeping() const {
        return isSleeping;
      }

      private:
        TaskHandle_t taskHandle;

          Watch::Drivers::SpiMaster& spi;
          Watch::Drivers::St7789& lcd;
          Watch::Drivers::SpiNorFlash& spiNorFlash;
          Watch::Drivers::TwiMaster& twiMaster;
          Watch::Drivers::Cst816S& touchPanel;
          Watch::Components::LittleVgl& lvgl;
          Watch::Controllers::Battery& batteryController;
          
          Watch::Controllers::Ble& bleController;
          Watch::Controllers::DateTime& dateTimeController;
          QueueHandle_t systemTasksMsgQueue;
          std::atomic<bool> isSleeping{false};
          std::atomic<bool> isGoingToSleep{false};
          std::atomic<bool> isWakingUp{false};
          Watch::Drivers::Watchdog watchdog;
          Watch::Drivers::WatchdogView watchdogView;
          Watch::Controllers::NotificationManager& notificationManager;
          Watch::Drivers::Acnt101& tempSensor;
          Watch::Applications::DisplayApp& displayApp;
          //Watch::Drivers::Acnt101Adc tempSensorAdc;
          //Watch::Drivers::Kx022& motionSensor;
          Watch::Drivers::Kx126& motionSensor;
          Watch::Controllers::MotionController& motionController;
          Watch::Controllers::NimbleController nimbleController;   
          Watch::Drivers::VchR02 heartRateSensor; 
          States state = States::One;   

        static constexpr uint8_t pinSpiSck = 16;
        static constexpr uint8_t pinSpiMosi = 14;
        static constexpr uint8_t pinSpiMiso = 15;
        static constexpr uint8_t pinSpiFlashCsn = 23;
        static constexpr uint8_t pinLcdCsn = 22;
        static constexpr uint8_t pinLcdDataCommand = 11;
        static constexpr uint8_t pinSpiAccCsn = 8;
        static constexpr uint8_t pinButton = 27;
        static constexpr uint8_t pinTouchIrq = 25; 
       // static constexpr uint8_t pinAccIrq = 5; 
        //static constexpr uint8_t pinPowerPresentIrq = 20; 


        static void Process(void* instance);
        void Work();
        void ReloadIdleTimer() const;
        void CheckLowbattery();
        void CheckCheckIn();
        void GoToRunning();
        void CheckFallImpact();
        void sendLowbatteryCellphone();
        void UpdateMotion();
        
        bool isFallDiscoveryTimerRunning = false;
        uint8_t FallDiscoveryTimer = 0;
        bool isImpactDiscoveryTimerRunning = false;

        uint8_t battery_notifyTimer =4;
        TimerHandle_t idleTimer;
        TimerHandle_t idleTimerAcc;
        TimerHandle_t idleTimerCommon;
        TimerHandle_t idleTimerTracking;
        TimerHandle_t idleTimerHeartbeat;
        TimerHandle_t idleTimerSendLowBattery;

        bool doNotGoToSleep = false;
        bool sendLowbattery= true;
        uint32_t tempData = 0;
        uint8_t Touch=0;
        bool checkcharging = false;
        bool precheckcharging = false;
        bool checklowbattery = true;
        bool isBleDiscoveryTimerRunning = false;
        uint8_t bleDiscoveryTimer = 0;
      
        bool checkbright= false;
        uint8_t BrightDiscoveryTimer = 0;
        uint8_t prehour=0;
        uint8_t checktime=0;
        uint8_t checknum =0;
        uint8_t preAlert =0;
        bool checkheartbeat = true;
        uint8_t checktimeheart=0;
        uint8_t checknumheart =0;
        float preAcc=0;
        float prepreAcc=0;;
        float maxPeak=0;
        uint16_t idmaxPeak=0;
        std::vector<float> arrAccPeak; 
        std::vector<float> arrAccTime; 
        uint16_t idStageOne =0;
        uint16_t idStageTwo =0;
        uint16_t flagTimerPeak =0;
        uint16_t flagTimerDisFall =0;
        uint8_t flagTimerStageOne=0;
        bool flagcheckStageOne= false;
     
#if configUSE_TRACE_FACILITY == 1
        SystemMonitor<FreeRtosMonitor> monitor;
#else
        SystemMonitor<DummyMonitor> monitor;
#endif

    };
  }
}
