#include <string>

#include "DisplayApp.h"
#include <FreeRTOS.h>
#include <task.h>
#include <libraries/log/nrf_log.h>
#include <nrf_font.h>
#include <queue.h>
#include "components/datetime/DateTimeController.h"
#include <drivers/Cst816s.h>
#include "displayapp/screens/FirmwareUpdate.h"
#include "displayapp/screens/FirmwareValidation.h"
#include "systemtask/SystemTask.h"
#include "displayapp/screens/ScreenDisplay.h"
#include "displayapp/screens/BlackScreen.h"

using namespace Watch::Applications;

namespace {
  static inline bool in_isr(void) {
    return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;
  }

  TouchEvents Convert(Watch::Drivers::Cst816S::TouchInfos info) {
    if (info.isTouch) {
      switch (info.gesture) {
        case Watch::Drivers::Cst816S::Gestures::SingleTap:
          return TouchEvents::Tap;
        case Watch::Drivers::Cst816S::Gestures::LongPress:
          return TouchEvents::LongTap;
        case Watch::Drivers::Cst816S::Gestures::DoubleTap:
          return TouchEvents::DoubleTap;
        case Watch::Drivers::Cst816S::Gestures::SlideRight:
          return TouchEvents::SwipeRight;
        case Watch::Drivers::Cst816S::Gestures::SlideLeft:
          return TouchEvents::SwipeLeft;
        case Watch::Drivers::Cst816S::Gestures::SlideDown:
          return TouchEvents::SwipeDown;
        case Watch::Drivers::Cst816S::Gestures::SlideUp:
          return TouchEvents::SwipeUp;
        case Watch::Drivers::Cst816S::Gestures::None:
        default:
          return TouchEvents::None;
      }
    }
    return TouchEvents::None;
  }
}

DisplayApp::DisplayApp(Drivers::St7789 &lcd, Components::LittleVgl &lvgl, Drivers::Cst816S &touchPanel,
                       Controllers::Battery &batteryController, Controllers::Ble &bleController,
                       Controllers::DateTime &dateTimeController, Drivers::WatchdogView &watchdog,
                       Watch::Drivers::Acnt101& tempSensor) :
        lcd{lcd},
        lvgl{lvgl},
        batteryController{batteryController},
        bleController{bleController},
        dateTimeController{dateTimeController},
        watchdog{watchdog},
        touchPanel{touchPanel},        
        tempSensor{tempSensor} {
        }

void IdleTimerTouchCallback(TimerHandle_t xTimer) {
  auto disTask = static_cast<DisplayApp *>(pvTimerGetTimerID(xTimer));
  disTask->TouchPolling();
}

void DisplayApp::Start() {
  msgQueue = xQueueCreate(queueSize, itemSize);

  if(bleController.IsConnected()) SwitchApp(0); else  {SwitchApp(7); batteryController.setIsVibrate();}

  if (pdPASS != xTaskCreate(DisplayApp::Process, "displayapp", 800, this, 0, &taskHandle)) {
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }
}


void DisplayApp::Process(void* instance) {
  auto* app = static_cast<DisplayApp*>(instance);
  //app->InitHw();
  xTaskNotifyGive(xTaskGetCurrentTaskHandle());
  
  while (true) {
    app->Refresh();
  }
}

void DisplayApp::InitHw() {
  idleTimerPolling = xTimerCreate ("idleTimerPolling", pdMS_TO_TICKS(80), pdTRUE, this, IdleTimerTouchCallback);
  xTimerStart(idleTimerPolling, 0);
}

void DisplayApp::Refresh() {
  TickType_t queueTimeout;
  switch (state) {
    case States::Idle:
      IdleState();
      queueTimeout = portMAX_DELAY;
      break;
    case States::Running:
      RunningState();
      queueTimeout = 25;
      break;
    default:
      queueTimeout = portMAX_DELAY;
      break;
  }

  Messages msg;
  if (xQueueReceive(msgQueue, &msg, queueTimeout)) {
    switch (msg) {
      case Messages::GoToSleep:     
        //SetTouchMode(DisplayApp::TouchModes::Gestures); 
        //PushMessage(Messages::Clock);    
        //lcd.DisplayOff(); 
        PushMessageToSystemTask(Watch::System::Messages::OnDisplayTaskSleeping);
        state = States::Idle;
        break;
      case Messages::GoToRunning:         
        //lcd.DisplayOn();     
        state = States::Running;
        break;
      case Messages::UpdateBleConnection: 
        checkupdate = false;
        checkFall = false;
        checkImpact =false;
        checkCheckin =false;
        batteryController.setIsVibrate();
        //validator.Validate(); 
        if(bleController.IsConnected()) { SwitchApp(1);   appIndex =1; }          
        else SwitchApp(7);           
        break;
      case Messages::TouchEvent: {
        if (state != States::Running) break;  

        auto info = touchPanel.GetTouchInfo();
        auto gesture = Convert(info);
        batteryController.setTouch(info.x, info.y);
        if (gesture == TouchEvents::None) break;

        if(!currentScreen->OnTouchEvent(gesture)) {
          switch (gesture) {
            case TouchEvents::SwipeDown:                         
              break;
            case TouchEvents::SwipeUp:
              break;   
            case TouchEvents::SwipeLeft:
             if (!bleController.IsConnected() || checkupdate || checkCheckin|| checkFall || checkImpact) break;
              //SetFullRefresh(DisplayApp::FullRefreshDirections::LeftAnim);
              SetFullRefresh(DisplayApp::FullRefreshDirections::Left);
              if(appIndex>0) {       
                appIndex--;
                SwitchApp(appIndex);               
            } else  {appIndex =4;   SwitchApp(appIndex); }      
              break;
           case TouchEvents::SwipeRight:
             if (!bleController.IsConnected() || checkupdate || checkCheckin|| checkFall || checkImpact) break;
              //SetFullRefresh(DisplayApp::FullRefreshDirections::RightAnim);
              SetFullRefresh(DisplayApp::FullRefreshDirections::Right);
            if(appIndex<4) {     
                appIndex++;
                SwitchApp(appIndex);               
              } else {appIndex =0;   SwitchApp(appIndex);  } 
              break;
            case TouchEvents::DoubleTap:
              nrf_gpio_pin_clear(2); 
              checkupdate = false;
              checkFall = false;
              checkImpact =false;
              checkCheckin =false;
              appIndex=0; 
              PushMessage(Messages::Clock);
              PushMessageToSystemTask(Watch::System::Messages::GoToSleep);
              break;
            default:
              break;
          }
        }
      }
          break;
      case Messages::ButtonPushed:
          nrf_gpio_pin_clear(2); 
          checkupdate = false;
          checkFall = false;
          checkImpact =false;
          checkCheckin =false;
          batteryController.setDisturnOff(false);             
          appIndex=0; 
          PushMessageToSystemTask(Watch::System::Messages::GoToSleep);
          break;

      case Messages::BleFirmwareUpdateStarted:
          checkupdate= true;
          batteryController.setDisturnOff(true);
          currentScreen.reset(nullptr);
          batteryController.setIsVibrate();
          SetFullRefresh(DisplayApp::FullRefreshDirections::None); 
          currentScreen = std::make_unique<Screens::FirmwareUpdate>(this, bleController);       
          break;

      case Messages::Impact:
          //batteryController.setDisturnOff(true);
          checkImpact= true;
          batteryController.setButtonData(0x12); 
          batteryController.setIsAlert(0x12); 
          SetFullRefresh(DisplayApp::FullRefreshDirections::None); 
          PushMessageToSystemTask(Watch::System::Messages::AlwaysDisplay);    
          SwitchApp(5);      
          break;

      case Messages::Fall: 
          checkFall = true;
          //batteryController.setDisturnOff(true);
          batteryController.setButtonData(0x08);
          batteryController.setIsAlert(0x08); 
          SetFullRefresh(DisplayApp::FullRefreshDirections::None); 
          PushMessageToSystemTask(Watch::System::Messages::AlwaysDisplay);     
          SwitchApp(6);      
          break;

      case Messages::CheckIn:  
          checkCheckin = true;
          //batteryController.setDisturnOff(true);
          SetFullRefresh(DisplayApp::FullRefreshDirections::None); 
          PushMessageToSystemTask(Watch::System::Messages::AlwaysDisplay);
          SwitchApp(10);      
          break;

      case Messages::Clock: 
          if(batteryController.IsvalidatorFirmware()) {
            validator.Validate(); 
            batteryController.validatorFirmware(false);
          }

          batteryController.setDisturnOff(false);
          if(batteryController.IsCharging()) SwitchApp(8);
          else if(bleController.IsConnected()) SwitchApp(0); 
          else  SwitchApp(7);       
          break;

      case Messages::Lowbattery: 
          SetFullRefresh(DisplayApp::FullRefreshDirections::None); 
          batteryController.setDisturnOff(false);
          SwitchApp(9);      
          break;
      case Messages::Charging:
          SetFullRefresh(DisplayApp::FullRefreshDirections::None); 
          batteryController.setDisturnOff(false);     
          SwitchApp(8);
          break;
      default:
          break;
    }
  }
  TouchPolling();
}

void DisplayApp::TouchPolling(){
  if (state != States::Idle) {
    auto info = touchPanel.GetTouchInfo();
    if (info.action == 2) { // 2 = contact
      if (!currentScreen->OnTouchEvent(info.x, info.y)) {
        //lvgl.SetNewTapEvent(info.x, info.y);
      }
    } 
  } else  {if(!currentScreen->OnTouchEvent(0, 0)) {}}
}


void DisplayApp::RunningState() {
  if (!currentScreen->Refresh()) {
    PushMessage(Messages::Clock);
  }
  lv_task_handler();
}

void DisplayApp::IdleState() {
}

void DisplayApp::PushMessage(Messages msg) {
  if(in_isr()) {
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(msgQueue, &msg, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  } else {
    xQueueSend(msgQueue, &msg, portMAX_DELAY);
  }
}

void DisplayApp::StartApp(uint8_t app, DisplayApp::FullRefreshDirections direction) {
   SetFullRefresh(direction);
   SwitchApp(app);
}

void DisplayApp::SetFullRefresh(DisplayApp::FullRefreshDirections direction) {
  switch (direction) {
    case DisplayApp::FullRefreshDirections::Down:
      lvgl.SetFullRefresh(Components::LittleVgl::FullRefreshDirections::Down);
      break;
    case DisplayApp::FullRefreshDirections::Up:
      lvgl.SetFullRefresh(Components::LittleVgl::FullRefreshDirections::Up);
      break;
    case DisplayApp::FullRefreshDirections::Left:
      lvgl.SetFullRefresh(Components::LittleVgl::FullRefreshDirections::Left);
      break;
    case DisplayApp::FullRefreshDirections::Right:
      lvgl.SetFullRefresh(Components::LittleVgl::FullRefreshDirections::Right);
      break;
    case DisplayApp::FullRefreshDirections::LeftAnim:
      lvgl.SetFullRefresh(Components::LittleVgl::FullRefreshDirections::LeftAnim);
      break;
    case DisplayApp::FullRefreshDirections::RightAnim:
      lvgl.SetFullRefresh(Components::LittleVgl::FullRefreshDirections::RightAnim);
      break;
    default:
      break;
  }
}

void DisplayApp::SetTouchMode(DisplayApp::TouchModes mode) {
  touchMode = mode;
}

void DisplayApp::SwitchApp(uint8_t app ){    
    currentScreen.reset(nullptr); 
    onClockApp = true;
    //currentScreen->OnTouchEvent(0, 0); 
    //SetTouchMode(DisplayApp::TouchModes::Gestures);    
    switch (app)
    {
      case 0: 
        checkupdate = false;
        checkFall = false;
        checkImpact = false;
        checkCheckin =false;
        batteryController.setDisturnOff(false); 
        appIndex=0;  
        systemTask->UpdateTimeOut(25000);
        currentScreen = std::make_unique<Screens::ScreenDisplay>(this, dateTimeController, batteryController, bleController, tempSensor,Screens::ScreenDisplay::Modes::Clock);       
        break;
      case 1:
        systemTask->UpdateTimeOut(15000);
        currentScreen = std::make_unique<Screens::ScreenDisplay>(this, dateTimeController, batteryController, bleController, tempSensor,Screens::ScreenDisplay::Modes::Test);       
        break;
      case 2:
        systemTask->UpdateTimeOut(50000);
        currentScreen = std::make_unique<Screens::ScreenDisplay>(this, dateTimeController, batteryController, bleController, tempSensor,Screens::ScreenDisplay::Modes::Sensor);  
        break;
      case 3:
        systemTask->UpdateTimeOut(50000);
        currentScreen = std::make_unique<Screens::ScreenDisplay>(this, dateTimeController, batteryController, bleController, tempSensor,Screens::ScreenDisplay::Modes::Temp);  
        break;
      case 4:
        systemTask->UpdateTimeOut(50000);
        currentScreen = std::make_unique<Screens::ScreenDisplay>(this, dateTimeController, batteryController, bleController, tempSensor,Screens::ScreenDisplay::Modes::Oxi);  
        break;
      case 5:
        systemTask->UpdateTimeOut(50000);
        currentScreen = std::make_unique<Screens::ScreenDisplay>(this, dateTimeController, batteryController, bleController, tempSensor,Screens::ScreenDisplay::Modes::Impact);  
        break;
      case 6:
        systemTask->UpdateTimeOut(50000);
        currentScreen = std::make_unique<Screens::ScreenDisplay>(this, dateTimeController, batteryController, bleController, tempSensor,Screens::ScreenDisplay::Modes::Fall);  
        break;
      case 7:
        currentScreen = std::make_unique<Screens::ScreenDisplay>(this, dateTimeController, batteryController, bleController, tempSensor,Screens::ScreenDisplay::Modes::PairDis); 
        break;
      case 8:
        currentScreen = std::make_unique<Screens::ScreenDisplay>(this, dateTimeController, batteryController, bleController, tempSensor,Screens::ScreenDisplay::Modes::Charging);  
        break;
      case 9:
        currentScreen = std::make_unique<Screens::ScreenDisplay>(this, dateTimeController, batteryController, bleController, tempSensor,Screens::ScreenDisplay::Modes::LowBattery);  
        break;
      case 10:
        currentScreen = std::make_unique<Screens::ScreenDisplay>(this, dateTimeController, batteryController, bleController, tempSensor,Screens::ScreenDisplay::Modes::CheckIn); 
        break;
      case 11:
        currentScreen = std::make_unique<Screens::BlackScreen>(this); 
        break;

      default:
        break;
    }
}

void DisplayApp::Register(Watch::System::SystemTask* systemTask) {
  this->systemTask = systemTask;
}

void DisplayApp::PushMessageToSystemTask(Watch::System::Messages message) {
  if (systemTask != nullptr)
    systemTask->PushMessage(message);
}
