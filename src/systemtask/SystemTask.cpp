#include <libraries/log/nrf_log.h>
#include <libraries/gpiote/app_gpiote.h>
#include <drivers/Cst816s.h>
#include "displayapp/LittleVgl.h"
#include <hal/nrf_rtc.h>
#include "components/ble/NotificationManager.h"
#include <host/ble_gatt.h>
#include <host/ble_hs_adv.h>
#include "SystemTask.h"
#include <nimble/hci_common.h>
#include <host/ble_gap.h>
#include <host/util/util.h>
#include <drivers/InternalFlash.h>
#include "main.h"
#include "components/ble/NimbleController.h"
#include "SEGGER_RTT.h" 

using namespace Watch::System;
using namespace std;

namespace {
  static inline bool in_isr(void) {
    return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;
  }
}

void IdleTimerCallback(TimerHandle_t xTimer) {
  auto sysTask = static_cast<SystemTask *>(pvTimerGetTimerID(xTimer));
  sysTask->OnIdle();
}

void IdleTimerAccCallback(TimerHandle_t xTimer) {
  auto sysTask = static_cast<SystemTask *>(pvTimerGetTimerID(xTimer));
  sysTask->CheckACC();
}
void IdleTimerCommonCallback(TimerHandle_t xTimer) {
  auto sysTask = static_cast<SystemTask *>(pvTimerGetTimerID(xTimer));
  sysTask->CheckCommon();
}

void IdleTimerTrackingCallback(TimerHandle_t xTimer) {
  auto sysTask = static_cast<SystemTask *>(pvTimerGetTimerID(xTimer));
  sysTask->CheckTracking();
}

void IdleTimerHeartbeatCallback(TimerHandle_t xTimer) {
  auto sysTask = static_cast<SystemTask *>(pvTimerGetTimerID(xTimer));
  sysTask->CheckHeartbeat();
}

void IdleTimerSendLowBattery(TimerHandle_t xTimer) {
  auto sysTask = static_cast<SystemTask *>(pvTimerGetTimerID(xTimer));
  sysTask->CheckSendLowBattery();
}

SystemTask::SystemTask(Drivers::SpiMaster &spi, 
                       Drivers::St7789 &lcd,
                       Watch::Drivers::SpiNorFlash& spiNorFlash,
                       Drivers::TwiMaster& twiMaster,Drivers::Cst816S &touchPanel,
                       Drivers::Kx126& motionSensor,
                       Watch::Controllers::MotionController& motionController,
                       //Drivers::Kx022& motionSensor,
                       //Drivers::Gh301& heartRateSensor,
                       Components::LittleVgl &lvgl,
                       Controllers::Battery &batteryController, Controllers::Ble &bleController,
                       Controllers::DateTime &dateTimeController,
                       Watch::Controllers::NotificationManager& notificationManager,
                       Watch::Drivers::Acnt101& tempSensor,
                       Watch::Applications::DisplayApp& displayApp) 
        :   spi{spi}, 
            lcd{lcd}, 
            spiNorFlash{spiNorFlash},
            twiMaster{twiMaster}, 
            touchPanel{touchPanel},
            lvgl{lvgl}, 
            batteryController{batteryController},                       
            bleController{bleController}, 
            dateTimeController{dateTimeController},
            watchdog{}, 
            watchdogView{watchdog}, 
            notificationManager{notificationManager},
            tempSensor{tempSensor},
            displayApp{displayApp},
            motionSensor{motionSensor},
            motionController{motionController},
            nimbleController(*this, bleController,dateTimeController, notificationManager, batteryController, spiNorFlash) {
}

void SystemTask::Start() {
  systemTasksMsgQueue = xQueueCreate(10, 1);
  if (pdPASS != xTaskCreate(SystemTask::Process, "MAIN", 350, this, 0, &taskHandle))
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
}

void SystemTask::Process(void *instance) {
  auto *app = static_cast<SystemTask *>(instance);
  app->Work();
}

void SystemTask::Work() {
  watchdog.Setup(5);
  watchdog.Start();
  APP_GPIOTE_INIT(2);

  spi.Init();
  spiNorFlash.Init();
  spiNorFlash.Wakeup();
  nimbleController.Init();
  nimbleController.StartAdvertising();
  lcd.Init();

  twiMaster.Init();
  touchPanel.Init();
  batteryController.Init();
  motionSensor.SoftReset();
  heartRateSensor.Enable();
  tempSensor.Init(); 

  spi.Sleep();
  spi.Init();

  motionSensor.Init();
  
  displayApp.Register(this);
  displayApp.Start();

  nrf_gpio_cfg_sense_input(pinButton, (nrf_gpio_pin_pull_t)GPIO_PIN_CNF_PULL_Pullup, (nrf_gpio_pin_sense_t)GPIO_PIN_CNF_SENSE_Low); 
  nrfx_gpiote_in_config_t pinConfig;
  pinConfig.skip_gpio_setup = true;
  pinConfig.hi_accuracy = false;
  pinConfig.is_watcher = false;
  pinConfig.sense = (nrf_gpiote_polarity_t)NRF_GPIOTE_POLARITY_HITOLO;
  pinConfig.pull = (nrf_gpio_pin_pull_t)GPIO_PIN_CNF_PULL_Pulldown;
  nrfx_gpiote_in_init(pinButton, &pinConfig, nrfx_gpiote_evt_handler);

  nrf_gpio_cfg_sense_input(pinTouchIrq, (nrf_gpio_pin_pull_t) GPIO_PIN_CNF_PULL_Pullup, (nrf_gpio_pin_sense_t) GPIO_PIN_CNF_SENSE_Low);
  pinConfig.skip_gpio_setup = true;
  pinConfig.hi_accuracy = false;
  pinConfig.is_watcher = false;
  pinConfig.sense = (nrf_gpiote_polarity_t) NRF_GPIOTE_POLARITY_HITOLO;
  pinConfig.pull = (nrf_gpio_pin_pull_t) GPIO_PIN_CNF_PULL_Pullup;

  nrfx_gpiote_in_init(pinTouchIrq, &pinConfig, nrfx_gpiote_evt_handler);
  /*
  nrf_gpio_cfg_sense_input(pinAccIrq, (nrf_gpio_pin_pull_t)GPIO_PIN_CNF_PULL_Pulldown, (nrf_gpio_pin_sense_t)GPIO_PIN_CNF_SENSE_Low);
  pinConfig.pull = (nrf_gpio_pin_pull_t)GPIO_PIN_CNF_PULL_Pulldown;
  nrfx_gpiote_in_init(pinAccIrq, &pinConfig, nrfx_gpiote_evt_handler); 

 
  nrf_gpio_cfg_sense_input(pinPowerPresentIrq, (nrf_gpio_pin_pull_t) NRF_GPIO_PIN_NOPULL, (nrf_gpio_pin_sense_t) GPIO_PIN_CNF_SENSE_Low);
  pinConfig.sense = NRF_GPIOTE_POLARITY_TOGGLE;
  pinConfig.pull = NRF_GPIO_PIN_NOPULL;
  pinConfig.is_watcher = false;
  pinConfig.hi_accuracy = false;
  pinConfig.skip_gpio_setup = true;
  nrfx_gpiote_in_init(pinPowerPresentIrq, &pinConfig, nrfx_gpiote_evt_handler);
 */
     
  idleTimer = xTimerCreate ("idleTimer", pdMS_TO_TICKS(25000), pdFALSE, this, IdleTimerCallback);
  idleTimerAcc = xTimerCreate ("idleTimerAcc", pdMS_TO_TICKS(50), pdTRUE, this, IdleTimerAccCallback);
  idleTimerCommon = xTimerCreate ("idleTimerCommon", pdMS_TO_TICKS(400), pdTRUE, this, IdleTimerCommonCallback);
  idleTimerTracking = xTimerCreate ("idleTimerAcc", pdMS_TO_TICKS(60000), pdTRUE, this, IdleTimerTrackingCallback);
  idleTimerHeartbeat = xTimerCreate ("idleTimerHeartbeat", pdMS_TO_TICKS(60000), pdTRUE, this, IdleTimerHeartbeatCallback);
  idleTimerSendLowBattery = xTimerCreate ("idleTimerSendLowBattery", pdMS_TO_TICKS(3600000), pdTRUE, this, IdleTimerSendLowBattery);
  xTimerStart(idleTimer, 0);
  xTimerStart(idleTimerAcc, 0);
  xTimerStart(idleTimerCommon, 0);  
  
  // Suppress endless loop diagnostic
  #pragma clang diagnostic push
  #pragma ide diagnostic ignored "EndlessLoop"
  while(true) {
    batteryController.Update(); 
    uint8_t msg;
    if (xQueueReceive(systemTasksMsgQueue, &msg, 100)) {    
      Messages message = static_cast<Messages >(msg); 
      switch(message) {
        case Messages::GoToRunning:       
          //spi.Wakeup();
          twiMaster.Wakeup();
          //touchPanel.Wakeup();
          lcd.Wakeup(); 
          if(!bleController.IsConnected()){
          //nimbleController.ReInit();
          nimbleController.StartAdvertising();
          }
          xTimerStart(idleTimer, 0);   
          spiNorFlash.Wakeup();  
           
          displayApp.PushMessage(Watch::Applications::DisplayApp::Messages::GoToRunning);
          isSleeping = false;
          isWakingUp = false;      
          break;
        case Messages::GoToSleep:
          xTimerStop(idleTimer, 0);
          isGoingToSleep = true;
          //displayApp.PushMessage(Watch::Applications::DisplayApp::Messages::Clock);
          displayApp.PushMessage(Watch::Applications::DisplayApp::Messages::GoToSleep);
          heartRateSensor.Disable(); 
          break;
        case Messages::BleConnected:
          ReloadIdleTimer();
          isBleDiscoveryTimerRunning = true;
          bleDiscoveryTimer = 5;
          //nimbleController.StartDiscovery();
          if(isSleeping && !isWakingUp) GoToRunning();           
          displayApp.PushMessage(Watch::Applications::DisplayApp::Messages::UpdateBleConnection);         
          break;
        case Messages::BleFirmwareUpdateStarted:
          batteryController.setGoToSleep(false);
          if(isSleeping && !isWakingUp) GoToRunning();
          displayApp.PushMessage(Watch::Applications::DisplayApp::Messages::BleFirmwareUpdateStarted);
          break;
        case Messages::BleFirmwareUpdateFinished:          
          if (bleController.State() == Watch::Controllers::Ble::FirmwareUpdateStates::Validated) {
            NVIC_SystemReset();
          }
          batteryController.setGoToSleep(true);
          xTimerStart(idleTimer, 0);
          break;
        case Messages::OnTouchEvent:
          ReloadIdleTimer();
          if(batteryController.getDisturnOff()) break;
          batteryController.setGoToSleep(true);
          break;
        case Messages::OnButtonEvent:
          ReloadIdleTimer();
          GoToRunning();
          state = States::One;          
          displayApp.PushMessage(Watch::Applications::DisplayApp::Messages::Clock);
          break;

        case Messages::TouchWakeUp: {
          twiMaster.Wakeup();
          auto touchInfo = touchPanel.GetTouchInfo();
          twiMaster.Sleep();
          if (touchInfo.isTouch && (touchInfo.gesture == Watch::Drivers::Cst816S::Gestures::DoubleTap)) {                                                                                              
          GoToRunning();
          displayApp.PushMessage(Watch::Applications::DisplayApp::Messages::Clock);
          }
        } break;

        case Messages::AlwaysDisplay:
         batteryController.setGoToSleep(false);
          xTimerStart(idleTimer, 0);
          break;
        case Messages::OnDisplayTaskSleeping:         
          spiNorFlash.Sleep();
          lcd.Sleep(); 
          //spi.Sleep();
          
          //if(!bleController.IsConnected()) nimbleController.StopAdv();
          NRF_POWER->TASKS_LOWPWR = 1;         
          isSleeping = true;
          isGoingToSleep = false;
          //touchPanel.Sleep();
          twiMaster.Sleep();
          break;
        case Messages::OnChargingEvent:
        batteryController.setIsVibrate();
          break;
        default: break;
      }
    }

     if (isBleDiscoveryTimerRunning) {
      if (bleDiscoveryTimer == 0) {
        isBleDiscoveryTimerRunning = false;
        nimbleController.StartDiscovery();
      } else {
        bleDiscoveryTimer--;
      }
     }

    //monitor.Process();
    uint32_t systick_counter = nrf_rtc_counter_get(portNRF_RTC_REG);
    dateTimeController.UpdateTime(systick_counter);
    if(nrf_gpio_pin_read(pinButton))  watchdog.Kick();
  }
  #pragma clang diagnostic pop
}

void SystemTask::UpdateMotion() {
  if (isGoingToSleep or isWakingUp)
    return;

 //if (isSleeping)
    //spi.Wakeup();

  auto motionValues = motionSensor.Process();
  
 // if (isSleeping)
   //pi.Sleep();
  //motionController.IsSensorOk(motionSensor.IsOk());
  motionController.Update(motionValues.x, motionValues.y, motionValues.z, motionValues.acc);
}

void SystemTask::OnButtonPushed() {     
  if(isGoingToSleep) return;
  if(!isSleeping) {
    PushMessage(Messages::OnButtonEvent);
    displayApp.PushMessage(Watch::Applications::DisplayApp::Messages::ButtonPushed);
  }
  else {
    if(!isWakingUp) {
       PushMessage(Messages::OnButtonEvent);
    }
  }
}

void SystemTask::GoToRunning() {
  if(isGoingToSleep || (!isSleeping) ||isWakingUp) return;
  isWakingUp = true;
  PushMessage(Messages::GoToRunning);
}

void SystemTask::OnTouchEvent() {
  if (isGoingToSleep)return;
  if (!isSleeping) {
    PushMessage(Messages::OnTouchEvent);
     displayApp.PushMessage(Watch::Applications::DisplayApp::Messages::TouchEvent);
  } else if (!isWakingUp) {
    PushMessage(Messages::TouchWakeUp);
  }
}

void SystemTask::PushMessage(Messages msg) {
  if (msg == Messages::GoToSleep) {
    isGoingToSleep = true;
  }

  if(in_isr()) {
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(systemTasksMsgQueue, &msg, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
      /* Actual macro used here is port specific. */
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    }
  } else {
    xQueueSend(systemTasksMsgQueue, &msg, portMAX_DELAY);
  }
}

void SystemTask::OnIdle() {
  if(doNotGoToSleep) return;
  nrf_gpio_pin_clear(2); 
  displayApp.PushMessage(Watch::Applications::DisplayApp::Messages::Clock);
  PushMessage(Messages::GoToSleep);
}

void SystemTask::ReloadIdleTimer() const {
  if(isSleeping || isGoingToSleep) return;
  xTimerReset(idleTimer, 0);
}

void SystemTask::ReadTempSensor() {
  if(tempSensor.getIsCount()==0)
  {
   tempSensor.timer_temp_start();
   tempSensor.setIsCount(true);
  }  
  else 
   tempData++;
}

 void SystemTask::ResetSensor() {
  tempSensor.setIsCount(false);
  tempSensor.ReadData(tempData);
  tempData=0;
}

void SystemTask::CheckFallImpact(){
if(bleController.IsConnected()) {
   UpdateMotion(); 
   batteryController.setAccData(motionController.ACC());
  if(batteryController.getfallyy()!=0x02) { 
      //Ax < 0.3g, Ay < 0.3g and Az < 0.3g for a time t > 300ms (referred to as Freefall later on)
        if((std::fabs(motionController.X())<0.6f) && (std::fabs(motionController.Y())<0.6f) && (std::fabs(motionController.Z())<0.6f)){
          idStageOne++;
          flagcheckStageOne= true;
        }
        if(flagcheckStageOne) flagTimerStageOne++;
        if((flagTimerStageOne==32)&& (idStageOne==32)) {state = States::Two; }
        else if(flagTimerStageOne>32) {  idStageOne=0; flagcheckStageOne =false;flagTimerStageOne=0;} 
      // Ay < −2g, Ax > 0g and Az > 0g (referred to as Arm Flying Up later on)
      // Atot > 6g (referred to as Extra Hard later on)
      if(((motionController.X()>0.0f) && (motionController.Y()<-2.0f) && (motionController.Z()>0.0f)) || (motionController.ACC()>batteryController.getfallHighpeak())) state = States::Two;

      switch (state) {
            case States::One:
              arrAccPeak.clear();
              arrAccTime.clear(); 
              flagTimerDisFall=0; 
              flagTimerPeak=0; 
              idmaxPeak=0;
              maxPeak=0;
              //SEGGER_RTT_printf(0, "Case One\n");                             
              break;
            case States::Two: // get the main peak and all residual peaks
              if(flagTimerPeak>(batteryController.getfalltime()/0.05f)){
                state = States::Three;
                preAcc=0;
                prepreAcc=0;
                flagTimerPeak=0;
                idStageTwo=0;
                flagTimerDisFall=0; 
                break;
              }
              if((preAcc >= prepreAcc) && (preAcc > motionController.ACC())){
                arrAccPeak.push_back(preAcc); 
                arrAccTime.push_back(flagTimerPeak);
               // SEGGER_RTT_printf(0, "Case Two\n");       
                idStageTwo++;
              }
              preAcc=motionController.ACC();
              prepreAcc=preAcc;
              flagTimerPeak++;
              break;
            case States::Three://checks for a high enough main peak (big enough impact with the ground)
              //A high enough peak means greater than 4g for "Freefall" and "Arm flying upwards", and greater than 6g for the third case
              maxPeak=*std::max_element(arrAccPeak.begin(),arrAccPeak.end());              
              if( maxPeak>batteryController.getfallHighpeak()){
                idmaxPeak=std::distance(arrAccPeak.begin(),max_element(arrAccPeak.begin(),arrAccPeak.end()));
                //SEGGER_RTT_printf(0, "Case Three\n"); 
                state = States::Four;
              } else  state = States::One;
              break;
            case States::Four: //checks for residuals of the fall.
              //A residual peak needs to be within 0.75 seconds after the main peak and needs to exceed 1.8g for it to be valid.
             // SEGGER_RTT_printf(0, "arrAccTime.size()=%d\n", uint8_t(arrAccTime.size()));
              //SEGGER_RTT_printf(0, "Case Four\n"); 
              for (size_t i = 0; i < arrAccTime.size(); i++)  {
                if(arrAccTime[idmaxPeak+i]>(arrAccTime[idmaxPeak]+15))   {
                  if((arrAccPeak[idmaxPeak+i-1] > batteryController.getfallLowpeak()) || (arrAccPeak[idmaxPeak+i] > batteryController.getfallLowpeak()) || (arrAccPeak[idmaxPeak+i+1] > batteryController.getfallLowpeak()) || (arrAccPeak[idmaxPeak+i+2] > batteryController.getfallLowpeak())|| (arrAccPeak[idmaxPeak+i+3] > batteryController.getfallLowpeak()))  {state = States::Five; break;}
                  else {state = States::One; break;}
                }
              }  
            break;  
        
          case States::Five: // Another way to pass the fifth stage is to have a residual peak larger than 3g. after 5 seconds
           if((arrAccPeak[arrAccTime.size()-1] > batteryController.getfallEndpeak()) || (arrAccPeak[arrAccTime.size()-2] >  batteryController.getfallEndpeak()) || (arrAccPeak[arrAccTime.size()-3] >  batteryController.getfallEndpeak()) || (arrAccPeak[arrAccTime.size()-4] >  batteryController.getfallEndpeak()) || (arrAccPeak[arrAccTime.size()-4] >  batteryController.getfallEndpeak()))
            {
              //SEGGER_RTT_printf(0, "Enable Impact\n");
              if(isSleeping && !isWakingUp) GoToRunning();
              displayApp.PushMessage(Watch::Applications::DisplayApp::Messages::Fall);
              batteryController.setfallyy(0x01);
              arrAccPeak.clear();
              arrAccTime.clear();  
              state = States::Six; 
              break;
            } 
            else state = States::One;
           break;
          case States::Six: // additional 15 seconds waiting for either an Activity or DOUBLE_TAP interrupt
             // SEGGER_RTT_printf(0, "Case Six\n"); 
              arrAccPeak.clear();
              arrAccTime.clear();    
              flagTimerDisFall++;              
            //  SEGGER_RTT_printf(0, "flagTimerDisFall=%d\n", uint16_t(flagTimerDisFall));
              if(flagTimerDisFall>500){
                state = States::One;
                break;
              } 
              if(flagTimerDisFall>150){
                if(motionController.ACC() >2.0f) {displayApp.PushMessage(Watch::Applications::DisplayApp::Messages::Clock);state = States::One; break;}
              }
              break;
            default:
              break;      
            }
            if(batteryController.getfallyy()==0x07){
                if(isSleeping && !isWakingUp) GoToRunning();
                displayApp.PushMessage(Watch::Applications::DisplayApp::Messages::Fall);
                batteryController.setfallyy(0x01);
              }
        } 

  if((batteryController.getimpactyy()!=0x02)) {
      if(motionController.ACC()>batteryController.getimpactzz()) isImpactDiscoveryTimerRunning =true;
      if(isImpactDiscoveryTimerRunning || (batteryController.getimpactyy()==0x05)){
        if(isSleeping && !isWakingUp) GoToRunning();
        displayApp.PushMessage(Watch::Applications::DisplayApp::Messages::Impact);
        batteryController.setimpactyy(0x01);
        isImpactDiscoveryTimerRunning= false;
      }
    }
} else state = States::One;
}

void SystemTask::CheckACC() {  
  CheckFallImpact();
}

void SystemTask::CheckCommon(){
  
    //nimbleController.ble_checkevent();
    nimbleController.ble_acc_checkevent();
    CheckCheckIn();
    if(batteryController.Isheartbeat()&& checkheartbeat) { xTimerStart(idleTimerHeartbeat, 0); checkheartbeat = false;}
    if(batteryController.getGoToSleep())  doNotGoToSleep = false;  else doNotGoToSleep = true; 

    if(batteryController.Istracking()){  
      if (batteryController.getIsAlert()!=preAlert) { 
        xTimerStart(idleTimerTracking, 0);
        checktime=0;
        checknum=0;
      }
      preAlert= batteryController.getIsAlert();
    }

    CheckLowbattery();  

	  if(!checkcharging && batteryController.IsCharging()){ 
      //  if(isSleeping && !isWakingUp) GoToRunning();
      //   displayApp.PushMessage(Watch::Applications::DisplayApp::Messages::Charging);  
    } else if(checkcharging && !batteryController.IsCharging() ){   
      //  if(isSleeping && !isWakingUp) GoToRunning();
      //   displayApp.PushMessage(Watch::Applications::DisplayApp::Messages::Charging);
    } 
    checkcharging=batteryController.IsCharging();
}

void SystemTask::CheckSendLowBattery(){   
  if(bleController.IsConnected()) { 
    batteryController.sendAlerts(0x05);    
  }
}
void SystemTask::CheckTracking(){
      if((checknum==batteryController.getnumtracking()) || !batteryController.Istracking()|| (batteryController.getIsAlert()==0x02)){
        xTimerStop(idleTimerTracking, 0);
        checktime=0;
        checknum=0;
        return;
     }
      checktime++;
      if(checktime==batteryController.gettimetracking()){
      batteryController.sendAlerts(0x07);
      checktime=0;
      checknum++;
    };
}

void SystemTask::CheckHeartbeat(){
  if(!batteryController.Isheartbeat()) { xTimerStop(idleTimerHeartbeat, 0); checkheartbeat = true; checktimeheart=0;return;}
    checktimeheart++;
   if(checktimeheart==batteryController.gettimeheartbeat()){
      batteryController.sendAlerts(0x10);
      xTimerStop(idleTimerHeartbeat, 0);
      checktimeheart=0;
      checkheartbeat = true;
      batteryController.setIsheartbeat(false);
   };     
}

void SystemTask::CheckLowbattery(){
    if((batteryController.PercentRemaining()<28)&& !batteryController.IsCharging() && !checklowbattery) { 
        batteryController.setButtonData(0x05);
        xTimerStart(idleTimerSendLowBattery, 0);  
        // if(isSleeping && !isWakingUp) GoToRunning();
        // displayApp.PushMessage(Watch::Applications::DisplayApp::Messages::Lowbattery);
        checklowbattery = true;
    } 
    if((batteryController.PercentRemaining()>35) || batteryController.IsCharging()) {
       checklowbattery = false;
       xTimerStop(idleTimerSendLowBattery, 0);  
    }
    if((batteryController.PercentRemaining()<15)&& !batteryController.IsCharging()){

     }
}

void SystemTask::CheckCheckIn(){
    uint8_t hour = dateTimeController.Hours();
    uint8_t minute = dateTimeController.Minutes();
    uint8_t second = dateTimeController.Seconds();
    batteryController.setCurrentHour(hour);
    batteryController.setCurrentMinute(minute);
    batteryController.setCurrentSecond(second);

    if((batteryController.isTimerStart1() && (batteryController.getCheckinTime1()== hour) && (minute==0) && (second<2)) ||(batteryController.isTimerStart2() && (batteryController.getCheckinTime2()== hour) && (minute==0) && (second<2))) {             
          if(isSleeping && !isWakingUp) GoToRunning();
          displayApp.PushMessage(Watch::Applications::DisplayApp::Messages::CheckIn); 
          batteryController.setGoToSleep(false);
          if (batteryController.isTimerStart1()) batteryController.setisTimer1Display(false) ;
          if (batteryController.isTimerStart2()) batteryController.setisTimer2Display(false) ;   
     }
}

void SystemTask::UpdateTimeOut(uint32_t timeout){
    xTimerChangePeriod(idleTimer, pdMS_TO_TICKS(timeout), 0);
}
