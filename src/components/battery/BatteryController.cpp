#include "BatteryController.h"
#include <hal/nrf_gpio.h>
#include <libraries/log/nrf_log.h>
#include <algorithm>
#include <lvgl/src/lv_core/lv_obj.h>
#include <math.h>
using namespace Watch::Controllers;
Battery *Battery::instance = nullptr;

Battery::Battery() {
  instance = this;
}

void Battery::Init() {
  nrf_gpio_cfg_input(chargingPin, (nrf_gpio_pin_pull_t)GPIO_PIN_CNF_PULL_Pullup);
  nrf_gpio_cfg_input(powerPresentPin, (nrf_gpio_pin_pull_t)GPIO_PIN_CNF_PULL_Pullup);
  MotorControllerInit();
}

void Battery::Update() { 
  isCharging = !nrf_gpio_pin_read(chargingPin);
  isPowerPresent = !nrf_gpio_pin_read(powerPresentPin);

  if (isReading) {
    return;
  }
  // Non blocking read
  isReading = true;
  SaadcInit();

  nrfx_saadc_sample();
  }

void Battery::AdcCallbackStatic(nrfx_saadc_evt_t const *event) {
  instance->SaadcEventHandler(event);
}

void Battery::SaadcInit() {
  nrfx_saadc_config_t adcConfig = NRFX_SAADC_DEFAULT_CONFIG;
  APP_ERROR_CHECK(nrfx_saadc_init(&adcConfig, AdcCallbackStatic));

  nrf_saadc_channel_config_t adcChannelConfig = {.resistor_p = NRF_SAADC_RESISTOR_DISABLED,
                                                 .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
                                                 .gain = NRF_SAADC_GAIN1_4,
                                                 .reference = NRF_SAADC_REFERENCE_INTERNAL,
                                                 .acq_time = NRF_SAADC_ACQTIME_40US,
                                                 .mode = NRF_SAADC_MODE_SINGLE_ENDED,
                                                 .burst = NRF_SAADC_BURST_ENABLED,
                                                 .pin_p = batteryVoltageAdcInput,
                                                 .pin_n = NRF_SAADC_INPUT_DISABLED};
  APP_ERROR_CHECK(nrfx_saadc_channel_init(0, &adcChannelConfig));
  APP_ERROR_CHECK(nrfx_saadc_buffer_convert(&saadc_value, 1));
}

void Battery::SaadcEventHandler(nrfx_saadc_evt_t const* p_event) {
  const uint16_t battery_max = 4170; 
  const uint16_t battery_min = 3200; 

  if (p_event->type == NRFX_SAADC_EVT_DONE) {

    APP_ERROR_CHECK(nrfx_saadc_buffer_convert(&saadc_value, 1));

    // A hardware voltage divider divides the battery voltage by 2
    // ADC gain is 1/4
    // thus adc_voltage = battery_voltage / 2 * gain = battery_voltage / 8
    // reference_voltage is 600mV
    // p_event->data.done.p_buffer[0] = (adc_voltage / reference_voltage) * 1024
    voltage = p_event->data.done.p_buffer[0] * (8 * 600) / 1024;

    if (voltage > battery_max) {
      percentRemaining = 100;
    } else if (voltage < battery_min) {
      percentRemaining = 0;
    } else {
      percentRemaining = (voltage - battery_min) * 100 / (battery_max - battery_min);

      if(isCharging ){
        percentRemaining = percentRemaining -4;
      }
    }

    nrfx_saadc_uninit();
    isReading = false;
  }
}


void Battery::setButtonData( uint8_t data) {
      buttonData = data;
      isTouch = true;
      MotorControllerSetDuration(200);
}

void Battery::sendAlerts( uint8_t data) {
      buttonData = data;
      isTouch = true;
}

void Battery::setLongTap( bool data) {  checkLongTap = data;}  
void Battery::setIsTouch( bool data) {  isTouch = data;}  
void Battery::setIsVibrate(void){ MotorControllerSetDuration(200);} 
void Battery::StopVibrate(void) {MotorControllerStop();}

void Battery::impactCharacteristic(uint8_t zz, uint8_t yy){
  impactyy=yy;
  if(yy==0x03) impactzz=zz;
}

void Battery::checkinCharacteristic(uint8_t zz, uint8_t yy){
  switch (yy){
  case 0x01:
     CheckinTime1 =zz;
    break;
  case 0x02:
    CheckinTime2 =zz;
   break;
  case 0x03:
    isTimer1Start=true;
    isTimer1Display= true;
   break;
  case 0x04:
    isTimer2Start=true;
    isTimer2Display= true;
  break;
  case 0x05:
    isTimer1Start=false;
   break;
  case 0x06:
    isTimer2Start=false;
  break;
  case 0x07:
    isTimer1Done=true;
   break;
  case 0x08:
    isTimer2Done=true;
  break;
  
  default:
    break;
  }
}

void Battery::fallCharacteristic(uint8_t zz, uint8_t yy){
  fallyy=yy;
  switch (yy){
  case 0x03:
    fallHighpeak = zz;
    break;
  case 0x04:
    falltime = zz;
    break;
  case 0x05:
    fallLowpeak = zz; 
    break; 
  case 0x08:
    fallEndpeak = zz; 
    break; 
  default:
    break;
  }
}

void Battery::trackCharacteristic(uint8_t zz, uint8_t yy,uint8_t nn){
  switch (nn){
  case 0x00:
    istracking=true;
    numtracking=zz;
    timetracking=yy;
    break;
  case 0x01:
    istracking=false;
   break;
  case 0x03:
    isheartbeat =true;
    timeheartbeat= yy;
    break;
  case 0x04:
    isheartbeat =false;    
    break;
  }
}

void Battery::setimpactyy(uint8_t data) {impactyy=data;}
void Battery::setfallyy(uint8_t data) {fallyy = data;}
void Battery::setisTimer1Done( bool data) {isTimer1Done=data;}
void Battery::setisTimer2Done( bool data) {isTimer2Done=data;}
void Battery::setCurrentHour(uint8_t data) { currentHour =data ;};
void Battery::setCurrentMinute(uint8_t data) { currentMinute = data;};
void Battery::setCurrentSecond(uint8_t data) { currentSecond= data;};
void Battery::setisTimer1Display(bool data) {isTimer1Display=data;};
void Battery::setisTimer2Display(bool data) {isTimer2Display=data;};
void Battery::setGoToSleep(bool data) {GoToSleep=data;};
void Battery::setAccData( float data) {AccData = data;}
void Battery::setTouch( uint8_t x, uint8_t y) {
  touchx=x;
  touchy=y;
}
void Battery::setIsAlert(uint8_t data){isAlert=data;}
void Battery::setIsheartbeat(bool data){isheartbeat=data;}
void Battery::setIstracking(bool data){istracking=data;}
void Battery::setIslowbattery(bool data){islowbattery=data;}
void Battery::setcheckVibrate(bool data){checkVibrate=data;}
void Battery::setisButtonPushed(bool data){isButtonPushed=data;}
void Battery::setDisturnOff(bool data){isDisturnOff=data;};

void Battery::setxyz( float _x,float _y,float _z) {
x=_x;
y=_y;
z=_z;
}

void Battery::setXmax(float x){xmax=x;};
void Battery::setYmax(float y){ymax=y;};
void Battery::setZmax(float z){zmax=z;};

void Battery::validatorFirmware(bool data){validator = data;}


