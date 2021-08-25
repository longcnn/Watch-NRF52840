#include "FirmwareUpdate.h"
#include <lvgl/lvgl.h>
#include "components/ble/BleController.h"
#include "../DisplayApp.h"

using namespace Watch::Applications::Screens;

extern lv_font_t San_Francisco_22;
extern lv_font_t San_Francisco_50;

FirmwareUpdate::FirmwareUpdate(Watch::Applications::DisplayApp *app, Watch::Controllers::Ble& bleController) :
      Screen(app), bleController{bleController} {

  lv_obj_t* backgroundLabel = lv_label_create(lv_scr_act(), nullptr);
  lv_label_set_long_mode(backgroundLabel, LV_LABEL_LONG_CROP);
  lv_obj_set_size(backgroundLabel, 240, 240);
  lv_obj_set_pos(backgroundLabel, 0, 0);
  lv_label_set_text_static(backgroundLabel, "");

  titleLabel = lv_label_create(lv_scr_act(), nullptr);
  lv_label_set_recolor(titleLabel, true);     
  lv_label_set_text(titleLabel, "#ff0000 Firmware update#");
  lv_obj_set_auto_realign(titleLabel, true);
 
  lv_obj_set_style_local_text_font(titleLabel, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &San_Francisco_22);
  lv_obj_align(titleLabel, nullptr, LV_ALIGN_IN_TOP_MID, 0, 50);

  percentLabel = lv_label_create(lv_scr_act(), nullptr);
  lv_label_set_text(percentLabel, "");
  lv_obj_set_auto_realign(percentLabel, true);
  lv_obj_set_style_local_text_font(percentLabel, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &San_Francisco_50);
  lv_obj_align(percentLabel, nullptr,  LV_ALIGN_CENTER, 0, 40);
  
  startTime = xTaskGetTickCount();
}

FirmwareUpdate::~FirmwareUpdate() {
  lv_obj_clean(lv_scr_act());
}

bool FirmwareUpdate::Refresh() {
  switch (bleController.State()) {
    default:
    case Watch::Controllers::Ble::FirmwareUpdateStates::Idle:
      // This condition makes sure that the app is exited if somehow it got
      // launched without a firmware update. This should never happen.
      if (state != States::Error) {
        if (xTaskGetTickCount() - startTime > (60 * 1024)) {
          UpdateError();
          state = States::Error;
        }
      } else if (xTaskGetTickCount() - startTime > (5 * 1024)) {
        running = false;
      }
      break;
    case Watch::Controllers::Ble::FirmwareUpdateStates::Running:
      if (state != States::Running)
        state = States::Running;
      DisplayProgression();
      break;
    case Watch::Controllers::Ble::FirmwareUpdateStates::Validated:
      if (state != States::Validated) {
        UpdateValidated();
        state = States::Validated;
      }
      break;
    case Watch::Controllers::Ble::FirmwareUpdateStates::Error:
      if (state != States::Error) {
        UpdateError();
        state = States::Error;
      }
      if (xTaskGetTickCount() - startTime > (5 * 1024)) {
        running = false;
      }
      break;
  }
  return running;
}

void FirmwareUpdate::DisplayProgression() const {
  float current = bleController.FirmwareUpdateCurrentBytes() / 1024.0f;
  float total = bleController.FirmwareUpdateTotalBytes() / 1024.0f;
  int16_t pc = (current / total) * 100.0f;
  sprintf(percentStr, "%d%%", pc);
  lv_label_set_text(percentLabel, percentStr);
}

void FirmwareUpdate::UpdateValidated() {
  lv_label_set_recolor(percentLabel, true);
  lv_label_set_text(percentLabel, "#00ff00 0000!#");
}

void FirmwareUpdate::UpdateError() {
  lv_label_set_recolor(percentLabel, true);
  lv_label_set_text(percentLabel, "#ff0000 3333!#");
  startTime = xTaskGetTickCount();
}

bool FirmwareUpdate::OnButtonPushed() {
  return true;
}