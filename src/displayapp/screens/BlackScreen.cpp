#include "BlackScreen.h"
#include "../DisplayApp.h"

using namespace Watch::Applications::Screens;

BlackScreen::BlackScreen(Watch::Applications::DisplayApp *app) : Screen(app) {
  backgroundLabel = lv_label_create(lv_scr_act(), nullptr);
  lv_label_set_long_mode(backgroundLabel, LV_LABEL_LONG_CROP);
  lv_obj_set_size(backgroundLabel, 240, 240);
  lv_obj_set_pos(backgroundLabel, 0, 0);
  lv_label_set_text(backgroundLabel, "");    
}

BlackScreen::~BlackScreen() {
  lv_obj_clean(lv_scr_act());
}

bool BlackScreen::Refresh() {
  return running;
}

bool BlackScreen::OnButtonPushed() {
  running = false;
  return true;
}
