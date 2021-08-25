#pragma once

#include <cstdint>
#include "Screen.h"
#include <lvgl/lvgl.h>

namespace Watch {
  namespace Applications {
    namespace Screens {

      class BlackScreen : public Screen{
        public:
          BlackScreen(DisplayApp* app);
          ~BlackScreen() override;

          bool Refresh() override;
          bool OnButtonPushed() override;

        private:
         lv_obj_t* backgroundLabel;    
          bool running = true;

      };
    }
  }
}
