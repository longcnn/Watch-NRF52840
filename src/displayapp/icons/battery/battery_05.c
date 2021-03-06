#include "lvgl/lvgl.h"

#ifndef LV_ATTRIBUTE_MEM_ALIGN
#define LV_ATTRIBUTE_MEM_ALIGN
#endif

#ifndef LV_ATTRIBUTE_IMG_BATTERY_05
#define LV_ATTRIBUTE_IMG_BATTERY_05
#endif

const LV_ATTRIBUTE_MEM_ALIGN LV_ATTRIBUTE_LARGE_CONST LV_ATTRIBUTE_IMG_BATTERY_05 uint8_t battery_05_map[] = {
 0x04, 0x02, 0x04, 0xff, 	/*Color of index 0*/
  0x9b, 0x99, 0x9b, 0xff, 	/*Color of index 1*/
  0xe4, 0xe2, 0xe3, 0xff, 	/*Color of index 2*/
  0x4c, 0x4a, 0x4c, 0xff, 	/*Color of index 3*/
  0xb4, 0xb2, 0xb4, 0xff, 	/*Color of index 4*/
  0x64, 0x66, 0x64, 0xff, 	/*Color of index 5*/
  0xf4, 0xf4, 0xf4, 0xff, 	/*Color of index 6*/
  0xac, 0xad, 0xac, 0xff, 	/*Color of index 7*/
  0x79, 0x7a, 0x79, 0xff, 	/*Color of index 8*/
  0x30, 0x31, 0x30, 0xff, 	/*Color of index 9*/
  0xc8, 0xc7, 0xc8, 0xff, 	/*Color of index 10*/
  0x5c, 0x5d, 0x5c, 0xff, 	/*Color of index 11*/
  0x9e, 0x9f, 0x9e, 0xff, 	/*Color of index 12*/
  0x70, 0x70, 0x70, 0xff, 	/*Color of index 13*/
  0xfc, 0xfc, 0xfc, 0xff, 	/*Color of index 14*/
  0x54, 0x52, 0x54, 0xff, 	/*Color of index 15*/

  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x05, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0xd0, 0x00, 
  0xfe, 0x8f, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xb8, 0xef, 0x00, 
  0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x00, 
  0x77, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x77, 0x00, 
  0x77, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4c, 0x8d, 
  0x77, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0x4c, 
  0x77, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0x71, 
  0x77, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0x71, 
  0x77, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4c, 0x71, 
  0x77, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0x41, 
  0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4c, 0xf3, 
  0x77, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x77, 0x00, 
  0xca, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa1, 0x00, 
  0x96, 0xa7, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x7a, 0x69, 0x00, 
  0x09, 0x14, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x41, 0x90, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  };

const lv_img_dsc_t battery_05 = {
    {
      LV_IMG_CF_INDEXED_4BIT,
        0,
        0,
        28,
        17
    },
   302,
   battery_05_map
};

