#include "lvgl/lvgl.h"


#ifndef LV_ATTRIBUTE_MEM_ALIGN
#define LV_ATTRIBUTE_MEM_ALIGN
#endif

#ifndef LV_ATTRIBUTE_IMG_BATTERY_100_C
#define LV_ATTRIBUTE_IMG_BATTERY_100_C
#endif

const LV_ATTRIBUTE_MEM_ALIGN LV_ATTRIBUTE_LARGE_CONST LV_ATTRIBUTE_IMG_BATTERY_100_C uint8_t battery_100_c_map[] = {
  0x05, 0x03, 0x04, 0xff, 	/*Color of index 0*/
  0x56, 0x92, 0x2e, 0xff, 	/*Color of index 1*/
  0xa5, 0xa2, 0xa5, 0xff, 	/*Color of index 2*/
  0x2f, 0x49, 0x19, 0xff, 	/*Color of index 3*/
  0x75, 0xc9, 0x3c, 0xff, 	/*Color of index 4*/
  0x5b, 0x5a, 0x58, 0xff, 	/*Color of index 5*/
  0xdd, 0xd9, 0xdd, 0xff, 	/*Color of index 6*/
  0xb6, 0xb4, 0xb6, 0xff, 	/*Color of index 7*/
  0x41, 0x69, 0x24, 0xff, 	/*Color of index 8*/
  0x8c, 0x8a, 0x8c, 0xff, 	/*Color of index 9*/
  0x6e, 0x70, 0x6b, 0xff, 	/*Color of index 10*/
  0xfa, 0xfb, 0xf9, 0xff, 	/*Color of index 11*/
  0x1f, 0x28, 0x18, 0xff, 	/*Color of index 12*/
  0x66, 0xab, 0x38, 0xff, 	/*Color of index 13*/
  0x7e, 0xd7, 0x41, 0xff, 	/*Color of index 14*/
  0x40, 0x4f, 0x37, 0xff, 	/*Color of index 15*/

  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x0a, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x57, 0xa6, 0x66, 0x66, 0x66, 0xa0, 0x00, 
  0x5b, 0x95, 0xa5, 0xaa, 0x5a, 0x5a, 0x5a, 0x62, 0xca, 0xaa, 0xaa, 0x52, 0xb5, 0x00, 
  0x27, 0x31, 0x11, 0x31, 0x11, 0x18, 0xc6, 0xb5, 0x11, 0x11, 0x11, 0x80, 0x72, 0x00, 
  0x77, 0xde, 0xee, 0x11, 0xee, 0xdf, 0x6b, 0x68, 0xee, 0xee, 0xee, 0xec, 0x27, 0x00, 
  0x77, 0x1e, 0x44, 0x48, 0x4d, 0xf6, 0xbb, 0x91, 0xe4, 0x44, 0x44, 0x4c, 0x22, 0x9a, 
  0x27, 0x1e, 0x44, 0xe1, 0x8f, 0x6b, 0xbb, 0xf1, 0xe4, 0x44, 0x44, 0x4c, 0x22, 0x72, 
  0x27, 0x1e, 0x44, 0x4d, 0xc6, 0xbb, 0xb6, 0xcc, 0xde, 0x44, 0x44, 0x4c, 0x22, 0x72, 
  0x27, 0x1e, 0x44, 0x4d, 0x8f, 0x96, 0xbb, 0x69, 0xf1, 0x44, 0x44, 0x4c, 0x22, 0x72, 
  0x27, 0x1e, 0x44, 0x44, 0xe8, 0xc7, 0xbb, 0xb6, 0xc1, 0xe4, 0x44, 0x4c, 0x22, 0x29, 
  0x27, 0x1e, 0x44, 0x44, 0x4d, 0xfb, 0xbb, 0x63, 0x18, 0x44, 0x44, 0x4c, 0x22, 0x72, 
  0x27, 0x1e, 0x44, 0x44, 0xe1, 0x9b, 0xb7, 0x3d, 0xe8, 0x1e, 0x44, 0x4c, 0x22, 0x5f, 
  0x72, 0x1e, 0xee, 0xee, 0xe8, 0x7b, 0x6f, 0x4e, 0xe4, 0x84, 0xee, 0xec, 0x27, 0x00, 
  0x27, 0xc8, 0x88, 0x88, 0x8f, 0xb7, 0x03, 0x88, 0x88, 0xc3, 0x88, 0x30, 0x62, 0x00, 
  0xcb, 0x62, 0x22, 0x27, 0x59, 0x62, 0x72, 0x22, 0x22, 0x22, 0x22, 0x26, 0xbc, 0x00, 
  0x0c, 0x27, 0x77, 0x77, 0xa9, 0xa2, 0x77, 0x77, 0x77, 0x77, 0x77, 0x72, 0xc0, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0xcc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
};
const lv_img_dsc_t battery_100_c = {
    {
      LV_IMG_CF_INDEXED_4BIT,
        0,
        0,
        28,
        17
    },
    302,
   battery_100_c_map
};
