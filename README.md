Watch_nRF52840
add compiler : gcc

https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm

add SDK Nordic :

https://www.nordicsemi.com/Software-and-tools/Software/nRF5-SDK/Download

add openocd :

http://openocd.org/

*BUILD App :

mkdir build


cmake ../

make all

*Just Flash Bootloader :

./scripts/nrf52/flash-boot.sh

Jush Flash Firmware :

./scripts/nrf52/flash-app.sh

*Flash  Bootloader and Firmware :

./scripts/nrf52/flash-app-full.sh

*DFU OTA :

use nordic App : NRFConnect (runs on Android and iOS).

file DFU: /build/src/watch-app-dfu-x.xx.x.zip
arger
https://devzone.nordicsemi.com/nordic/nordic-blog/b/blog/posts/common-faqs-on-dfu



# nRF52840_Watch
*22/04/2021
# 1.install Driver link:https://www.segger.com/downloads/jlink/JLink_Linux_V700a_x86_64.deb 
# 2.interface SWD link: https://programmersought.com/article/57052980800/

# 3. how to connect pc with JLink
  3.1 open folder contain diriver JLink
  3.2 sudo JLinkExe
  3.3 command "connect" 
  3.4 Please specify target interface:
    J) JTAG (Default)
    S) SWD
    T) cJTAG
    => choose S ( interface SWD)
  3.5 
 * 04/23/2021
# 4.Tomorrow, bring line bus
# 5. OPP and Master in C
# 6. FreeRTOS
# 7. Bluetooth
# 8. What is the JLink?
# 9. Install Windown10 in Ubuntu???


# cd build/
# make all 
# cd ..
# ./scripts/nrf52/flash-boot.sh
# How to work with Cmake
  Link: https://cmake.org/cmake/help/latest/guide/tutorial/index.html
# Sensor kx126, Tri-Axis
  Link https://kionixfs.azureedge.net/en/datasheet/KX126-1063-Specifications-Rev-3.0.pdf
# How to Work With MakeFile
  Link: https://viblo.asia/p/dao-dau-voi-cmake-thong-qua-vi-du-07LKXNbelV 
# Connected with JLink in Terminal VSCode
# Firmware - Done
# FreeRTOS - https://www.freertos.org/fr-content-src/uploads/2018/07/161204_Mastering_the_FreeRTOS_Real_Time_Kernel-A_Hands-On_Tutorial_Guide.pdf
#05/07/2021
# Problem ??
  How to show "screen-charging" when charging?
  fix code in systemtask
# Debug with Segger, use command
  sudo JLinkRTTViewerExe... to show Vcc for GPIO..
# command file hex
+ nrfjprog --program /home/hthang/Watch_nRF52840/build/src/watch-app-0.14.5.hex --chiperase --verify
+ nrfjprog --reset // reset

# C++ 
https://cppdeveloper.com/category/c-nang-cao/page/2/

# Xét trường hợp cạnh lên cạnh xuống, cạnh lên display, cạnh xuống thì ko display..
# How to Display " Status Charging" when plug in the charger
# FreeRTOS
  https://vidieukhien.xyz/2018/05/25/stm32f10x-get-start-in-freertos/
# what is the bootloader?
# Resize Image
https://www.img2go.com/result#j=940a7b71-25b2-483e-8e16-948e4215ce1a
# Convert Image 
https://lvgl.io/tools/imageconverter
# Problem Charging-Circle use tool lvgl
https://docs.lvgl.io/7.11/widgets/arc.html
https://docplayer.net/156307579-Littlevgl-documentation.html
#Document Embedded
https://doc.embedded-wizard.de/any-type?v=8.10
# make color in lvgl- to change color of screen display
https://color-hex.org/color/78756f



  

  


