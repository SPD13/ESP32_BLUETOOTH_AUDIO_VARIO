#ifndef EPAPERDISPLAY_H_
#define EPAPERDISPLAY_H_

#include "audio.h"
#include <SPI.h>
// base class GxEPD2_GFX can be used to pass references or pointers to the display instance as parameter, uses ~1.2k more code
// enable or disable GxEPD2_GFX base class
#define ENABLE_GxEPD2_GFX 0
#include <GxEPD2_BW.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMono9pt7b.h>
#include <Fonts/Picopixel.h>
#include <Fonts/FreeMonoBold18pt7b.h>

void display_init();
void display_add_boot_message(String BootMessage);
void display_replace_boot_message(String BootMessage);
void display_refresh_data(int32_t altm, int32_t cps);
void display_show_modal_message(String titleText, String messageText[], int textNbItems);
void display_show_zz_message();
void display_clear_modal();
void display_off();
void display_on();
void display_toggle();

#endif
