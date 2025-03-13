#include <Arduino.h>
#include "config.h"
#include "epaper_display.h"
#include "floatToString.h"

GxEPD2_BW<GxEPD2_290_BS, GxEPD2_290_BS::HEIGHT> display(GxEPD2_290_BS(/*CS=5*/ EPAPER_PIN_CS, /*DC=*/ EPAPER_PIN_DC, /*RES=*/ EPAPER_PIN_RES, /*BUSY=*/ EPAPER_PIN_BUSY)); // DEPG0290BS 128x296, SSD1680

#define MAX_BOOT_MESSAGES 40

bool in_modal = false;
bool display_status = true;
bool alive_marker = false;
uint8_t boot_line = 0;
String boot_messages[MAX_BOOT_MESSAGES];
static unsigned int TextBootTitleX = 5;
static unsigned int TextBootTitleY = 5;
static unsigned int VarioBarStartX = 5;
static unsigned int VarioBarRectX = 20;
static unsigned int VarioBarRectY = 14;
unsigned int VarioBarStartY; // Dynamically calculated based on screen height
static unsigned int VarioNumberStartX = 30;
unsigned int VarioNumberStartY; // Dynamically calculated based on screen height
static unsigned int AltNumberStartX = 30;
static unsigned int AltNumberStartY = 250;
unsigned int AudioStatusIconX; // Dynamically calculated based on screen width
static unsigned int AudioStatusIconY = 10;
unsigned int AliveMarkerX = 10; // Dynamically calculated based on screen width
unsigned int AliveMarkerY = 10; // Dynamically calculated based on screen height

void display_init() {
    display.init(115200,true,50,false);
    VarioBarStartY = (display.height()-(20*VarioBarRectY))/2;
    VarioNumberStartY = display.height()/2;
    AudioStatusIconX = display.width() - 20;
    AliveMarkerX = display.width() - 20;
    AliveMarkerY = display.height() - 20;
    Serial.println("[ePaper] - Init");
}

void display_boot_messages() {
    if (in_modal) return; //if modal message on, don't display
    const char WelcomeText[] = "Open Vario";
    display.setRotation(0);
    display.setFont(&FreeMonoBold9pt7b);
    display.setPartialWindow(0, 0, display.width(), display.height());
    display.setTextColor(GxEPD_BLACK);
    int16_t tbx, tby; uint16_t tbw, tbh;
    display.getTextBounds(WelcomeText, 0, 0, &tbx, &tby, &tbw, &tbh);
    // center the bounding box by transposition of the origin:
    display.firstPage();
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(TextBootTitleX, TextBootTitleY+tbh);
    display.print(WelcomeText);
    display.setFont(&Picopixel);
    for (int i = 0; i<=boot_line; i++) {
        display.setCursor(TextBootTitleX, TextBootTitleY+40+(i*8));
        display.print(boot_messages[i]);
    }
    //Web config message
    display.setCursor(TextBootTitleX, display.height() - 3*8);
    display.print(("To start WEB Config mode"));
    display.setCursor(TextBootTitleX, display.height() - 2*8);
    display.print(("Press & Hold Button 1"));

    while (display.nextPage());
    //display.hibernate();
}

void display_add_boot_message(String BootMessage) {
    boot_messages[boot_line]=BootMessage;
    boot_line++;
    if (boot_line > MAX_BOOT_MESSAGES) {
        boot_line = 0; //Overflow at the top
    }
    display_boot_messages();
}

void display_replace_boot_message(String BootMessage) {
    if (boot_line<1) return;
    boot_messages[boot_line-1]=BootMessage;
    display_boot_messages();
}

void display_show_modal_message(String titleText, String messageText[], int textNbItems) {
    if (textNbItems == 0) return;
    in_modal = true;
    display.setPartialWindow(0, 0, display.width(), display.height());
    display.setRotation(3);
    display.firstPage();
    display.setFont(&FreeMono9pt7b);
    display.setTextColor(GxEPD_BLACK);
    unsigned static int itemHeight = 14;
    int windowStartY = (display.height() - ((itemHeight*textNbItems) + 10))/2;
    display.drawRect(0, windowStartY, display.width(), (itemHeight*textNbItems) + 10, GxEPD_BLACK);
    display.fillRect(1, windowStartY+1, display.width()-2, (itemHeight*textNbItems) + 8, GxEPD_WHITE);
    for (int i=0; i<textNbItems; i++) {
        display.setCursor(4, windowStartY + (i+0.5)*itemHeight + 5);
        display.print(messageText[i]);
    }
    //Title
    int16_t tbx, tby; uint16_t tbw, tbh;
    display.setTextSize(1);
    display.setFont(&FreeMonoBold9pt7b);
    display.getTextBounds(titleText, 0, 0, &tbx, &tby, &tbw, &tbh);
    display.fillRect(0, windowStartY-(tbh+20), display.width(), tbh + 20, GxEPD_BLACK);
    display.setCursor(display.width()/2-tbw/2, windowStartY-tbh);
    display.setTextColor(GxEPD_WHITE);
    display.print(titleText);

    while (display.nextPage());
}

void display_show_zz_message() {
    in_modal = true;
    display.setFullWindow();
    display.setRotation(3);
    display.firstPage();
    display.fillScreen(GxEPD_WHITE);
    display.setFont(&FreeMonoBold18pt7b);
    display.setTextColor(GxEPD_BLACK);
    int windowStartY = 50;
    int16_t tbx, tby; uint16_t tbw, tbh;
    display.getTextBounds("Z", 0, 0, &tbx, &tby, &tbw, &tbh);
    display.drawRect(0, windowStartY, display.width(), tbh+30, GxEPD_BLACK);
    display.setCursor(4, windowStartY + 30);
    display.print("Z");
    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(4+tbw+5, windowStartY + 30);
    display.print("Z...");
    //Title
    display.setTextSize(1);
    display.setFont(&FreeMonoBold9pt7b);
    display.getTextBounds("SCREEN OFF", 0, 0, &tbx, &tby, &tbw, &tbh);
    display.fillRect(0, windowStartY-(tbh+20), display.width(), tbh + 20, GxEPD_BLACK);
    display.setCursor(display.width()/2-tbw/2, windowStartY-tbh);
    display.setTextColor(GxEPD_WHITE);
    display.print("SCREEN OFF");
    while (display.nextPage());
}

void display_clear_modal() {
    display.setFullWindow();
    display.setRotation(3);
    display.firstPage();
    display.fillScreen(GxEPD_WHITE);
    while (display.nextPage());
    in_modal = false;
}

void display_refresh_data(int32_t altm, int32_t cps) {
    if (in_modal) return; //if modal message on, don't display
    display.setPartialWindow(0, 0, display.width(), display.height());
    display.setRotation(0);
    display.firstPage();
    //Convert cps
    float mps = ((float) cps)/100.;

    //Vario Bar Chart
    display.fillScreen(GxEPD_WHITE);
    for (int i = 0; i<20; i++) {
        display.drawRect(VarioBarStartX, VarioBarStartY+i*VarioBarRectY, VarioBarRectX, VarioBarRectY, GxEPD_BLACK);
        if (i == 10) {
            //Draw Mid Point
            display.fillRect(VarioBarStartX-5, VarioBarStartY+i*VarioBarRectY-1, 5, 3, GxEPD_BLACK);
            display.fillRect(VarioBarStartX+VarioBarRectX, VarioBarStartY+i*VarioBarRectY-1, 5, 3, GxEPD_BLACK);
        }
    }
    //Vario bar value, cap
    if (mps>9.99) mps=9.99;
    if (mps<-9.99) mps=-9.99;
    display.fillRect(VarioBarStartX, VarioBarStartY+(10-mps)*VarioBarRectY, VarioBarRectX, mps*VarioBarRectY, GxEPD_BLACK);
    //Vario Numeric value
    display.setFont(&FreeMonoBold18pt7b);
    int16_t tbx, tby; uint16_t tbw, tbh;
    char S[6];
    display.setTextColor(GxEPD_BLACK);
    dtostrf(mps,4,1,S);
    String mps_str = String(S);
    display.getTextBounds(mps_str, 0, 0, &tbx, &tby, &tbw, &tbh);
    display.setCursor(VarioNumberStartX, VarioNumberStartY-(tbh/2));
    display.print(mps_str);
    //Vario labels
    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(VarioNumberStartX+20, VarioNumberStartY-(tbh/2)-30);
    display.print("Climb");
    display.setFont(&FreeMono9pt7b);
    display.setCursor(VarioNumberStartX+30, VarioNumberStartY-(tbh/2)+20);
    display.print("m/s");
    //Altitude numeric value
    display.setFont(&FreeMono9pt7b);
    display.getTextBounds(String(altm), 0, 0, &tbx, &tby, &tbw, &tbh);
    display.setCursor(display.width()-tbw-10, AltNumberStartY);
    display.print(String(altm));
    //Altitude labels
    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(AltNumberStartX, AltNumberStartY-(tbh/2)-20);
    display.print("Altitude");
    display.setFont(&FreeMono9pt7b);
    display.setCursor(AltNumberStartX+30, AltNumberStartY-(tbh/2)+20);
    display.print("m");
    //Status display
    display.fillRect(AudioStatusIconX, AudioStatusIconY+4, 5, 5, GxEPD_BLACK);
    display.fillRect(AudioStatusIconX+5, AudioStatusIconY+3, 1, 7, GxEPD_BLACK);
    display.fillRect(AudioStatusIconX+6, AudioStatusIconY+2, 1, 9, GxEPD_BLACK);
    display.fillRect(AudioStatusIconX+7, AudioStatusIconY+1, 1, 11, GxEPD_BLACK);
    display.fillRect(AudioStatusIconX+8, AudioStatusIconY, 1, 12, GxEPD_BLACK);
    if (IsMuted) {
        display.drawLine(AudioStatusIconX + 10, AudioStatusIconY+3, AudioStatusIconX + 15, AudioStatusIconY + 8, GxEPD_BLACK);
        display.drawLine(AudioStatusIconX + 15, AudioStatusIconY+3, AudioStatusIconX + 10, AudioStatusIconY + 8, GxEPD_BLACK);
    } else {
        display.drawLine(AudioStatusIconX + 10, AudioStatusIconY + 2, AudioStatusIconX + 10, AudioStatusIconY + 10, GxEPD_BLACK);
        display.drawLine(AudioStatusIconX + 12, AudioStatusIconY + 4, AudioStatusIconX + 12, AudioStatusIconY + 8, GxEPD_BLACK);
    }
    //Alive marker
    if (alive_marker) {
        display.fillRect(AliveMarkerX, AliveMarkerY, 10, 10, GxEPD_BLACK);
        alive_marker = false;
    } else {
        display.drawRect(AliveMarkerX, AliveMarkerY, 10, 10, GxEPD_BLACK);
        alive_marker = true;
    }

    while (display.nextPage());
}

void display_off() {
    display_status = false;
    display_show_zz_message();
}

void display_on() {
    display_status = true;
    display_clear_modal();
}

void display_toggle() {
    if (display_status)
        display_off();
    else
        display_on();
}