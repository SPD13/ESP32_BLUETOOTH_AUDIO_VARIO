#include <Arduino.h>
#include "config.h"
#include "epaper_display.h"
#include "floatToString.h"

GxEPD2_BW<GxEPD2_290_BS, GxEPD2_290_BS::HEIGHT> display(GxEPD2_290_BS(/*CS=5*/ EPAPER_PIN_CS, /*DC=*/ EPAPER_PIN_DC, /*RES=*/ EPAPER_PIN_RES, /*BUSY=*/ EPAPER_PIN_BUSY)); // DEPG0290BS 128x296, SSD1680

#define MAX_BOOT_MESSAGES 40

bool in_modal = false;
uint8_t boot_line = 0;
String boot_messages[MAX_BOOT_MESSAGES];
static unsigned int TextBootTitleX = 5;
static unsigned int TextBootTitleY = 5;
static unsigned int VarioBarStartX = 5;
static unsigned int VarioBarRectX = 20;
static unsigned int VarioBarRectY = 10;
unsigned int VarioBarStartY; // Dynamically calculated based on screen height
static unsigned int VarioNumberStartX = 50;
unsigned int VarioNumberStartY; // Dynamically calculated based on screen height

void display_init() {
    display.init(115200,true,50,false);
    VarioBarStartY = (display.height()-(20*VarioBarRectY))/2;
    VarioNumberStartY = display.height()/2;
    Serial.println("[ePaper] - Init");
}

void display_boot_messages() {
    if (in_modal) return; //if modal message on, don't display
    const char WelcomeText[] = "Open Vario";
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

void display_refresh_data(int32_t altm, int32_t cps) {
    if (in_modal) return; //if modal message on, don't display
    display.setPartialWindow(0, 0, display.width(), display.height());
    display.firstPage();
    //Convert cps
    float mps = ((float) cps)/100.;

    //Vario Bar Chart
    display.fillScreen(GxEPD_WHITE);
    for (int i = 0; i<20; i++) {
        display.drawRect(VarioBarStartX, VarioBarStartY+i*VarioBarRectY, VarioBarRectX, VarioBarRectY, GxEPD_BLACK);
        if (i == 10) {
            //Draw Mid Point
            display.drawLine(VarioBarStartX-2, VarioBarStartY+i*VarioBarRectY, VarioBarStartX+2, VarioBarStartY+i*VarioBarRectY, GxEPD_BLACK);
        }
    }
    //Display current bar value
    if (mps>10) mps=10;
    if (mps<-10) mps=-10;
    if (mps>=0)
        display.fillRect(VarioBarStartX, VarioBarStartY+(10-mps)*VarioBarRectY, VarioBarRectX, mps*VarioBarRectY, GxEPD_BLACK);
    else
        display.fillRect(VarioBarStartX, VarioBarStartY+(10+mps)*VarioBarRectY, VarioBarRectX, mps*VarioBarRectY, GxEPD_BLACK);
    //Text
    display.setFont(&FreeMonoBold9pt7b);
    display.setTextSize(2);
    int16_t tbx, tby; uint16_t tbw, tbh;
    char S[6];
    //Set new text
    display.setTextColor(GxEPD_BLACK);
    floatToString(mps, S, sizeof(S), 2);
    display.getTextBounds(S, 0, 0, &tbx, &tby, &tbw, &tbh);
    display.setCursor(VarioNumberStartX, VarioNumberStartY-(tbh/2));
    display.print(S);
    display.setTextSize(1);
    display.setCursor(VarioNumberStartX, VarioNumberStartY-(tbh/2)-30);
    display.print("Climb");
    display.setCursor(VarioNumberStartX, VarioNumberStartY-(tbh/2)+30);
    display.print("m/s");
    while (display.nextPage());
}