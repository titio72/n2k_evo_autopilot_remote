#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "Display.h"

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

EVODisplay::EVODisplay(): init(false), tw(NULL), display(NULL) {
    tw = new TwoWire(1);
    display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, tw, OLED_RESET);
}

EVODisplay::~EVODisplay() {
    delete display;
    delete tw;
}

void EVODisplay::setup() {
    if (!init) {
        tw->begin(26, 25);
        if (!display->begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
            Serial.println("[DS] SSD1306 allocation failed\n");
        } else {
            init = true;
        }
        display->clearDisplay();
    }
}

void EVODisplay::draw_text(const char* text) {
    if (init && text) {
        display->clearDisplay();
        display->setTextSize(2);      // Normal 1:1 pixel scale
        display->setTextColor(SSD1306_WHITE); // Draw white text
        display->setCursor(0, 0);     // Start at top-left corner
        display->cp437(true);         // Use full 256 char 'Code Page 437' font
        display->write(text, strlen(text));
        display->display();
    }
}
