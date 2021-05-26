#ifndef DISPLAY_H
#define DISPLAY_H

class TwoWire;
class Adafruit_SSD1306;

class EVODisplay {

public:
    EVODisplay();
    ~EVODisplay();

    void draw_text(const char* text);
    void reset(const char* text);

    void setup();
    void loop(unsigned long milliseconds);

private:
    bool init;
    TwoWire* tw;
    Adafruit_SSD1306* display;
};


#endif