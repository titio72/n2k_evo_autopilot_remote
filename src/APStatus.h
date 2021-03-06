#ifndef AP_STATUS_H
#define AP_STATUS_H

#include <N2kMessages.h>

#define AP_UNKNOWN -1
#define AP_STANDBY 0
#define AP_AUTO 1
#define AP_WIND_VANE 2
#define AP_TRACK 3
#define AP_TRACK_DEV 4
#define BLINK_TIME 100

typedef void (*status_listener)(unsigned char event);

class APStatus {
public:
    APStatus(int statusPin);
    ~APStatus();

public:
    void setup(status_listener l = NULL);


    void onPGN(const tN2kMsg &m);

    int getWind();
    int getLockedHeading();
    int getStatus();
    const char* getStatusStr();

    const char* getDescription(char* c);

    // use for testing only
    void overrideStatus(int status);
    void overrideLockedHeading(int head);
    void overrideWind(int wind);

private:
    int status;
    int lockedHeading;
    int windVane;
    int pin;

    status_listener listener;
};



#endif
