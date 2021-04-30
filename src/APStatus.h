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

class APStatus {
public:
    APStatus(int statusPin);
    ~APStatus();

public:
    void setup();

    void onPGN(const tN2kMsg &m);

    int getLockedHeading();
    int getStatus();

    // use for testing only
    void overrideStatus(int status);

private:
    int status;
    int lockedHeading;
    int pin;
};



#endif
