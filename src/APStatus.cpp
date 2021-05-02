#include <Arduino.h>
#include <math.h>
#include "APStatus.h"

APStatus::APStatus(int statusPin): status(AP_UNKNOWN), lockedHeading(-1), pin(statusPin) {
}

APStatus::~APStatus() {
    digitalWrite(pin, LOW);
}

void APStatus::setup() {
    pinMode(pin, OUTPUT);
}

int APStatus::getLockedHeading() {
    return lockedHeading;
}

int APStatus::getStatus() {
    return status;
}

void APStatus::overrideStatus(int s) {
    status = s;
    digitalWrite(pin, status>0?HIGH:LOW);
}

void APStatus::onPGN(const tN2kMsg &m)
{
 // printf("Msg %ul\n", m.PGN);
  int pgn = m.PGN;
  int index = 0;
  int x = 0;
  switch (pgn) {
    case 65360:
      index = 5;
      x = (int)((RadToDeg(m.Get2ByteDouble(0.0001, index))) + 360) % 360;
      if (x!=lockedHeading) {
        Serial.printf("New locked heading %d\n", x);
      }
      lockedHeading = x;
      break;
    case 65379:
      index = 2;
      int ap_m = m.GetByte(index);
      int ap_sm = m.GetByte(index);
      int ap_d = m.GetByte(index);

      int old_status = status;
      const char* s_status = "Unknown";
      if (ap_m == 0 && ap_sm == 0) {
        status = AP_STANDBY;
        s_status = "StandBy";
      } else if (ap_m == 64 && ap_sm == 0) {
        status = AP_AUTO;
        s_status = "Auto";
      } else if (ap_m == 0 && ap_sm == 1) {
        status = AP_WIND_VANE;
        s_status = "Vane";
      } else if (ap_m == 128 && ap_sm == 1) {
        status = AP_TRACK;
        s_status = "Track";
      } else if (ap_m == 129 && ap_sm == 1) {
        status = AP_TRACK_DEV;
        s_status = "TrackDev";
      }  else {
        status = AP_UNKNOWN;
        s_status = "Unknown";
      }
      if (old_status!=status) {
        Serial.printf("Received status (%d %d %d) %s\n", ap_m, ap_sm, ap_d, s_status);
      }
      digitalWrite(pin, status>0?HIGH:LOW);
      break;
  }
}

