#include <Arduino.h>
#include <math.h>
#include "APStatus.h"

APStatus::APStatus(int statusPin): status(AP_UNKNOWN), lockedHeading(0), pin(statusPin) {
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
  int pgn = m.PGN;
  int index = 0;
  switch (pgn) {
    case 65360:
      index = 5;
      lockedHeading = RadToDeg(m.Get2ByteUDouble(0.0001, index));
      Serial.printf("Received locked heading %d\n", lockedHeading);
      break;
    case 65379:
      index = 2;
      int ap_m = m.GetByte(index);
      int ap_sm = m.GetByte(index);
      int ap_d = m.GetByte(index);

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
      Serial.printf("Received status (%d %d %d) %s\n", ap_m, ap_sm, ap_d, s_status);
      digitalWrite(pin, status>0?HIGH:LOW);
      break;
  }
}

