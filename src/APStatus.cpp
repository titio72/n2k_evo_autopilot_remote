#include <Arduino.h>
#include <math.h>
#include "APStatus.h"

APStatus::APStatus(int statusPin): status(AP_UNKNOWN), lockedHeading(-1), pin(statusPin) {
}

APStatus::~APStatus() {
    digitalWrite(pin, LOW);
}

void APStatus::setup(status_listener l) {
    listener = l;
    pinMode(pin, OUTPUT);
}

int APStatus::getLockedHeading() {
    return lockedHeading;
}

int APStatus::getStatus() {
    return status;
}

const char* APStatus::getStatusStr() {
  switch (status)
  {
  case AP_STANDBY: return "Standby";
  case AP_AUTO: return "Auto";
  case AP_WIND_VANE: return "Vane";
  case AP_TRACK: return "Track";
  case AP_TRACK_DEV: return "Trackdev";
  default: return "Unknown";
  }

}

void APStatus::overrideStatus(int s) {
    status = s;
    digitalWrite(pin, status>0?HIGH:LOW);
    if (listener) listener(0x10);
}

void APStatus::overrideLockedHeading(int head) {
    lockedHeading = head;
    if (listener) listener(0x01);
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
      x = (int)((RadToDeg(m.Get2ByteUDouble(0.0001, index))) + 360) % 360;
      if (x!=lockedHeading) {
        Serial.printf("[AP] New locked heading %d\n", x);
        if (listener) listener(0x01);
      }
      lockedHeading = x;
      break;
    case 65379:
      index = 2;
      int ap_m = m.GetByte(index);
      int ap_sm = m.GetByte(index);
      int ap_d = m.GetByte(index);

      int old_status = status;
      if (ap_m == 0 && ap_sm == 0) {
        status = AP_STANDBY;
      } else if (ap_m == 64 && ap_sm == 0) {
        status = AP_AUTO;
      } else if (ap_m == 0 && ap_sm == 1) {
        status = AP_WIND_VANE;
      } else if (ap_m == 128 && ap_sm == 1) {
        status = AP_TRACK;
      } else if (ap_m == 129 && ap_sm == 1) {
        status = AP_TRACK_DEV;
      }  else {
        status = AP_UNKNOWN;
      }
      if (old_status!=status) {
        Serial.printf("[AP] New status (%d %d %d) %s\n", ap_m, ap_sm, ap_d, getStatusStr());
        if (listener) listener(0x10);
      }
      digitalWrite(pin, status>0?HIGH:LOW);
      break;
  }
}

const char* APStatus::getDescription(char* c) {
    switch (status) {
      case AP_AUTO:
        sprintf(c, "%s [%d]", getStatusStr(), getLockedHeading());
        break;
      default:
        sprintf(c, "%s", getStatusStr());
        break;
    }
    return c;
}
