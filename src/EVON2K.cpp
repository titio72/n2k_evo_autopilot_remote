// use this block with esp and transceiver
//#define USE_N2K_CAN USE_N2K_ESP32_CAN
//#define ESP32_CAN_RX_PIN GPIO_NUM_22
//#define ESP32_CAN_TX_PIN GPIO_NUM_23

// use this block with MCP2515
#define USE_N2K_CAN USE_N2K_MCP_CAN
#define N2k_SPI_CS_PIN 5
#define N2k_CAN_INT_PIN 0xff
#define USE_MCP_CAN_CLOCK_SET 8

#define DEST 204
#define SOURCE 23

#define DEBUG_AP

#include <Arduino.h>
#include <NMEA2000_CAN.h>
#include <RCSwitch.h>
#include <N2kMessages.h>
#include <math.h>

#include "EvoN2K.h"
#include "APStatus.h"


APStatus* status = NULL;
int request_locked_heading = -1;
unsigned long lastHeadTime = 0;

void EVON2K::switchStatus(int s) {
  tN2kMsg m;
  m.Init(2, 126208, SOURCE, DEST);
  unsigned char b[] = {
                    0x01,  // 126208 type (1 means "command")
                    0x63,  // PGN 65379
                    0xff,  // PGN 65379
                    0x00,  // PGN 65379
                    0xf8,  // priority + reserved
                    0x04,  // 4 params
                    0x01,  // first param - 1 of PGN 65379 (manufacturer code)
                    0x3b,  // 1851 Raymarine
                    0x07,  // 1851 Raymarine
                    0x03,  // second param -  3 of pgn 65369 (Industry code)
                    0x04,  // Ind. code 4
                    0x04,  // third parameter - 4 of pgn 65379 (mode)
                    0x00,  // code 0 (STDBY)
                    0x00,  // fourth param - 0 (does not exists, seems to be a Raymarine hack)
                    0x05   // value of weird raymarine param
  };
  if (s==AP_AUTO) b[12] = 0x40;
  for (int i = 0; i<15; i++) m.AddByte(b[i]);
  bool res = NMEA2000.SendMsg(m);
  Serial.printf("Switching pilot %s %s\n", (s==AP_AUTO)?"On":"Off", res?"Ok":"Fail");

  #ifdef DEBUG_AP
  status->overrideStatus((s==AP_AUTO)?AP_AUTO:AP_STANDBY);
  #endif
}

int EVON2K::setLockedHeading(int delta) {
  if (status->getStatus()!=AP_AUTO) {
      Serial.printf("Unsupported status (only AUTO [0] is supported): %d\n", status->getStatus());
      return -9;
  } else if (request_locked_heading==-1) {
    if (status->getLockedHeading()==-1) {
      Serial.printf("Set pilot heading error: %s\n", "no value for locked heading");
      return -1;
    } else {
      // initialize the request with th current value of the "locked head" of the AP
      request_locked_heading = status->getLockedHeading();
    }
  }

  lastHeadTime = time(0);

  int oldRequest = request_locked_heading;
  request_locked_heading += delta;
  request_locked_heading = (request_locked_heading + 360) % 360;
  double rads = DegToRad(request_locked_heading);
  long lHeading = (long)round(rads / 0.0001);

  unsigned char byte0 = (unsigned char) (lHeading & 0xff);
  unsigned char byte1 = (unsigned char) ((lHeading >> 8) & 0xff);

  unsigned char b[] = {
                0x01,
                0x50, // pgn 65360
                0xff, // pgn 65360
                0x00, // pgn 65360
                0xf8, // priority + reserved
                0x03, // n params
                0x01, 0x3b, 0x07, // param 1
                0x03, 0x04, // param 2
                0x06, byte0, byte1 // param 3: heading
  };
  tN2kMsg m;
  m.Init(2, 126208, SOURCE, DEST);
  for (int i = 0; i<14; i++) m.AddByte(b[i]);
  bool res = NMEA2000.SendMsg(m);
  if (res) {
    Serial.printf("Set pilot heading %d (%d)\n", request_locked_heading, oldRequest);
  } else {
    request_locked_heading = oldRequest;
    Serial.printf("Set pilot heading error\n");
  }
  return 0;
}

void on_msg(const tN2kMsg &msg) {
    status->onPGN(msg);
}

void EVON2K::setup(APStatus* s) {
  request_locked_heading = -1;
  lastHeadTime = 0;
  status = s;
  Serial.printf("Initializing N2K\n");
  NMEA2000.ExtendReceiveMessages(NULL);
  NMEA2000.SetN2kCANReceiveFrameBufSize(1500);
  NMEA2000.SetN2kCANMsgBufSize(16);
  NMEA2000.SetProductInformation("00000001", 100,
                                /*12345678901234567890123456789012*/
                                 "ABRemote                        ", "1.0.0.0 (2021-05-01)", "1.0.0.0 (2021-05-01)"
                                 );
  NMEA2000.SetDeviceInformation(1, /*Unique number. Use e.g. Serial number.*/ 150 /* Autopilot */, 40 /* Steering */, 2047);
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, SOURCE);
  NMEA2000.EnableForward(false);
  NMEA2000.SetMsgHandler(on_msg);
  bool initialized = NMEA2000.Open();
  Serial.printf("Initialized N2K %s\n", initialized?"OK":"KO");

}

void resetHeading() {
  if (request_locked_heading!=-1) {
    long now = time(0);
    if ((now-lastHeadTime)>5) {
      request_locked_heading = -1;
      Serial.printf("Debug: reset pilot heading\n");
    }
  }
}

void EVON2K::poll() {
  NMEA2000.ParseMessages();
  resetHeading();
}
