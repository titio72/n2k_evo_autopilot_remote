#define USE_N2K_CAN USE_N2K_MCP_CAN
#define N2k_SPI_CS_PIN 5
#define N2k_CAN_INT_PIN 0xff
#define USE_MCP_CAN_CLOCK_SET 8
#include "EvoN2K.h"
#include "APStatus.h"
#include <Arduino.h>
#include <RCSwitch.h>
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include <math.h>

APStatus* status = NULL;

int request_locked_heading = -1;
int locked_heading = -1;
unsigned long lastHeadTime = 0;

void EVON2K::switchStatus(int s) {
  tN2kMsg m;
  m.Init(2, 126208, 99, 255);
  byte b[] = {
                    (byte) 0x01,  // 126208 type (1 means "command")
                    (byte) 0x63,  // PGN 65379
                    (byte) 0xff,  // PGN 65379
                    (byte) 0x00,  // PGN 65379
                    (byte) 0xf8,  // priority + reserved
                    (byte) 0x04,  // 4 params
                    (byte) 0x01,  // first param - 1 of PGN 65379 (manufacturer code)
                    (byte) 0x3b,  // 1851 Raymarine
                    (byte) 0x07,  // 1851 Raymarine
                    (byte) 0x03,  // second param -  3 of pgn 65369 (Industry code)
                    (byte) 0x04,  // Ind. code 4
                    (byte) 0x04,  // third parameter - 4 of pgn 65379 (mode)
                    (byte) 0x00,  // code 0 (STDBY)
                    (byte) 0x00,  // fourth param - 0 (does not exists, seems to be a Raymarine hack)
                    (byte) 0x05   // value of weird raymarine param
  };
  if (s==AP_AUTO) b[12] = 0x40;
  for (int i = 0; i<15; i++) m.AddByte(b[i]);
  bool res = NMEA2000.SendMsg(m);
  Serial.printf("Switching pilot %s %s\n", (s==AP_AUTO)?"On":"Off", res?"Ok":"Fail");

  // debug
  status->overrideStatus((s==AP_AUTO)?AP_AUTO:AP_STANDBY);
  // end debug
}

int EVON2K::setLockedHeading(int delta) {
  if (status->getStatus()!=AP_AUTO) {
      Serial.printf("Unsupported status (only AUTO [0] is supported): %d\n", status->getStatus());
      return -9;
  } else if (request_locked_heading==-1) {
    if (locked_heading==-1) {
      Serial.printf("Set pilot heading error: %s\n", "no value for locked heading");
      return -1;
    } else {
      // initialize the request with th current value of the "locked head" of the AP
      request_locked_heading = locked_heading;
    }
  }

  lastHeadTime = time(0);

  int oldRequest = request_locked_heading;
  request_locked_heading += delta;
  request_locked_heading = (request_locked_heading+360)%360;
  long lHeading = (long)round(request_locked_heading * M_PI / 180.0 / 0.0001);

  byte byte0 = (byte) (lHeading & 0xff);
  byte byte1 = (byte) (lHeading >> 8);

  byte b[] = {
                (byte)0x01,
                (byte)0x50, // pgn 65360
                (byte)0xff, // pgn 65360
                (byte)0x00, // pgn 65360
                (byte)0xf8, // priority + reserved
                (byte)0x03, // n params
                (byte)0x01, (byte)0x3b, (byte)0x07, // param 1
                (byte)0x03, (byte)0x04, // param 2
                (byte)0x06, byte0, byte1 // param 3: heading
  };
  tN2kMsg m;
  m.Init(2, 126208, 99, 255);
  for (int i = 0; i<14; i++) m.AddByte(b[i]);
  bool res = NMEA2000.SendMsg(m);
  if (res) {
    Serial.printf("Set pilot heading %d\n", request_locked_heading);
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
  locked_heading = 32; // set to 32 for testing purposes, should be -1 and driven by the AP input
  lastHeadTime = 0;
  status = s;
  Serial.printf("Initializing N2K\n");
  NMEA2000.SetN2kCANSendFrameBufSize(3);
  NMEA2000.SetN2kCANReceiveFrameBufSize(150),
  NMEA2000.SetN2kCANMsgBufSize(15);
  Serial.printf("Initializing N2K Product Info\n");
  NMEA2000.SetProductInformation("00000001", // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                /*1234567890123456789012345678901234567890*/
                                 "ABN2k                           ",  // Manufacturer's Model ID
                                 "1.0.2.25 (2019-07-07)",  // Manufacturer's Software version code
                                 "1.0.2.0 (2019-07-07)" // Manufacturer's Model version
                                 );
  Serial.printf("Initializing N2K Device Info\n");
  NMEA2000.SetDeviceInformation(1, // Unique number. Use e.g. Serial number.
                                132, // Device function=Analog to NMEA 2000 Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                25, // Device class=Inter/Intranetwork Device. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2046 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                               );
  Serial.printf("Initializing N2K mode\n");
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 15);
  NMEA2000.EnableForward(false); // Disable all msg forwarding to USB (=Serial)
  NMEA2000.SetMsgHandler(on_msg);
  Serial.printf("Initializing N2K Port & Handlers\n");
  bool initialized = NMEA2000.Open();
  Serial.printf("Initializing N2K %s\n", initialized?"OK":"KO");

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