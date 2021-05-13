// use this block with esp and transceiver
#define USE_N2K_CAN USE_N2K_ESP32_CAN
#define ESP32_CAN_RX_PIN GPIO_NUM_22
#define ESP32_CAN_TX_PIN GPIO_NUM_23

// use this block with MCP2515
//#define USE_N2K_CAN USE_N2K_MCP_CAN
//#define N2k_SPI_CS_PIN 5
//#define N2k_CAN_INT_PIN 0xff
//#define USE_MCP_CAN_CLOCK_SET 8

#define DEST 204
#define SOURCE 23

enum key_codes {
  KEY_PLUS_1 = 0x07f8,
  KEY_PLUS_10 = 0x08f7,
  KEY_MINUS_1 = 0x05fa,
  KEY_MINUS_10 = 0x06f9,
  KEY_MINUS_1_MINUS_10 = 0x21de,
  KEY_PLUS_1_PLUS_10 = 0x22dd,
  KEY_TACK_PORTSIDE = KEY_MINUS_1_MINUS_10,
  KEY_TACK_STARBORD = KEY_PLUS_1_PLUS_10
};

//#define DEBUG_AP

#include <Arduino.h>
#include <NMEA2000_CAN.h>
#include <RCSwitch.h>
#include <N2kMessages.h>
#include <math.h>

#include "EvoN2K.h"
#include "APStatus.h"

const unsigned long TransmitMessages[] PROGMEM = {126208UL,   // Set Pilot Mode
                                                  126720UL,   // Send Key Command
                                                  65288UL,    // Send Seatalk Alarm State
                                                  0
                                                 };

const unsigned long ReceiveMessages[] PROGMEM = { 127250UL,   // Read Heading
                                                  65288UL,    // Read Seatalk Alarm State
                                                  65379UL,    // Read Pilot Mode
                                                  65360UL,    // Read Pilot locked head
                                                  0
                                                };


APStatus* status = NULL;
int request_locked_heading = -1;
unsigned long lastHeadTime = 0;

void EVON2K::switchStatus(int s) {
  tN2kMsg m;
  m.Init(2, 126208, SOURCE, DEST);
  u_char b[] = {
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
  Serial.printf("[NT] Switching pilot %s {%s}\n", (s==AP_AUTO)?"On":"Off", res?"Ok":"Ok");

  #ifdef DEBUG_AP
  status->overrideStatus((s==AP_AUTO)?AP_AUTO:AP_STANDBY);
  #endif
}

int EVON2K::setLockedHeading(int delta) {
  if (status->getStatus()!=AP_AUTO) {
      Serial.printf("[NT] Unsupported status (only AUTO [0] is supported): %d\n", status->getStatus());
      return -9;
  } else if (request_locked_heading==-1) {
    if (status->getLockedHeading()==-1) {
      Serial.printf("[NT] Set pilot heading error: %s\n", "no value for locked heading");
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

  u_char byte0 = (u_char) (lHeading & 0xff);
  u_char byte1 = (u_char) ((lHeading >> 8) & 0xff);

  u_char b[] = {
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
    Serial.printf("[NT] Set pilot heading %d (%d) {Ok}\n", request_locked_heading, oldRequest);
  } else {
    request_locked_heading = oldRequest;
    Serial.printf("[NT] Set pilot heading error {Ko}\n");
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
  Serial.printf("[NT] Initializing N2K\n");
  NMEA2000.ExtendReceiveMessages(NULL);
  NMEA2000.SetN2kCANReceiveFrameBufSize(1500);
  NMEA2000.SetN2kCANMsgBufSize(16);
  NMEA2000.SetProductInformation("00000001", 100,
                                /*12345678901234567890123456789012*/
                                 "ABRemote                        ", "1.0.0.0 (2021-05-01)", "1.0.0.0 (2021-05-01)"
                                 );
  NMEA2000.SetDeviceInformation(1, /*Unique number. Use e.g. Serial number.*/ 150 /* Autopilot */, 40 /* Steering */, 2047);
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, SOURCE);
  //NMEA2000.ExtendTransmitMessages(TransmitMessages);
  NMEA2000.ExtendReceiveMessages(ReceiveMessages);
  NMEA2000.EnableForward(false);
  NMEA2000.SetMsgHandler(on_msg);
  bool initialized = NMEA2000.Open();
  Serial.printf("[NT] Initialized N2K {%s}\n", initialized?"Ok":"Ko");

}

void resetHeading() {
  if (request_locked_heading!=-1) {
    long now = time(0);
    if ((now-lastHeadTime)>5) {
      request_locked_heading = -1;
      Serial.printf("[NT] reset pilot heading\n");
    }
  }
}

void EVON2K::poll() {
  NMEA2000.ParseMessages();
  resetHeading();
}

static boolean KeyCommand(uint16_t command) {
  tN2kMsg m;
  m.Init(2, 126720UL, SOURCE, DEST);

  u_char commandByte0, commandByte1;
  commandByte0 = command >> 8;
  commandByte1 = command & 0xff;
  u_char buffer[] = {
    0x3b, 0x9f, 0xf0, 0x81, 0x86, 0x21, commandByte0, commandByte1,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xc1, 0xc2, 0xcd,
    0x66, 0x80, 0xd3, 0x42, 0xb1, 0xc8
  };
  for (int i = 0; i<22; i++) m.AddByte(buffer[i]);

  return NMEA2000.SendMsg(m);
}

void EVON2K::starboard1() {
  bool b = KeyCommand(KEY_PLUS_1);
  Serial.printf("[NT] Go starboard by 1 {%s}\n", b?"Ok":"Ko");
}

void EVON2K::starboard10() {
  bool b = KeyCommand(KEY_PLUS_10);
  Serial.printf("[NT] Go starboard by 10 {%s}\n", b?"Ok":"Ko");
}

void EVON2K::port1() {
  bool b = KeyCommand(KEY_MINUS_1);
  Serial.printf("[NT] Go port by 1 {%s}\n", b?"Ok":"Ko");
}

void EVON2K::port10() {
  bool b = KeyCommand(KEY_MINUS_10);
  Serial.printf("[NT] Go port by 10 {%s}\n", b?"Ok":"Ko");
}

void EVON2K::tackPort() {
  bool b = KeyCommand(KEY_TACK_PORTSIDE);
  Serial.printf("[NT] Tack port {%s}\n", b?"Ok":"Ko");
}

void EVON2K::tackStarboard() {
  bool b = KeyCommand(KEY_TACK_STARBORD);
  Serial.printf("[NT] Tack starboard {%s}\n", b?"Ok":"Ko");
}