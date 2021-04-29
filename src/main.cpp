#include <Arduino.h>
#include <RCSwitch.h>
#define USE_N2K_CAN USE_N2K_MCP_CAN
#define N2k_SPI_CS_PIN 5
#define N2k_CAN_INT_PIN 0xff
#define USE_MCP_CAN_CLOCK_SET 8
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include <math.h>

static const char* bin2tristate(const char* bin);
static char * dec2binWzerofill(unsigned long Dec, unsigned int bitLength);

void output(unsigned long decimal, unsigned int length, unsigned int delay, unsigned int* raw, unsigned int protocol) {

  const char* b = dec2binWzerofill(decimal, length);
  Serial.print("Decimal: ");
  Serial.print(decimal);
  Serial.print(" (");
  Serial.print( length );
  Serial.print("Bit) Binary: ");
  Serial.print( b );
  Serial.print(" Tri-State: ");
  Serial.print( bin2tristate( b) );
  Serial.print(" PulseLength: ");
  Serial.print(delay);
  Serial.print(" microseconds");
  Serial.print(" Protocol: ");
  Serial.println(protocol);
  
  Serial.print("Raw data: ");
  for (unsigned int i=0; i<= length*2; i++) {
    Serial.print(raw[i]);
    Serial.print(",");
  }
  Serial.println();
  Serial.println();
}

static const char* bin2tristate(const char* bin) {
  static char returnValue[50];
  int pos = 0;
  int pos2 = 0;
  while (bin[pos]!='\0' && bin[pos+1]!='\0') {
    if (bin[pos]=='0' && bin[pos+1]=='0') {
      returnValue[pos2] = '0';
    } else if (bin[pos]=='1' && bin[pos+1]=='1') {
      returnValue[pos2] = '1';
    } else if (bin[pos]=='0' && bin[pos+1]=='1') {
      returnValue[pos2] = 'F';
    } else {
      return "not applicable";
    }
    pos = pos+2;
    pos2++;
  }
  returnValue[pos2] = '\0';
  return returnValue;
}

static char * dec2binWzerofill(unsigned long Dec, unsigned int bitLength) {
  static char bin[64]; 
  unsigned int i=0;

  while (Dec > 0) {
    bin[32+i++] = ((Dec & 1) > 0) ? '1' : '0';
    Dec = Dec >> 1;
  }

  for (unsigned int j = 0; j< bitLength; j++) {
    if (j >= bitLength - i) {
      bin[j] = bin[ 31 + i - (j - (bitLength - i)) ];
    } else {
      bin[j] = '0';
    }
  }
  bin[bitLength] = '\0';
  
  return bin;
}

RCSwitch m = RCSwitch();

#define SINGLE_CLICK_DELAY_THRESHOLD 150
#define AP_UNKNOWN -1
#define AP_STANDBY 0
#define AP_AUTO 1
#define AP_WIND_VANE 2
#define AP_TRACK 3
#define AP_TRACK_DEV 4
#define BLINK_TIME 100
#define AP_STATUS_PIN 14
#define AP_BLINK_PIN 13

int setHeading = 32; // set to 32 for testing purposes, should be -1 and driven by the AP input
int requestSetHeading = -1;
unsigned long lastHeadTime = 0;
int status = AP_UNKNOWN;

void msg_handler(const tN2kMsg &m)
{
  int pgn = m.PGN;
  int index = 0;
  switch (pgn) {
    case 65360:
      index = 5;
      setHeading = RadToDeg(m.Get2ByteDouble(0.0001, index));
      Serial.printf("Received locked heading %d\n", setHeading);
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
      digitalWrite(AP_STATUS_PIN, status>0?HIGH:LOW);
      break;
  }
}

void setup() {
  Serial.begin(115200);
  m.enableReceive(22);

  pinMode(AP_BLINK_PIN, OUTPUT);
  pinMode(AP_STATUS_PIN, OUTPUT);

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
  NMEA2000.SetMsgHandler(msg_handler);
  Serial.printf("Initializing N2K Port & Handlers\n");
  bool initialized = NMEA2000.Open();
  Serial.printf("Initializing N2K %s\n", initialized?"OK":"KO");
}

void SwitchPilot(bool onOff) {
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
  if (onOff) b[12] = 0x40;
  for (int i = 0; i<15; i++) m.AddByte(b[i]);
  bool res = NMEA2000.SendMsg(m);
  Serial.printf("Switching pilot %s %s\n", onOff?"On":"Off", res?"Ok":"Fail");

  // debug
  status = onOff?AP_AUTO:AP_STANDBY;
  digitalWrite(AP_STATUS_PIN, onOff?HIGH:LOW);
  // end debug
}

int SetHeading(int delta) {
  if (status!=AP_AUTO) {
      Serial.printf("Unsupported status (only AUTO [0] is supported): %d\n", status);
      return -9;
  } else if (requestSetHeading==-1) {
    if (setHeading==-1) {
      Serial.printf("Set pilot heading error: %s\n", "no value for locked heading");
      return -1;
    } else {
      // initialize the request with th current value of the "locked head" of the AP
      requestSetHeading = setHeading;
    }
  }

  lastHeadTime = time(0);

  int oldRequest = requestSetHeading;
  requestSetHeading += delta;
  requestSetHeading = (requestSetHeading+360)%360;
  long lHeading = (long)round(requestSetHeading * M_PI / 180.0 / 0.0001);

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
    Serial.printf("Set pilot heading %d\n", requestSetHeading);
  } else {
    requestSetHeading = oldRequest;
    Serial.printf("Set pilot heading error\n");
  }
  return 0;
}

void resetHeading() {
  if (requestSetHeading!=-1) {
    long now = time(0);
    if ((now-lastHeadTime)>5) {
      requestSetHeading = -1;
      Serial.printf("Debug: reset pilot heading\n");
    }
  }

}

void loop() {
  static bool led = false;
  static unsigned long led_time = 0;

  unsigned long t = millis();
  static long t0 = 0;
  if (m.available()) {
    digitalWrite(13, HIGH);
    led = true;
    led_time = t;
    if ((t - t0)>SINGLE_CLICK_DELAY_THRESHOLD) {
      //char c[] = {0, 0, 0};
      switch (m.getReceivedValue()) {
        case 0xafb7e1:
          //c[0] = 'A'; c[1] = 0;
          SetHeading(-1); break;
        case 0xafb7e2:
          //c[0] = 'B'; c[1] = 0;
          SetHeading(+1); break;
        case 0xafb7e3:
          //c[0] = 'A'; c[1] = 'B';
          SwitchPilot(false); break;
        case 0xafb7e4:
          //c[0] = 'C'; c[1] = 0;
          SetHeading(-10); break;
        case 0xafb7e5:
          //c[0] = 'A'; c[1] = 'C';
          break;
        case 0xafb7e6:
          //c[0] = 'B'; c[1] = 'C';
          break;
        case 0xafb7e8:
          //c[0] = 'D'; c[1] = 0;
          SetHeading(+10); break;
        case 0xafb7e9:
          //c[0] = 'A'; c[1] = 'D';
          break;
        case 0xafb7ea:
          //c[0] = 'B'; c[1] = 'D';
          break;
        case 0xafb7ec:
          //c[0] = 'C'; c[1] = 'D';
          SwitchPilot(true);
          break;
      }
      //Serial.printf("-- Clicked %s\n", c);
    }
    t0 = t;
    m.resetAvailable();
  } else {
    if (led && (t-led_time)>BLINK_TIME) {
      digitalWrite(13, LOW);
      led = false;
    }
  }
  NMEA2000.ParseMessages();
  resetHeading();
  delay(100);
}
