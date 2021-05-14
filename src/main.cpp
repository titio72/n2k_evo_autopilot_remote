#define SINGLE_CLICK_DELAY_THRESHOLD 150
#define AP_STATUS_PIN 14
#define AP_BLINK_PIN 13
#define PROGRAM_PIN 34
#define RADIO_PIN 17
#define BEEP_PIN 16

#include <Arduino.h>
#include <EEPROM.h>
#include <RCSwitch.h>
#include <math.h>
#include "APStatus.h"
#include "EvoN2K.h"
#include "RFUtil.h"
#include "BTInterface.h"

RCSwitch m = RCSwitch();
APStatus ap = APStatus(AP_STATUS_PIN);
BTInterface bt("ABRemote");
unsigned long remote = 0;
bool program = false;

void write_remote_id() {
    Serial.printf("[RM] Wrinting remote %lx to conf\n", remote);
    EEPROM.writeULong(0, remote);
    if (!EEPROM.commit()) {
        Serial.printf("[RM] Failed to write remote code to conf\n");
    }
}

void on_status(u_char event) {
  static char buffer[128];
  if (event & 0x01) {
    sprintf(buffer, "[AP] Locked heading {%d}", ap.getLockedHeading());
    bt.send_data((uint8_t*)buffer, strlen(buffer));
  } else {
    sprintf(buffer, "[AP] Status {%s} {%d}", ap.getStatusStr(), ap.getStatus());
    bt.send_data((uint8_t*)buffer, strlen(buffer));
  }
}

const char* on_command(const char* command) {
  Serial.printf("[BT] Received command '%s'\n", command);
  if (strcmp("AUTO", command)==0) {
    EVON2K::switchStatus(AP_AUTO);
    return "OK";
  } else if (strcmp("STANDBY", command)==0) {
    EVON2K::switchStatus(AP_STANDBY);
    return "OK";
  } else if (strcmp("+1", command)==0) {
    EVON2K::port1();
    return "OK";
  } else if (strcmp("+10", command)==0) {
    EVON2K::port10();
    return "OK";
  } else if (strcmp("-1", command)==0) {
    EVON2K::starboard1();
    return "OK";
  } else if (strcmp("-10", command)==0) {
    EVON2K::starboard10();
    return "OK";
  } else if (strcmp("TP", command)==0) {
    EVON2K::tackPort();
    return "OK";
  } else if (strcmp("TS", command)==0) {
    EVON2K::tackStarboard();
    return "OK";
  } else if (command && (command[0]=='+' || command[0]=='-')) {
    int delta = atoi(command);
    if (delta) EVON2K::setLockedHeading(delta);
    return "OK";
  } else {
    return "Unknownd command";
  }
}

void read_remote_id() {
  remote = EEPROM.readULong(0);
  Serial.printf("[RM] Read remote %lx from conf\n", remote);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  bt.set_on_command_callback(on_command);
  bt.begin();

  EEPROM.begin(sizeof(unsigned long)+1);
  read_remote_id();

  m.enableReceive(RADIO_PIN);
  ap.setup(on_status);
  EVON2K::setup(&ap);
  pinMode(AP_BLINK_PIN, OUTPUT);
  pinMode(PROGRAM_PIN, INPUT);
  pinMode(BEEP_PIN, OUTPUT);
}

void loop_normal(unsigned long t) {
  static bool led = false;
  static unsigned long led_time = 0;

  static long t0 = 0;
  if (m.available()) {
    digitalWrite(AP_BLINK_PIN, HIGH);
    digitalWrite(BEEP_PIN, HIGH);
    led = true;
    led_time = t;
    if ((t - t0)>SINGLE_CLICK_DELAY_THRESHOLD) {
      RFUtil r(m.getReceivedValue(), remote);
      Serial.printf("[RF] Received combination %d%d%d%d\n", r.is_A_pressed(), r.is_B_pressed(), r.is_C_pressed(), r.is_D_pressed());
      switch (r.getAction()) {
        case SET_STATUS_ACTION:
          EVON2K::switchStatus(r.get_status());
          break;
        case SET_LOCKED_HEADING_ACTION:
          EVON2K::setLockedHeading(r.get_delta_degrees());
          break;
        case GO_STARBOARD_BY_1_ACTION:
          EVON2K::starboard1();
          break;
        case GO_STARBOARD_BY_10_ACTION:
          EVON2K::starboard10();
          break;
        case GO_PORT_BY_1_ACTION:
          EVON2K::port1();
          break;
        case GO_PORT_BY_10_ACTION:
          EVON2K::port10();
          break;
        case TACK_PORT_ACTION:
          EVON2K::tackPort();
          break;
        case TACK_STARBOARD_ACTION:
          EVON2K::tackStarboard();
          break;
        default:
          break;
      }
    }
    t0 = t;
    m.resetAvailable();
  } else {
    if (led && (t-led_time)>BLINK_TIME) {
      digitalWrite(AP_BLINK_PIN, LOW);
      digitalWrite(BEEP_PIN, LOW);
      led = false;
    }
  }
  delay(10);
}

void loop_program(unsigned long t) {
  if (m.available()) {
    remote = m.getReceivedValue() & 0xffff00;
    write_remote_id();
    program = false;
    digitalWrite(AP_BLINK_PIN, LOW);
    printf("[RM] Switching to normal mode.\n");
    m.resetAvailable();
    delay(1000);
  } else {
    delay(100);
  }
}

void loop() {
  unsigned long t = millis();
  if (program) {
    loop_program(t);
  } else {
    loop_normal(t);
  }
  if (!program && digitalRead(PROGRAM_PIN)==HIGH) {
    program = true;
    digitalWrite(AP_BLINK_PIN, HIGH);
    printf("[RM] Switching to program mode. Click a button to complete.\n");
  }
  bt.loop(t);
  EVON2K::poll();
}

