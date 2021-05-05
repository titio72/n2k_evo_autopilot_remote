#define SINGLE_CLICK_DELAY_THRESHOLD 150
#define AP_STATUS_PIN 14
#define AP_BLINK_PIN 13
#define PROGRAM_PIN 34
#define RADIO_PIN 17

#include <Arduino.h>
#include <EEPROM.h>
#include <RCSwitch.h>
#include <math.h>
#include "APStatus.h"
#include "EvoN2K.h"
#include "RFUtil.h"

RCSwitch m = RCSwitch();
APStatus ap = APStatus(AP_STATUS_PIN);
unsigned long remote = 0;
bool program = false;
const bool debug = true;

void write_remote() {
    Serial.printf("Wrinting remote %lx to conf\n", remote);
    EEPROM.writeULong(0, remote);
    if (!EEPROM.commit()) {
        Serial.printf("Failed to write remote code to conf\n");
    }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  EEPROM.begin(sizeof(unsigned long)+1);
  remote = EEPROM.readULong(0);
  if (remote==0) {
    remote = 0xafb700;
    write_remote();
  } else {
    Serial.printf("Read remote %lx from conf\n", remote);
  }

  m.enableReceive(RADIO_PIN);
  ap.setup();
  EVON2K::setup(&ap, debug);
  pinMode(AP_BLINK_PIN, OUTPUT);
  pinMode(PROGRAM_PIN, INPUT);
}

void loop_normal() {
  static bool led = false;
  static unsigned long led_time = 0;

  unsigned long t = millis();
  static long t0 = 0;
  if (m.available()) {
    digitalWrite(AP_BLINK_PIN, HIGH);
    led = true;
    led_time = t;
    if ((t - t0)>SINGLE_CLICK_DELAY_THRESHOLD) {
      RFUtil r(m.getReceivedValue(), remote);
      if (r.getAction()==SET_STATUS_ACTION) {
          EVON2K::switchStatus(r.get_status());
      } else if (r.getAction()==SET_LOCKED_HEADING_ACTION) {
        EVON2K::setLockedHeading(r.get_delta_degrees());
      }
    }
    t0 = t;
    m.resetAvailable();
  } else {
    if (led && (t-led_time)>BLINK_TIME) {
      digitalWrite(13, LOW);
      led = false;
    }
  }
  EVON2K::poll();
  delay(100);
}

void loop_program() {
  if (m.available()) {
    remote = m.getReceivedValue() & 0xffff00;
    write_remote();
    program = false;
    digitalWrite(AP_BLINK_PIN, LOW);
    printf("Switching to normal mode.\n");
  }
  m.resetAvailable();
  delay(100);
}

void loop() {
  if (program) {
    loop_program();
  } else {
    loop_normal();
  }
  if (!program && digitalRead(PROGRAM_PIN)==HIGH) {
    program = true;
    digitalWrite(AP_BLINK_PIN, HIGH);
    printf("Switching to program mode. Click a button to complete.\n");
  }

}