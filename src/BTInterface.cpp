#include <Arduino.h>
#include "BTInterface.h"

BTInterface::BTInterface(const char* name) {
    device_name = name;
}

BTInterface::~BTInterface() {}

void bt_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    //Log::trace("[BT] Event {%d}\n", event);
}

void BTInterface::begin() {
    serialBT.begin(device_name);
    serialBT.register_callback(bt_callback);
}

void BTInterface::end() {
    serialBT.end();
}

size_t BTInterface::send_data(const uint8_t* data, size_t len) {
    size_t res = 0;
    if (len>0 && data) {
        res = serialBT.write(data, len);
        res = serialBT.write('\r');
        res = serialBT.write('\n');
        serialBT.flush();
    }
    return res;
}

void BTInterface::loop(unsigned long ms) {
    static char buffer[128];
    static int bf_pos = 0;
    while (serialBT.available()) {
        char c = (char)serialBT.read();
        if (c!=13) {
            buffer[bf_pos] = c;
            buffer[bf_pos + 1] = 0;
            bf_pos++;
        } else {
            Serial.printf("[BT] Received command {%s}\n", buffer);
            if (p_command_callback) {
                const char* response = p_command_callback(buffer);
                if (response) send_data((uint8_t*)response, strlen(response));
            }
            bf_pos = 0;
            buffer[0] = 0;
        }
    }
}