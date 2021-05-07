#ifndef _BTINTERFACE_H
#define _BTINTERFACE_H

#include <BluetoothSerial.h>

typedef const char* (*on_bt_command)(const char* cmd);

class BTInterface {
    public:
        BTInterface(const char* device_name);
        ~BTInterface();

        void begin();

        void end();

        size_t send_data(const uint8_t* data, size_t len);

        void loop(unsigned long ms);

        void set_on_command_callback(on_bt_command callback) { p_command_callback = callback; }

    private:
        BluetoothSerial serialBT;
        const char* device_name;

        on_bt_command p_command_callback;
};

#endif