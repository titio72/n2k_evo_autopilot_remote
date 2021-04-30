#include "RFUtil.h"
#include "APStatus.h"

int RFUtil::getAction() {
    return action;
}

int RFUtil::get_delta_degrees() {
    return delta;
}

int RFUtil::get_status() {
    return status;
}

bool RFUtil::is_A_pressed() {
    return b_A;
}

bool RFUtil::is_B_pressed() {
    return b_B;
}

bool RFUtil::is_C_pressed() {
    return b_C;
}

bool RFUtil::is_D_pressed() {
    return b_D;
}

RFUtil::RFUtil(unsigned long v, unsigned long remote_code) {
    b_A = false; b_B = false; b_C = false; b_D = false;
    action = -1;
    delta = 0;
    status = AP_UNKNOWN;
    if ((v & 0xffff00) == remote_code) {
      unsigned long buttons = v & 0x0000ff;
      switch (buttons) {
        case 0xe1:
            b_A = true;
            action = SET_LOCKED_HEADING_ACTION;
            delta = -1;
            break;
        case 0xe2:
            b_B = true;
            action = SET_LOCKED_HEADING_ACTION;
            delta = +1;
            break;
        case 0xe3:
            b_A = true; b_B = true;
            action = SET_STATUS_ACTION;
            status = AP_STANDBY;
            break;
        case 0xe4:
            b_C = true;
            action = SET_LOCKED_HEADING_ACTION;
            delta = -10;
            break;
        case 0xe5:
            b_A = true; b_C = true;
            break;
        case 0xe6:
            b_B = true; b_C = true;
            break;
        case 0xe8:
            b_D = true;
            action = SET_LOCKED_HEADING_ACTION;
            delta = +10;
            break;
        case 0xe9:
            b_A = true; b_D = true;
            break;
        case 0xea:
            b_B = true; b_D = true;
            break;
        case 0xec:
            b_C = true; b_D = true;
            action = SET_STATUS_ACTION;
            status = AP_AUTO;
            break;
      }
    }
}