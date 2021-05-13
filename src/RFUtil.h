#ifndef RF_UTIL_H
#define RF_UTIL_H

enum RF_AP_ACTIONS {
    NO_ACTION,
    SET_STATUS_ACTION,
    SET_LOCKED_HEADING_ACTION,
    GO_PORT_BY_1_ACTION,
    GO_PORT_BY_10_ACTION,
    GO_STARBOARD_BY_1_ACTION,
    GO_STARBOARD_BY_10_ACTION,
    TACK_PORT_ACTION,
    TACK_STARBOARD_ACTION,
};

class RFUtil {

public:
    RFUtil(unsigned long rf_value, unsigned long remoter_code);

    RF_AP_ACTIONS getAction();
    int get_delta_degrees();
    int get_status();

    bool is_A_pressed();
    bool is_B_pressed();
    bool is_C_pressed();
    bool is_D_pressed();

private:
    RF_AP_ACTIONS action;
    int delta;
    int status;

    bool b_A;
    bool b_B;
    bool b_C;
    bool b_D;
};



#endif
