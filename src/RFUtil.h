#ifndef RF_UTIL_H
#define RF_UTIL_H

#define SET_STATUS_ACTION 0
#define SET_LOCKED_HEADING_ACTION 1

class RFUtil {

public:
    RFUtil(unsigned long rf_value, unsigned long remoter_code);

    int getAction();
    int get_delta_degrees();
    int get_status();

    bool is_A_pressed();
    bool is_B_pressed();
    bool is_C_pressed();
    bool is_D_pressed();

private:
    int action;
    int delta;
    int status;

    bool b_A;
    bool b_B;
    bool b_C;
    bool b_D;
};



#endif
