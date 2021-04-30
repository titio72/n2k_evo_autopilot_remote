#ifndef EVON2K_H
#define EVON2K_H

#include <N2kMessages.h>

class APStatus;

class EVON2K {

public:
    static void setup(APStatus* ap);

public:
    static void switchStatus(int status);
    static int setLockedHeading(int degrees);
    static void poll();
};


#endif
