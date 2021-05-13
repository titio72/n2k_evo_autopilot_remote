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
    static void starboard1();
    static void starboard10();
    static void port1();
    static void port10();
    static void tackPort();
    static void tackStarboard();
};


#endif
