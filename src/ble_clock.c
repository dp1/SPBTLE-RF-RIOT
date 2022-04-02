#include "ble_clock.h"
#include "xtimer.h"

tClockTime Clock_Time(void) {
    return xtimer_now_usec() / 1000;
}
void Clock_Wait(tClockTime t) {
    xtimer_usleep(t * 1000);
}
