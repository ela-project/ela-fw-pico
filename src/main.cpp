#include "pico/stdlib.h"
#include "analyzer.hpp"
#include "hardware/clocks.h"
#include "hardware/pll.h"

int main() {
    set_sys_clock_khz(120000, true);

    analyzer_agent.init();
    for (;;) {
        analyzer_agent.idle();
    }
    return 0;
}
