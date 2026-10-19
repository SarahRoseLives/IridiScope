#include "util/iritime.h"

#include <ctime>

namespace iridi {

std::string fmt_iritime(uint32_t t) {
    // Different Iridium epochs are known; ERA3 is current.
    // ERA3: 2025-02-14T18:14:17Z : 1739556857
    double ux = (double)t * 90.0 / 1000.0 + 1739556857.0;
    time_t sec = (time_t)ux;
    struct tm *tmv = gmtime(&sec);
    if (!tmv) return "?";
    char buf[40];
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", tmv);
    return buf;
}

} // namespace iridi
