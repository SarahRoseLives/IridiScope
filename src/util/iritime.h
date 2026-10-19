// Iridium time (LBFC / IBC iri_time) formatting.
#ifndef IRIDISCOPE_IRITIME_H
#define IRIDISCOPE_IRITIME_H

#include <cstdint>
#include <string>

namespace iridi {

// Format an Iridium time counter (90 ms units) as UTC. Uses the ERA3 epoch.
std::string fmt_iritime(uint32_t t);

} // namespace iridi

#endif // IRIDISCOPE_IRITIME_H
