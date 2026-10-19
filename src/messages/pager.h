// Pager message reassembly (Iridium IMS "MSG"/"MS3" frames).
// Ported from iridium-toolkit's reassembler/msg.py.
#ifndef IRIDISCOPE_PAGER_H
#define IRIDISCOPE_PAGER_H

#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include "parser/iridium_parser.h"

namespace iridi {

struct PagerMessage {
    uint64_t timestamp_ns = 0;
    double frequency = 0;
    int ric = 0;
    int fmt = 0;
    int seq = 0;
    int parts = 0;
    bool ok = false;
    std::string text;
};

class PagerAssembler {
public:
    PagerAssembler() = default;

    void reset();
    void feed(const irid::ParsedMessage &pm); // thread-safe
    std::vector<PagerMessage> messages() const;
    uint64_t count() const { return count_; }

private:
    struct Entry {
        int ric = 0, fmt = 0, seq = 0;
        int parts = 1;
        double frequency = 0;
        std::vector<char> have;
        std::vector<std::vector<uint8_t>> data; // fmt 5
        std::vector<std::string> text;          // fmt 3
        int checksum = 0;
        double time = 0;
        uint64_t timestamp_ns = 0;
    };

    void emit(Entry &e);
    void expire(double now);

    mutable std::mutex mtx_;
    std::vector<Entry> buf_;
    std::deque<PagerMessage> msgs_;
    uint64_t count_ = 0;
};

} // namespace iridi

#endif // IRIDISCOPE_PAGER_H
