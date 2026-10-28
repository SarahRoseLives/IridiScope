// Message database: every decoded ACARS, SMS, SBD and pager message, voice
// call and ring alert is written to an SQLite file (one per session, in
// logs/), so a session can be searched later with any SQLite tool
// (DB Browser for SQLite, sqlite3, Python, ...).
//
// Writes are queued and committed by a background thread about once a second
// in a single transaction, so decoding is never held up by the disk.
#ifndef IRIDISCOPE_MSG_DB_H
#define IRIDISCOPE_MSG_DB_H

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct sqlite3;

namespace iridi {

struct AcarsMessage;
struct SmsMessage;
struct SbdMessage;
struct PagerMessage;
struct VoiceCall;

class MsgDb {
public:
    static MsgDb &instance();

    // Opens logs/iridiscope_YYYYMMDD_HHMMSS.db (created on first message).
    void set_enabled(bool on);
    bool enabled() const { return enabled_.load(); }
    std::string path() const;
    uint64_t rows_written() const { return rows_.load(); }
    // Messages seen this session (counted even with the database off):
    // 1 ACARS, 2 SMS, 3 SBD, 4 pager, 5 voice calls saved, 6 ring alerts.
    uint64_t count(int table) const { return table > 0 && table < 8 ? counts_[table].load() : 0; }
    void shutdown();   // flush and close (app exit)

    void acars(const AcarsMessage &m);
    void sms(const SmsMessage &m);
    void sbd(const SbdMessage &m);
    void pager(const PagerMessage &m);
    void voice(const VoiceCall &c);
    void ring_alert(uint64_t ts_ns, int sat, int beam, uint32_t tmsi, int msc_id);

private:
    MsgDb() = default;
    ~MsgDb() { shutdown(); }
    struct Row {
        int table = 0;
        std::vector<std::string> text;
        std::vector<double> num;
    };
    void push(Row &&r);
    void writer_main();
    bool open_db();

    std::atomic<bool> enabled_{true};
    mutable std::mutex mtx_;
    std::condition_variable cv_;
    std::deque<Row> q_;
    bool stop_ = false;
    std::thread th_;
    sqlite3 *db_ = nullptr;
    std::string path_;
    std::atomic<uint64_t> rows_{0};
    std::atomic<uint64_t> counts_[8] = {};
};

} // namespace iridi

#endif // IRIDISCOPE_MSG_DB_H
