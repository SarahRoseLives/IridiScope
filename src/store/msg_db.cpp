#include "store/msg_db.h"
#include "core/priority.h"

#include "messages/acars.h"
#include "messages/pager.h"
#include "voice/voice_manager.h"

#include <sqlite3.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>

#if defined(_WIN32)
#include <direct.h>
#define MDB_MKDIR(p) _mkdir(p)
#else
#include <sys/stat.h>
#define MDB_MKDIR(p) mkdir(p, 0755)
#endif

namespace iridi {

namespace {
enum Table { T_ACARS = 1, T_SMS, T_SBD, T_PAGER, T_VOICE, T_RING };

const char *kSchema = R"SQL(
PRAGMA journal_mode=WAL;
PRAGMA synchronous=NORMAL;
CREATE TABLE IF NOT EXISTS acars (
  id INTEGER PRIMARY KEY, time_utc TEXT, ts REAL, freq_mhz REAL, dir TEXT,
  reg TEXT, flight TEXT, label TEXT, label_desc TEXT, mode TEXT, msg_num TEXT,
  sublabel TEXT, mfi TEXT, text TEXT, decoded TEXT, errors TEXT);
CREATE TABLE IF NOT EXISTS sms (
  id INTEGER PRIMARY KEY, time_utc TEXT, ts REAL, freq_mhz REAL, dir TEXT,
  sender TEXT, recipient TEXT, text TEXT, dcs TEXT);
CREATE TABLE IF NOT EXISTS sbd (
  id INTEGER PRIMARY KEY, time_utc TEXT, ts REAL, freq_mhz REAL, dir TEXT,
  type TEXT, length INTEGER, info TEXT, text TEXT, payload_hex TEXT,
  lat REAL, lon REAL);
CREATE TABLE IF NOT EXISTS pager (
  id INTEGER PRIMARY KEY, time_utc TEXT, ts REAL, freq_mhz REAL,
  ric INTEGER, fmt INTEGER, seq INTEGER, parts INTEGER, text TEXT);
CREATE TABLE IF NOT EXISTS voice_calls (
  id INTEGER PRIMARY KEY, time_utc TEXT, ts REAL, end_ts REAL, freq_mhz REAL,
  duration_s REAL, speech_s REAL, frames INTEGER, missing INTEGER,
  error_pct REAL, handoffs INTEGER, type TEXT, wav TEXT, announcement TEXT);
CREATE TABLE IF NOT EXISTS ring_alerts (
  id INTEGER PRIMARY KEY, time_utc TEXT, ts REAL, sat INTEGER, beam INTEGER,
  tmsi TEXT, msc_id INTEGER);
CREATE INDEX IF NOT EXISTS ix_acars_ts ON acars(ts);
CREATE INDEX IF NOT EXISTS ix_acars_reg ON acars(reg);
CREATE INDEX IF NOT EXISTS ix_sms_ts ON sms(ts);
CREATE INDEX IF NOT EXISTS ix_sbd_ts ON sbd(ts);
CREATE INDEX IF NOT EXISTS ix_pager_ts ON pager(ts);
CREATE INDEX IF NOT EXISTS ix_pager_ric ON pager(ric);
CREATE INDEX IF NOT EXISTS ix_voice_ts ON voice_calls(ts);
CREATE INDEX IF NOT EXISTS ix_ring_ts ON ring_alerts(ts);
CREATE INDEX IF NOT EXISTS ix_ring_tmsi ON ring_alerts(tmsi);
)SQL";

// Columns per table: 'T' = text value, 'N' = number value, in order.
struct Spec { const char *sql; const char *cols; };
const Spec kSpecs[] = {
    {nullptr, nullptr},
    {"INSERT INTO acars(time_utc,ts,freq_mhz,dir,reg,flight,label,label_desc,mode,msg_num,sublabel,mfi,text,decoded,errors)"
     " VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)", "TNNTTTTTTTTTTTT"},
    {"INSERT INTO sms(time_utc,ts,freq_mhz,dir,sender,recipient,text,dcs) VALUES(?,?,?,?,?,?,?,?)", "TNNTTTTT"},
    {"INSERT INTO sbd(time_utc,ts,freq_mhz,dir,type,length,info,text,payload_hex,lat,lon)"
     " VALUES(?,?,?,?,?,?,?,?,?,?,?)", "TNNTTNTTTNN"},
    {"INSERT INTO pager(time_utc,ts,freq_mhz,ric,fmt,seq,parts,text) VALUES(?,?,?,?,?,?,?,?)", "TNNNNNNT"},
    {"INSERT INTO voice_calls(time_utc,ts,end_ts,freq_mhz,duration_s,speech_s,frames,missing,error_pct,handoffs,type,wav,announcement)"
     " VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?)", "TNNNNNNNNNTTT"},
    {"INSERT INTO ring_alerts(time_utc,ts,sat,beam,tmsi,msc_id) VALUES(?,?,?,?,?,?)", "TNNNTN"},
};

std::string utc(uint64_t ts_ns) {
    const std::time_t t = (std::time_t)(ts_ns / 1000000000ULL);
    std::tm u{};
#if defined(_WIN32)
    gmtime_s(&u, &t);
#else
    gmtime_r(&t, &u);
#endif
    char b[96];
    std::snprintf(b, sizeof(b), "%04d-%02d-%02d %02d:%02d:%02d.%03dZ", u.tm_year + 1900, u.tm_mon + 1, u.tm_mday,
                  u.tm_hour, u.tm_min, u.tm_sec, (int)((ts_ns / 1000000ULL) % 1000));
    return b;
}
double secs(uint64_t ts_ns) { return (double)ts_ns / 1e9; }
const char *dirn(int ul) { return ul ? "UL" : "DL"; }
} // namespace

MsgDb &MsgDb::instance() {
    static MsgDb db;
    return db;
}

void MsgDb::set_enabled(bool on) { enabled_.store(on); }

std::string MsgDb::path() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return path_;
}

void MsgDb::push(Row &&r) {
    if (r.table > 0 && r.table < 8) counts_[r.table].fetch_add(1);
    if (!enabled_.load()) return;
    std::lock_guard<std::mutex> lk(mtx_);
    if (stop_) return;
    if (q_.size() > 200000) return;   // disk stalled: don't grow without bound
    q_.push_back(std::move(r));
    if (!th_.joinable()) th_ = std::thread(&MsgDb::writer_main, this);
}

bool MsgDb::open_db() {
    MDB_MKDIR("logs");
    const std::time_t t = std::time(nullptr);
    std::tm lt{};
#if defined(_WIN32)
    localtime_s(&lt, &t);
#else
    localtime_r(&t, &lt);
#endif
    char name[64];
    std::strftime(name, sizeof(name), "logs/iridiscope_%Y%m%d_%H%M%S.db", &lt);
    if (sqlite3_open(name, &db_) != SQLITE_OK) {
        if (db_) sqlite3_close(db_);
        db_ = nullptr;
        return false;
    }
    sqlite3_busy_timeout(db_, 2000);
    char *err = nullptr;
    sqlite3_exec(db_, kSchema, nullptr, nullptr, &err);
    if (err) sqlite3_free(err);
    std::lock_guard<std::mutex> lk(mtx_);
    path_ = name;
    return true;
}

void MsgDb::writer_main() {
    if (!open_db()) {
        std::lock_guard<std::mutex> lk(mtx_);
        q_.clear();
        return;
    }
    sqlite3_stmt *st[7] = {};
    for (int i = 1; i <= 6; i++) sqlite3_prepare_v2(db_, kSpecs[i].sql, -1, &st[i], nullptr);
    int prio_gen = 0;
    for (;;) {
        iridi_refresh_thread_priority(IRIDI_PRIO_NORMAL, &prio_gen);
        std::deque<Row> batch;
        {
            std::unique_lock<std::mutex> lk(mtx_);
            cv_.wait_for(lk, std::chrono::seconds(1), [this] { return stop_ || q_.size() >= 2000; });
            batch.swap(q_);
            if (batch.empty() && stop_) break;
        }
        if (batch.empty()) continue;
        sqlite3_exec(db_, "BEGIN", nullptr, nullptr, nullptr);
        for (const Row &r : batch) {
            sqlite3_stmt *s = st[r.table];
            if (!s) continue;
            const char *cols = kSpecs[r.table].cols;
            size_t ti = 0, ni = 0;
            for (int c = 0; cols[c]; c++) {
                if (cols[c] == 'T') {
                    const std::string &v = r.text[ti++];
                    if (v.empty()) sqlite3_bind_null(s, c + 1);
                    else sqlite3_bind_text(s, c + 1, v.c_str(), (int)v.size(), SQLITE_TRANSIENT);
                } else {
                    const double v = r.num[ni++];
                    if (std::isnan(v)) sqlite3_bind_null(s, c + 1);
                    else sqlite3_bind_double(s, c + 1, v);
                }
            }
            sqlite3_step(s);
            sqlite3_reset(s);
            rows_.fetch_add(1);
        }
        sqlite3_exec(db_, "COMMIT", nullptr, nullptr, nullptr);
    }
    for (auto *s : st) if (s) sqlite3_finalize(s);
    sqlite3_close(db_);
    db_ = nullptr;
}

void MsgDb::shutdown() {
    {
        std::lock_guard<std::mutex> lk(mtx_);
        stop_ = true;
    }
    cv_.notify_all();
    if (th_.joinable()) th_.join();
}

void MsgDb::acars(const AcarsMessage &m) {
    Row r; r.table = T_ACARS;
    r.text = {utc(m.timestamp_ns), dirn(m.ul), m.reg, m.flight, m.label, m.label_desc, m.mode, m.msg_num,
              m.sublabel, m.mfi, m.text, m.arinc622_text, m.errors};
    r.num = {secs(m.timestamp_ns), m.frequency / 1e6};
    push(std::move(r));
}

void MsgDb::sms(const SmsMessage &m) {
    Row r; r.table = T_SMS;
    r.text = {utc(m.timestamp_ns), dirn(m.ul), m.from, m.to, m.text, m.dcs};
    r.num = {secs(m.timestamp_ns), m.frequency / 1e6};
    push(std::move(r));
}

void MsgDb::sbd(const SbdMessage &m) {
    Row r; r.table = T_SBD;
    r.text = {utc(m.timestamp_ns), dirn(m.ul), m.type, m.info, m.text, m.payload_hex};
    const bool pos = m.nal_lat != 0 || m.nal_lon != 0;
    r.num = {secs(m.timestamp_ns), m.frequency / 1e6, (double)m.length, pos ? m.nal_lat : NAN, pos ? m.nal_lon : NAN};
    push(std::move(r));
}

void MsgDb::pager(const PagerMessage &m) {
    Row r; r.table = T_PAGER;
    r.text = {utc(m.timestamp_ns), m.text};
    r.num = {secs(m.timestamp_ns), m.frequency / 1e6, (double)m.ric, (double)m.fmt, (double)m.seq, (double)m.parts};
    push(std::move(r));
}

void MsgDb::voice(const VoiceCall &c) {
    Row r; r.table = T_VOICE;
    r.text = {utc(c.start_ns), c.subtype.empty() ? "VOC" : c.subtype, c.wav_path, c.announcement};
    auto r2 = [](double v) { return std::round(v * 100.0) / 100.0; };
    r.num = {secs(c.start_ns), secs(c.end_ns), std::round(c.channel_hz) / 1e6, r2(c.duration_s), r2(c.speech_s),
             (double)c.frames, (double)c.missing, r2(100.0 * c.error_ratio), (double)c.handoffs};
    push(std::move(r));
}

void MsgDb::ring_alert(uint64_t ts_ns, int sat, int beam, uint32_t tmsi, int msc_id) {
    Row r; r.table = T_RING;
    char t[16]; std::snprintf(t, sizeof(t), "%08X", tmsi);
    r.text = {utc(ts_ns), t};
    r.num = {secs(ts_ns), (double)sat, (double)beam, (double)msc_id};
    push(std::move(r));
}

} // namespace iridi
