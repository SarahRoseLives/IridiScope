#include "messages/pager.h"

#include <cstdio>

namespace iridi {

namespace {

std::string bytes_to_text(const std::vector<uint8_t> &b) {
    std::string out;
    for (uint8_t v : b) {
        if (v == 3) continue; // ETX
        if (v >= 32 && v < 127) {
            out += (char)v;
        } else {
            char t[8];
            snprintf(t, sizeof(t), "[%d]", v);
            out += t;
        }
    }
    return out;
}

} // namespace

void PagerAssembler::reset() {
    std::lock_guard<std::mutex> lk(mtx_);
    buf_.clear();
    msgs_.clear();
    count_ = 0;
}

void PagerAssembler::expire(double now) {
    for (size_t i = 0; i < buf_.size();) {
        if (now - buf_[i].time > 2000.0) buf_.erase(buf_.begin() + i);
        else ++i;
    }
}

void PagerAssembler::emit(Entry &e) {
    PagerMessage m;
    m.timestamp_ns = e.timestamp_ns;
    m.frequency = e.frequency;
    m.ric = e.ric;
    m.fmt = e.fmt;
    m.seq = e.seq;
    m.parts = e.parts;

    if (e.fmt == 5) {
        std::vector<uint8_t> content;
        for (int i = 0; i < e.parts; i++) {
            const auto &d = e.data[i];
            content.insert(content.end(), d.begin(), d.end());
        }
        while (!content.empty() && content.back() == 3) content.pop_back();
        int s = 0;
        for (uint8_t b : content) s = (s + b) % 128;
        m.ok = (((~s) & 127) == e.checksum);
        m.text = bytes_to_text(content);
    } else if (e.fmt == 3) {
        std::string txt;
        for (int i = 0; i < e.parts; i++) txt += e.text[i];
        while (!txt.empty() && txt.back() == 'c') txt.pop_back();
        m.ok = !txt.empty();
        for (char c : txt)
            if (c < '0' || c > '9') { m.ok = false; break; }
        m.text = txt;
    } else {
        m.ok = false;
    }

    msgs_.push_back(m);
    if (msgs_.size() > 1000) msgs_.pop_front();
    count_++;
}

void PagerAssembler::feed(const irid::ParsedMessage &pm) {
    if (!pm.has_msg) return;
    std::lock_guard<std::mutex> lk(mtx_);

    double t = (double)pm.timestamp_ns / 1e9;
    int parts = pm.msg_parts + 1;
    if (parts < 1) parts = 1;

    size_t idx = buf_.size();
    for (size_t i = 0; i < buf_.size(); i++) {
        Entry &x = buf_[i];
        if (x.ric == pm.msg_ric && x.seq == pm.msg_seq && x.fmt == pm.msg_fmt) {
            idx = i;
            break;
        }
    }

    if (idx < buf_.size()) {
        Entry &e = buf_[idx];
        if (e.checksum != pm.msg_checksum && (t - e.time > 600.0 || pm.msg_part >= e.parts))
            buf_.erase(buf_.begin() + idx), idx = buf_.size();
    }

    if (idx >= buf_.size()) {
        Entry e;
        e.ric = pm.msg_ric;
        e.fmt = pm.msg_fmt;
        e.seq = pm.msg_seq;
        e.parts = parts;
        e.frequency = pm.frequency;
        e.have.assign(parts, 0);
        e.data.resize(parts);
        e.text.resize(parts);
        e.checksum = pm.msg_checksum;
        e.time = t;
        e.timestamp_ns = pm.timestamp_ns;
        buf_.push_back(std::move(e));
        idx = buf_.size() - 1;
    }

    Entry &e = buf_[idx];
    if (pm.msg_part < 0 || pm.msg_part >= e.parts) { expire(t); return; }
    if (pm.msg_fmt == 5) e.data[pm.msg_part] = pm.msg_bytes;
    else e.text[pm.msg_part] = pm.msg_text;
    e.have[pm.msg_part] = 1;
    e.time = t;
    e.timestamp_ns = pm.timestamp_ns;
    e.checksum = pm.msg_checksum;

    bool complete = true;
    for (char h : e.have)
        if (!h) { complete = false; break; }

    if (complete) {
        // Keep the completed entry around so retransmissions of the same
        // message (often on several simplex channels) are not emitted again.
        if (!e.sent) emit(e);
        e.sent = true;
    }

    expire(t);
}

std::vector<PagerMessage> PagerAssembler::messages() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return std::vector<PagerMessage>(msgs_.begin(), msgs_.end());
}

} // namespace iridi
