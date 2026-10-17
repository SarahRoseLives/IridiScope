// Minimal JSON parser (objects, arrays, strings, numbers, bool, null).
// Header-only; used for the small adsbdb responses.
#ifndef IRIDISCOPE_JSON_MINI_H
#define IRIDISCOPE_JSON_MINI_H

#include <cstddef>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

namespace jsonmini {

struct Value {
    enum Type { Null, Bool, Number, String, Array, Object } type = Null;
    bool b = false;
    double num = 0.0;
    std::string str;
    std::vector<Value> arr;
    std::vector<std::pair<std::string, Value>> obj;

    const Value *find(const std::string &key) const {
        if (type != Object) return nullptr;
        for (const auto &kv : obj)
            if (kv.first == key) return &kv.second;
        return nullptr;
    }
    std::string s(const std::string &key) const {
        const Value *v = find(key);
        return (v && v->type == String) ? v->str : std::string();
    }
};

class Parser {
public:
    explicit Parser(const std::string &s) : s_(s) {}
    bool parse(Value &out) {
        skip();
        if (!parseValue(out)) return false;
        skip();
        return true;
    }

private:
    const std::string &s_;
    size_t i_ = 0;

    void skip() {
        while (i_ < s_.size() && (s_[i_] == ' ' || s_[i_] == '\t' || s_[i_] == '\n' ||
                                  s_[i_] == '\r'))
            ++i_;
    }

    bool parseValue(Value &v) {
        skip();
        if (i_ >= s_.size()) return false;
        char c = s_[i_];
        if (c == '{') return parseObject(v);
        if (c == '[') return parseArray(v);
        if (c == '"') { v.type = Value::String; return parseString(v.str); }
        if (c == 't') {
            if (s_.compare(i_, 4, "true") == 0) { i_ += 4; v.type = Value::Bool; v.b = true; return true; }
            return false;
        }
        if (c == 'f') {
            if (s_.compare(i_, 5, "false") == 0) { i_ += 5; v.type = Value::Bool; v.b = false; return true; }
            return false;
        }
        if (c == 'n') {
            if (s_.compare(i_, 4, "null") == 0) { i_ += 4; v.type = Value::Null; return true; }
            return false;
        }
        return parseNumber(v);
    }

    bool parseString(std::string &out) {
        if (i_ >= s_.size() || s_[i_] != '"') return false;
        ++i_;
        out.clear();
        while (i_ < s_.size()) {
            char c = s_[i_++];
            if (c == '"') return true;
            if (c != '\\') { out += c; continue; }
            if (i_ >= s_.size()) return false;
            char e = s_[i_++];
            switch (e) {
            case '"':  out += '"';  break;
            case '\\': out += '\\'; break;
            case '/':  out += '/';  break;
            case 'b':  out += '\b'; break;
            case 'f':  out += '\f'; break;
            case 'n':  out += '\n'; break;
            case 'r':  out += '\r'; break;
            case 't':  out += '\t'; break;
            case 'u': {
                if (i_ + 4 > s_.size()) return false;
                unsigned cp = 0;
                for (int k = 0; k < 4; ++k) {
                    char h = s_[i_++];
                    cp <<= 4;
                    if (h >= '0' && h <= '9') cp += (unsigned)(h - '0');
                    else if (h >= 'a' && h <= 'f') cp += (unsigned)(h - 'a' + 10);
                    else if (h >= 'A' && h <= 'F') cp += (unsigned)(h - 'A' + 10);
                    else return false;
                }
                if (cp < 0x80) {
                    out += (char)cp;
                } else if (cp < 0x800) {
                    out += (char)(0xC0 | (cp >> 6));
                    out += (char)(0x80 | (cp & 0x3F));
                } else {
                    out += (char)(0xE0 | (cp >> 12));
                    out += (char)(0x80 | ((cp >> 6) & 0x3F));
                    out += (char)(0x80 | (cp & 0x3F));
                }
                break;
            }
            default: return false;
            }
        }
        return false;
    }

    bool parseNumber(Value &v) {
        size_t start = i_;
        if (i_ < s_.size() && (s_[i_] == '-' || s_[i_] == '+')) ++i_;
        while (i_ < s_.size()) {
            char c = s_[i_];
            if ((c >= '0' && c <= '9') || c == '.' || c == 'e' || c == 'E' || c == '+' || c == '-')
                ++i_;
            else
                break;
        }
        if (i_ == start) return false;
        v.type = Value::Number;
        v.num = std::strtod(s_.substr(start, i_ - start).c_str(), nullptr);
        return true;
    }

    bool parseArray(Value &v) {
        ++i_;
        v.type = Value::Array;
        skip();
        if (i_ < s_.size() && s_[i_] == ']') { ++i_; return true; }
        while (true) {
            Value e;
            if (!parseValue(e)) return false;
            v.arr.push_back(std::move(e));
            skip();
            if (i_ >= s_.size()) return false;
            if (s_[i_] == ',') { ++i_; continue; }
            if (s_[i_] == ']') { ++i_; return true; }
            return false;
        }
    }

    bool parseObject(Value &v) {
        ++i_;
        v.type = Value::Object;
        skip();
        if (i_ < s_.size() && s_[i_] == '}') { ++i_; return true; }
        while (true) {
            skip();
            std::string key;
            if (!parseString(key)) return false;
            skip();
            if (i_ >= s_.size() || s_[i_] != ':') return false;
            ++i_;
            Value e;
            if (!parseValue(e)) return false;
            v.obj.emplace_back(std::move(key), std::move(e));
            skip();
            if (i_ >= s_.size()) return false;
            if (s_[i_] == ',') { ++i_; continue; }
            if (s_[i_] == '}') { ++i_; return true; }
            return false;
        }
    }
};

inline bool parse(const std::string &text, Value &out) {
    Parser p(text);
    return p.parse(out);
}

} // namespace jsonmini

#endif // IRIDISCOPE_JSON_MINI_H
