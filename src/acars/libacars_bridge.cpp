#include "acars/libacars_bridge.h"

#ifdef HAS_LIBACARS

extern "C" {
#include <libacars/libacars.h>
#include <libacars/acars.h>
#include <libacars/reassembly.h>
#include <libacars/vstring.h>
#include <libacars/adsc.h>
#include <libacars/cpdlc.h>
}

#include <mutex>
#include <sys/time.h>

namespace iridi {

namespace {

std::once_flag g_init;
la_reasm_ctx *g_ctx = nullptr;
std::mutex g_mtx;

void ensure_init() {
    std::call_once(g_init, [] {
        la_config_set_int("acars_bearer", LA_ACARS_BEARER_SATCOM);
    });
}

std::string cstr(const char *s) { return s ? std::string(s) : std::string(); }

} // namespace

bool libacars_available() { return true; }

void libacars_reset() {
    std::lock_guard<std::mutex> lk(g_mtx);
    if (g_ctx) {
        la_reasm_ctx_destroy(g_ctx);
        g_ctx = nullptr;
    }
}

AcarsDecoded decode_acars_libacars(const std::vector<uint8_t> &raw, int ul, double time_s) {
    AcarsDecoded out;
    if (raw.size() < 2 || raw[0] != 0x01) return out;

    const uint8_t *p = raw.data() + 1;
    int n = (int)raw.size() - 1;
    if (n >= 8 && p[0] == 0x03) {
        p += 8;
        n -= 8;
    }
    if (n < 13) return out;

    std::lock_guard<std::mutex> lk(g_mtx);
    ensure_init();
    if (!g_ctx) g_ctx = la_reasm_ctx_new();
    if (!g_ctx) return out;

    struct timeval tv;
    tv.tv_sec = (time_t)time_s;
    tv.tv_usec = (long)((time_s - (double)tv.tv_sec) * 1000000.0);
    la_msg_dir dir = ul ? LA_MSG_DIR_AIR2GND : LA_MSG_DIR_GND2AIR;

    la_proto_node *tree = la_acars_parse_and_reassemble(p, n, dir, g_ctx, tv);
    if (!tree) return out;

    la_proto_node *acars_node = la_proto_tree_find_acars(tree);
    if (!acars_node || !acars_node->data) {
        la_proto_tree_destroy(tree);
        return out;
    }
    la_acars_msg *msg = (la_acars_msg *)acars_node->data;
    if (msg->reasm_status == LA_REASM_IN_PROGRESS) {
        la_proto_tree_destroy(tree);
        return out;
    }

    out.ok = true;
    out.crc_ok = msg->crc_ok;
    out.err = msg->err;
    out.final_block = msg->final_block;
    out.mode.assign(1, msg->mode ? msg->mode : ' ');
    out.reg = cstr(msg->reg);
    out.ack.assign(1, msg->ack ? msg->ack : ' ');
    out.label = cstr(msg->label);
    out.sublabel = cstr(msg->sublabel);
    out.mfi = cstr(msg->mfi);
    out.block_id.assign(1, msg->block_id ? msg->block_id : ' ');
    out.msg_num = cstr(msg->msg_num);
    out.flight = cstr(msg->flight_id);
    out.text = cstr(msg->txt);

    if (la_proto_tree_find_adsc(tree)) out.kind = "ADS-C";
    else if (la_proto_tree_find_cpdlc(tree)) out.kind = "CPDLC";

    la_vstring *vt = la_proto_tree_format_text(nullptr, tree);
    if (vt) {
        if (vt->str) out.arinc622_text.assign(vt->str, vt->len);
        la_vstring_destroy(vt, true);
    }
    la_vstring *vj = la_proto_tree_format_json(nullptr, tree);
    if (vj) {
        if (vj->str) out.arinc622_json.assign(vj->str, vj->len);
        la_vstring_destroy(vj, true);
    }

    la_proto_tree_destroy(tree);
    return out;
}

} // namespace iridi

#else // !HAS_LIBACARS

namespace iridi {

bool libacars_available() { return false; }
void libacars_reset() {}
AcarsDecoded decode_acars_libacars(const std::vector<uint8_t> &, int, double) {
    return AcarsDecoded{};
}

} // namespace iridi

#endif // HAS_LIBACARS
