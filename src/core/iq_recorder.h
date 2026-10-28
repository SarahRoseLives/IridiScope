// Raw IQ recording: writes the radio's samples to 16-bit stereo WAV files
// (the same layout SDR# / airspy_rx use), on a background thread so the
// detector is never held up by the disk.
//
// Files are named IridiScope_YYYYMMDD_HHMMSSZ_<center>Hz_IQ.wav, so IridiScope
// (and other tools) pick up the centre frequency and the start time when the
// file is played back. A new file is started every ~2 GB (WAV's 4 GB limit)
// and after a long gap in the radio stream; short gaps are written as silence
// so time within a file stays correct.
#ifndef IRIDISCOPE_IQ_RECORDER_H
#define IRIDISCOPE_IQ_RECORDER_H

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace iridi {

class IqRecorder {
public:
    ~IqRecorder() { stop(); }

    // Starts recording into `dir` (created if missing). Returns false with a
    // reason if the first file can't be created.
    bool start(const std::string &dir, int sample_rate, double center_hz, std::string &err);
    void stop();
    bool active() const { return active_.load(); }

    // Engine thread: samples just read from the radio, and how many samples
    // were lost just before them.
    void push(const float *iq, size_t n, uint64_t gap_before);

    // Status for the UI.
    uint64_t bytes_written() const { return bytes_.load(); }
    double seconds_recorded() const { return rate_ > 0 ? (double)samples_.load() / rate_ : 0.0; }
    uint64_t dropped_blocks() const { return dropped_blocks_.load(); }
    std::string current_file() const;
    int files_written() const { return files_.load(); }

private:
    struct Block { std::vector<int16_t> s; bool new_file = false; uint64_t start_ns = 0; };
    void writer_main();
    bool open_file(uint64_t start_ns);
    void close_file();

    std::string dir_;
    int rate_ = 0;
    double center_hz_ = 0;
    std::atomic<bool> active_{false};

    // Engine-thread state.
    uint64_t stream_samples_ = 0;   // samples since start, incl. gaps (for file times)
    uint64_t start_ns_ = 0;
    uint64_t file_samples_ = 0;     // samples in the current file (engine-side count)

    mutable std::mutex mtx_;
    std::condition_variable cv_;
    std::deque<Block> queue_;
    size_t queued_bytes_ = 0;
    bool stopping_ = false;
    std::thread writer_;

    // Writer-thread state.
    FILE *f_ = nullptr;
    uint64_t data_bytes_ = 0;
    std::string file_;
    int since_header_ = 0;

    std::atomic<uint64_t> bytes_{0}, samples_{0}, dropped_blocks_{0};
    std::atomic<int> files_{0};
};

} // namespace iridi

#endif // IRIDISCOPE_IQ_RECORDER_H
