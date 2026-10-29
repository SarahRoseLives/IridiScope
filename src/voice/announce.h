// Recognises the network's recorded announcements ("The subscriber cannot be
// reached...", "You cannot access an emergency responder using shortcuts
// such as 911...") in decoded call audio, so those calls can be filed apart
// from real conversations.
//
// Each reference recording in assets/announcements/<name>.wav is a template.
// A call matches when, at its best alignment against a template, the
// spectral shape of its speech follows the template frame by frame
// (20 ms frames, 20 bands, normalised). Different speech scores ~0.0-0.2,
// another playout of the same announcement ~0.6-1.0. Drop more reference
// WAVs into that folder to recognise more announcements.
#ifndef IRIDISCOPE_ANNOUNCE_H
#define IRIDISCOPE_ANNOUNCE_H

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace iridi {

class AnnouncementMatcher {
public:
    static AnnouncementMatcher &instance();
    // Loads every .wav in dir (8 kHz 16-bit mono) as a template. Returns how many.
    int load(const std::string &dir);
    int templates() const;
    // Name of the matching announcement ("" if none), and the best score.
    std::string classify(const std::vector<int16_t> &pcm, double *score = nullptr) const;

    static constexpr double kThreshold = 0.45;

private:
    struct Feat { std::vector<float> v; std::vector<char> on; int n = 0; };
    struct Tmpl { std::string name; Feat f; };
    Feat features(const std::vector<int16_t> &pcm, size_t max_samples) const;
    static double best_match(const Feat &c, const Feat &t);
    mutable std::mutex mtx_;
    std::vector<Tmpl> t_;
    std::vector<float> mean_;   // average band levels of the templates
};

} // namespace iridi

#endif // IRIDISCOPE_ANNOUNCE_H
