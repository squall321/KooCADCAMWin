#pragma once
// @lat: [[engine/skills#screw_terminal_strip_array]]
//
// screw_terminal_strip_array — COMPOUND screw terminal block mounting.
//
// Drills N evenly-spaced threaded mounting holes for a screw terminal
// strip (e.g., Phoenix Contact MKDS-style).  Standard pitches:
//   5.08 mm (= 0.2"), 7.5 mm, 10.16 mm (= 0.4")
// covering 90 % of industrial screw-terminal block formats.
//
// Sub-feature chain:
//   build tap-pilot cylinder × position_count → cutMany.
//
// DFM gates:
//   DFM-INPUT     : position_count < 2, strip_length_mm <= 0.
//   DFM-THREAD-KEY: screw_thread_size not in central ISO M-thread table.
//   DFM-STRIP-PITCH: pitch < 4.0 mm (Phoenix smallest = 5.08; nothing below 4).
//   DFM-STRIP-FIT : (count-1) × pitch + screw_clearance > strip_length_mm.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace screw_terminal_strip_array {

constexpr const char* kSkillId = "screw_terminal_strip_array";

struct Input
{
    int         face_id              = -1;
    double      strip_origin_x_mm    = 0.0;   // first hole centre X
    double      strip_origin_y_mm    = 0.0;   // first hole centre Y
    int         position_count       = 0;
    double      position_pitch_mm    = 0.0;   // 5.08 / 7.5 / 10.16 (Phoenix)
    std::string screw_thread_size    = "M3";  // M2.5 | M3 | M4
    double      strip_length_mm      = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace screw_terminal_strip_array
}  // namespace koocadcam::skill
