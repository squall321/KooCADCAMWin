#pragma once
// @lat: [[engine/skills#mic_capsule_thread_compound]]
//
// mic_capsule_thread_compound — microphone capsule threaded mount.
//
// Slice 16 — audio / musical instrument hardware.
//
// Sub-features (sequential CUTs):
//   1. cosmetic thread (outer) — wide cylinder cut representing the thread OD
//   2. clearance pilot      — slightly smaller cylinder cut, deeper, that
//                             represents the unthreaded clearance bore
//                             below the threaded region.
//
// subfeature_count = 2.
//
// Supported mount styles (audio industry standard):
//   "5/8-27 UNS"  — universal mic stand thread (5/8" OD, 27 TPI UNS — NOT in
//                   the central UNC/UNF table; handled by a local single-row
//                   constant inside this skill).
//   "3/8-16 UNC"  — Euro/photographic adapter thread (UNC, in central table)
//   "1/4-20 UNC"  — camera tripod-style adapter (UNC, in central table)
//
// DFM gates:
//   DFM-INPUT       : positive depth
//   DFM-MOUNT-STYLE : mount_style in supported set
//   DFM-DEPTH       : mount_depth must be >= 3 * pitch (or 4 mm minimum)

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace mic_capsule_thread_compound {

constexpr const char* kSkillId = "mic_capsule_thread_compound";

struct Input
{
    FaceDatum   face_id;
    double      center_x_mm    = 0.0;
    double      center_y_mm    = 0.0;
    gp_Dir      axis_dir       { 0.0, 0.0, -1.0 };
    std::string mount_style    = "5/8-27 UNS";   // "5/8-27 UNS" | "3/8-16 UNC" | "1/4-20 UNC"
    double      mount_depth_mm = 10.0;
};

// Lookup helpers (public for tests).
double nominalDiaFor(const std::string& mount_style);
double pitchFor    (const std::string& mount_style);

SkillOutput  apply   (const Workpiece& wp, const Input& in);
DFMReport    validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace mic_capsule_thread_compound
}  // namespace koocadcam::skill
