#pragma once
// @lat: [[engine/skills#thermal_pad_recess_with_to220]]
//
// thermal_pad_recess_with_to220 — COMPOUND TO-220/TO-247/TO-263/D2PAK
// thermal-pad recess + screw mounting hole.
//
// Sub-feature chain (2 ops):
//   1. CUT shallow rectangular recess sized to device package footprint +
//      thermal_pad clearance (typ. 0.1 mm).
//   2. CUT a threaded clearance hole for the package's standard mounting
//      screw (TO-220/TO-263 → M3, TO-247 → M4, D2PAK → M3) using the
//      central ISO M-thread table (_iso_thread_table.hpp) tap_pilot_dia.
//
// Device-package footprint table (manufacturer-standardised dims, mm):
//   TO-220  : 10.0 × 4.6   (tab thickness ~ 1.27)  → M3 screw
//   TO-247  : 16.0 × 5.5   (tab ~ 2.0)              → M4 screw
//   TO-263  : 10.0 × 8.5   (SMD D2PAK family)       → M3 screw (typ. clip)
//   D2PAK   : 10.3 × 8.4   (SMD power package)      → M3 screw
//
// DFM gates:
//   DFM-PKG       : device_package not in table.
//   DFM-PAD-THK   : pad_thickness_mm <= 0.05 or > 5.0 (sanity).
//   DFM-THREAD-KEY: screw_thread_size_key not in central ISO table.
//   DFM-PKG-MATCH : passed screw doesn't match recommended size for the package
//                   (warning only — caller may use a different screw).

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace thermal_pad_recess_with_to220 {

constexpr const char* kSkillId = "thermal_pad_recess_with_to220";

struct Input
{
    int         face_id              = -1;
    double      center_x_mm          = 0.0;
    double      center_y_mm          = 0.0;
    std::string device_package       = "TO-220";  // TO-220 | TO-247 | TO-263 | D2PAK
    double      pad_thickness_mm     = 0.5;
    std::string screw_thread_size_key = "M3";     // ISO M, from central table
};

// Footprint constants exposed for tests.
struct PackageFootprint
{
    double width_mm        = 0.0;   // wider dimension (X)
    double depth_mm        = 0.0;   // shorter dimension (Y)
    double screw_offset_mm = 0.0;   // hole offset from one edge along width
    const char* recommended_screw = "";
};

// Lookup; returns zeroed footprint when package unknown.
PackageFootprint footprintFor(const std::string& device_package);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace thermal_pad_recess_with_to220
}  // namespace koocadcam::skill
