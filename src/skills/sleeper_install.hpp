#pragma once
// @lat: [[engine/skills#sleeper_install]]
//
// sleeper_install — placement of railway sleepers (cross-ties) on the
// ballast bed before the rail is fixed.  Sleeper type drives the spacing
// rule of thumb (concrete monoblock ≈ 600 mm, wood ≈ 650 mm, steel
// trough ≈ 700 mm).  KooCADCAM records sleeper_type, centre-to-centre
// spacing_mm, and count installed.  Macroscopic B-rep unchanged.
//
//      ▭   ▭   ▭   ▭   ▭   ▭   ▭         ← count sleepers, spacing_mm CTC
//
// Signature pattern:
//   - pattern.kind             = "sleeper_install"
//   - pattern.sleeper_type     = "concrete" | "wood" | "steel"
//   - pattern.spacing_mm       = centre-to-centre spacing
//   - pattern.count            = sleepers installed
//   - pattern.geometry_changed = false
//
// DFM checks:
//   sleeper_type ∈ {concrete,wood,steel}    (DFM-SL-TYPE info if unknown)
//   spacing_mm   ∈ [450, 900]               (DFM-SL-SPACING error if outside)
//   count        >= 1                       (DFM-INPUT error)
//
// Synthesis:
//   NO geometric change. Clone + stamp.
//
// Recognize:
//   Metadata replay only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace sleeper_install {

constexpr const char* kSkillId = "sleeper_install";

struct Input
{
    std::string  sleeper_type = "concrete"; // "concrete" | "wood" | "steel"
    double       spacing_mm   = 600.0;      // CTC spacing
    int          count        = 100;        // sleepers installed
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace sleeper_install
}  // namespace koocadcam::skill
