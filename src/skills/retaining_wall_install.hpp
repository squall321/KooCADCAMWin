#pragma once
// @lat: [[engine/skills#retaining_wall_install]]
//
// retaining_wall_install — install a retaining wall along a road, bridge
// abutment, or cut slope.  Captures wall_type (gravity, mechanically
// stabilized reinforced, or driven sheet pile), height_m, and length_m.
// KooCADCAM records this as metadata; the wall body lives in the civil
// model upstream.
//
//                      ┌────────────┐
//                      │░░░░░░░░░░░░│  ← height_m
//          ════════════│░░░░░░░░░░░░│        wall (gravity / reinforced / sheet)
//                      │░░░░░░░░░░░░│        length_m runs into the page
//                      └────────────┘
//                            ↑
//                       backfill side
//
// Signature pattern:
//   - pattern.kind             = "retaining_wall_install"
//   - pattern.wall_type        = "gravity" | "reinforced" | "sheet"
//   - pattern.height_m         = exposed wall height (m)
//   - pattern.length_m         = wall run length (m)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   wall_type   ∈ {"gravity","reinforced","sheet"}    (DFM-WALL-TYPE error)
//   height_m    > 0                                    (DFM-INPUT error)
//   length_m    > 0                                    (DFM-INPUT error)
//   gravity   + height_m > 6   ⇒ unstable              (DFM-WALL-HEIGHT info)
//   sheet     + height_m > 8   ⇒ tie-back recommended  (DFM-WALL-HEIGHT info)
//
// Synthesis:
//   NO geometric change.  Clone + stamp.
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

namespace retaining_wall_install {

constexpr const char* kSkillId = "retaining_wall_install";

struct Input
{
    std::string  wall_type   = "reinforced";  // "gravity" | "reinforced" | "sheet"
    double       height_m    = 4.0;           // exposed wall height (m)
    double       length_m    = 20.0;          // wall run length (m)
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace retaining_wall_install
}  // namespace koocadcam::skill
