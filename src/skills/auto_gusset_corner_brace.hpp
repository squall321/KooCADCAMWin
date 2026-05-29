#pragma once
// @lat: [[engine/skills#auto_gusset_corner_brace]]
//
// auto_gusset_corner_brace — CONTEXT-AWARE compound macro that fuses a
// triangular gusset (corner brace) at an internal corner edge of the
// workpiece.  Detects the two flanking faces sharing the corner edge and
// builds a 3-leg gusset spanning a chosen leg_length along each face.
//
// Sub-features (subfeature_count = 3):
//   1. Triangular gusset prism (fuse)                        [fuse]
//   2. Small fillet bead along the corner edge               [fuse]
//   3. Tip relief cut at the apex (to avoid stress riser)    [cut]
//
// Lookup table — Stress concentration factor (Kt) per ASME B31.3 corner-
// gusset chart:
//   "light"  → tip_radius = 0.5 mm, Kt ≈ 1.4
//   "medium" → tip_radius = 1.0 mm, Kt ≈ 1.2
//   "heavy"  → tip_radius = 2.0 mm, Kt ≈ 1.1
//
// DFM:
//   DFM-INPUT        : corner_edge_id out of range / leg_length ≤ 0.
//   DFM-GUSSET-SPEC  : unknown service class.
//   DFM-GUSSET-LEG   : leg_length > 0.5 × min(face dimension).
//   DFM-GUSSET-ANGLE : flanking face angle > 100° (gusset insufficient).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace auto_gusset_corner_brace {

constexpr const char* kSkillId = "auto_gusset_corner_brace";

struct Input
{
    int         corner_edge_id  = -1;
    double      leg_length_mm   = 8.0;
    double      gusset_thick_mm = 3.0;
    std::string service_class   = "medium";   // light/medium/heavy
};

SkillOutput   apply   (const Workpiece& wp, const Input& in);
DFMReport     validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace auto_gusset_corner_brace
}  // namespace koocadcam::skill
