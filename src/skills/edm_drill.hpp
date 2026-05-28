#pragma once
// @lat: [[engine/skills#edm_drill]]
//
// edm_drill — Small-hole EDM drilling (a.k.a. "EDM hole-popper" or
// "fast-hole EDM").  A rotating, hollow brass/copper electrode fed with
// dielectric flushing under high pressure erodes a straight micro-hole.
// Used for:
//   - cooling holes in turbine blades (0.3–1.0 mm dia)
//   - wire-EDM starter holes in hardened tool steel
//   - injection-mold venting (typ. 0.1–0.3 mm dia)
//   - any application requiring 30:1 or higher depth/dia ratios on
//     hard materials where twist drills cannot survive.
//
// Geometry: identical to drill_hole (straight cylindrical bore) but with a
// MUCH smaller diameter envelope (0.05–1.0 mm) and a MUCH larger depth/dia
// ceiling (50:1 vs 8:1 for twist drills).
//
//                       entry_face (planar)
//                            │
//                  ──────────┼──────────
//                            │   ▼ axis
//                            │  ╱ ╲
//                            │ ╱   ╲   ← 0.05 ≤ dia ≤ 1.0 mm
//                            │╲   ╱│
//                            │ ╲ ╱ │   ↑ depth ≤ 50 × dia
//                            │     │   │
//                            └─────┘   ↓
//
// Signature pattern:
//   - 1 cylindrical face (the bore)
//   - 2 circular edges (entry + exit OR entry + blind-bottom)
//   - 1 planar face (blind bottom) — sharp corner (no EDM tool-radius leg)
//
// DFM checks:
//   DFM-INPUT      diameter ≤ 0, depth ≤ 0
//   DFM-EDMD-DIA-LO  diameter < 0.05 mm  (electrode supply impractical) → error
//   DFM-EDMD-DIA-HI  diameter > 1.0 mm  (use sinker EDM or drill_hole)  → error
//   DFM-EDMD-RATIO   depth/dia > 50    (50:1 absolute envelope)         → error
//   DFM-EDMD-RATIO-WARN  depth/dia > 30 (warning — flush limit)
//
// Recognize:
//   Same topological pattern as drill_hole.  We use the SMALL-DIAMETER
//   + HIGH-RATIO envelope as the EDM-drill discriminator: hole satisfies
//   (dia ≤ 1.0 mm AND depth/dia > 8) → candidate.  Confidence 0.85.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace edm_drill {

constexpr const char* kSkillId    = "edm_drill";
constexpr double      kMinDia_mm  = 0.05;
constexpr double      kMaxDia_mm  = 1.0;
constexpr double      kMaxRatio   = 50.0;

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir       { 0.0, 0.0, -1.0 };
    double      diameter_mm    = 0.0;
    double      depth_mm       = 0.0;
    bool        through_hole   = false;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace edm_drill
}  // namespace koocadcam::skill
