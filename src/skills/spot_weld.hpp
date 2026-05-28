#pragma once
// @lat: [[engine/skills#spot_weld]]
//
// spot_weld — a small fused bead joining two parts at a single point.
// Geometrically: a SHORT cylindrical disc-like BUMP fused onto a planar face.
//
//            ╭─────╮      ← weld bead (cylinder, very low height)
//            │     │   ↕ weld_height_mm  (typ. 0.3 – 1.5 mm)
//   ─────────┴─────┴────────────  ← entry_face (planar)
//
//            ↑           ↑
//          weld_dia_mm (typ. 4 – 8 mm)
//
// Signature pattern (what the skill leaves on the workpiece):
//   - 1 cylindrical face (the bead side wall)
//   - 1 circular edge where the bead meets the planar face
//   - 1 small planar disc on top of the bead
//
// DFM checks:
//   weld_dia    ∈ [3, 12]   (electrode tip practical range)
//   weld_height ≤ weld_dia × 0.4    (otherwise it is a multi-pass build-up, not a spot weld)
//
// Recognize:
//   Iterates cylindrical faces; for each, check that
//     height / diameter < 0.5  AND  diameter ≤ 12 mm
//   and that the cylinder protrudes outward (one circle on a larger planar
//   face, the other on a small disc face).  Geometry is additive — confusion
//   with drill_hole avoided because drill_hole bores INWARD (the disc is
//   inside) whereas spot_weld bumps OUTWARD.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace spot_weld {

constexpr const char* kSkillId = "spot_weld";

struct Input
{
    FaceDatum   entry_face;             // planar face onto which the bead is fused
    double      position_x_mm  = 0.0;   // bead centre on the entry face
    double      position_y_mm  = 0.0;
    double      weld_dia_mm    = 0.0;   // bead diameter (typ. 4 – 8 mm)
    double      weld_height_mm = 0.0;   // bead height   (typ. 0.3 – 1.5 mm)
    std::string material       = "carbon_steel";
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace spot_weld
}  // namespace koocadcam::skill
