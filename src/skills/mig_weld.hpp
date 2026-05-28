#pragma once
// @lat: [[engine/skills#mig_weld]]
//
// mig_weld — Metal Inert Gas (a.k.a. GMAW).  A consumable wire is
// continuously fed into an inert-gas arc, depositing material much faster
// than TIG.  Beads are taller and wider than TIG / seam, suitable for
// structural fillet and butt joints in steel up to ~25 mm thickness.
//
//   Side view (relative to TIG):
//        TIG:    __                MIG:    ______
//              ╱    ╲                   ╱        ╲      ← taller, wider
//   ────────╱________╲────       ────╱____________╲────
//             ~3 mm                       ~6 mm
//
// Footprint: stadium bead, additively fused onto a planar face.  Same
// geometric shape family as TIG / seam — only the dimensions and the
// tooling metadata change.
//
// Signature pattern:
//   - 2 half-cylinder bead-end faces
//   - 2 long planar walls parallel to start→end direction
//   - 1 top planar face (stadium-shaped)
//   - pattern.kind = "mig_weld"
//   - wire_dia_mm + wire_feed_mm_per_s recorded
//
// DFM checks:
//   bead_width  ∈ [2.0, 12.0]
//   wire_dia    ∈ [0.6, 1.6]    (typical solid GMAW wire range)
//   bead_height / bead_width ≤ 0.5
//   |end - start| > bead_width
//
// Recognize:
//   Identical geometry to TIG / seam — metadata-driven only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace mig_weld {

constexpr const char* kSkillId = "mig_weld";

struct Input
{
    FaceDatum   entry_face;
    double      start_x_mm           = 0.0;
    double      start_y_mm           = 0.0;
    double      end_x_mm             = 0.0;
    double      end_y_mm             = 0.0;
    double      bead_width_mm        = 0.0;   // typ. 4 – 10 mm
    double      bead_height_mm       = 0.0;   // typ. 1 – 4 mm
    std::string material             = "steel";
    std::string gas                  = "argon_co2";   // "argon_co2" | "argon"
    double      wire_dia_mm          = 1.0;
    double      wire_feed_mm_per_s   = 80.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace mig_weld
}  // namespace koocadcam::skill
