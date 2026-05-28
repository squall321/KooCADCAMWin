#pragma once
// @lat: [[engine/skills#tig_weld]]
//
// tig_weld — Tungsten Inert Gas welding.  A FINE, controllable arc bead laid
// down along a continuous path using a non-consumable tungsten electrode and
// inert shielding gas (argon or argon-helium).  Compared with MIG / seam,
// TIG produces the narrowest, cleanest beads but at the lowest deposition
// rate; it is the process of choice for thin sections, aluminum and
// stainless joints where appearance and metallurgical purity matter.
//
//   Top view (entry_face), bead path is the dashed line:
//                                ╭───────────────────╮
//        start (x,y) ────────────│═══════════════════│──────── end (x,y)
//                                ╰───────────────────╯
//                                ↑                  ↑
//                                ╰──── bead_width_mm (2 – 6 mm typ.)
//
//   Side view (perpendicular to start→end direction):
//                                ___________________
//                              ╱                     ╲    ↕ bead_height_mm
//   ─────────────────────────╱_______________________╲────  ← entry_face
//                              (0.5 – 2 mm typ.)
//
// Footprint identical to seam_weld (cylinder + box + cylinder fused
// additively onto a planar face).  Tooling metadata is the chief
// differentiator — tool_type = "tig_torch", material+gas pair recorded.
//
// Signature pattern:
//   - 2 half-cylinder bead-end faces
//   - 2 long planar walls parallel to start→end direction
//   - 1 top planar face (stadium-shaped)
//   - pattern.kind = "tig_weld"
//   - additive bead PROTRUDING above the parent face
//
// DFM checks:
//   bead_width  ∈ [1.0, 8.0]
//   bead_height / bead_width ≤ 0.5    (single-pass keyhole, not buildup)
//   |end - start| > bead_width        (otherwise prefer a tack weld)
//   gas vs material compatibility (info-level finding, never a hard error):
//     argon          : OK on steel / stainless / aluminum
//     argon_helium   : best for aluminum thicker than 6 mm
//
// Recognize:
//   Same geometric topology as seam_weld so we cannot tell them apart from
//   shape alone.  Recognition is METADATA-DRIVEN: we look at the workpiece's
//   feature history; only signatures whose `skill_id == "tig_weld"` are
//   returned.  This avoids double-reporting the same bead as both seam_weld
//   AND tig_weld and reflects how a real CAPP pipeline distinguishes the two
//   (via process plan annotation, not pure geometry).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace tig_weld {

constexpr const char* kSkillId = "tig_weld";

struct Input
{
    FaceDatum   entry_face;
    double      start_x_mm     = 0.0;
    double      start_y_mm     = 0.0;
    double      end_x_mm       = 0.0;
    double      end_y_mm       = 0.0;
    double      bead_width_mm  = 0.0;   // typ. 2 – 6 mm
    double      bead_height_mm = 0.0;   // typ. 0.5 – 2 mm
    std::string material       = "steel";       // "steel" | "aluminum" | "stainless"
    std::string gas            = "argon";       // "argon" | "argon_helium"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace tig_weld
}  // namespace koocadcam::skill
