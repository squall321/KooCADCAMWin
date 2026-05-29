#pragma once
// @lat: [[engine/skills#glue_laminate]]
//
// glue_laminate — assemble a glulam (glued-laminated timber) beam by
// stacking and gluing N lamina plies under pressure until cure.  Standard
// adhesives are melamine-urea (MUF), phenol-resorcinol (PRF), and one-part
// polyurethane (PUR).  Total beam height = ply_count × ply_thickness_mm.
// KooCADCAM phase-1 records the lay-up so DFM can flag exotic ratios; the
// beam B-rep is the formwork model and is unchanged.
//
//      ━━━━━━━━━━━━━━━━━━━━━━━━━━━━   ← ply N    (top lamina)
//      ━━━━━━━━━━━━━━━━━━━━━━━━━━━━   ← glue line (adhesive)
//      ━━━━━━━━━━━━━━━━━━━━━━━━━━━━   ← ply N-1
//      ...                                ply_thickness_mm typical 25–50 mm
//      ━━━━━━━━━━━━━━━━━━━━━━━━━━━━   ← ply 1    (bottom lamina)
//
// Signature pattern:
//   - pattern.kind                = "glue_laminate"
//   - pattern.ply_count           = number of lamina plies
//   - pattern.ply_thickness_mm    = single lamina thickness
//   - pattern.adhesive_type       = "MUF" | "PRF" | "PUR"
//   - pattern.total_thickness_mm  = ply_count × ply_thickness_mm
//   - pattern.geometry_changed    = false
//
// DFM checks:
//   ply_count        ≥ 2                               (DFM-INPUT error)
//   ply_thickness_mm ∈ [10, 50]                        (DFM-GLULAM-PLY error)
//   adhesive_type    ∈ {"MUF","PRF","PUR"}             (DFM-GLULAM-ADHESIVE error)
//   total_thickness  > 1800 mm                         (DFM-GLULAM-DEPTH info — handling/transport)
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

namespace glue_laminate {

constexpr const char* kSkillId = "glue_laminate";

struct Input
{
    int          ply_count          = 6;        // number of lamina plies
    double       ply_thickness_mm   = 38.0;     // single lamina thickness
    std::string  adhesive_type      = "MUF";    // "MUF" | "PRF" | "PUR"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace glue_laminate
}  // namespace koocadcam::skill
