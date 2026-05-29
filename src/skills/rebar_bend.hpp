#pragma once
// @lat: [[engine/skills#rebar_bend]]
//
// rebar_bend — cold-bend reinforcement bar to a specified angle around a
// mandrel pin.  A site/shop step: bar diameter, bend angle, and the inside
// bend radius dictate the rebar bender's pin selection and the local strain
// imposed on the steel.  The bar's outer geometry is treated as out-of-
// model for KooCADCAM phase-1 — this skill records the bend metadata so
// downstream BIM / structural QA can verify ACI 318 §25.3 bend diameters.
//
//      straight bar                bent bar (angle θ, inside radius r)
//      ─────────────►              ─────────╮
//                                            ╲
//                                             ╲   bend_radius_mm = r
//                                              ╲  bend_angle_deg = θ
//                                               ╲
//
// Signature pattern:
//   - pattern.kind             = "rebar_bend"
//   - pattern.dia_mm           = bar Ø (e.g. 10, 13, 16, 19, 22, 25 mm)
//   - pattern.bend_angle_deg   = bend angle [0, 180]
//   - pattern.bend_radius_mm   = inside bend radius
//   - pattern.bend_ratio       = bend_radius_mm / dia_mm
//   - pattern.geometry_changed = false
//
// DFM checks:
//   dia_mm         ∈ [6, 57]                  (DFM-REBAR-DIA error)
//   bend_angle_deg ∈ (0, 180]                 (DFM-INPUT error)
//   bend_radius_mm ≥ 3 × dia_mm               (DFM-REBAR-RADIUS error — ACI minimum for #3-#8)
//   bend_radius_mm < 12 × dia_mm              (DFM-REBAR-RADIUS info if larger — gentle bend, wastes capacity)
//
// Synthesis:
//   NO geometric change in KooCADCAM model.  Clone + stamp.
//
// Recognize:
//   Metadata replay only — bar shape lives outside the workpiece B-rep.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace rebar_bend {

constexpr const char* kSkillId = "rebar_bend";

struct Input
{
    double  dia_mm           = 16.0;    // bar Ø (typical #5 = 16 mm)
    double  bend_angle_deg   = 90.0;    // bend angle [0, 180]
    double  bend_radius_mm   = 64.0;    // inside bend radius (≥ 3 × dia)
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace rebar_bend
}  // namespace koocadcam::skill
