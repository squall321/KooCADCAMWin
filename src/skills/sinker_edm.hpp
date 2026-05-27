#pragma once
// @lat: [[engine/skills#sinker_edm]]
//
// sinker_edm — sinker (cavity / die-sink) electrical-discharge machining.
//
// A pre-formed copper or graphite ELECTRODE is sunk into the workpiece
// while a controlled spark discharge erodes material in the shape of the
// electrode's negative.  Used for:
//   - injection-mold cavities and cores
//   - tight-tolerance pockets with sharp internal corners (EDM has no
//     tool-radius limitation — unlike end-mills which always leave a
//     corner-R = tool-radius)
//   - hard materials (tool steels, carbide) where milling is impractical
//
// This skill is a DISPATCHER over the underlying pocket geometry skills:
// EDM only differs from a milled pocket by the cutting physics — the
// resulting cavity is geometrically identical.  We delegate synthesis to
// mill_rect_pocket / mill_circular_pocket and stamp the signature with
// EDM tooling metadata.
//
//                       entry_face (planar)
//                            │
//                ────────────┼──────────────
//                            │     ▼ axis
//                            │
//                  ╔═════════╧═════════╗   ↑ depth_mm
//                  ║  (electrode-      ║   │
//                  ║   shaped cavity)  ║   ↓
//                  ╚═══════════════════╝
//
// `electrode_shape` selects the dispatch target:
//   - "rect_pocket"        → mill_rect_pocket with sharp corners (corner_r=0)
//                            because EDM doesn't leave tool-radius fillets.
//                            `dims` must contain: length_mm, width_mm.
//   - "cylindrical_cavity" → mill_circular_pocket with sharp bottom corner.
//                            `dims` must contain: dia_mm.
//
// Signature pattern:
//   Inherited from the underlying skill, but with `pattern.kind = "sinker_edm"`
//   and `tooling.tool_type = "edm_electrode"`.
//
// DFM checks:
//   DFM-EDM-DEPTH : depth_mm > 50 mm    (warning — exceeds typical sinker
//                                        stroke; flush/secondary electrode
//                                        needed)
//   shape-specific dims validated per-shape (≥ 0.5 mm)
//
// Recognize:
//   ZERO — recognition is delegated to the underlying recognizer.  Without
//   metadata, a sinker_edm pocket and a milled pocket are geometrically
//   indistinguishable.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace sinker_edm {

constexpr const char* kSkillId = "sinker_edm";

struct Input
{
    FaceDatum       entry_face;
    double          position_x_mm  = 0.0;
    double          position_y_mm  = 0.0;
    std::string     electrode_shape;     // "rect_pocket" | "cylindrical_cavity"
    nlohmann::json  dims = nlohmann::json::object();  // shape-specific dims
    double          depth_mm       = 0.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace sinker_edm
}  // namespace koocadcam::skill
