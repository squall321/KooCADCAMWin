#pragma once
// @lat: [[engine/skills#gasket_face_with_drain_groove]]
//
// gasket_face_with_drain_groove — COMPOUND MACRO: prepared gasket face
// with a concentric drain (weep) groove for leakage detection.
//
// Geometry:
//
//      ─────────────────────────────────────────  ← gasket face plane
//      │░░░░░░░░░░░ pocket relief ░░░░░░░░░░░░░│  ↑ relief_depth (50 μm typ.)
//      │░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░│  ↓
//      │     ┌────────────────────────────┐    │
//      │     │ drain groove (concentric)  │    │  drain
//      │     └────────────────────────────┘    │
//      │░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░│
//      ─────────────────────────────────────────
//
// Compound chain (3 sub-features cut/fused):
//   1. FACE RELIEF — shallow circular pocket (large dia, 50 μm deep) that
//      establishes the gasket-mating plane.  Roughness-controlled face mill.
//   2. ID CHAMFER — 0.5 mm × 45° chamfer at inner edge (sealing-bead side).
//   3. DRAIN GROOVE — concentric thin annular groove (width 1.0 mm, depth
//      0.5 mm) at radius = drain_radius_mm, for leak-path detection per
//      ASME B16.5 §6.4.4 "Vented gasket recommendations".
//
// Signature pattern:
//   kind = "gasket_face_with_drain_groove"
//   is_compound = true, subfeature_count = 3
//   spec_key = gasket class (e.g. "ANSI-150" / "ANSI-300" / "DIN-PN16")
//   roughness_class (Ra), drain_radius_mm
//
// DFM gates:
//   - face_dia_mm ≥ 10 mm (smallest practical gasket prep).
//   - drain_radius_mm strictly between (face_dia/4) and (face_dia/2 − 1.0)
//     — must be inside the face and not at the OD.
//   - roughness_class ∈ {"smooth", "stock", "serrated"} per ASME B16.5
//     Table 6 — else DFM-GASKET-ROUGHNESS warning.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace gasket_face_with_drain_groove {

constexpr const char* kSkillId = "gasket_face_with_drain_groove";

struct Input
{
    FaceDatum   face_id;
    double      center_x_mm        = 0.0;
    double      center_y_mm        = 0.0;
    gp_Dir      axis_dir           { 0.0, 0.0, -1.0 };
    double      face_dia_mm        = 0.0;     // overall prepared face diameter
    double      relief_depth_mm    = 0.05;    // 50 µm relief
    double      drain_radius_mm    = 0.0;     // concentric drain radius
    double      drain_width_mm     = 1.0;     // groove width
    double      drain_depth_mm     = 0.5;     // groove depth
    double      id_chamfer_mm      = 0.5;     // ID chamfer
    std::string gasket_class;                 // e.g. "ANSI-150"
    std::string roughness_class    = "stock"; // smooth | stock | serrated
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace gasket_face_with_drain_groove
}  // namespace koocadcam::skill
