#pragma once
// @lat: [[engine/skills#rivet_dimple_compound]]
//
// rivet_dimple_compound — COMPOUND aerospace dimpled-hole for a flush solid
// rivet (MS33558 / NASM33558).  Used on thin sheet skins (≤ 1.2 mm) where the
// sheet is locally dimpled (conical depression) so the countersunk rivet head
// sits flush with the surrounding skin.
//
//   sub-feature 1: through pilot hole at the rivet body diameter
//   sub-feature 2: conical dimple (countersink) at the entry face
//                  82° / 100° / 120° included angle
//                  depth = sheet_thickness × 0.7  (per MS33558 envelope)
//
// Sequential pr::cut(wp, cutter) loop is used (NOT cutMany with overlapping
// cutters) — slice-14 root-cause guard: OCCT 8.0 silently no-ops or
// wipes-out compound cuts when the cutter sub-shapes overlap concentrically.
//
// Signature:
//   pattern.kind                  = "rivet_dimple_compound"
//   pattern.is_compound           = true
//   pattern.aerospace_feature_type = "flush_rivet_dimple"
//   pattern.fastener_dia          = rivet_dia_mm
//   pattern.derived_volume_removed_mm3
//
// DFM gates:
//   DFM-INPUT             : all dimensions > 0
//   DFM-RIVET-DIA         : rivet_dia ∈ {3.2, 4.0, 4.8} mm (MS20426)
//   DFM-DIMPLE-ANGLE      : dimple_angle ∈ {82, 100, 120}
//   DFM-SHEET-THICKNESS   : sheet_thickness ≥ rivet_dia × 0.3
//   DFM-DIMPLE-DEPTH      : dimple_depth ≤ sheet_thickness × 0.8

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace rivet_dimple_compound {

constexpr const char* kSkillId = "rivet_dimple_compound";

struct Input
{
    int    face_id            = -1;
    double center_x_mm        = 0.0;
    double center_y_mm        = 0.0;
    double rivet_dia_mm       = 3.2;     // 3.2 / 4.0 / 4.8 (MS20426)
    double sheet_thickness_mm = 0.0;
    double dimple_angle_deg   = 100.0;   // 82 / 100 / 120 (MS33558 default 100)
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace rivet_dimple_compound
}  // namespace koocadcam::skill
