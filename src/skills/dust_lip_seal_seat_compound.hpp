#pragma once
// @lat: [[engine/skills#dust_lip_seal_seat_compound]]
//
// dust_lip_seal_seat_compound — COMPOUND MACRO: shaft-housing seal seat
// with an integrated dust lip (also called "secondary lip" or "wiper") plus
// a nose chamfer.  Standard for SKF / National / CR rotary shaft seals.
//
// 4 stacked sub-features cut into the housing bore (from inlet to back):
//
//       ▼ axis (shaft enters here)
//   ── nose chamfer 30° ── counterbore (seal OD) ── main bore (shaft OD) ── dust lip relief (smaller, deeper) ──
//
// Sub-features (each is a real Boolean cut/fuse):
//   1. MAIN BORE — straight cylinder, dia = bore_dia_mm, depth = bore_depth_mm.
//      This is the running shaft fit (typically H7).
//   2. COUNTERBORE (seal OD) — wider step at the inlet that holds the seal
//      outer can (dia = bore_dia_mm + 2·seal_can_OD_offset, depth = seal_thickness).
//      Standard SKF press-fit: counterbore_dia = shaft_dia + 2 × seal_OD_thickness.
//   3. DUST LIP RELIEF — narrower undercut DEEPER than the main bore, where
//      the secondary (dust) lip of the seal sits.  Dia = bore_dia − 2·lip_relief_depth.
//   4. NOSE CHAMFER — 30° lead-in cone at the inlet to guide the shaft (per
//      SKF Catalog § "Shaft requirements" — 15-30° chamfer × 0.25-0.5 mm).
//
// Signature pattern:
//   kind = "dust_lip_seal_seat_compound"
//   is_compound = true, subfeature_count = 4
//   spec_key = seal_series  (e.g. "CR-25-37-7" — SKF nominal)
//
// DFM gates:
//   - bore_dia_mm ≥ 3 mm (smallest SKF CR shaft seal).
//   - bore_depth_mm ≥ seal_thickness_mm (or DFM-SEAL-DEPTH error).
//   - lip_relief_depth_mm > 0 AND lip_relief_depth_mm < bore_dia_mm/4
//     (else seal lip would lose contact area).
//   - counterbore_offset_mm ≥ 0.1 mm (press-fit minimum interference).
//
// Context-aware recognize():
//   1. Iterate workpiece cylindrical faces — find chains of ≥ 2 coaxial
//      cylinders where outermost (entry) > main bore > undercut.
//   2. Verify presence of a chamfer (cone face) at the inlet.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace dust_lip_seal_seat_compound {

constexpr const char* kSkillId = "dust_lip_seal_seat_compound";

struct Input
{
    FaceDatum   entry_face;
    double      center_x_mm           = 0.0;
    double      center_y_mm           = 0.0;
    gp_Dir      axis_dir              { 0.0, 0.0, -1.0 };
    double      bore_dia_mm           = 0.0;
    double      bore_depth_mm         = 0.0;
    double      seal_thickness_mm     = 7.0;   // SKF CR standard 7 mm
    double      seal_can_OD_offset_mm = 0.5;   // counterbore step offset
    double      lip_relief_depth_mm   = 0.5;   // dust lip undercut depth
    double      nose_chamfer_mm       = 0.5;   // 30° × 0.5 mm chamfer
    std::string seal_series;                   // e.g. "CR-25-37-7"
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace dust_lip_seal_seat_compound
}  // namespace koocadcam::skill
