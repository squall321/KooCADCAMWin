#pragma once
// @lat: [[engine/skills#iso_h7_bore_spec]]
//
// iso_h7_bore_spec — ISO 286-1 H7 fit-class precision bore.
//
// COMPOUND macro:
//   1. drill at nominal diameter (rough)
//   2. ream to deviated (final) diameter from ISO 286-1 H7 table
//   3. lead-in chamfer at entry (conical frustum)
//
// ISO 286-1 H7 table (lower deviation 0, upper deviation = +ITgrade µm):
//   Ø 1–3   → +10 µm
//   Ø 3–6   → +12 µm
//   Ø 6–10  → +15 µm
//   Ø 10–18 → +18 µm
//   Ø 18–30 → +21 µm
//   Ø 30–50 → +25 µm
//   Ø 50–80 → +30 µm
//   Ø 80–120→ +35 µm
//
// Caller supplies nominal_dia_mm; spec table delivers the deviation, and
// the bore is reamed at (nominal + upper_dev/2) to land mid-tolerance.
//
// Signature pattern:
//   - 1 cylindrical face at FINAL (deviated) diameter
//   - 1 chamfered conical face at the entry
//   - 1 planar bottom face (blind)
//   - signature.pattern.kind == "iso_h7_bore_spec"
//   - signature.pattern.spec_key == "H7"
//
// DFM:
//   nominal_dia_mm must fall in [1, 100] mm (table range).
//   depth/dia ≤ 5 (reamer rigidity).
//
// Recognize:
//   Cylindrical face with diameter matching one of the table mid-tolerances
//   within 0.05 mm AND adjacent conical-frustum face at entry.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace iso_h7_bore_spec {

constexpr const char* kSkillId = "iso_h7_bore_spec";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm   = 0.0;
    double      position_y_mm   = 0.0;
    gp_Dir      axis_dir        { 0.0, 0.0, -1.0 };
    double      nominal_dia_mm  = 0.0;    // table-keyed
    double      depth_mm        = 0.0;
    double      chamfer_mm      = 0.3;    // standard lead-in
    std::string spec_key        = "H7";   // fixed for this skill
};

// Returns upper-deviation in µm for a given nominal_dia_mm.  Returns -1.0
// when the diameter is out of table range (1..100 mm).
double upperDeviationMicrons(double nominal_dia_mm);

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace iso_h7_bore_spec
}  // namespace koocadcam::skill
