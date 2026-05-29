#pragma once
// @lat: [[engine/skills#x_ring_groove_spec]]
//
// x_ring_groove_spec — COMPOUND MACRO: AS568 X-ring (quad-ring / 4-lobe)
// groove on a flat face.
//
// X-rings ("quad rings") have a 4-lobed cross-section — twice the sealing
// lines of an O-ring at the same CS — which lowers compression-set and
// hysteresis but requires a slightly deeper groove than the equivalent
// O-ring to clear the four lobes:
//
//   Parker "Quad-Ring Brand Seals" Catalog (PTD 5705 §3.4):
//     groove depth G = 0.78 × CS   (vs 0.75 × CS for O-ring)
//     groove width W = 1.40 × CS   (same as O-ring AS568 radial)
//     lead-in 15° per Parker §3.4.3 (same as O-ring)
//
// Sub-feature chain (3 cuts):
//   1. MAIN ANNULAR GROOVE — annular ring tool, axis along axis_dir.
//   2. ID 15° lead-in cone — for assembly clearance of X-ring lobes.
//   3. OD 15° lead-in cone — symmetric.
//
// Signature pattern:
//   kind = "x_ring_groove_spec"
//   is_compound = true, subfeature_count = 3, spec_key = AS568 dash size
//   seal_profile = "quad_ring" (distinguishes from o-ring metadata)
//
// DFM gates:
//   - dash_size must be in the X-ring spec table.
//   - mean_dia − W/2 must be > 0.
//   - depth must be ≥ 0.2 mm (form-tool minimum).
//   - Quad ring is incompatible with HIGH dynamic temperature (>120 °C)
//     unless material override is supplied — warning only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace x_ring_groove_spec {

constexpr const char* kSkillId = "x_ring_groove_spec";

struct Input
{
    FaceDatum   face_id;
    double      center_x_mm  = 0.0;
    double      center_y_mm  = 0.0;
    gp_Dir      axis_dir     { 0.0, 0.0, -1.0 };
    double      mean_dia_mm  = 0.0;
    std::string dash_size;             // AS568 dash, e.g. "-212"
};

double crossSectionFor(const std::string& dash_size);
double recommendedDepthFor(double cs_mm);   // 0.78 * cs (quad)
double recommendedWidthFor(double cs_mm);   // 1.40 * cs (same as O-ring)

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace x_ring_groove_spec
}  // namespace koocadcam::skill
