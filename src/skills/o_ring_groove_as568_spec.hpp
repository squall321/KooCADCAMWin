#pragma once
// @lat: [[engine/skills#o_ring_groove_as568_spec]]
//
// o_ring_groove_as568_spec — COMPOUND MACRO: AS568 spec-driven O-ring
// groove on a flat face.  Difference vs `o_ring_groove_face`:
//   • Embeds ≥10 AS568 dash sizes (-006/-011/-016/-111/-116/-212/-224/
//     -325/-425/-908) — covers the entire AS568 range from micro
//     (-006, CS=1.78) to specialty oversize (-908, CS=6.99).
//   • Uses Parker O-ring Handbook ORD 5700 §4-2 RADIAL/static gland design
//     constants: depth G = 0.75·CS, width W = 1.40·CS, lead-in 15° (per
//     Parker §4-2 footnote "5° to 15° lead-in chamfer recommended for
//     groove ID/OD").
//
// Sub-feature chain (3 cuts, fused → cut once):
//   1. MAIN ANNULAR GROOVE — annular ring tool, axis along axis_dir,
//      ID = mean − W/2, OD = mean + W/2, height = G.
//   2. ID 15° LEAD-IN CONE — cone frustum widens the groove ID toward the
//      face surface so the O-ring "drops in".
//   3. OD 15° LEAD-IN CONE — cone frustum widens the groove OD toward the
//      face surface (symmetric to #2).
//
// Signature pattern:
//   kind = "o_ring_groove_as568_spec"
//   is_compound = true
//   subfeature_count = 3
//   spec_key = AS568 dash size (e.g. "-212")
//
// DFM gates:
//   - dash_size must be in the lookup table (else DFM-AS568-SPEC error).
//   - mean_dia − W/2 must be > 0 (groove ID positive).
//   - mean_dia + W/2 must fit on the face (caller-supplied face area
//     bounded via wp.bbox() heuristic).
//   - face_id must resolve to a planar face on the workpiece.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace o_ring_groove_as568_spec {

constexpr const char* kSkillId = "o_ring_groove_as568_spec";

struct Input
{
    FaceDatum   face_id;
    double      center_x_mm  = 0.0;
    double      center_y_mm  = 0.0;
    gp_Dir      axis_dir     { 0.0, 0.0, -1.0 };
    double      mean_dia_mm  = 0.0;
    std::string dash_size;                 // e.g. "-006", "-212", "-908"
};

// Lookup helpers — exposed so tests can verify the embedded AS568 table.
double crossSectionFor(const std::string& dash_size);
double recommendedDepthFor (double cs_mm);   // 0.75 * cs  (Parker §4-2)
double recommendedWidthFor (double cs_mm);   // 1.40 * cs  (Parker §4-2)

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace o_ring_groove_as568_spec
}  // namespace koocadcam::skill
