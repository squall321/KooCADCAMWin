#pragma once
// @lat: [[engine/skills#o_ring_groove_face]]
//
// o_ring_groove_face — COMPOUND MACRO: face-seal O-ring groove on a flat
// face (axial compression seal) with ID + OD 5° lead-in chamfers.
//
//   ────┬──────────────────────────────────────┬────  ← face (z = face_top)
//       │ ╲ID 5°lead       OD 5°lead╱          │
//       │  ╲      ╔════════╗       ╱           │
//       │   ╲     ║         ║      ╱           │      depth G
//       │    ╲    ║         ║     ╱            │
//       │     ╲___║         ║____╱             │
//       │         ╚═════════╝                  │
//       │         ◄── W ──►                    │
//       │                                      │
//       │ ◄── mean_dia ──►                     │
//
// Sub-feature chain (geometric):
//   1. MAIN ANNULAR GROOVE — annular ring of inner_R = mean - W/2, outer_R =
//      mean + W/2, height = G, axis along normal to face.  Cut via
//      pr::annularRing + pr::cut.
//   2. ID CHAMFER — 5° lead-in cone frustum (inner edge) so the O-ring
//      slides into the groove cleanly during assembly.  Cone frustum tool
//      goes from groove ID at face plane to (ID - C_lead) at face plane -
//      C_lead·tan(85°).
//   3. OD CHAMFER — 5° lead-in cone frustum (outer edge), symmetric to #2.
//
// Parker O-ring Handbook (Table 4-3, face-seal static):
//   G (depth)  ≈ 0.74 · CS    (same squeeze as radial static)
//   W (width)  ≈ 1.50 · CS    (wider than radial — accommodates groove fill)
//   Lead-in C  ≈ 0.5 mm × tan(5°) chamfer at ID/OD
//
// Signature pattern: kind = "o_ring_groove_face", is_compound = true,
// subfeature_count = 3 (groove + ID chamfer + OD chamfer).
//
// DFM gates (AS568):
//   - o_ring_size must be in the lookup table.
//   - mean_dia - W/2 must be > 0 (ID must be positive).
//   - mean_dia + W/2 + 1.0 mm must be ≤ stock outer (1 mm outer wall).
//   - face_id must resolve to a planar face.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace o_ring_groove_face {

constexpr const char* kSkillId = "o_ring_groove_face";

struct Input
{
    FaceDatum   face_id;
    double      center_x_mm  = 0.0;
    double      center_y_mm  = 0.0;
    gp_Dir      axis_dir     { 0.0, 0.0, -1.0 };  // into-the-face direction
    double      mean_dia_mm  = 0.0;               // diameter at groove centerline
    std::string o_ring_size;                      // AS568 dash number
};

double crossSectionFor(const std::string& as568_dash);
double recommendedFaceDepthFor(double cs_mm);
double recommendedFaceWidthFor(double cs_mm);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace o_ring_groove_face
}  // namespace koocadcam::skill
