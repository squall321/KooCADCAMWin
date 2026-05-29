#pragma once
// @lat: [[engine/skills#spline_shaft_compound]]
//
// spline_shaft_compound — REAL compound feature on a cylindrical shaft:
//
//   sub-cuts (N + 1):
//     1. shaft-end chamfer cone (entry chamfer for spline engagement)
//     2..N+1. N parallel-sided axial spline grooves equispaced around the
//             shaft, milled to spline_depth_mm.
//
// DIN 5480 standard for involute splines:
//   - Module m: 0.5, 0.6, 0.75, 1, 1.25, 1.5, 2, 2.5, 3, 4, 5, 6, 8, 10 mm
//   - Pressure angle: 30° (involute)
//   - Tooth count: 6..82
//   - Reference dia (pitch dia) = m * (z + x) where x = profile shift
//   - Major dia      = ref_dia + 0.9 * m
//   - Minor dia      = ref_dia - 1.1 * m
//
// In slice-1 we approximate the splines as parallel-sided rectangular
// grooves (NOT involute flanks) — this matches DIN 5463 parallel-sided
// splines, which is acceptable for the majority of light-duty applications.
//
// DFM gates:
//   DFM-SPLINE-MOD      module outside DIN 5480 set
//   DFM-SPLINE-N        N < 6 or N > 82
//   DFM-SPLINE-OD       reference_dia + module > shaft_dia
//   DFM-SPLINE-LEN      spline_length_mm > shaft_length_mm

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace spline_shaft_compound {

constexpr const char* kSkillId = "spline_shaft_compound";

struct Input
{
    double  shaft_dia_mm        = 0.0;     // raw shaft OD
    double  shaft_length_mm     = 0.0;     // total shaft length (cylinder thickness)
    double  module_mm           = 0.0;     // DIN 5480 module
    int     num_splines         = 0;       // tooth count z
    double  spline_length_mm    = 0.0;     // axial extent of spline section
    double  end_chamfer_mm      = 1.0;     // entry chamfer at shaft end
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace spline_shaft_compound
}  // namespace koocadcam::skill
