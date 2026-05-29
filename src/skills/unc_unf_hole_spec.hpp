#pragma once
// @lat: [[engine/skills#unc_unf_hole_spec]]
//
// unc_unf_hole_spec — imperial UNC/UNF clearance hole per ASME B18.2.8.
//
//   • fastener_size : "#2-56" .. "1/2-13" (UNC/UNF nominal sizes)
//   • fit_class     : "normal" | "close" | "loose"  (ASME B18.2.8 Table 1)
//   • Output: clearance through-hole + entry chamfer.  All dims internally
//     in mm (the table is converted from inches).
//
// Spec lookup (ASME B18.2.8, normal/close/loose, mm):
//   #2-56   : 0.094"/0.106"/0.116"   → 2.39/2.69/2.95 mm
//   #4-40   : 0.116"/0.140"/0.149"   → 2.95/3.56/3.78 mm
//   #6-32   : 0.144"/0.166"/0.177"   → 3.66/4.22/4.50 mm
//   #8-32   : 0.170"/0.196"/0.206"   → 4.32/4.98/5.23 mm
//   #10-24  : 0.196"/0.221"/0.228"   → 4.98/5.61/5.79 mm
//   1/4-20  : 0.281"/0.297"/0.323"   → 7.14/7.54/8.20 mm
//   3/8-16  : 0.406"/0.422"/0.453"   → 10.31/10.72/11.51 mm
//   1/2-13  : 0.531"/0.562"/0.609"   → 13.49/14.27/15.47 mm
//
// Signature pattern:
//   { kind, is_compound=true, subfeature_count=2,
//     fastener_size, fit_class, clearance_dia_mm,
//     chamfer_dia_mm, chamfer_depth_mm, axis_dir }
//
// DFM gates:
//   DFM-IMP-UNKNOWN : fastener_size not in 8-entry table → error
//   DFM-IMP-FIT     : fit_class not in {normal, close, loose} → error
//   DFM-IMP-DEPTH   : stock_thickness < 1.0 × clearance_dia → warning

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace unc_unf_hole_spec {

constexpr const char* kSkillId = "unc_unf_hole_spec";

struct Input
{
    FaceDatum    entry_face;
    double       position_x_mm    = 0.0;
    double       position_y_mm    = 0.0;
    gp_Dir       axis_dir         { 0.0, 0.0, -1.0 };
    std::string  fastener_size    = "1/4-20";
    std::string  fit_class        = "normal";   // normal | close | loose
    double       chamfer_size_mm  = 0.5;
    double       chamfer_angle_deg = 45.0;      // imperial chamfer convention
};

// Exposed for tests.
double clearanceDiameterFor(const std::string& fastener_size,
                            const std::string& fit_class);

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace unc_unf_hole_spec
}  // namespace koocadcam::skill
