#pragma once
// @lat: [[engine/skills#bolt_hole_metric_spec]]
//
// bolt_hole_metric_spec — spec-driven metric clearance hole for through bolts.
//
//   • thread_size  : "M3" .. "M30" (ISO standard sizes)
//   • fit_class    : "close" | "medium" | "free"   (ISO 273 fit-class tables)
//   • Produces a clearance through-hole + an entry chamfer (compound, 2 cuts).
//
// Spec lookup table (embedded as static const in .cpp) per ISO 273:
//   close  : M3=3.2, M4=4.3, M5=5.3, M6=6.4, M8=8.4, M10=10.5,
//            M12=13, M16=17, M20=21, M24=25, M30=31
//   medium : M3=3.4, M4=4.5, M5=5.5, M6=6.6, M8=9, M10=11, M12=14,
//            M16=18, M20=22, M24=26, M30=33
//   free   : M3=3.6, M4=4.8, M5=5.8, M6=7, M8=10, M10=12, M12=15,
//            M16=19, M20=24, M24=28, M30=35
//
// Entry chamfer: 30° break, depth = 0.5 mm (default), to deburr the hole
//                  rim per ISO 13715 and to ease bolt insertion.
//
// Signature pattern:
//   { kind, is_compound=true, subfeature_count=2,
//     thread_size, fit_class,
//     clearance_dia_mm, chamfer_dia_mm, chamfer_depth_mm,
//     cylindrical_face_count=1, conical_face_count=1 }
//
// DFM gates (citing ISO 273 / IT13):
//   DFM-METRIC-UNKNOWN  : thread_size not in 8+ supported sizes → error
//   DFM-METRIC-FIT      : fit_class not in {close, medium, free} → error
//   DFM-METRIC-DEPTH    : stock_thickness < 1.0 × clearance_dia → warning
//                         (insufficient bearing area per VDI 2230)

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace bolt_hole_metric_spec {

constexpr const char* kSkillId = "bolt_hole_metric_spec";

struct Input
{
    FaceDatum    entry_face;
    double       position_x_mm   = 0.0;
    double       position_y_mm   = 0.0;
    gp_Dir       axis_dir        { 0.0, 0.0, -1.0 };
    std::string  thread_size     = "M6";
    std::string  fit_class       = "medium";       // close | medium | free
    double       chamfer_size_mm = 0.5;            // entry chamfer leg
    double       chamfer_angle_deg = 30.0;         // 30° break per ISO 13715
};

// Returns the ISO 273 clearance hole diameter for a given size + fit, or
// 0.0 if the size/fit combination is unknown.  Exposed for unit tests.
double clearanceDiameterFor(const std::string& thread_size,
                            const std::string& fit_class);

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace bolt_hole_metric_spec
}  // namespace koocadcam::skill
