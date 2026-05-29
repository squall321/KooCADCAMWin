#pragma once
// @lat: [[engine/skills#flange_face_with_gasket_groove]]
//
// flange_face_with_gasket_groove — REAL multi-cut compound for a finished
// flange face conforming to ASME B16.20 ring-joint dimensions.
//
// Sub-features (REAL geometric cuts on a flange-blank workpiece):
//   1. flat-machined facing pass on the top face   [fuse-cap then face-mill]
//   2. concentric gasket groove (annular cut)       [annularRing cut]
//
// We model the operation as:
//   step A: cut a thin facing pass off the flange top (fuse-then-cut: fuse
//           a thin disc above flange top to make a clean reference plane,
//           then cut a slightly thinner facing disc).
//   step B: cut the annular gasket groove into the freshly faced surface.
//
// subfeature_count = 2.
//
// DFM standard cited:
//   • ASME B16.20 — gasket groove ID/OD per nominal pipe class; surface
//     finish Ra 1.6–3.2 µm; groove depth tolerance ±0.13 mm; bottom-fillet
//     radius ≥ 0.4 mm; concentricity ≤ 0.25 mm to bolt circle.
//
// Recognize:
//   1. Find a planar top face with two concentric circular edges of
//      different radii (the groove boundary).
//   2. Confirm an annular planar face at the groove bottom, axially below
//      the top face by groove depth.
//   Confidence 0.85 (geometric) / 1.0 (replay).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace flange_face_with_gasket_groove {

constexpr const char* kSkillId = "flange_face_with_gasket_groove";

struct Input
{
    FaceDatum   entry_face;
    double      center_x_mm    = 0.0;
    double      center_y_mm    = 0.0;
    gp_Dir      axis_dir       { 0.0, 0.0, -1.0 };    // facing down into flange
    double      flange_outer_dia_mm = 180.0;          // for face-pass radius
    double      facing_pass_depth_mm = 0.5;            // axial material removed
    double      gasket_groove_id_mm  = 120.0;
    double      gasket_groove_od_mm  = 130.0;
    double      gasket_groove_depth_mm = 1.5;
    double      groove_fillet_r_mm     = 0.4;          // recorded for tooling (not modeled in BRep)
};

SkillOutput  apply   (const Workpiece& wp, const Input& in);
DFMReport    validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace flange_face_with_gasket_groove
}  // namespace koocadcam::skill
