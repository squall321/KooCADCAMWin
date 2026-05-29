#pragma once
// @lat: [[engine/skills#bezel_groove_assembly]]
//
// bezel_groove_assembly — COMPOUND FEATURE (4 primitive cuts).
//
// Realistic watch-bezel retention groove: an annular U-channel cut into the
// case-top face with a tip chamfer on the entry edge and a bottom fillet for
// stress relief.  Chains 4 primitive ops:
//
//   1. outer annular cut at radius = outer_dia/2 (sets outer wall)
//   2. inner annular cut at radius = inner_dia/2 (sets inner wall)
//      → outer + inner are FUSED first then cut once (overlapping concentric
//        cylinders pattern from counterbore.cpp)
//   3. tip chamfer on the top entry circle (cone frustum subtracted)
//   4. bottom fillet (small toroidal sweep at the groove floor — approximated
//      as a thin annular sub-cut)
//
// Output topology: 1 annular pocket (outer cyl + inner cyl + bottom annulus +
// chamfer cone band + fillet land).  Single Workpiece, single FeatureSignature
// with pattern.is_compound = true, subfeature_count = 4.
//
// DFM gates (DIN 471-style snap-ring retention groove):
//   - outer_dia > inner_dia + 0.4 mm (min groove width)
//   - groove_depth ≥ 0.3 mm and ≤ outer_dia/8 (avoid over-deep cut)
//   - tip chamfer angle 15°-60° (typical DIN 6784 watch bezel)
//   - case must be ≥ outer_dia / 4 thick (stress on retention land)
//
// Recognize: metadata replay only (compound groove is hard to fingerprint
// geometrically — multiple annular faces can stem from many feature kinds).
// Returns confidence 1.0 when matched from wp.features() history.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace bezel_groove_assembly {

constexpr const char* kSkillId = "bezel_groove_assembly";

struct Input
{
    FaceDatum   case_face;            // top face of the case (where the groove enters)
    double      center_x_mm    = 0.0;
    double      center_y_mm    = 0.0;
    gp_Dir      axis_dir       { 0.0, 0.0, -1.0 };  // groove cut direction (into case)
    double      outer_dia_mm   = 0.0;
    double      inner_dia_mm   = 0.0;
    double      groove_depth_mm = 0.0;
    double      taper_deg      = 30.0;   // tip chamfer half-angle
    double      bottom_fillet_mm = 0.15;  // small radius at groove floor
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace bezel_groove_assembly
}  // namespace koocadcam::skill
