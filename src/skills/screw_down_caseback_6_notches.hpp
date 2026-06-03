#pragma once
// @lat: [[engine/skills#screw_down_caseback_6_notches]]
//
// screw_down_caseback_6_notches — COMPOUND FEATURE (thread relief + spanner
// notches), bottom of watch case.
//
// Screw-down case back: a cylindrical thread relief is cut into the bottom of
// the case to receive an external-thread caseback (approximated as a straight
// cylinder cut at thread minor diameter + small chamfer entry), and N
// rectangular spanner notches (typically 6) are cut around the perimeter to
// allow a case-opener tool to engage and unscrew the back.
//
// Sub-feature chain:
//   1. cylindrical thread relief cut (caseback_dia at thread minor depth)
//   2..N+1. rectangular spanner notches (small boxes radial-outward,
//           rotated about the case axis)
//
// DFM: caseback_dia > 0, thread_pitch ∈ [0.5, 2.0], notch_count ∈ [4, 8],
// all dims > 0, notch_depth < caseback_dia/4.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace screw_down_caseback_6_notches {

constexpr const char* kSkillId = "screw_down_caseback_6_notches";

struct Input
{
    FaceDatum   case_bottom_face;
    gp_Ax1      case_axis            { gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1) };
    double      caseback_dia_mm   = 0.0;
    double      thread_pitch_mm   = 1.0;
    double      thread_depth_mm   = 1.5;
    int         notch_count       = 6;
    double      notch_width_mm    = 1.5;
    double      notch_depth_mm    = 0.6;
    double      notch_radial_extent_mm = 2.0;   // tangential length of notch
    double      bottom_z_mm       = 0.0;        // bottom Z; 0 -> infer
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace screw_down_caseback_6_notches
}  // namespace koocadcam::skill
