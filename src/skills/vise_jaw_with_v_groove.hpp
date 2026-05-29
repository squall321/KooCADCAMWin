#pragma once
// @lat: [[engine/skills#vise_jaw_with_v_groove]]
//
// vise_jaw_with_v_groove — soft jaw machined for round-stock gripping.
//
// A rectangular vise jaw (e.g. for a Kurt 6" vise) with a horizontal 90°
// included-angle V-groove machined along its working face, plus 2
// counterbored mounting holes on the jaw back so it can be bolted to the
// vise body (per ANSI/ASME B5.41 — Vise jaw mounting; DIN 6346 — soft
// jaws).
//
// Geometry chain on the input rectangular jaw stock:
//   1. CUT   triangular prism = the V-groove, along the full jaw length
//            (90° included angle, depth controls how big a Ø it grips).
//   2-N. For each mounting hole:
//        cut pilotCyl       — clearance hole through jaw
//        cut counterboreCyl — for SHCS head (head sits flush on back)
//
// For 2 mounting holes total chain = 1 + 2·2 = 5 sub-cuts → subfeature_count = 5.
//
// DFM:
//   DIN 6346  — V-groove included angle 60°-120° (90° standard for round stock)
//   ASME B5.41 — mounting bolt spacing ≥ 25 mm typical M8 / 30 mm M10
//   DFM-002    — pilot Ø ≥ 0.8 mm
//   groove depth ≤ 0.4 × jaw height (else jaw too thin under groove)

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace vise_jaw_with_v_groove {

constexpr const char* kSkillId = "vise_jaw_with_v_groove";

struct MountHole
{
    double x_mm = 0.0;       // along jaw length
    double z_mm = 0.0;       // along jaw height
};

struct Input
{
    FaceDatum               working_face;
    double                  v_groove_angle_deg     = 90.0;   // included angle
    double                  v_groove_depth_mm      = 6.0;    // groove depth from face
    double                  v_groove_z_mm          = 10.0;   // groove centre-line height
    double                  pilot_dia_mm           = 6.6;    // M6 clearance default
    double                  counterbore_dia_mm     = 10.0;   // M6 SHCS head + slip
    double                  counterbore_depth_mm   = 6.0;
    std::vector<MountHole>  mount_holes;                     // ≥ 1; SHCS positions
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace vise_jaw_with_v_groove
}  // namespace koocadcam::skill
