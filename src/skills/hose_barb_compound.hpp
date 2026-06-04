#pragma once
// @lat: [[engine/skills#hose_barb_compound]]
//
// hose_barb_compound — hose barb fitting (HVAC / fluid).
//
// Slice 15 — HVAC/fluid.
//
// Sub-features (sequential FUSES, no compound boolean — slice-14 root cause):
//   1. base cylindrical body (fuse #1, base_dia × tip_length)
//   2..(1+N). N annular ridge cones, each tapering FROM barb_max_dia TO
//             base_dia along +Z, stacked at barb_pitch_mm intervals.
//             Sequential fuses with overlap to model the multi-ridge barb.
//   Tip is tapered (final cone reduces over taper_to_tip_mm).
//
// subfeature_count = 1 + barb_count + 1 (base + N ridges + tip taper).
//
// DFM:
//   DFM-BARB-N : barb_count must be in [2, 8].
//   DFM-INPUT  : geometric inputs must be > 0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace hose_barb_compound {

constexpr const char* kSkillId = "hose_barb_compound";

struct Input
{
    FaceDatum   face_id;
    double      center_x_mm        = 0.0;
    double      center_y_mm        = 0.0;
    gp_Dir      axis_dir           { 0.0, 0.0, 1.0 };
    double      base_dia_mm        = 8.0;
    int         barb_count         = 3;        // 2..8
    double      barb_pitch_mm      = 3.5;      // axial pitch between ridges
    double      barb_max_dia_mm    = 10.0;     // OD of each ridge
    double      taper_to_tip_mm    = 2.0;      // tapered tip length above last ridge
};

SkillOutput  apply   (const Workpiece& wp, const Input& in);
DFMReport    validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace hose_barb_compound
}  // namespace koocadcam::skill
