#pragma once
// @lat: [[engine/skills#gauge_block_step]]
//
// gauge_block_step — compound stepped reference block, e.g. for height
// gauges, calibration of indicators, gauge-pin verification.
//
// Per JIS B 7506 / ANSI/ASME B89.1.9 stepped gauge specification, a step
// block is a rectangular reference body with N planar steps at calibrated
// depths.  Each step is created by removing a rectangular slab from the
// top, where each successive slab extends further in +X and has a deeper
// floor.  Steps land at the standard depths 1 / 2 / 3 / 5 / 10 mm
// (selectable subset).
//
// Geometry chain on the input rectangular block (length × width × height):
//   For step i ∈ [1, N]:
//     cut rectBox(origin = (x_start_i, 0, top), DX = step_length, DY = width,
//                 DZ = -step_depth_i)
//
// N sub-cuts → subfeature_count = N.  Plus the original top face = N+1
// horizontal planar faces at distinct Z heights after the cut.
//
// DFM:
//   ASME B89.1.9 — each step depth ≥ 0.5 mm (resolution)
//   each step length ≥ 5 mm so an indicator probe ball clears
//   step depths must be monotonically increasing
//   total step extent ≤ block length − end clearance (= 5 mm)

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace gauge_block_step {

constexpr const char* kSkillId = "gauge_block_step";

struct Input
{
    FaceDatum            top_face;
    double               x_start_mm      = 5.0;     // origin of first step (along +X from block 0)
    double               step_length_mm  = 10.0;    // each step's length along +X
    double               y_full_mm       = 0.0;     // 0 → use full block Y width
    std::vector<double>  step_depths_mm;            // depth of each step floor from top
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace gauge_block_step
}  // namespace koocadcam::skill
