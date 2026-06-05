#pragma once
// @lat: [[engine/skills#hydraulic_quick_coupler_iso7241]]
//
// hydraulic_quick_coupler_iso7241 — Hydraulic flat-face quick coupler port
// per ISO 7241-A (slice 16, agricultural/forestry — ag implements).
//
// ISO 7241-A flat-face couplers are the standard ag/construction interface
// for connecting/disconnecting hydraulic lines under pressure-less
// conditions.  Each dash size has:
//   - central port bore for the male coupler tip to enter
//   - O-ring face groove on the mating flat (AS568 ring)
//   - spotface counterbore for the threaded body shoulder
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. central port bore (cylinder cut DOWN from top face)
//   2. AS568 O-ring face groove (annular ring)
//   3. spotface counterbore (wider cylinder for body shoulder)
//
// subfeature_count = 3.
//
// DFM:
//   DFM-INPUT     : every dimension must be > 0.
//   DFM-DASH      : dash_size in { "-4", "-6", "-8" }.
//   DFM-THREAD    : thread_size_key must exist in BSP G or NPT central table.
//   DFM-AS568     : derived AS568 dash key must exist in central AS568 table.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace hydraulic_quick_coupler_iso7241 {

constexpr const char* kSkillId = "hydraulic_quick_coupler_iso7241";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      center_xy        { 0.0, 0.0, 0.0 };
    std::string dash_size;                            // "-4" | "-6" | "-8"
    std::string thread_size_key;                      // from _hydraulic_ports.hpp BSP G or NPT
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace hydraulic_quick_coupler_iso7241
}  // namespace koocadcam::skill
