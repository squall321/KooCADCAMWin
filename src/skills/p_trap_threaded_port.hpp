#pragma once
// @lat: [[engine/skills#p_trap_threaded_port]]
//
// p_trap_threaded_port — P-trap fitting with NPT-threaded inlet & outlet.
//
// Slice 15 — HVAC/fluid.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. inlet NPT port (drill + taper cone fused, then cut)
//   2. U-shape trap channel (2 vertical bores + 1 horizontal connector
//      — approximates BRepOffsetAPI_MakePipe of a U-spine but is robust)
//   3. outlet NPT port (drill + taper cone fused, then cut)
//
// subfeature_count = 1 (inlet) + 3 (U-trap legs) + 1 (outlet) = 5.
//
// DFM:
//   DFM-NPT-CODE : inlet/outlet thread_size_key not in central NPT table.
//   DFM-INPUT    : geometric inputs must be > 0.
//   DFM-TRAP-R   : trap_radius < port drill dia × 1.5.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace p_trap_threaded_port {

constexpr const char* kSkillId = "p_trap_threaded_port";

struct Input
{
    FaceDatum   face_id;
    double      center_x_mm           = 0.0;
    double      center_y_mm           = 0.0;
    double      trap_radius_mm        = 12.0;
    std::string inlet_thread_size_key;          // NPT "1/2" | "3/4" | "1"
    std::string outlet_thread_size_key;
};

SkillOutput  apply   (const Workpiece& wp, const Input& in);
DFMReport    validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace p_trap_threaded_port
}  // namespace koocadcam::skill
