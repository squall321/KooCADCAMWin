#pragma once
// @lat: [[engine/skills#needle_valve_seat]]
//
// needle_valve_seat — precision throttling needle valve internals.
//
// Sub-features (REAL cuts):
//   1. Tapered needle pocket — conical cavity (typical 30° included angle)
//      that the tapered needle tip seats into.
//   2. Control bore — tiny (< 1 mm) precision cylindrical orifice that exits
//      the bottom (point) of the needle pocket; this is the controlled
//      flow path.
//   3. Stem boss counterbore — a wider cylindrical seat above the needle
//      pocket for the threaded stem bushing.
//   4. Stem threaded relief — a slightly wider relief between the boss
//      and the pocket so the stem threads don't engage the seat.
//
// DFM (ISA 75.11 — Inherent Flow Characteristic of Control Valves +
//     ANSI/ISA-75.01 control valve sizing):
//   - Control bore dia: 0.2 mm ≤ d ≤ 1 mm for "needle" classification
//     (per ISA 75.11 Annex on small-Cv valves).
//   - Needle taper included angle 20..45° (manufacturer norm; tight angle
//     → fine throttling, wide angle → coarse but more flow at given lift).
//   - Stem boss ID ≥ needle pocket entry × 1.4 (so the stem nut clears).
//
// Recognize: cone face + adjacent narrow cylindrical face (the orifice) +
// wider top cylindrical counterbore.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace needle_valve_seat {

constexpr const char* kSkillId = "needle_valve_seat";

struct Input
{
    FaceDatum   entry_face;                            // top face — stem enters from here
    double      position_x_mm         = 0.0;
    double      position_y_mm         = 0.0;
    gp_Dir      axis_dir              { 0.0, 0.0, -1.0 };// downward needle axis

    double      pocket_entry_dia_mm   = 6.0;            // top of the cone
    double      pocket_taper_deg      = 30.0;           // total included angle
    double      pocket_length_mm      = 8.0;            // axial cone length
    double      control_bore_dia_mm   = 0.6;            // < 1 mm, precision
    double      control_bore_depth_mm = 6.0;            // through-depth below cone tip
    double      stem_boss_dia_mm      = 12.0;           // top counterbore for stem bushing
    double      stem_boss_depth_mm    = 5.0;            // boss depth from entry
    double      thread_relief_dia_mm  = 7.0;            // relief between boss and cone
    double      thread_relief_depth_mm = 2.0;           // relief axial length
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace needle_valve_seat
}  // namespace koocadcam::skill
