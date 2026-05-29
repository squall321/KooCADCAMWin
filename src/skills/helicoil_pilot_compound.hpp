#pragma once
// @lat: [[engine/skills#helicoil_pilot_compound]]
//
// helicoil_pilot_compound — COMPOUND skill that prepares a blind hole to
// receive a Heli-Coil wire-thread insert.  The target_thread_size is the
// THREAD that the insert will expose to the user (M3, M4, ...); the actual
// pilot is OVERSIZE per the Heli-Coil Recommended STI Tap Drill chart.
//
// Chains:
//   sub-feature 1: drill_pilot (Heli-Coil tap-drill diameter for STI tap)
//   sub-feature 2: thread_pilot — the chip-clearance section below the
//                  tap engagement (geometry-wise: same pilot dia, extra
//                  0.5 × insert-length, no thread modelled in slice-1)
//   sub-feature 3: 90° countersink at entry for clean insert installation
//
// Heli-Coil STI tap-drill chart (metric coarse, 1×D inserts):
//
//   target | STI tap-drill dia | rec. csk top dia
//   -------+-------------------+-------------------
//   M2     | 2.10              | 2.6
//   M2.5   | 2.65              | 3.2
//   M3     | 3.10              | 3.7
//   M4     | 4.10              | 4.8
//   M5     | 5.20              | 6.0
//   M6     | 6.25              | 7.2
//   M8     | 8.30              | 9.4
//   M10    | 10.40             | 11.7
//
// DFM:
//   DFM-002        : tap-drill dia < 0.8 mm (impossible — guards table)
//   DFM-HELI-SIZE  : unknown thread size
//   DFM-HELI-DEPTH : insert_length_mm <= 0
//
// Recognize: metadata replay, confidence 1.0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace helicoil_pilot_compound {

constexpr const char* kSkillId = "helicoil_pilot_compound";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir       { 0.0, 0.0, -1.0 };
    std::string target_thread_size;       // "M2".."M10"
    double      insert_length_mm = 0.0;   // 1×D nominal length of insert
};

double stiTapDrillFor (const std::string& size);
double cskTopDiaFor   (const std::string& size);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace helicoil_pilot_compound
}  // namespace koocadcam::skill
