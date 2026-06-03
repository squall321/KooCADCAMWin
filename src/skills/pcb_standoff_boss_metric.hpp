#pragma once
// @lat: [[engine/skills#pcb_standoff_boss_metric]]
//
// pcb_standoff_boss_metric — PHONE compound: PCB mounting boss with a metric
// threaded blind hole. Designed for phone metal frame PCB attach (M1.6 / M2 /
// M2.5 / M3).
//
// Sub-features (compound):
//   1. cylindrical boss  (fuse)  — diameter ≈ pilot_dia + 2 × wall_min,
//                                   height = boss_height_mm, axis +Z (face up).
//      Material is ADDED FIRST.
//   2. tapped pilot hole (cut)   — pilot_dia from central thread table,
//                                   depth = thread_depth_mm. CUT AFTER add.
//
// DFM (real checks per central tables):
//   DFM-INPUT       : boss_height_mm > 0; thread_depth_mm > 0
//   DFM-BAD-SPEC    : thread_size_key ∈ {"M1.6","M2","M2.5","M3"}
//   DFM-MIN-HOLE    : pilot_dia ≥ 0.3 mm
//   DFM-MIN-WALL    : (boss_dia − pilot_dia) / 2 ≥ 0.4 mm (≥ 0.4 mm wall)
//   DFM-AR          : boss_height_mm / boss_dia ≤ 5
//   DFM-DEPTH       : thread_depth_mm ≤ boss_height_mm − 0.3 (leave a floor)
//
// Signature pattern:
//   pattern.kind                       = "pcb_standoff_boss_metric"
//   pattern.is_compound                = true
//   pattern.is_phone_feature           = true
//   pattern.phone_feature_type         = "pcb_standoff_boss"
//   pattern.thread_size_key            = "M1.6" | "M2" | "M2.5" | "M3"
//   pattern.boss_dia_mm / pilot_dia_mm / boss_height_mm / thread_depth_mm

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>

namespace koocadcam::skill {

class Workpiece;

namespace pcb_standoff_boss_metric {

constexpr const char* kSkillId = "pcb_standoff_boss_metric";

struct Input
{
    int         face_id          = -1;
    double      center_x_mm      = 0.0;
    double      center_y_mm      = 0.0;
    double      boss_height_mm   = 2.0;
    std::string thread_size_key  = "M2";   // M1.6 | M2 | M2.5 | M3
    double      thread_depth_mm  = 1.4;
};

// Lookup: small-screw metric pilot diameter (mm). Returns 0.0 if unsupported.
double pilotDiameterFor(const std::string& thread_size_key);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace pcb_standoff_boss_metric
}  // namespace koocadcam::skill
