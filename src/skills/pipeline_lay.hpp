#pragma once
// @lat: [[engine/skills#pipeline_lay]]
//
// pipeline_lay — surface gathering / trunkline pipeline laying campaign
// record.  Captures the pipe geometry (OD + wall) and the route extent
// (length + terrain) for a segment of installed pipeline.  This is a
// process record for downstream construction planning; the workpiece
// (line-pipe joint or fitting model) is NOT geometrically modified.
//
//     ════════════════════════════════════════════════════════════════
//     │  pipe_dia_mm OD                                              │
//     │  wall_thick_mm                                               │
//     │  length_km  over  terrain_class                              │
//     ════════════════════════════════════════════════════════════════
//
// Signature pattern:
//   - pattern.kind             = "pipeline_lay"
//   - pattern.pipe_dia_mm
//   - pattern.wall_thick_mm
//   - pattern.length_km
//   - pattern.terrain_class    = "flat" | "rolling" | "mountain" | "subsea" | "swamp"
//   - pattern.geometry_changed = false
//
// DFM checks:
//   pipe_dia_mm > 0                              (DFM-INPUT error)
//   pipe_dia_mm ∈ [60.3, 1219.0] mm              (DFM-PL-DIA info — typ 2"–48")
//   wall_thick_mm > 0                            (DFM-INPUT error)
//   wall_thick_mm < pipe_dia_mm / 2              (DFM-PL-WALL error — wall ≥ radius)
//   wall_thick_mm / pipe_dia_mm < 0.005          (DFM-PL-DR info — D/t > 200)
//   length_km > 0                                (DFM-INPUT error)
//   terrain_class non-empty                      (DFM-INPUT warning)
//
// Synthesis:
//   NO geometric change.  Clone workpiece + stamp signature.
//
// Recognize:
//   Metadata-only — pipeline route has no part-level B-rep.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace pipeline_lay {

constexpr const char* kSkillId = "pipeline_lay";

struct Input
{
    double       pipe_dia_mm    = 323.9;       // 12 3/4" trunkline
    double       wall_thick_mm  = 9.5;
    double       length_km      = 50.0;
    std::string  terrain_class  = "rolling";   // "flat"|"rolling"|"mountain"|"subsea"|"swamp"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace pipeline_lay
}  // namespace koocadcam::skill
