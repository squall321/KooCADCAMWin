#pragma once
// @lat: [[engine/skills#quarry_extract]]
//
// quarry_extract — record of removing a discrete block (or rubble batch) from
// a quarry face.  This is an UPSTREAM process: at this stage the workpiece
// stand-in is just stock representing the as-extracted block.  No CAD-level
// geometric change happens here — we stamp the extraction batch metadata
// (rock type, volume, extraction method) so downstream sand_wash / gravel_grade
// / stone_cut steps know what they are processing.
//
// Signature pattern:
//   - pattern.kind             = "quarry_extract"
//   - pattern.rock_type        = "granite" | "limestone" | "marble" | "basalt" | "sandstone"
//   - pattern.volume_m3        = bulk volume extracted in m^3
//   - pattern.extraction_method= "drill_blast" | "diamond_wire" | "channeling" | "ripping"
//   - pattern.geometry_changed = false
//
// DFM checks:
//   DFM-INPUT             volume_m3 ≤ 0 (error)
//   DFM-QUARRY-METHOD     extraction_method outside known set (info)
//   DFM-QUARRY-METHOD-FIT diamond_wire / channeling on dimension stone OK;
//                         drill_blast on marble damages slabs (warning)
//
// Synthesis:
//   NO geometric change.  Clone the workpiece (shape + material) and stamp
//   the signature.
//
// Recognize:
//   Metadata-only — the extraction event leaves no B-rep trace.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace quarry_extract {

constexpr const char* kSkillId = "quarry_extract";

struct Input
{
    std::string rock_type         = "granite";
    double      volume_m3         = 5.0;
    std::string extraction_method = "diamond_wire";
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace quarry_extract
}  // namespace koocadcam::skill
