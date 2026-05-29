#pragma once
// @lat: [[engine/skills#finish_apparel]]
//
// finish_apparel — record-only final processing of a garment: washing,
// pressing, or labeling.  These steps adjust surface properties (hand,
// shrinkage, label attach) but leave the macroscopic B-rep unchanged.
// This skill stamps the finish-record signature so MES and QC can
// downstream verify wash temp / press cycle / label code.
//
// Signature pattern:
//   - pattern.kind             = "finish_apparel"
//   - pattern.technique        = "washing" | "pressing" | "labeling"
//   - pattern.bath_temp_c      = <input>   (washing) or 0 for non-wash
//   - pattern.geometry_changed = false
//
// DFM checks:
//   technique     ∈ {washing, pressing, labeling}    (DFM-INPUT error)
//   bath_temp_c   ∈ [0, 95] for washing              (DFM-FIN-TEMP warning)
//   bath_temp_c   == 0 ok for non-washing            (info if non-zero)
//
// Synthesis:
//   NO geometric change — clone workpiece and stamp signature.
//
// Recognize:
//   Metadata-only replay.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace finish_apparel {

constexpr const char* kSkillId = "finish_apparel";

struct Input
{
    std::string  technique    = "pressing";   // "washing" | "pressing" | "labeling"
    double       bath_temp_c  = 0.0;          // only meaningful for washing
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace finish_apparel
}  // namespace koocadcam::skill
