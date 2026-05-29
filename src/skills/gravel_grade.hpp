#pragma once
// @lat: [[engine/skills#gravel_grade]]
//
// gravel_grade — vibrating-screen classification: split a feed gravel
// stream into size classes (e.g., "5-10mm", "10-20mm", "20-40mm") and
// pick the target class for downstream use.  CAD geometry is unchanged
// at the workpiece level — we stamp the batch grading metadata.
//
// Signature pattern:
//   - pattern.kind                    = "gravel_grade"
//   - pattern.size_distribution_classes = json array of strings
//   - pattern.throughput_kg_h         = input
//   - pattern.target_class            = which classification was selected
//   - pattern.class_count             = derived size
//   - pattern.geometry_changed        = false
//
// DFM checks:
//   DFM-INPUT                throughput_kg_h ≤ 0 (error)
//   DFM-INPUT                size_distribution_classes empty (error)
//   DFM-GRAVEL-TARGET        target_class not present in distribution (warning)
//   DFM-GRAVEL-CLASS-COUNT   class_count > 6 (info — too many decks)
//
// Synthesis:
//   NO geometric change.  Clone and stamp.
//
// Recognize:
//   Metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace gravel_grade {

constexpr const char* kSkillId = "gravel_grade";

struct Input
{
    std::vector<std::string> size_distribution_classes =
        { "5-10mm", "10-20mm", "20-40mm" };
    double      throughput_kg_h = 30000.0;
    std::string target_class    = "10-20mm";
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace gravel_grade
}  // namespace koocadcam::skill
