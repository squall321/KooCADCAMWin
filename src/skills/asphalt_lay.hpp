#pragma once
// @lat: [[engine/skills#asphalt_lay]]
//
// asphalt_lay — place and compact an asphalt course on a road or bridge
// deck.  Each pass is described by mix_class (binder | wearing),
// lay_thickness_mm and area_m2.  KooCADCAM records this as metadata; the
// finished surface elevation lives in the road model upstream.
//
//          ════════════════════════════════════     ← finished wearing course
//          ████████████████████████████████████     ← lay_thickness_mm
//          ────────────────────────────────────       binder course / deck below
//
// Signature pattern:
//   - pattern.kind             = "asphalt_lay"
//   - pattern.mix_class        = "binder" | "wearing"
//   - pattern.lay_thickness_mm = compacted layer thickness (mm)
//   - pattern.area_m2          = paved area (m2)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   mix_class         ∈ {"binder","wearing"}              (DFM-ASPH-CLASS error)
//   lay_thickness_mm  > 0                                  (DFM-INPUT error)
//   area_m2           > 0                                  (DFM-INPUT error)
//   lay_thickness_mm  < 25  for wearing                    (DFM-ASPH-THK info)
//   lay_thickness_mm  > 100                                (DFM-ASPH-THK info — split into two lifts)
//
// Synthesis:
//   NO geometric change.  Clone + stamp.
//
// Recognize:
//   Metadata replay only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace asphalt_lay {

constexpr const char* kSkillId = "asphalt_lay";

struct Input
{
    std::string  mix_class        = "wearing";   // "binder" | "wearing"
    double       lay_thickness_mm = 50.0;        // compacted thickness (mm)
    double       area_m2          = 1000.0;      // paved area (m2)
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace asphalt_lay
}  // namespace koocadcam::skill
