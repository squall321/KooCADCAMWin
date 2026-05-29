#pragma once
// @lat: [[engine/skills#ballast_tamp]]
//
// ballast_tamp — mechanical tamping of ballast under sleepers to restore
// track geometry after lift.  A tamping machine lifts the sleeper by
// `lift_mm`, vibrates tamping tines into the ballast, and squeezes the
// stones under the sleeper.  KooCADCAM records the lift, the line speed
// class, and how many tamping passes were performed.  Macroscopic B-rep
// of the rail is unchanged — this is purely a substructure operation.
//
// Signature pattern:
//   - pattern.kind             = "ballast_tamp"
//   - pattern.lift_mm          = sleeper lift before tamp (mm)
//   - pattern.line_class       = "freight" | "main" | "high_speed"
//   - pattern.passes_count     = >= 1
//   - pattern.geometry_changed = false
//
// DFM checks:
//   lift_mm     ∈ (0, 60]                       (DFM-BT-LIFT error if outside)
//   line_class  ∈ {freight,main,high_speed}     (DFM-BT-CLASS info if unknown)
//   passes_count >= 1                           (DFM-INPUT error)
//   high_speed line yet passes_count < 2        (DFM-BT-PASS info advisory)
//
// Synthesis:
//   NO geometric change. Clone + stamp.
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

namespace ballast_tamp {

constexpr const char* kSkillId = "ballast_tamp";

struct Input
{
    double       lift_mm      = 20.0;     // sleeper lift before tamping
    std::string  line_class   = "main";   // "freight" | "main" | "high_speed"
    int          passes_count = 1;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace ballast_tamp
}  // namespace koocadcam::skill
