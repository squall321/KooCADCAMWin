#pragma once
// @lat: [[engine/skills#fixture_setup]]
//
// fixture_setup — metadata-only operation that records how the workpiece is
// held during subsequent machining: fixture type (vise, three-jaw chuck,
// custom soft jaws, magnetic chuck, …), the XYZ locations where the part
// is clamped, and one or more work-offsets (G54 / G55 / …) into which the
// fixture origin is translated by the controller.
//
// Geometry is UNCHANGED.  The workpiece shape, material, and prior feature
// list are passed through verbatim with a single new signature appended.
//
//        ┌─────────────────┐                       ┌─────────────────┐
//        │                 │   fixture_setup       │                 │
//        │   stock         │  ─────────────────▶  │   stock         │
//        │                 │   (vise, G54, …)      │  (no shape Δ)   │
//        └─────────────────┘                       └─────────────────┘
//
// Signature pattern:
//   - pattern.kind            = "fixture_setup"
//   - pattern.fixture_type    = <input>
//   - pattern.clamp_locations = [[x,y,z], …]
//   - pattern.work_offsets    = ["G54", "G55", …]
//   - pattern.geometry_changed = false
//
// DFM checks:
//   DFM-INPUT       fixture_type empty                       (error)
//   DFM-INPUT       clamp_locations empty                    (error)
//   DFM-INPUT       work_offsets empty                       (error)
//   FX-OFFSET-FORM  any work_offset not matching ^G5[4-9]    (warning)
//
// Recognize: metadata replay only.

#include "Skill.hpp"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace fixture_setup {

constexpr const char* kSkillId = "fixture_setup";

struct Input
{
    std::string                       fixture_type     = "vise";
    std::vector<std::array<double,3>> clamp_locations;        // [{x,y,z}]
    std::vector<std::string>          work_offsets     = { "G54" };
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace fixture_setup
}  // namespace koocadcam::skill
