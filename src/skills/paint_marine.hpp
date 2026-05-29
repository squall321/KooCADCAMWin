#pragma once
// @lat: [[engine/skills#paint_marine]]
//
// paint_marine — application of a multi-coat marine protective system
// (e.g. zinc primer + epoxy intermediate + anti-fouling top coat) onto the
// vessel's hull / superstructure.  KooCADCAM records the named coating
// system, total dry-film thickness (μm), and coated area (m²).  No
// macroscopic geometric change.
//
//             ▓▓▓▓▓▓▓▓▓▓▓▓▓▓   ← coat_system, dft_um total
//             ░░░░░░░░░░░░░░       over area_m2
//             ████████████████   ← steel substrate
//
// Signature pattern:
//   - pattern.kind             = "paint_marine"
//   - pattern.coat_system      = e.g. "zinc_epoxy_AF" | "PU_topcoat_HB"
//   - pattern.dft_um           = total dry-film thickness (μm)
//   - pattern.area_m2          = coated area (m²)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   coat_system   non-empty                     (DFM-INPUT error)
//   dft_um        ∈ [50, 1500]                  (DFM-PAINT-DFT error if outside)
//   area_m2       > 0                           (DFM-INPUT error)
//   dft_um        > 800 ⇒ stripe-coat + extended
//                          cure advisory        (DFM-PAINT-DFT info)
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

namespace paint_marine {

constexpr const char* kSkillId = "paint_marine";

struct Input
{
    std::string  coat_system = "zinc_epoxy_AF";  // named multi-coat system
    double       dft_um      = 350.0;            // total dry-film thickness (μm)
    double       area_m2     = 100.0;            // coated area (m²)
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace paint_marine
}  // namespace koocadcam::skill
