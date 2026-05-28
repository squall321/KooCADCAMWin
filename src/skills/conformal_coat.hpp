#pragma once
// @lat: [[engine/skills#conformal_coat]]
//
// conformal_coat — Thin polymer protective film applied to a populated PCB to
// guard against moisture, dust, salt fog, fungus, and dendritic growth.
// Sprayed, dipped, or selectively dispensed; cured by UV, heat, or moisture.
// The macroscopic PCB B-rep is UNCHANGED (typical coatings are 25–150 μm).
// What is recorded is the coating chemistry, target thickness, and coverage.
//
//            ┌──────────────────────────────────────┐
//            │ ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ │  ← thin conformal film
//            │ ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ │     (25 – 150 μm)
//            │  □ □ □  ◯  ▣  ▣  ◫  ◫  ▤             │  ← components underneath
//            └──────────────────────────────────────┘
//
// Signature pattern:
//   - pattern.kind             = "conformal_coat"
//   - pattern.coat_type        = "acrylic" | "silicone" | "polyurethane" |
//                                "epoxy" | "parylene"
//   - pattern.thickness_um     = <input>
//   - pattern.coverage_pct     = <input>   (typ. 95 – 100; "keep-out" pads reduce)
//   - pattern.ipc_class
//       "IPC-CC-830_thin"   thickness < 25 μm
//       "IPC-CC-830"        25 ≤ thickness ≤ 130 μm
//       "IPC-CC-830_thick"  thickness > 130 μm
//   - pattern.geometry_changed = false
//
// DFM checks:
//   DFM-INPUT              thickness_um ≤ 0          (error)
//   DFM-INPUT              coverage_pct ≤ 0          (error)
//   DFM-CC-THK-LOW         thickness_um < 25 μm      (error — below IPC-CC-830 floor)
//   DFM-CC-THK-HIGH        thickness_um > 300 μm     (error — bubble / cure risk)
//   DFM-CC-COVERAGE        coverage_pct > 100        (error — physical impossibility)
//   DFM-CC-COVERAGE-LOW    coverage_pct < 80         (info — large keep-out zones)
//   DFM-CC-TYPE            coat_type not in known catalog (info)
//
// Recognize:
//   Impossible from B-rep (film sub-OCCT).  Returns ONLY signatures replayed
//   from `workpiece.features()` filtered by skill_id — confidence 1.0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace conformal_coat {

constexpr const char* kSkillId = "conformal_coat";

struct Input
{
    std::string  coat_type     = "acrylic";  // acrylic | silicone | polyurethane | epoxy | parylene
    double       thickness_um  = 50.0;       // typical 25 – 150 μm
    double       coverage_pct  = 100.0;      // 0 – 100
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace conformal_coat
}  // namespace koocadcam::skill
