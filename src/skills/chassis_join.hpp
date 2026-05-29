#pragma once
// @lat: [[engine/skills#chassis_join]]
//
// chassis_join — automotive Body-In-White (BIW) chassis-joining station.
// Combines spot/seam welds, sealer/structural adhesive beads, and
// mechanical bolt fastening of underbody / cradle / floor-pan / side-frame
// subassemblies into the chassis assembly.  This is an assembly-line
// operation, not a part-removal one — the macroscopic B-rep we carry on
// the parent workpiece is unchanged; only the join record is stamped.
//
//      ┌─────────────┐    welds + sealer + bolts    ┌─────────────┐
//      │ subassembly │ ───────────────────────────▶ │   chassis   │
//      │  + chassis  │                              │   (joined)  │
//      └─────────────┘                              └─────────────┘
//
// Signature pattern:
//   - pattern.kind             = "chassis_join"
//   - pattern.weld_count       = int (spot + seam welds)
//   - pattern.sealer_meters    = total sealer / adhesive bead length (m)
//   - pattern.bolt_count       = mechanical fasteners
//   - pattern.geometry_changed = false
//
// DFM checks:
//   weld_count    ≥ 0                              (DFM-INPUT error if < 0)
//   sealer_meters ≥ 0                              (DFM-INPUT error if < 0)
//   bolt_count    ≥ 0                              (DFM-INPUT error if < 0)
//   weld_count + bolt_count > 0                    (DFM-CHASSIS-EMPTY error)
//   weld_count > 200                               (DFM-CHASSIS-WELD info — overweld)
//
// Synthesis:
//   NO geometric change.  Clone shape + material, stamp signature.
//
// Recognize:
//   Metadata-only — no geometric fingerprint.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace chassis_join {

constexpr const char* kSkillId = "chassis_join";

struct Input
{
    int     weld_count    = 60;        // total welds (spot + seam)
    double  sealer_meters = 3.0;       // sealer / structural adhesive length (m)
    int     bolt_count    = 12;        // mechanical fasteners
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace chassis_join
}  // namespace koocadcam::skill
