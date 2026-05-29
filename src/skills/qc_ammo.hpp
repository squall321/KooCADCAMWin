#pragma once
// @lat: [[engine/skills#qc_ammo]]
//
// qc_ammo — record final QC inspection of a finished cartridge lot for
// authorized civilian/commercial small-arms manufacturing.  The skill is
// metadata-only: it stamps cartridge OAL (overall length), accept count,
// and reject count so process planning + lot release can downstream verify
// the lot is in-spec and traceable.
//
// Signature pattern:
//   - pattern.kind             = "qc_ammo"
//   - pattern.cartridge_oal_mm = overall length (case base to bullet tip)
//   - pattern.accept_count     = units passing all checks
//   - pattern.reject_count     = units failing any check
//   - pattern.yield_pct        = 100 × accept / (accept + reject)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   cartridge_oal_mm ∈ (5.0, 150.0)   (DFM-AMMO-OAL error if outside)
//   accept_count + reject_count > 0   (DFM-AMMO-QCSAMPLE error)
//   accept_count >= 0, reject_count >= 0 (DFM-INPUT error)
//   yield_pct < 95.0 → info finding (lot review recommended)
//
// Synthesis: metadata-only.
// Recognize: metadata replay.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace qc_ammo {

constexpr const char* kSkillId = "qc_ammo";

struct Input
{
    double cartridge_oal_mm = 29.69;
    int    accept_count     = 100;
    int    reject_count     = 0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace qc_ammo
}  // namespace koocadcam::skill
