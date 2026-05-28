#pragma once
// @lat: [[engine/skills#vibratory_sortation]]
//
// vibratory_sortation — bowl-feeder orientation + sorting step.  Bulk
// parts are loaded into a vibratory bowl which agitates them up a
// spiral track; in-spec / correctly-oriented parts continue to the
// downstream station, mis-oriented parts are kicked back via tooling
// gates.  Geometric no-op on a single workpiece; the skill records the
// throughput rate and the orientation yield so cell-level capacity
// planning can roll it up.
//
//                ╭─────────────────╮
//                │ ↑ ↑ ↑ ↑ ↑       │   ← parts climb spiral track
//                │  ↑ ↑ ↑ ↑ ↑      │
//                │ ╲             ╱ │
//                │  ╲     ●     ╱  │   ← bowl
//                │   ╲___▼▼▼___╱   │
//                ╰────────────────╯
//                      ▼ feed_rate_per_min (oriented parts/min)
//
// Signature pattern:
//   - pattern.kind                    = "vibratory_sortation"
//   - pattern.feed_rate_per_min       = <double>
//   - pattern.orientation_quality_pct = <double>  in [0, 100]
//   - pattern.cycle_time_per_part_s   = 60 / feed_rate_per_min
//   - pattern.geometry_changed        = false
//
// DFM checks:
//   feed_rate_per_min > 0                          (DFM-INPUT error)
//   orientation_quality_pct ∈ [0, 100]             (DFM-INPUT error if out)
//   feed_rate_per_min ≤ 600                        (DFM-VIB-RATE info — industrial cap)
//   orientation_quality_pct < 80                   (DFM-VIB-YIELD warning — re-tune track)
//   orientation_quality_pct < 50                   (DFM-VIB-YIELD error — re-design)
//
// Synthesis:
//   NO geometric change.  Clone + stamp.
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

namespace vibratory_sortation {

constexpr const char* kSkillId = "vibratory_sortation";

struct Input
{
    double  feed_rate_per_min       = 60.0;
    double  orientation_quality_pct = 95.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace vibratory_sortation
}  // namespace koocadcam::skill
