#pragma once
// @lat: [[engine/skills#drop_test]]
//
// drop_test — destructive impact test record.  A finished part is dropped
// from a fixed height onto a defined surface to verify that mechanical
// integrity (housings, latches, displays) survives a drop event.  The
// macroscopic B-rep we hand back is UNCHANGED — the skill stamps a
// signature so downstream QA/reliability can audit which drop spec was
// applied to which unit, and so a watch-and-replay flow can recover the
// test parameters from the part record alone.
//
//          ───────────────────  drop_height_mm above floor
//             │                │
//             │  test  unit    │   axis = impact_axis  (e.g. -Z)
//             │                │
//             ▼                ▼
//        ━━━━━━━━━━━━━━━  surface_type (concrete / steel / wood / …)
//
// Signature pattern:
//   - pattern.kind             = "drop_test"
//   - pattern.drop_height_mm
//   - pattern.impact_axis      ("+X" | "-X" | "+Y" | "-Y" | "+Z" | "-Z")
//   - pattern.surface_type     ("concrete" | "steel" | "wood" | "rubber")
//   - pattern.expected_g_peak  (lookup-derived rough estimate)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   drop_height_mm > 0                            (DFM-INPUT error)
//   drop_height_mm ∈ [50, 3000]                   (DFM-DROP-HEIGHT info)
//   impact_axis    ∈ {±X, ±Y, ±Z}                 (DFM-DROP-AXIS info)
//   surface_type   ∈ known set                    (DFM-DROP-SURFACE info)
//
// Synthesis:
//   NO geometric change.  Clone shape + material verbatim, append
//   FeatureSignature.  stock_removed_mm3 = 0.
//
// Recognize:
//   Destructive test leaves no B-rep trace recoverable by CAD alone.
//   Metadata-only replay from `workpiece.features()`.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace drop_test {

constexpr const char* kSkillId = "drop_test";

struct Input
{
    double      drop_height_mm = 1000.0;          // ISO 2248 typical 1 m
    std::string impact_axis    = "-Z";            // "+X|-X|+Y|-Y|+Z|-Z"
    std::string surface_type   = "concrete";      // "concrete|steel|wood|rubber"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace drop_test
}  // namespace koocadcam::skill
