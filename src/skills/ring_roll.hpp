#pragma once
// @lat: [[engine/skills#ring_roll]]
//
// ring_roll — ring rolling.  A pierced doughnut blank is rotated between an
// outer main roll and an inner mandrel; the squeeze gradually reduces the
// wall thickness, which (via volume preservation) grows the ring's outer
// diameter and reduces its height.  Used to make seamless rings — bearing
// races, flanges, turbine cases.
//
//          ┌──────────────┐                              ┌──────────────────┐
//   ───►   │ pierced blank│   main roll + mandrel  ───►  │ thinner, larger  │
//          │   ID0 / OD0  │      axial rolls             │   ID1 / OD1      │
//          └──────────────┘                              └──────────────────┘
//
// Synthesis strategy:
//   Metadata-only — slice-1 does not synthesize the new ring shape because
//   that is normally provided by an upstream CAD model of the finished part.
//   We stamp the signature recording the target ID / OD / height and the
//   ID/OD ratio so downstream layers can verify dimensional consistency.
//
// Signature pattern:
//   - kind             = "ring_roll"
//   - target_id_mm
//   - target_od_mm
//   - target_height_mm
//   - id_od_ratio
//   - geometry_changed = false
//
// DFM checks:
//   DFM-INPUT       : target_od_mm <= target_id_mm; any dim <= 0 — error
//   DFM-RR-RATIO    : id_od_ratio < 0.50 — info (very thick wall)
//   DFM-RR-RATIO    : id_od_ratio > 0.95 — error (paper-thin ring; collapse)
//   DFM-RR-HEIGHT   : height > target_od_mm — warning (column ring; tip risk)
//
// Recognize:
//   Pure metadata replay; geometric fallback could detect a hollow disk but
//   slice-1 keeps it metadata-only for simplicity.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace ring_roll {

constexpr const char* kSkillId = "ring_roll";

struct Input
{
    double  target_id_mm     = 0.0;   // finished inner diameter
    double  target_od_mm     = 0.0;   // finished outer diameter
    double  target_height_mm = 0.0;   // axial height of the ring
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace ring_roll
}  // namespace koocadcam::skill
