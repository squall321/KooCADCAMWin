#pragma once
// @lat: [[engine/skills#shred]]
//
// shred — mechanical shredding / size reduction of EOL parts.  Records the
// machine throughput rate and the final particle (grain) size after
// shredding.  The OCCT B-rep of the input is NOT modified — the actual
// shreds are sub-OCCT scale and physically destroyed; we keep the original
// for traceability + record the process metadata.
//
//        ┌─────────────┐                       ┌─────────────┐
//        │   intact    │       shred           │   intact    │
//        │   part      │  ────────────────▶   │   (sig only)│
//        │             │  (X kg/h, p mm)       │   particle: │
//        └─────────────┘                       │   ▒ ▒ ▒ ▒ ▒ │
//                                              └─────────────┘
//          shape unchanged;  signature records final_particle_size_mm
//
// Signature pattern:
//   - pattern.kind                    = "shred"
//   - pattern.throughput_kg_h         = machine throughput (kg/hr)
//   - pattern.final_particle_size_mm  = nominal output piece size (mm)
//   - pattern.geometry_changed        = false
//
// DFM checks:
//   throughput_kg_h        > 0   (DFM-INPUT error)
//   final_particle_size_mm > 0   (DFM-INPUT error)
//   final_particle_size_mm < 1   (DFM-SHRED-FINE info — very fine grind:
//                                  energy / dust hazard, may need cryo)
//   final_particle_size_mm > 200 (DFM-SHRED-COARSE info — coarse, likely
//                                  primary shred only; downstream sorting
//                                  may be impaired)
//   throughput_kg_h > 5000       (DFM-SHRED-THRU info — exceeds typical
//                                  single-shaft commercial throughput)
//
// Synthesis:
//   NO geometric change.  Clone the workpiece (shape + material) + stamp.
//
// Recognize:
//   Impossible from B-rep alone (shredded pieces are sub-OCCT and not the
//   workpiece we hold).  Returns ONLY signatures replayed from
//   `workpiece.features()`, confidence 1.0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace shred {

constexpr const char* kSkillId = "shred";

struct Input
{
    double  throughput_kg_h         = 500.0;
    double  final_particle_size_mm  = 20.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace shred
}  // namespace koocadcam::skill
