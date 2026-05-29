#pragma once
// @lat: [[engine/skills#block_assembly]]
//
// block_assembly — shipyard PE/EU/GE-stage block assembly.  Sub-blocks of
// hull steel are jigged on the assembly platen, tack-welded, then fully
// welded into a finished hull block (or grand block) ready for outfitting
// and erection on the dock.  The macroscopic B-rep we carry through the
// process planner is unchanged by this operation — what we record is the
// METADATA: block identity, mass, and total weld-seam length.
//
//        ┌────────────────────────┐
//        │   sub-block A          │   ┐
//        ├────────────────────────┤   │  fit-up + tack
//        │   sub-block B          │   ├──────────────▶  finished block
//        ├────────────────────────┤   │  + full weld
//        │   sub-block C          │   ┘
//        └────────────────────────┘
//
// Signature pattern:
//   - pattern.kind             = "block_assembly"
//   - pattern.block_id         = <input>
//   - pattern.mass_t           = <input>     // metric tons of finished block
//   - pattern.weld_meters      = <input>     // total weld seam length, m
//   - pattern.geometry_changed = false
//
// DFM checks:
//   block_id      non-empty                    (DFM-INPUT warning)
//   mass_t        > 0                          (DFM-INPUT error)
//   mass_t        ≤ 4000                       (DFM-BLOCK-MASS info — > 4000 t
//                                               grand-block, needs goliath crane)
//   weld_meters   ≥ 0                          (DFM-INPUT error if negative)
//
// Synthesis:
//   NO geometric change.  Clone shape + material and stamp the signature.
//
// Recognize:
//   Block assembly leaves no B-rep trace at the workpiece-block granularity
//   used by the planner; recognition is metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace block_assembly {

constexpr const char* kSkillId = "block_assembly";

struct Input
{
    std::string block_id     = "BLK-001";
    double      mass_t       = 200.0;     // metric tons
    double      weld_meters  = 500.0;     // total weld seam length, m
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace block_assembly
}  // namespace koocadcam::skill
