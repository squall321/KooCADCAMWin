#pragma once
// @lat: [[engine/skills#pvd_coat]]
//
// pvd_coat — Physical Vapor Deposition.  Hard, thin ceramic coating
// (typically 0.5 – 10 μm) deposited in vacuum on the target face.  Common
// compounds: TiN (gold), TiAlN (dark violet-grey), CrN (silver-grey),
// DLC (near-black diamond-like carbon).
//
//        ┌─────────────┐
//        │ ═══════════ │   ← PVD ceramic film (0.5 – 10 μm, very hard)
//        │             │
//        │  (tool /    │
//        │   substrate)│   ← workpiece (unchanged)
//        └─────────────┘
//
// Signature pattern:
//   - pattern.kind              = "pvd_coat"
//   - pattern.target_face_id
//   - pattern.coating_compound  = "TiN" | "TiAlN" | "CrN" | "DLC"
//   - pattern.thickness_um
//   - pattern.hardness_hv       (reported; warned if outside compound range)
//   - geometry IDENTICAL to input
//
// DFM checks:
//   thickness_um ∈ [0.5, 10]              (DFM-PVD-THK error if outside)
//   coating_compound known                 (DFM-PVD-COMPOUND info if unknown)
//   hardness_hv in compound's expected range  (DFM-PVD-HARDNESS info)
//
// Synthesis:
//   NO geometric change.  Returns a clone with the signature stamped.
//
// Recognize:
//   Metadata-only replay.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace pvd_coat {

constexpr const char* kSkillId = "pvd_coat";

struct Input
{
    FaceDatum   entry_face;
    std::string coating_compound = "TiN";    // "TiN" | "TiAlN" | "CrN" | "DLC"
    double      thickness_um     = 3.0;      // typical cutting-tool PVD
    double      hardness_hv      = 0.0;      // 0 → use compound midpoint
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace pvd_coat
}  // namespace koocadcam::skill
