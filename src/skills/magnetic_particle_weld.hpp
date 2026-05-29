#pragma once
// @lat: [[engine/skills#magnetic_particle_weld]]
//
// magnetic_particle_weld — magnetic-particle inspection (MT) of a ferromag-
// netic weld, per AWS D1.x acceptance.  Metadata-only.
//
//   yoke ▮▮       ─ B field ─       ▮▮ yoke
//        N ──────────────────────────S
//                   weld bead
//        iron particles cluster at flux leakage = surface / near-surface crack
//
// Pattern:
//   pattern.kind             = "magnetic_particle_weld"
//   pattern.weld_id          = <input>
//   pattern.technique        = "wet" | "dry"
//   pattern.accept           = <input bool>
//   pattern.geometry_changed = false
//
// DFM:
//   DFM-INPUT          weld_id empty
//   DFM-NDT-TECHNIQUE  technique not in {wet, dry}
//   DFM-NDT-MATERIAL   workpiece material non-ferromagnetic (info)

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace magnetic_particle_weld {

constexpr const char* kSkillId = "magnetic_particle_weld";

struct Input
{
    std::string  weld_id;
    std::string  technique = "wet";    // "wet" | "dry"
    bool         accept    = true;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace magnetic_particle_weld
}  // namespace koocadcam::skill
