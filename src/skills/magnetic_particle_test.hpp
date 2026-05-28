#pragma once
// @lat: [[engine/skills#magnetic_particle_test]]
//
// magnetic_particle_test (MT / MPI) — INSPECTION skill: detects surface
// and near-surface discontinuities by inducing a magnetic field in a
// FERROMAGNETIC workpiece, then applying ferromagnetic particles
// suspended in liquid (wet) or as a powder (dry).  Flaws create flux
// leakage that pulls particles into a visible indication.
//
// Geometrically a NO-OP.
//
//      yoke (DC / AC)               flaw → flux leakage
//         ━━━━━━                          ↑   ↑
//        ╱      ╲                         ↑   ↑
//       ●        ●  ─ flux ─▶  ─────╲╱╲──── ←── particles cluster
//        ╲      ╱                         here
//         ━━━━━━
//
// What we record:
//   - target face datum
//   - technique               — "wet" | "dry"
//   - severity_class          — "none" | "linear" | "rounded" | "cluster"
//   - surface_prep            — "as_machined" | "ground" | "polished"
//
// DFM check (BLOCKING for material):
//   MT only works on FERROMAGNETIC materials.  Non-ferrous (aluminum,
//   brass, titanium, austenitic stainless) → ERROR.  This is a real
//   physical limitation, not a soft-warning.
//
// DFM checks:
//   target face datum resolves
//   technique ∈ {wet, dry}
//   severity_class ∈ {none, linear, rounded, cluster}
//   surface_prep ∈ {as_machined, ground, polished}
//   material is ferromagnetic (error if not)
//
// Recognize:
//   Metadata-only operation — no geometric fingerprint.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace magnetic_particle_test {

constexpr const char* kSkillId = "magnetic_particle_test";

struct Input
{
    FaceDatum     target_face_datum;
    std::string   technique        = "wet";          // "wet" | "dry"
    std::string   severity_class   = "none";         // "none" | "linear" | "rounded" | "cluster"
    std::string   surface_prep     = "as_machined";  // "as_machined" | "ground" | "polished"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace magnetic_particle_test
}  // namespace koocadcam::skill
