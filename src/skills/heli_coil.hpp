#pragma once
// @lat: [[engine/skills#heli_coil]]
//
// heli_coil — a specific kind of threaded insert: a stainless-steel wire
// coiled into the form of a 60° thread.  Common brand: Heli-Coil®.
//
//                       ┌─ entry_face (planar)
//                       ▼
//                 ──────┬──────────┬──────
//                       │ helical  │
//                       │ wire     │       ↑ insert_length_mm
//                       │ coil     │       │  (≥ 1.5 × thread dia)
//                       │  ⨯       │       │
//                       │          │       ↓
//                       └──────────┘
//                       ↑
//               install pilot (same as threaded_insert)
//
// Slice-1: B-rep geometry is IDENTICAL to threaded_insert — only the
// signature.skill_id / pattern.kind and a few coil-specific metadata fields
// differ.  The wire helix itself is not modelled in geometry.
//
// Differences from threaded_insert:
//   - coil_pitch_mm   : matches the destination thread's pitch (ISO 68-1 coarse)
//   - coil_count      : how many coils (turns) of wire; 1 ≤ count ≤ 3
//   - signature kind  : "heli_coil" instead of "threaded_insert"
//
// DFM checks:
//   DFM-HC-SIZE  : thread_size in supported set (M3/M4/M5/M6/M8)
//   DFM-HC-LEN   : insert_length_mm ≥ 1.5 × thread dia
//   DFM-HC-PITCH : coil_pitch_mm > 0
//   DFM-HC-COUNT : coil_count ∈ [1, 3]
//
// Recognize:
//   Metadata-driven (history replay) for high confidence.  Topology
//   fallback matches the install-pilot diameter and TAGS as heli_coil
//   with low confidence (cannot distinguish from threaded_insert by
//   geometry alone).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace heli_coil {

constexpr const char* kSkillId = "heli_coil";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm     = 0.0;
    double      position_y_mm     = 0.0;
    gp_Dir      axis_dir          { 0.0, 0.0, -1.0 };
    std::string thread_size;                       // "M3" | "M4" | "M5" | "M6" | "M8"
    double      insert_length_mm  = 0.0;
    std::string insert_material   = "stainless_316";
    double      coil_pitch_mm     = 0.0;            // = ISO coarse pitch for thread_size
    int         coil_count        = 1;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

// Default ISO-68 coarse pitch (mm) for the supported sizes.  Returns 0.0
// if unknown.  Used by validate() to fill in a sensible default when the
// caller passes coil_pitch_mm == 0.
double defaultCoarsePitch(const std::string& thread_size);

}  // namespace heli_coil
}  // namespace koocadcam::skill
