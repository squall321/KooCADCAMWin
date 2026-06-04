#pragma once
// @lat: [[engine/skills#locking_compression_plate_hole]]
//
// locking_compression_plate_hole — COMPOUND LCP "combi-hole" feature:
//
//   sub-feature 1: oblong dynamic-compression slot (DCP-style)
//                  rectangle + 2 end half-cylinders (stadium shape)
//   sub-feature 2: threaded locking-screw hole at ONE end of the slot
//                  (represented as a plain cylindrical bore; thread is
//                  declared in the signature/tooling metadata — actual
//                  helical thread modelled by the tap_thread skill if
//                  used downstream).
//
// Combined "combi-hole" enables either compression-mode fixation (regular
// cortical screw slides in the oblong portion) OR locking-mode fixation
// (locking head-screw threads into the round end), and bridges the two —
// the foundation of the LCP (Locking Compression Plate, AO/Synthes).
//
// Standards reference:
//   AO/Synthes LCP design (Frigg 2003)
//   ISO 5836 (bone screws), ASTM F382 (bone-plate testing)
//
// DFM:
//   DFM-LCP-SLOT     : compression slot length < locking_thread_dia × 1.5
//   DFM-LCP-THICK    : plate_thickness < 0.5 × locking_thread_dia
//   DFM-LCP-PITCH    : locking_thread_pitch_mm outside [0.4, 2.0]
//
// signature.pattern.kind                  = "locking_compression_plate_hole"
// signature.pattern.is_compound           = true
// signature.pattern.medical_feature_type  = "lcp_combi_hole"

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace locking_compression_plate_hole {

constexpr const char* kSkillId = "locking_compression_plate_hole";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir       { 0.0, 0.0, -1.0 };
    gp_Dir      slot_dir       { 1.0, 0.0, 0.0 };   // long axis of the oblong slot
    double      dynamic_compression_slot_length_mm = 0.0;
    double      locking_thread_dia_mm   = 0.0;
    double      locking_thread_pitch_mm = 0.0;
    double      plate_thickness_mm      = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace locking_compression_plate_hole
}  // namespace koocadcam::skill
