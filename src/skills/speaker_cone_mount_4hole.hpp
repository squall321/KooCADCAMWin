#pragma once
// @lat: [[engine/skills#speaker_cone_mount_4hole]]
//
// speaker_cone_mount_4hole — speaker driver mounting pattern (4-bolt or 8-bolt).
//
// Slice 16 — audio / musical instrument hardware.
//
// Sub-features (sequential CUTs):
//   1. central cone opening (through-hole cylinder for the cone)
//   2..(1+N). N bolt clearance holes on a PCD (4 or 8 holes)
//   1 extra. optional rear bumper recess cylinder (counterbore) — only if
//            recess_dia_mm > 0.
//
// subfeature_count = 1 + bolt_count + (recess?1:0).
//
// Standard speaker sizes (mm OD reference): 100/130/165/200/250 (4"/5"/6.5"/
// 8"/10").  Bolt counts: 4 (typical) or 8 (high-power / pro audio).
//
// DFM gates:
//   DFM-INPUT          : positive dims
//   DFM-CONE-DIA       : cone_dia_mm in {100, 130, 165, 200, 250}
//   DFM-BOLT-COUNT     : bolt_count in {4, 8}
//   DFM-PCD            : pcd > cone_dia + 2 * bolt_hole_dia
//   DFM-THREAD-KEY     : bolt_thread_size_key must exist in central
//                        metric OR UNC/UNF table.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace speaker_cone_mount_4hole {

constexpr const char* kSkillId = "speaker_cone_mount_4hole";

struct Input
{
    FaceDatum   face_id;
    double      center_x_mm    = 0.0;
    double      center_y_mm    = 0.0;
    gp_Dir      axis_dir       { 0.0, 0.0, -1.0 };
    double      cone_dia_mm    = 165.0;   // 100/130/165/200/250
    int         mount_bolt_count = 4;      // 4 or 8
    double      mount_pcd_mm   = 190.0;
    std::string bolt_thread_size_key = "M5";
    double      recess_dia_mm  = 0.0;     // 0 → no rear bumper recess
    double      recess_depth_mm = 0.0;
    double      panel_thickness_mm = 18.0; // host panel thickness (for cone cut)
};

// Public for tests.
bool isStandardConeDia(double cone_dia_mm);

SkillOutput  apply   (const Workpiece& wp, const Input& in);
DFMReport    validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace speaker_cone_mount_4hole
}  // namespace koocadcam::skill
