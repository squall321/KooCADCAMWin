#pragma once
// @lat: [[engine/skills#c_mount_lens_thread]]
//
// c_mount_lens_thread — OPTICAL compound: C-mount / CS-mount / M42 / M52
// lens-mount thread interface.
//
// Sub-features (3 ops):
//   1. cut cylindrical pilot bore (thread minor dia)             (CUT)
//   2. cut cosmetic thread band (annular ring at pilot, shallow) (CUT)
//   3. cut flange recess (planar pad at standard flange dist)    (CUT)
//
// Standards:
//   "C"   : 25.4 mm OD, 32 TPI (1"-32 UN), flange dist 17.526 mm
//   "CS"  : 25.4 mm OD, 32 TPI (1"-32 UN), flange dist 12.5 mm
//   "M42" : 42 mm OD, M42×1
//   "M52" : 52 mm OD, M52×0.75
//
// DFM:
//   DFM-INPUT       : face_id valid
//   DFM-MOUNT-STYLE : mount_style ∈ {C, CS, M42, M52}
//   DFM-FACE        : face must be planar

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>

namespace koocadcam::skill {

class Workpiece;

namespace c_mount_lens_thread {

constexpr const char* kSkillId = "c_mount_lens_thread";

struct Input
{
    int         face_id     = -1;
    double      center_x_mm = 0.0;
    double      center_y_mm = 0.0;
    std::string mount_style = "C";   // "C" | "CS" | "M42" | "M52"
};

struct MountSpec {
    double od_mm;
    double pitch_mm;
    double flange_distance_mm;
    double pilot_dia_mm;   // thread minor diameter (cosmetic pilot bore)
};

bool lookupMount(const std::string& style, MountSpec& out);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace c_mount_lens_thread
}  // namespace koocadcam::skill
