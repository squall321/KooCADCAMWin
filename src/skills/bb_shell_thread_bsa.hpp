#pragma once
// @lat: [[engine/skills#bb_shell_thread_bsa]]
//
// bb_shell_thread_bsa — Bicycle bottom-bracket shell threaded bore
// (BSA / English thread, 1.37in x 24 TPI, ~34.8 mm major dia).
//
// The BSA standard threaded BB shell is the classic English-thread interface
// for bicycle bottom brackets.  The shell is a short cylindrical tube whose
// inner bore is threaded from both ends (drive side is left-hand, non-drive
// right-hand, but geometrically both are the same major bore here).  At each
// end a shallow thread-relief annular groove is undercut so the tap can run
// clean to the shoulder.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. central through bore (major-dia cylinder cut along the axis)
//   2. drive-side thread-relief annular groove (annular ring)
//   3. non-drive-side thread-relief annular groove (annular ring)
//
// subfeature_count = 3.
//
// DFM:
//   DFM-INPUT     : every dimension must be > 0.
//   DFM-SHELL     : shell_width_mm must be a standard 68 or 73 mm BSA shell.
//   DFM-BORE      : shell_bore_dia_mm must straddle the BSA 34.8 mm major.
//   DFM-STOCK     : bore + relief must fit inside the stock bounding box.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace bb_shell_thread_bsa {

constexpr const char* kSkillId = "bb_shell_thread_bsa";

struct Input
{
    FaceDatum face_id;
    gp_Pnt    axis_origin           { 0.0, 0.0, 0.0 };  // shell axis entry (top end)
    double    shell_width_mm        = 68.0;             // 68 or 73 (BSA)
    double    shell_bore_dia_mm     = 34.8;             // BSA 1.37in major bore
    double    thread_relief_width_mm = 3.0;             // undercut groove width
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace bb_shell_thread_bsa
}  // namespace koocadcam::skill
