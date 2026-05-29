#pragma once
// @lat: [[engine/skills#pivot_pin_clevis_compound]]
//
// pivot_pin_clevis_compound — clevis joint pocket family.
//
// A clevis is a U-shaped slotted body with a cross-pin (the pivot) and a
// cotter-pin retention slot, used to attach a pivoting tongue to a fixed
// base.  Found in deployable mechanisms, hinge fittings, and aerospace
// control surfaces (MIL-S-5556).
//
// COMPOUND CHAIN (≥ 4 primitive cuts/fuses):
//   1. U-POCKET — rectangular slot through the lug body
//   2. CROSS-PIN BORE — perpendicular cylindrical bore through both ears
//   3. COTTER SLOT — transverse narrow slot intersecting the pin bore
//   4. BUSHING SEAT — counterbored shoulder at the bore entry
//
// Spec table (clevis_size → MIL-S-5556 dims):
//   "CL-04"   : u_width 4.0, pin_dia 4.0, ear_thk 3.0, cotter 1.0, seat 5.0
//   "CL-06"   : u_width 6.0, pin_dia 5.0, ear_thk 4.0, cotter 1.2, seat 7.0
//   "CL-08"   : u_width 8.0, pin_dia 6.0, ear_thk 5.0, cotter 1.6, seat 10.0
//   "CL-10"   : u_width 10.0, pin_dia 8.0, ear_thk 6.0, cotter 2.0, seat 12.0
//
// Signature pattern.kind = "pivot_pin_clevis_compound", is_compound = true,
//                  subfeature_count = 4, spec key = clevis_size.
//
// DFM gates:
//   DFM-EAR-THK   : ear_thickness ≥ pin_dia × 0.5 (shear strength)
//   DFM-002       : cotter_slot ≥ 0.8 mm
//   DFM-SPEC      : clevis_size must be in spec table

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace pivot_pin_clevis_compound {

constexpr const char* kSkillId = "pivot_pin_clevis_compound";

struct Input
{
    FaceDatum    entry_face;             // U-pocket opens against this face
    double       center_x_mm   = 0.0;
    double       center_y_mm   = 0.0;
    gp_Dir       u_axis        { 0.0, 0.0, -1.0 };  // pocket opens along this
    gp_Dir       pin_axis      { 0.0, 1.0,  0.0 };  // cross-pin axis (perpendicular)
    double       u_depth_mm    = 0.0;               // pocket depth into material
    std::string  clevis_size   = "CL-06";           // spec key
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

struct ClevisSpec {
    const char* key;
    double      u_width_mm;
    double      pin_dia_mm;
    double      ear_thk_mm;
    double      cotter_slot_mm;
    double      seat_dia_mm;
};
const ClevisSpec* lookupClevisSpec(const std::string& key);

}  // namespace pivot_pin_clevis_compound
}  // namespace koocadcam::skill
