#pragma once
// @lat: [[engine/skills#sprocket_blank_with_bore]]
//
// sprocket_blank_with_bore — COMPOUND macro that produces a
// finished sprocket blank from cylindrical stock:
//
//   sub-cuts (7):
//     1. central bore                                (Ø bore_dia)
//     2-5. four equispaced lightening holes          (45° offset, Ø derived)
//     6. top-face circumferential chamfer            (0.5 × 45°)
//     7. bottom-face circumferential chamfer         (0.5 × 45°)
//
// Tooth-cutting is NOT performed at this step (sprockets are hob-cut
// downstream).  This skill produces the BLANK that feeds the gear-hob.
//
// DFM gates (ANSI B29.1 sprocket blank dimensioning):
//   DFM-INPUT       basic positivity checks
//   DFM-002         bore_dia < 0.8 mm
//   DFM-SPROCKET-WALL  hub wall thickness < 3 mm
//   DFM-SPROCKET-LIGHT  lightening-hole would collide with bore or OD

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace sprocket_blank_with_bore {

constexpr const char* kSkillId = "sprocket_blank_with_bore";

struct Input
{
    double outer_dia_mm      = 0.0;
    double bore_dia_mm       = 0.0;
    double thickness_mm      = 0.0;
    double lightening_count  = 4;     // number of lightening holes (default 4)
    double chamfer_size_mm   = 0.5;   // top + bottom face chamfer (default 0.5 mm)
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace sprocket_blank_with_bore
}  // namespace koocadcam::skill
