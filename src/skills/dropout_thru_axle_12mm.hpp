#pragma once
// @lat: [[engine/skills#dropout_thru_axle_12mm]]
//
// dropout_thru_axle_12mm — Rear dropout thru-axle bore (12 mm) with a
// threaded engagement section and an integrated derailleur-hanger bolt hole.
//
// Modern rear dropouts use a 12 mm thru-axle.  The non-drive dropout has a
// plain 12 mm clearance bore; the drive dropout is threaded (commonly
// M12 x 1.5) so the axle threads home.  A small derailleur-hanger bolt hole
// sits an offset below the axle bore to fasten the replaceable hanger.
//
// Geometry (the two coaxial sections are STACKED along the axis at different
// Z ranges — they do NOT overlap radially-inside, so each is a clean
// sequential cut):
//   1. 12 mm thru-axle clearance bore (upper section of the axle line)
//   2. threaded engagement bore (tap pilot dia, lower section, coaxial,
//      below the clearance bore)
//   3. derailleur-hanger bolt hole (offset below, parallel axis)
//
// subfeature_count = 3.
//
// DFM:
//   DFM-INPUT     : every dimension must be > 0.
//   DFM-AXLE      : axle_dia_mm must be the 12 mm thru-axle standard.
//   DFM-THREAD    : thread_key must exist in the central ISO M-thread table.
//   DFM-STOCK     : the hanger hole offset must stay inside the stock.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace dropout_thru_axle_12mm {

constexpr const char* kSkillId = "dropout_thru_axle_12mm";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      center_xy        { 0.0, 0.0, 0.0 };  // thru-axle bore XY
    double      axle_dia_mm      = 12.0;             // 12 mm thru-axle
    std::string thread_key       = "M12";           // central ISO M-thread key
    double      hanger_bolt_dia_mm = 4.0;            // derailleur-hanger bolt
    double      hanger_offset_mm = 16.0;             // offset below axle bore
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace dropout_thru_axle_12mm
}  // namespace koocadcam::skill
