#pragma once
// @lat: [[engine/skills#blind_threaded_insert_seat]]
//
// blind_threaded_insert_seat — COMPOUND skill that prepares a blind hole
// for a self-clinching / press-in / glue-in threaded insert (Avibank-style
// pem-nut, Tappex Trisert, Big-Head etc.).  Chains:
//
//   sub-feature 1: drill_pilot (insert OD + light press fit)
//   sub-feature 2: counterbore-style head seat (slightly larger dia for the
//                  head/flange of the insert) at a precise depth
//   sub-feature 3: small 45° chamfer at entry so the insert seats cleanly
//   sub-feature 4: bottom relief well (slightly deeper, slightly wider) so
//                  binder/epoxy excess and any insert tail have room.
//
// The "binder excess" relief is what distinguishes this from a plain
// counterbore — it's specifically for adhesive-style inserts (Tappex Trisert,
// Plasti-Inserts) where epoxy is dispensed first and the insert is pushed in,
// displacing excess binder into a designed reservoir.
//
// Material-specific table (insert OD × head dia × engagement depth):
//
//   insert_size | insert OD | head OD | engagement depth | binder reservoir
//   ------------+-----------+---------+------------------+-----------------
//   M2          | 3.5       | 4.5     | 4.0              | 0.5
//   M2.5        | 4.0       | 5.0     | 4.5              | 0.5
//   M3          | 4.5       | 5.5     | 5.5              | 0.7
//   M4          | 5.5       | 7.0     | 7.0              | 0.8
//   M5          | 7.0       | 9.0     | 8.5              | 1.0
//   M6          | 9.0       | 11.0    | 10.0             | 1.2
//   M8          | 11.0      | 13.5    | 13.0             | 1.5
//
// Material modifier:
//   insert_material = "thermoplastic"  → +0.1 mm radial slip
//   insert_material = "thermoset"      → +0.2 mm radial slip
//   insert_material = "metal"          → +0.0 mm (interference fit)
//   insert_material = "composite_fbg"  → +0.3 mm + 50% deeper binder reservoir
//
// DFM:
//   DFM-002              : derived pilot < 0.8 mm
//   DFM-INSERT-SIZE      : insert_size unknown
//   DFM-INSERT-MATERIAL  : insert_material not in known set
//   DFM-INSERT-THICKNESS : stock too thin
//
// Recognize: metadata replay, confidence 1.0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace blind_threaded_insert_seat {

constexpr const char* kSkillId = "blind_threaded_insert_seat";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir       { 0.0, 0.0, -1.0 };
    std::string insert_size;              // "M2".."M8"
    std::string insert_material = "metal"; // metal | thermoplastic | thermoset | composite_fbg
};

double insertOuterDiameterFor (const std::string& size);
double insertHeadDiameterFor  (const std::string& size);
double insertEngagementFor    (const std::string& size);
double binderReservoirFor     (const std::string& size);
double materialSlipFor        (const std::string& material);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace blind_threaded_insert_seat
}  // namespace koocadcam::skill
