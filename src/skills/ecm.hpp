#pragma once
// @lat: [[engine/skills#ecm]]
//
// ecm — Electrochemical Machining.
//
// ECM dissolves metal ATOM-BY-ATOM by Faradaic anodic reaction between a
// shaped cathode tool and the conductive workpiece in a flowing electrolyte
// stream.  The resulting cavity is geometrically similar to sinker EDM
// (electrode-shaped pocket) but with two characteristic differences:
//
//   1. NO heat-affected zone (cold process — chemistry, not sparks).
//   2. Internal corners are NATURALLY ROUNDED by the electrolyte flow field
//      (the current density is highest where the gap is smallest, and the
//      gap is constrained to several hundred microns), producing a fillet
//      on the bottom edge of the cavity.  We model this as a cylinder cut
//      followed by `BRepFilletAPI_MakeFillet` on the bottom rim.
//
// Geometry:
//
//                       entry_face (planar)
//                            │
//                ────────────┼──────────────
//                            │     ↓ axis
//                  ╔═════════╧═════════╗   ↑ depth_mm
//                  ║                   ║   │
//                  ║   (rounded bot    ║   │
//                  ║    by fillet R)   ║   │
//                  ╚═══════════════════╝   ↓
//                       ↑ corner_r_mm (default 0.5 mm)
//
// Signature pattern:
//   - 1 cylindrical face (wall)
//   - 1 toroidal face (bottom-rim fillet)
//   - 1 planar face (flat bottom disc)
//   - 2 circular edges (top rim, ring between torus & disc)
//
// DFM checks:
//   DFM-INPUT       diameter ≤ 0, depth ≤ 0
//   DFM-ECM-DIA     diameter < 1.0 mm  (electrolyte flow becomes unstable)
//   DFM-ECM-DEPTH   depth > 25 mm     (warning — flush pump capacity)
//   DFM-ECM-CORNER  corner_r_mm > diameter/2  (geometry collapses) → error
//
// Recognize:
//   Metadata-replay primary path: a synthesised cylinder + bottom torus is
//   ambiguous with mill_circular_pocket + corner fillet.  We only emit a
//   high-confidence candidate when wp.features() contains a prior ECM
//   signature; otherwise zero.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace ecm {

constexpr const char* kSkillId = "ecm";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir       { 0.0, 0.0, -1.0 };
    double      diameter_mm    = 0.0;
    double      depth_mm       = 0.0;
    double      corner_r_mm    = 0.5;   // fillet on the bottom rim
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace ecm
}  // namespace koocadcam::skill
