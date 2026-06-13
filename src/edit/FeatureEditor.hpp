#pragma once
// @lat: [[engine/reverse-route#feature-editor]]
//
// FeatureEditor — defeature-based in-place geometry editing (plan B4.1).
//
// The old edit path changed a hole by Boolean-fusing fill material back in
// (annular ring + far_center heuristic) — fragile on real geometry and the
// reason "move" was reverted (a mis-recovered fill length once produced a
// 1046 mm tube).  This layer replaces it structurally:
//
//   1. DEFEATURE — BRepAlgoAPI_Defeaturing removes the hole's faces and
//      HEALS the solid (BOPAlgo_RemoveFeatures extends the surrounding
//      faces).  The hole is gone; the material is back; no fill geometry
//      was ever constructed.
//   2. RECUT    — cut the new hole (any diameter, any position on the part)
//      as a plain coaxial cylinder.
//
// Enlarge / shrink / move thereby all become MATERIAL-REMOVAL operations,
// which OCCT Booleans handle robustly — including on multi-solid
// assemblies, where fusing was the failure mode.
//
// Phase 1 scope: single-cylinder hole family (drill_hole, bore_cylindrical,
// mill_circular_pocket, drill_through_hole).  Compound features
// (counterbore, bore_with_shelf) follow once their multi-face removal sets
// are wired (plan B4.2).

#include <TopoDS_Shape.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json_fwd.hpp>

#include <optional>
#include <string>

namespace koocadcam::edit {

// A plain cylindrical hole, as recovered by the precise recognizers.
struct HoleSpec
{
    gp_Pnt entry;              // entry point on the part surface
    gp_Dir axis{ 0, 0, -1 };   // drilling direction (into the material)
    double diameter_mm = 0.0;
    double depth_mm    = 0.0;  // ignored when through
    bool   through     = false;
};

// Build a HoleSpec from a recognized candidate's recovered_params
// (position_x/y/z_mm, axis_dir, diameter_mm, depth_mm, through_hole).
// Returns nullopt when required fields are missing.
std::optional<HoleSpec> holeSpecFromRecovered(const nlohmann::json& recovered);

// Remove the hole's faces (cylindrical wall(s) matched by axis+radius, plus
// a blind bottom face if present) and heal the solid.  On failure returns a
// null shape and fills `err`.  The result has MORE volume than the input
// (material restored) — callers can sanity-check that.
TopoDS_Shape defeatureHole(const TopoDS_Shape& shape, const HoleSpec& hole,
                           std::string& err);

// Cut `hole` into the shape (coaxial cylinder; spans the whole part when
// through, entry+depth when blind).
TopoDS_Shape cutHole(const TopoDS_Shape& shape, const HoleSpec& hole);

// The full edit: defeature `oldHole`, then cut `newHole` on the healed
// solid.  Diameter, depth and position may all differ — enlarge, shrink and
// move are the same operation here.  Null shape + `err` on failure.
TopoDS_Shape editHole(const TopoDS_Shape& shape, const HoleSpec& oldHole,
                      const HoleSpec& newHole, std::string& err);

// ── Compound coaxial feature: counterbore / stepped bore (plan B4.2) ───────
//
// A counterbore is a wide shallow SEAT over a narrow deep PILOT, sharing one
// axis.  The single-cylinder defeature can't undo it (two radii + a shoulder
// annulus + a blind bottom), so editing it needs a region defeature.
struct CounterboreSpec
{
    gp_Pnt entry;
    gp_Dir axis{ 0, 0, -1 };
    double pilot_dia_mm   = 0.0;
    double pilot_depth_mm = 0.0;
    double seat_dia_mm    = 0.0;
    double seat_depth_mm  = 0.0;
};

std::optional<CounterboreSpec>
counterboreSpecFromRecovered(const nlohmann::json& recovered);

// Remove EVERY face inside the coaxial region (entry → axialDepth, radius ≤
// maxRadius): both cylinder walls, the step annulus and the blind bottom.
// Heals the solid (material restored).  Null + err on failure.
TopoDS_Shape defeatureCoaxialRegion(const TopoDS_Shape& shape,
                                    const gp_Pnt& entry, const gp_Dir& axis,
                                    double maxRadius, double axialDepth,
                                    std::string& err);

// Cut a counterbore: SEAT cylinder then PILOT cylinder, sequentially (never
// one compound Boolean — overlapping coaxial cutters are the OCCT 8.0 wipe
// trap, learned in bore_with_shelf).
TopoDS_Shape cutCounterbore(const TopoDS_Shape& shape,
                            const CounterboreSpec& cb);

// Defeature `oldCb`'s region, then cut `newCb` — resize and/or move a
// counterbore as one material-removal edit.  Null + err on failure.
TopoDS_Shape editCounterbore(const TopoDS_Shape& shape,
                             const CounterboreSpec& oldCb,
                             const CounterboreSpec& newCb, std::string& err);

}  // namespace koocadcam::edit
