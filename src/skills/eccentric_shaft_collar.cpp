// @lat: [[engine/skills#eccentric_shaft_collar]]

#include "eccentric_shaft_collar.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::eccentric_shaft_collar {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────
//
// References: DIN 705 (clamp / set-collar dimensions) + ISO 68-1 pilot
// diameters (mapped through standard M-series tap drills).

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.outer_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "eccentric_shaft_collar: outer_dia_mm must be > 0");
    }
    if (in.thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "eccentric_shaft_collar: thickness_mm must be > 0");
    }
    if (in.bore_dia_mm < 1.0) {
        r.add("DFM-002", "error",
              "eccentric_shaft_collar: bore_dia_mm " +
              std::to_string(in.bore_dia_mm) + " mm < min 1.0 mm");
    }
    if (in.eccentricity_mm < 0.0) {
        r.add("DFM-INPUT", "error",
              "eccentric_shaft_collar: eccentricity_mm must be >= 0");
    }
    // DIN 705 wall ratio: bore must be ≤ 0.6 of OD.
    if (in.outer_dia_mm > 0.0 && in.bore_dia_mm > 0.6 * in.outer_dia_mm) {
        r.add("DFM-COLLAR-WALL", "error",
              "eccentric_shaft_collar: bore_dia " +
              std::to_string(in.bore_dia_mm) +
              " > 0.6 × outer_dia (" + std::to_string(in.outer_dia_mm) +
              ") — wall too thin per DIN 705");
    }
    // Off-axis must keep clearance to outer wall (≥ 0.05·OD).
    const double bMax = 0.45 * in.outer_dia_mm;
    if (in.bore_dia_mm / 2.0 + in.eccentricity_mm > bMax) {
        r.add("DFM-COLLAR-ECC", "error",
              "eccentric_shaft_collar: bore_r + eccentricity " +
              std::to_string(in.bore_dia_mm / 2.0 + in.eccentricity_mm) +
              " > 0.45 × OD (" + std::to_string(bMax) +
              ") — bore would punch through outer wall");
    }
    if (in.set_screw_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "eccentric_shaft_collar: set_screw_dia_mm " +
              std::to_string(in.set_screw_dia_mm) + " mm < min 0.8 mm");
    }
    if (in.set_screw_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "eccentric_shaft_collar: set_screw_depth_mm must be > 0");
    }
    // Set screw radial penetration must not break into the eccentric bore.
    // Worst case: set screw goes from +Y outer wall, reaches inward by
    // depth.  Distance from outer wall along -Y to the bore wall is:
    //   (outer_dia/2) - sqrt(max(0, (bore_dia/2)^2 - eccentricity^2)) - eccentricity_y
    // We assume set screw on +Y (eccentricity is on +X), so worst-case clearance
    //   = outer_dia/2 - (bore_dia/2)   (when bore is at origin Y but eccentric in X)
    const double minClear = in.outer_dia_mm / 2.0 - in.bore_dia_mm / 2.0;
    if (in.set_screw_depth_mm > minClear - 0.5) {
        r.add("DFM-COLLAR-SETSCREW", "warning",
              "eccentric_shaft_collar: set_screw_depth " +
              std::to_string(in.set_screw_depth_mm) +
              " mm approaches bore wall (clearance ≈ " +
              std::to_string(minClear) + " mm)");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Chain:
//   step 0: stock comes in; we ADD the collar disc onto the workpiece (fuse).
//   step 1: CUT eccentric through bore — offset by +X eccentricity.
//   step 2: CUT radial set-screw hole — axis +Y, length = set_screw_depth.
//
// Volume removed (analytic):
//   V_bore       = π · (bore_r)² · thickness
//   V_setscrew   = π · (ss_r)²   · set_screw_depth
//   V_collar_add = π · (OD/2)²   · thickness
//   Net (collar formed in stock-vacuum) =
//     V_collar_add − V_bore − V_setscrew

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "eccentric_shaft_collar DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double overhang = 0.05;
    const double bottomZ  = in.center_z_mm;

    // ── step 0: ADD the collar cylinder via fuse ─────────────────────────
    const gp_Pnt collarOrigin(in.center_x_mm, in.center_y_mm, bottomZ);
    const gp_Ax2 collarAx(collarOrigin, gp::DZ());
    const TopoDS_Shape collarDisc = pr::cylinder(
        collarAx, in.outer_dia_mm / 2.0, in.thickness_mm);

    const TopoDS_Shape fused = pr::fuse(wp.shape(), collarDisc);

    // ── step 1: CUT eccentric through bore (+X offset by eccentricity) ──
    const gp_Pnt boreOrigin(
        in.center_x_mm + in.eccentricity_mm,
        in.center_y_mm,
        bottomZ - overhang);
    const gp_Ax2 boreAx(boreOrigin, gp::DZ());
    const TopoDS_Shape boreTool = pr::cylinder(
        boreAx, in.bore_dia_mm / 2.0, in.thickness_mm + 2.0 * overhang);

    const TopoDS_Shape afterBore = pr::cut(fused, boreTool);

    // ── step 2: CUT radial set-screw hole (entry from +Y outer wall) ────
    //
    // gp_Ax2(P, V, Vx): cylinder will extend along V from P.  We want the
    // hole to point INWARD from +Y (i.e., along -Y).  Place the tool start
    // outside the OD on +Y and direct -Y.
    const double ssStartY = in.center_y_mm + in.outer_dia_mm / 2.0 + overhang;
    const double ssCenterZ = bottomZ + in.thickness_mm / 2.0;
    const gp_Pnt ssOrigin(in.center_x_mm, ssStartY, ssCenterZ);
    // Local Z (axial) = -Y.  Local X (perp) = +X.
    const gp_Ax2 ssAx(ssOrigin, gp_Dir(0.0, -1.0, 0.0), gp_Dir(1.0, 0.0, 0.0));
    const TopoDS_Shape ssTool = pr::cylinder(
        ssAx, in.set_screw_dia_mm / 2.0, in.set_screw_depth_mm + overhang);

    const TopoDS_Shape newShape = pr::cut(afterBore, ssTool);

    // ── signature ────────────────────────────────────────────────────────
    json params = {
        { "outer_dia_mm",       in.outer_dia_mm },
        { "thickness_mm",       in.thickness_mm },
        { "bore_dia_mm",        in.bore_dia_mm },
        { "eccentricity_mm",    in.eccentricity_mm },
        { "set_screw_dia_mm",   in.set_screw_dia_mm },
        { "set_screw_depth_mm", in.set_screw_depth_mm },
        { "center_x_mm",        in.center_x_mm },
        { "center_y_mm",        in.center_y_mm },
        { "center_z_mm",        in.center_z_mm },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_compound",          true },
        { "subfeature_count",     3 },                  // disc + bore + set-screw
        { "cylindrical_face_count", 3 },                // outer + bore + set-screw
        { "axial_through_bores",  1 },
        { "radial_blind_holes",   1 },
        { "outer_dia_mm",         in.outer_dia_mm },
        { "bore_dia_mm",          in.bore_dia_mm },
        { "eccentricity_mm",      in.eccentricity_mm },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill;boring_bar;tap_drill";
    tooling.tool_dia_mm       = std::max(in.bore_dia_mm, in.set_screw_dia_mm);
    tooling.tool_length_mm    = in.thickness_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.05;

    const double rB = in.bore_dia_mm / 2.0;
    const double rS = in.set_screw_dia_mm / 2.0;
    const double rO = in.outer_dia_mm / 2.0;
    const double vCollarAdd = M_PI * rO * rO * in.thickness_mm;
    const double vBoreRemove = M_PI * rB * rB * in.thickness_mm;
    const double vSsRemove   = M_PI * rS * rS * in.set_screw_depth_mm;
    tooling.stock_removed_mm3 = vBoreRemove + vSsRemove;
    tooling.est_cycle_time_s  = std::max(2.0, in.thickness_mm / 20.0 + 4.0);
    tooling.extra = {
        { "added_material_mm3", vCollarAdd },
        { "tool_sequence", {
            { { "tool_type", "drill" },
              { "tool_dia_mm", in.bore_dia_mm },
              { "depth_mm",   in.thickness_mm },
              { "note",       "eccentric through bore" } },
            { { "tool_type", "drill" },
              { "tool_dia_mm", in.set_screw_dia_mm },
              { "depth_mm",    in.set_screw_depth_mm },
              { "note",        "radial set-screw pilot (M-series tap drill)" } },
        } },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::eccentric_shaft_collar applied: OD={} bore={} ecc={} faces {}→{}",
                  in.outer_dia_mm, in.bore_dia_mm, in.eccentricity_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// PRIMARY (geometric): scan cylindrical faces, find:
//   • an "outer" cyl: largest radius, axis ≈ +Z, centered near workpiece XY centroid
//   • an "eccentric bore": smaller radius, axis ≈ +Z, centre offset from outer
//   • a "set-screw radial": smaller radius, axis ≈ horizontal (perp to +Z)
//
// FALLBACK (metadata): scan feature history for kSkillId.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // ── metadata fallback first (cheapest, exact) ────────────────────────
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "feature_history" } };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // ── geometric path ───────────────────────────────────────────────────
    struct CylInfo {
        int    faceIdx = -1;
        double radius  = 0.0;
        gp_Pnt loc;
        gp_Dir dir;
    };
    std::vector<CylInfo> cyls;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        const gp_Cylinder cy = surf.Cylinder();
        CylInfo c;
        c.faceIdx = i;
        c.radius  = cy.Radius();
        c.loc     = cy.Axis().Location();
        c.dir     = cy.Axis().Direction();
        cyls.push_back(c);
    }
    if (cyls.size() < 3) return out;

    // Find the largest axial (Z) cyl = outer.
    int outerIdx = -1;
    double maxR = 0.0;
    for (size_t i = 0; i < cyls.size(); ++i) {
        if (std::abs(std::abs(cyls[i].dir.Z()) - 1.0) > 1e-2) continue;
        if (cyls[i].radius > maxR) { maxR = cyls[i].radius; outerIdx = static_cast<int>(i); }
    }
    if (outerIdx < 0) return out;

    // The eccentric bore: another +Z cyl with smaller radius and OFFSET centre.
    int boreIdx = -1;
    double bestEcc = 0.0;
    for (size_t i = 0; i < cyls.size(); ++i) {
        if (static_cast<int>(i) == outerIdx) continue;
        if (std::abs(std::abs(cyls[i].dir.Z()) - 1.0) > 1e-2) continue;
        if (cyls[i].radius >= cyls[outerIdx].radius) continue;
        const double dx = cyls[i].loc.X() - cyls[outerIdx].loc.X();
        const double dy = cyls[i].loc.Y() - cyls[outerIdx].loc.Y();
        const double d  = std::sqrt(dx*dx + dy*dy);
        if (d > bestEcc) { bestEcc = d; boreIdx = static_cast<int>(i); }
    }
    if (boreIdx < 0) return out;

    // The set-screw hole: a cyl with axis perpendicular to +Z (Z component ≈ 0).
    int ssIdx = -1;
    for (size_t i = 0; i < cyls.size(); ++i) {
        if (static_cast<int>(i) == outerIdx) continue;
        if (static_cast<int>(i) == boreIdx)  continue;
        if (std::abs(cyls[i].dir.Z()) > 1e-2) continue;  // not radial
        if (cyls[i].radius < cyls[boreIdx].radius) {
            ssIdx = static_cast<int>(i);
            break;
        }
    }
    // The set-screw is optional in recognition — accept partial match.
    double conf = 0.6;
    if (ssIdx >= 0) conf = 0.85;

    json recovered = {
        { "outer_dia_mm",       2.0 * cyls[outerIdx].radius },
        { "bore_dia_mm",        2.0 * cyls[boreIdx].radius },
        { "eccentricity_mm",    bestEcc },
        { "center_x_mm",        cyls[outerIdx].loc.X() },
        { "center_y_mm",        cyls[outerIdx].loc.Y() },
        { "center_z_mm",        cyls[outerIdx].loc.Z() },
        { "set_screw_dia_mm",   ssIdx >= 0 ? 2.0 * cyls[ssIdx].radius : 0.0 },
        { "set_screw_depth_mm", ssIdx >= 0 ? 4.0 : 0.0 },  // can't recover length
    };
    json matched = {
        { "outer_cyl_face_id", cyls[outerIdx].faceIdx },
        { "bore_cyl_face_id",  cyls[boreIdx].faceIdx },
        { "set_screw_face_id", ssIdx >= 0 ? cyls[ssIdx].faceIdx : -1 },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, conf, matched });
    return out;
}

}  // namespace koocadcam::skill::eccentric_shaft_collar
