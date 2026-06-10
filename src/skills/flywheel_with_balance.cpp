// @lat: [[engine/skills#flywheel_with_balance]]

#include "flywheel_with_balance.hpp"

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
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::flywheel_with_balance {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────
//
// Standards cited:
//   ISO 1940-1: rotor balance quality (drives the need for balance drills)
//   AGMA 2015: minimum hub-OD ratio for flywheels

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.outer_dia_mm < 20.0) {
        r.add("DFM-FLYWHEEL-SIZE", "error",
              "flywheel_with_balance: outer_dia " +
              std::to_string(in.outer_dia_mm) +
              " < 20 mm — too small to be a productive flywheel (ISO 1940-1)");
    }
    if (in.thickness_mm < 3.0) {
        r.add("DFM-FLYWHEEL-SIZE", "error",
              "flywheel_with_balance: thickness " +
              std::to_string(in.thickness_mm) + " < 3 mm — too thin for balance drills");
    }
    if (in.hub_bore_dia_mm < 1.0) {
        r.add("DFM-002", "error",
              "flywheel_with_balance: hub_bore_dia " +
              std::to_string(in.hub_bore_dia_mm) + " < min 1 mm");
    }
    // AGMA 2015 hub-OD ratio.
    if (in.outer_dia_mm > 0.0 && in.hub_bore_dia_mm > 0.6 * in.outer_dia_mm) {
        r.add("DFM-FLYWHEEL-HUB", "error",
              "flywheel_with_balance: hub_bore (" +
              std::to_string(in.hub_bore_dia_mm) +
              ") > 0.6 × OD (" + std::to_string(in.outer_dia_mm) +
              ") — hub wall too thin (AGMA 2015)");
    }

    if (in.balance_drill_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "flywheel_with_balance: balance_drill_dia " +
              std::to_string(in.balance_drill_dia_mm) + " < 0.8 mm");
    }
    if (in.balance_drill_depth_mm > 0.9 * in.thickness_mm) {
        r.add("DFM-FLYWHEEL-DEPTH", "error",
              "flywheel_with_balance: balance_drill_depth " +
              std::to_string(in.balance_drill_depth_mm) +
              " > 0.9 × thickness — drill would punch through during balancing");
    }
    if (in.balance_count < 3) {
        r.add("DFM-FLYWHEEL-COUNT", "error",
              "flywheel_with_balance: balance_count " +
              std::to_string(in.balance_count) +
              " < 3 — at least 3 symmetric positions needed (ISO 1940-1)");
    }

    // PCD must fit between hub OD and outer OD with drill diameter margin.
    const double minPcd = in.hub_bore_dia_mm + 2.0 * in.balance_drill_dia_mm;
    const double maxPcd = in.outer_dia_mm   - 2.0 * in.balance_drill_dia_mm;
    if (in.balance_pcd_mm < minPcd || in.balance_pcd_mm > maxPcd) {
        r.add("DFM-FLYWHEEL-PCD", "error",
              "flywheel_with_balance: balance_pcd " +
              std::to_string(in.balance_pcd_mm) + " not in [" +
              std::to_string(minPcd) + ", " + std::to_string(maxPcd) +
              "] — balance drills would touch hub or rim");
    }

    if (in.chamfer_mm < 0.0 || in.chamfer_mm > in.thickness_mm / 2.0) {
        r.add("DFM-FLYWHEEL-CHAMFER", "error",
              "flywheel_with_balance: chamfer_mm " +
              std::to_string(in.chamfer_mm) +
              " not in [0, thickness/2] — would break the rim or be missing");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "flywheel_with_balance DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double overhang = 0.05;
    const double bottomZ  = in.center_z_mm;
    const double topZ     = bottomZ + in.thickness_mm;

    // ── step 1: ADD flywheel disc body via fuse ──────────────────────────
    const gp_Pnt discOrigin(in.center_x_mm, in.center_y_mm, bottomZ);
    const gp_Ax2 discAx(discOrigin, gp::DZ());
    const TopoDS_Shape disc = pr::cylinder(discAx, in.outer_dia_mm / 2.0,
                                           in.thickness_mm);
    const TopoDS_Shape fused = pr::fuse(wp.shape(), disc);

    // ── step 2: CUT central hub through-bore ─────────────────────────────
    const gp_Pnt hubOrigin(in.center_x_mm, in.center_y_mm, bottomZ - overhang);
    const gp_Ax2 hubAx(hubOrigin, gp::DZ());
    const TopoDS_Shape hubTool = pr::cylinder(
        hubAx, in.hub_bore_dia_mm / 2.0, in.thickness_mm + 2.0 * overhang);

    // ── step 3: build N balance drill cylinders, fuse into a single tool ─
    const double rPcd = in.balance_pcd_mm / 2.0;
    const double rBd  = in.balance_drill_dia_mm / 2.0;

    TopoDS_Shape drillTool;
    bool drillToolInit = false;
    for (int i = 0; i < in.balance_count; ++i) {
        const double th = 2.0 * M_PI * static_cast<double>(i) /
                          static_cast<double>(in.balance_count);
        const double px = in.center_x_mm + rPcd * std::cos(th);
        const double py = in.center_y_mm + rPcd * std::sin(th);
        // Drill goes from top face into the disc (axis -Z, but we cylinder
        // primitive extends along +Z from base; we'll place base BELOW the
        // entry point by the depth amount).
        const gp_Pnt dOrigin(px, py, topZ - in.balance_drill_depth_mm);
        const gp_Ax2 dAx(dOrigin, gp::DZ());
        const TopoDS_Shape oneDrill = pr::cylinder(
            dAx, rBd, in.balance_drill_depth_mm + overhang);
        if (!drillToolInit) {
            drillTool = oneDrill;
            drillToolInit = true;
        } else {
            drillTool = pr::fuse(drillTool, oneDrill);
        }
    }

    // Combine hub + balance drills as a single fused cutter, then ONE cut.
    TopoDS_Shape combinedCut = hubTool;
    if (drillToolInit) {
        combinedCut = pr::fuse(combinedCut, drillTool);
    }

    TopoDS_Shape afterDrills = pr::cut(fused, combinedCut);

    // ── step 4: CUT outer rim chamfer (45° annular cone-frustum) ─────────
    //
    // The chamfer ring sits on the TOP rim, from outerR back to (outerR - chamfer_mm),
    // axially from (topZ - chamfer_mm) to topZ.  We use a cone frustum cone
    // subtraction so that the resulting outer rim has a 45° break.
    TopoDS_Shape newShape = afterDrills;
    if (in.chamfer_mm > 1e-6) {
        const double rOuter = in.outer_dia_mm / 2.0;
        const double rInner = rOuter - in.chamfer_mm;  // inner radius at top of chamfer
        // Cone frustum: bottom (z = topZ - chamfer_mm) has radius = rOuter + overhang,
        //               top    (z = topZ + overhang)    has radius = rInner.
        // Cutting this conical RING removes the rim corner at 45°.
        // We achieve this with a small "cone shell": a cone frustum minus
        // the central cylinder.  Cone params: r1=rOuter+overhang at bottom,
        // r2=rInner at top, height=chamfer_mm+overhang.
        const gp_Pnt coneOrigin(in.center_x_mm, in.center_y_mm,
                                topZ - in.chamfer_mm);
        const gp_Ax2 coneAx(coneOrigin, gp::DZ());
        // We need to cut the OUTSIDE only. Build an annular cone ring with
        // inner straight cylinder = rInner so the cone subtraction touches
        // the rim corner cleanly.
        try {
            const TopoDS_Shape chamferTool = pr::annularConeRing(
                coneAx,
                rOuter + 0.5,   // outer bottom slightly beyond OD (safety)
                rInner,         // outer top = rInner (the chamfer tip)
                rInner - 0.1,   // inner straight cylinder (just below rInner)
                in.chamfer_mm + overhang);
            newShape = pr::cut(newShape, chamferTool);
        } catch (const std::exception& ex) {
            spdlog::debug("flywheel_with_balance: chamfer cut skipped — {}", ex.what());
            // keep newShape as-is; chamfer is optional
        }
    }

    // ── signature ────────────────────────────────────────────────────────
    json balancePositions = json::array();
    for (int i = 0; i < in.balance_count; ++i) {
        const double th = 2.0 * M_PI * static_cast<double>(i) /
                          static_cast<double>(in.balance_count);
        balancePositions.push_back({
            { "x", in.center_x_mm + rPcd * std::cos(th) },
            { "y", in.center_y_mm + rPcd * std::sin(th) },
            { "theta_rad", th },
        });
    }

    json params = {
        { "outer_dia_mm",           in.outer_dia_mm },
        { "thickness_mm",           in.thickness_mm },
        { "hub_bore_dia_mm",        in.hub_bore_dia_mm },
        { "balance_pcd_mm",         in.balance_pcd_mm },
        { "balance_drill_dia_mm",   in.balance_drill_dia_mm },
        { "balance_drill_depth_mm", in.balance_drill_depth_mm },
        { "balance_count",          in.balance_count },
        { "chamfer_mm",             in.chamfer_mm },
        { "center_x_mm",            in.center_x_mm },
        { "center_y_mm",            in.center_y_mm },
        { "center_z_mm",            in.center_z_mm },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "is_compound",            true },
        // disc + hub_bore + N drills + chamfer_ring
        { "subfeature_count",       2 + in.balance_count + (in.chamfer_mm > 0 ? 1 : 0) },
        { "cylindrical_face_count", 2 + in.balance_count },
        { "balance_drill_count",    in.balance_count },
        { "balance_positions",      balancePositions },
        { "outer_dia_mm",           in.outer_dia_mm },
        { "hub_bore_dia_mm",        in.hub_bore_dia_mm },
        { "balance_pcd_mm",         in.balance_pcd_mm },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill;chamfer_mill";
    tooling.tool_dia_mm       = in.balance_drill_dia_mm;
    tooling.tool_length_mm    = in.thickness_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 280.0;
    tooling.feed_per_tooth_mm = 0.05;
    const double rH = in.hub_bore_dia_mm / 2.0;
    const double vHub  = M_PI * rH * rH * in.thickness_mm;
    const double vBd   = M_PI * rBd * rBd * in.balance_drill_depth_mm *
                         static_cast<double>(in.balance_count);
    tooling.stock_removed_mm3 = vHub + vBd;
    tooling.est_cycle_time_s  = std::max(3.0,
        in.thickness_mm / 30.0 + in.balance_count * 1.5);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },
              { "tool_dia_mm", in.hub_bore_dia_mm },
              { "depth_mm",    in.thickness_mm },
              { "note",        "hub through-bore" } },
            { { "tool_type", "drill" },
              { "tool_dia_mm", in.balance_drill_dia_mm },
              { "depth_mm",    in.balance_drill_depth_mm },
              { "pass_count",  in.balance_count },
              { "note",        "balance drills at PCD" } },
            { { "tool_type", "chamfer_mill" },
              { "tool_dia_mm", in.chamfer_mm * 2.0 },
              { "note",        "outer rim 45° break" } },
        } },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::flywheel_with_balance applied: OD={} t={} drills={} faces {}→{}",
                  in.outer_dia_mm, in.thickness_mm, in.balance_count,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // metadata fallback
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

    // geometric path: collect axial cylinders, find outer + hub + drills.
    struct CylInfo {
        int faceIdx = -1;
        double radius = 0.0;
        gp_Pnt loc;
    };
    std::vector<CylInfo> axialCyls;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        const gp_Cylinder cy = surf.Cylinder();
        if (std::abs(std::abs(cy.Axis().Direction().Z()) - 1.0) > 1e-2) continue;
        CylInfo c{ i, cy.Radius(), cy.Axis().Location() };
        axialCyls.push_back(c);
    }
    if (axialCyls.size() < 3) return out;

    // Outer = largest radius. Hub = smaller, near same XY as outer. Drills =
    // small, OFF from outer's XY by approximately equal radial distance.
    int outerIdx = 0;
    for (size_t i = 1; i < axialCyls.size(); ++i)
        if (axialCyls[i].radius > axialCyls[outerIdx].radius) outerIdx = static_cast<int>(i);

    const gp_Pnt& outerLoc = axialCyls[outerIdx].loc;
    const double outerR = axialCyls[outerIdx].radius;

    int hubIdx = -1;
    double hubBestR = 1e30;
    for (size_t i = 0; i < axialCyls.size(); ++i) {
        if (static_cast<int>(i) == outerIdx) continue;
        const double dx = axialCyls[i].loc.X() - outerLoc.X();
        const double dy = axialCyls[i].loc.Y() - outerLoc.Y();
        const double d  = std::sqrt(dx*dx + dy*dy);
        if (d > outerR * 0.1) continue;            // not concentric
        if (axialCyls[i].radius < hubBestR) {
            hubBestR = axialCyls[i].radius;
            hubIdx   = static_cast<int>(i);
        }
    }
    if (hubIdx < 0) return out;

    // Balance drills: cylinders off-centre, small radius (< hub), grouped at
    // similar radial distance.
    std::vector<int> drillCands;
    double sumRadial = 0.0;
    double sumDia    = 0.0;
    for (size_t i = 0; i < axialCyls.size(); ++i) {
        if (static_cast<int>(i) == outerIdx) continue;
        if (static_cast<int>(i) == hubIdx)   continue;
        const double dx = axialCyls[i].loc.X() - outerLoc.X();
        const double dy = axialCyls[i].loc.Y() - outerLoc.Y();
        const double d  = std::sqrt(dx*dx + dy*dy);
        if (d < outerR * 0.1) continue;        // too close to centre
        if (d > outerR * 0.95) continue;       // too close to rim
        if (axialCyls[i].radius > axialCyls[hubIdx].radius) continue;
        drillCands.push_back(static_cast<int>(i));
        sumRadial += d;
        sumDia    += 2.0 * axialCyls[i].radius;
    }
    if (drillCands.size() < 3) return out;

    const double pcd = drillCands.empty() ? 0.0 :
                       (sumRadial / drillCands.size()) * 2.0;
    const double bDia = drillCands.empty() ? 0.0 :
                        sumDia / drillCands.size();

    double conf = 0.75;
    if (drillCands.size() >= 5) conf = 0.85;

    json recovered = {
        { "outer_dia_mm",           2.0 * outerR },
        { "thickness_mm",           0.0 },  // can't recover from cyl axes alone
        { "hub_bore_dia_mm",        2.0 * axialCyls[hubIdx].radius },
        { "balance_pcd_mm",         pcd },
        { "balance_drill_dia_mm",   bDia },
        { "balance_drill_depth_mm", 0.0 },
        { "balance_count",          static_cast<int>(drillCands.size()) },
        { "chamfer_mm",             0.0 },
        { "center_x_mm",            outerLoc.X() },
        { "center_y_mm",            outerLoc.Y() },
        { "center_z_mm",            outerLoc.Z() },
    };
    json matched = {
        { "outer_face_id",  axialCyls[outerIdx].faceIdx },
        { "hub_face_id",    axialCyls[hubIdx].faceIdx },
        { "drill_face_count", static_cast<int>(drillCands.size()) },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, conf, matched });
    return out;
}

}  // namespace koocadcam::skill::flywheel_with_balance
