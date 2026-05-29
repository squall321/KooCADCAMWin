// @lat: [[engine/skills#cam_follower_roller_seat]]

#include "cam_follower_roller_seat.hpp"

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

namespace koocadcam::skill::cam_follower_roller_seat {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────
//
// Standards: ANSI B92.1 (rolling-element seat clearance), ISO 286-2 (H8
// running fit for cam followers).

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.roller_dia_mm < 2.0) {
        r.add("DFM-FOLLOWER-SIZE", "error",
              "cam_follower_roller_seat: roller_dia " +
              std::to_string(in.roller_dia_mm) +
              " < 2 mm — smaller than the smallest std cam follower");
    }
    if (in.roller_length_mm < 2.0) {
        r.add("DFM-FOLLOWER-SIZE", "error",
              "cam_follower_roller_seat: roller_length " +
              std::to_string(in.roller_length_mm) + " < 2 mm");
    }
    if (in.radial_clearance_mm < 0.05 || in.radial_clearance_mm > 0.5) {
        r.add("DFM-FOLLOWER-FIT", "error",
              "cam_follower_roller_seat: radial_clearance " +
              std::to_string(in.radial_clearance_mm) +
              " outside ISO 286-2 H8 range [0.05, 0.5] mm");
    }
    if (in.arm_clearance_width_mm < 0.8) {
        r.add("DFM-002", "error",
              "cam_follower_roller_seat: arm_clearance_width " +
              std::to_string(in.arm_clearance_width_mm) + " < 0.8 mm");
    }
    if (in.arm_clearance_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "cam_follower_roller_seat: arm_clearance_depth must be > 0");
    }
    if (in.retention_dia_mm < 1.5) {
        r.add("DFM-002", "error",
              "cam_follower_roller_seat: retention_dia " +
              std::to_string(in.retention_dia_mm) +
              " < 1.5 mm — sub-M2");
    }
    if (in.retention_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "cam_follower_roller_seat: retention_depth must be > 0");
    }
    if (in.retention_dia_mm > 0.6 * in.roller_dia_mm) {
        r.add("DFM-FOLLOWER-AXLE", "error",
              "cam_follower_roller_seat: retention_dia (" +
              std::to_string(in.retention_dia_mm) +
              ") > 0.6 × roller_dia (" + std::to_string(in.roller_dia_mm) +
              ") — axle cannot fit through roller bore");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "cam_follower_roller_seat DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double overhang = 0.05;
    const double rPocket  = in.roller_dia_mm / 2.0 + in.radial_clearance_mm;
    const double lPocket  = in.roller_length_mm + 2.0 * in.radial_clearance_mm;

    // ── step 1: roller body pocket — cylinder axis +Y ───────────────────
    //
    // gp_Ax2(P, V, Vx) — V is the local Z (axial) of the cylinder.  We
    // want axial = +Y, so V = (0,1,0).  Vx (local X) = +X.
    const gp_Pnt rpStart(
        in.center_x_mm,
        in.center_y_mm - lPocket / 2.0 - overhang,
        in.center_z_mm);
    const gp_Ax2 rpAx(rpStart, gp_Dir(0.0, 1.0, 0.0), gp_Dir(1.0, 0.0, 0.0));
    const TopoDS_Shape rollerPocket = pr::cylinder(
        rpAx, rPocket, lPocket + 2.0 * overhang);

    // ── step 2: two arm clearance slots — boxes on +/-X sides ───────────
    //
    // Each slot is a box of size (arm_clearance_width × arm_clearance_depth ×
    // (something larger than lPocket)) sitting flush with the roller pocket
    // on its ±X side.  The arm swings through the +/-X opening.
    //
    // Box origin convention: box(ax, dx, dy, dz) — ax origin is the corner,
    // dx along axis X, dy along axis Y, dz along axis Z.
    const double slotLen = lPocket + 2.0 * overhang;
    const double slotW   = in.arm_clearance_width_mm;
    const double slotD   = in.arm_clearance_depth_mm;

    // Slot on +X side: origin at (center_x + rPocket, center_y - slotLen/2, center_z - slotW/2)
    const gp_Pnt slotPxOrigin(
        in.center_x_mm + rPocket,
        in.center_y_mm - slotLen / 2.0,
        in.center_z_mm - slotW / 2.0);
    const gp_Ax2 slotPxAx(slotPxOrigin, gp::DZ());     // V = +Z; gives us local X = +X
    const TopoDS_Shape slotPxBox = pr::box(slotPxAx, slotD, slotLen, slotW);

    // Slot on -X side: origin at (center_x - rPocket - slotD, …)
    const gp_Pnt slotNxOrigin(
        in.center_x_mm - rPocket - slotD,
        in.center_y_mm - slotLen / 2.0,
        in.center_z_mm - slotW / 2.0);
    const gp_Ax2 slotNxAx(slotNxOrigin, gp::DZ());
    const TopoDS_Shape slotNxBox = pr::box(slotNxAx, slotD, slotLen, slotW);

    // ── step 3: retention pilot bore — axis +Z, centered, full thickness ─
    const gp_Pnt retOrigin(in.center_x_mm, in.center_y_mm,
                           in.center_z_mm - in.retention_depth_mm / 2.0 - overhang);
    const gp_Ax2 retAx(retOrigin, gp::DZ());
    const TopoDS_Shape retTool = pr::cylinder(
        retAx, in.retention_dia_mm / 2.0,
        in.retention_depth_mm + 2.0 * overhang);

    // Fuse all 4 cutters into one compound tool, then a single cut.
    TopoDS_Shape allCut = pr::fuse(rollerPocket, slotPxBox);
    allCut = pr::fuse(allCut, slotNxBox);
    allCut = pr::fuse(allCut, retTool);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), allCut);

    // ── signature ────────────────────────────────────────────────────────
    json params = {
        { "roller_dia_mm",          in.roller_dia_mm },
        { "roller_length_mm",       in.roller_length_mm },
        { "radial_clearance_mm",    in.radial_clearance_mm },
        { "arm_clearance_width_mm", in.arm_clearance_width_mm },
        { "arm_clearance_depth_mm", in.arm_clearance_depth_mm },
        { "retention_dia_mm",       in.retention_dia_mm },
        { "retention_depth_mm",     in.retention_depth_mm },
        { "center_x_mm",            in.center_x_mm },
        { "center_y_mm",            in.center_y_mm },
        { "center_z_mm",            in.center_z_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    4 },   // pocket + 2 slots + retention bore
        { "cylindrical_face_count", 2 },   // roller pocket + retention bore
        { "rect_slot_count",     2 },
        { "roller_dia_mm",       in.roller_dia_mm },
        { "retention_dia_mm",    in.retention_dia_mm },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill;drill";
    tooling.tool_dia_mm       = std::max(rPocket * 2.0, in.retention_dia_mm);
    tooling.tool_length_mm    = lPocket + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 280.0;
    tooling.feed_per_tooth_mm = 0.04;
    const double vPocket = M_PI * rPocket * rPocket * lPocket;
    const double vSlot   = slotD * slotLen * slotW;
    const double vRet    = M_PI * (in.retention_dia_mm / 2.0)
                           * (in.retention_dia_mm / 2.0)
                           * in.retention_depth_mm;
    tooling.stock_removed_mm3 = vPocket + 2.0 * vSlot + vRet;
    tooling.est_cycle_time_s  = std::max(4.0,
        lPocket * 0.2 + in.retention_depth_mm * 0.15 + 2.0);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::cam_follower_roller_seat applied: roller={}x{} ret={} faces {}→{}",
                  in.roller_dia_mm, in.roller_length_mm, in.retention_dia_mm,
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

    // geometric path: find a Y-axis cylinder (roller pocket) and a Z-axis
    // cylinder (retention bore) intersecting it.
    struct CylInfo {
        int faceIdx;
        double radius;
        gp_Pnt loc;
        gp_Dir dir;
    };
    std::vector<CylInfo> cyls;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        const gp_Cylinder cy = surf.Cylinder();
        cyls.push_back({ i, cy.Radius(), cy.Axis().Location(), cy.Axis().Direction() });
    }

    // Y-axis cyl (roller pocket): largest such cyl.
    int yIdx = -1;
    double yMaxR = 0.0;
    for (size_t i = 0; i < cyls.size(); ++i) {
        if (std::abs(std::abs(cyls[i].dir.Y()) - 1.0) > 1e-2) continue;
        if (cyls[i].radius > yMaxR) { yMaxR = cyls[i].radius; yIdx = static_cast<int>(i); }
    }
    if (yIdx < 0) return out;

    // Z-axis cyl (retention bore): smaller, near the Y-cyl axis location.
    int zIdx = -1;
    const gp_Pnt& yLoc = cyls[yIdx].loc;
    for (size_t i = 0; i < cyls.size(); ++i) {
        if (static_cast<int>(i) == yIdx) continue;
        if (std::abs(std::abs(cyls[i].dir.Z()) - 1.0) > 1e-2) continue;
        if (cyls[i].radius >= yMaxR) continue;
        const double dx = cyls[i].loc.X() - yLoc.X();
        const double d  = std::sqrt(dx*dx);
        if (d > yMaxR * 1.5) continue;       // must be near the pocket centre
        zIdx = static_cast<int>(i);
        break;
    }

    double conf = (zIdx >= 0) ? 0.8 : 0.55;

    json recovered = {
        { "roller_dia_mm",          2.0 * yMaxR * 0.95 }, // backout clearance approx
        { "roller_length_mm",       0.0 },
        { "radial_clearance_mm",    yMaxR * 0.05 },
        { "arm_clearance_width_mm", 4.0 },
        { "arm_clearance_depth_mm", 4.0 },
        { "retention_dia_mm",       zIdx >= 0 ? 2.0 * cyls[zIdx].radius : 0.0 },
        { "retention_depth_mm",     0.0 },
        { "center_x_mm",            yLoc.X() },
        { "center_y_mm",            yLoc.Y() },
        { "center_z_mm",            yLoc.Z() },
    };
    json matched = {
        { "roller_pocket_face_id", cyls[yIdx].faceIdx },
        { "retention_face_id",     zIdx >= 0 ? cyls[zIdx].faceIdx : -1 },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, conf, matched });
    return out;
}

}  // namespace koocadcam::skill::cam_follower_roller_seat
