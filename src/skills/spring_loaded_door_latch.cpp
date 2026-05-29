// @lat: [[engine/skills#spring_loaded_door_latch]]

#include "spring_loaded_door_latch.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::spring_loaded_door_latch {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.latch_pocket_size_mm < 6.0) {
        r.add("DFM-SPDL-POCKET", "error",
              "spring_loaded_door_latch: latch_pocket_size " +
              std::to_string(in.latch_pocket_size_mm) +
              " mm < 6 mm functional minimum");
    }
    if (in.screw_hole_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "spring_loaded_door_latch: screw_hole_dia " +
              std::to_string(in.screw_hole_dia_mm) + " mm < 0.8 mm");
    }
    if (in.plunger_bore_dia_mm < 1.6) {
        r.add("DFM-SPDL-PLUNGER", "error",
              "spring_loaded_door_latch: plunger_bore_dia " +
              std::to_string(in.plunger_bore_dia_mm) +
              " mm < 1.6 mm");
    }
    if (in.receiver_slot_width_mm < in.plunger_bore_dia_mm) {
        r.add("DFM-SPDL-RECV", "error",
              "spring_loaded_door_latch: receiver_slot_width " +
              std::to_string(in.receiver_slot_width_mm) +
              " mm < plunger_bore_dia (plunger can't pass)");
    }
    if (in.screw_spacing_mm <= 2.0 * in.screw_hole_dia_mm) {
        r.add("DFM-SPDL-SCREW-SPACE", "error",
              "spring_loaded_door_latch: screw_spacing " +
              std::to_string(in.screw_spacing_mm) +
              " mm must be > 2 × screw_hole_dia");
    }
    if (in.latch_pocket_depth_mm <= 0.0
        || in.plunger_bore_depth_mm <= 0.0
        || in.receiver_slot_depth_mm <= 0.0
        || in.screw_hole_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "spring_loaded_door_latch: all depths must be > 0");
    }
    // Plunger bore must reach beyond the pocket back wall.  Conservative:
    // plunger_bore_depth >= latch_pocket_depth / 2.
    if (in.plunger_bore_depth_mm < in.latch_pocket_depth_mm * 0.5) {
        r.add("DFM-SPDL-PLUNGER-DEPTH", "error",
              "spring_loaded_door_latch: plunger_bore_depth " +
              std::to_string(in.plunger_bore_depth_mm) +
              " mm < 0.5 × pocket depth — plunger can't reach receiver");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "spring_loaded_door_latch DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("spring_loaded_door_latch: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;

    if (std::abs(in.axis_dir.X()) > 1e-6 || std::abs(in.axis_dir.Y()) > 1e-6
        || in.axis_dir.Z() >= 0) {
        throw SkillError("spring_loaded_door_latch: only axis_dir = -Z supported in slice 1");
    }

    const double zTop = zMax;

    // ── Sub-feature 1: latch body pocket (square) ─────────────────────
    const double half = in.latch_pocket_size_mm / 2.0;
    const gp_Pnt pocketOrigin(
        in.position_x_mm - half,
        in.position_y_mm - half,
        zTop - in.latch_pocket_depth_mm);
    const gp_Ax2 pocketAx(pocketOrigin, gp::DZ());
    const TopoDS_Shape latchPocket = pr::box(
        pocketAx,
        in.latch_pocket_size_mm,
        in.latch_pocket_size_mm,
        in.latch_pocket_depth_mm + kOverhang);

    // ── Sub-feature 2 & 3: 2 screw holes along +X centerline ──────────
    const double screwR = in.screw_hole_dia_mm / 2.0;
    const gp_Pnt screwA(
        in.position_x_mm - in.screw_spacing_mm / 2.0,
        in.position_y_mm,
        zTop + kOverhang);
    const gp_Pnt screwB(
        in.position_x_mm + in.screw_spacing_mm / 2.0,
        in.position_y_mm,
        zTop + kOverhang);
    const gp_Ax2 saAx(screwA, gp_Dir(0.0, 0.0, -1.0));
    const gp_Ax2 sbAx(screwB, gp_Dir(0.0, 0.0, -1.0));
    const TopoDS_Shape screwHoleA = pr::cylinder(
        saAx, screwR, in.screw_hole_depth_mm + 2.0 * kOverhang);
    const TopoDS_Shape screwHoleB = pr::cylinder(
        sbAx, screwR, in.screw_hole_depth_mm + 2.0 * kOverhang);

    // ── Sub-feature 4: plunger bore — horizontal (+X axis cylinder) ───
    // The plunger axis is horizontal so the spring-loaded plunger pops out
    // sideways toward the receiver.  We start it outside the pocket on the
    // -X side and run it through the pocket back wall.
    const double plungerZ = zTop - in.latch_pocket_depth_mm * 0.5;
    const gp_Pnt plungerStart(
        in.position_x_mm - half - kOverhang,
        in.position_y_mm,
        plungerZ);
    const gp_Ax2 plungerAx(plungerStart, gp_Dir(1.0, 0.0, 0.0));
    const TopoDS_Shape plungerBore = pr::cylinder(
        plungerAx, in.plunger_bore_dia_mm / 2.0,
        in.plunger_bore_depth_mm + in.latch_pocket_size_mm + 2.0 * kOverhang);

    // ── Sub-feature 5: receiver slot — rectangle on the -X face ───────
    // The strike receiver opening on the door face.  Cut INTO the +X side
    // of the pocket at the plunger Z, extending past the plunger nose.
    const double recvX = in.position_x_mm + half;
    const gp_Pnt recvOrigin(
        recvX - kOverhang,
        in.position_y_mm - in.receiver_slot_width_mm / 2.0,
        plungerZ - in.receiver_slot_height_mm / 2.0);
    const gp_Ax2 recvAx(recvOrigin, gp::DZ());
    const TopoDS_Shape receiverSlot = pr::box(
        recvAx,
        in.receiver_slot_depth_mm + kOverhang,
        in.receiver_slot_width_mm,
        in.receiver_slot_height_mm);

    // Fuse all then single cut
    TopoDS_Shape fused = pr::fuse(latchPocket, screwHoleA);
    fused = pr::fuse(fused, screwHoleB);
    fused = pr::fuse(fused, plungerBore);
    fused = pr::fuse(fused, receiverSlot);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    json params = {
        { "entry_face_id",            *entryId },
        { "position_x_mm",            in.position_x_mm },
        { "position_y_mm",            in.position_y_mm },
        { "axis_dir",                 { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "latch_pocket_size_mm",     in.latch_pocket_size_mm },
        { "latch_pocket_depth_mm",    in.latch_pocket_depth_mm },
        { "screw_hole_dia_mm",        in.screw_hole_dia_mm },
        { "screw_hole_depth_mm",      in.screw_hole_depth_mm },
        { "screw_spacing_mm",         in.screw_spacing_mm },
        { "plunger_bore_dia_mm",      in.plunger_bore_dia_mm },
        { "plunger_bore_depth_mm",    in.plunger_bore_depth_mm },
        { "receiver_slot_width_mm",   in.receiver_slot_width_mm },
        { "receiver_slot_height_mm",  in.receiver_slot_height_mm },
        { "receiver_slot_depth_mm",   in.receiver_slot_depth_mm },
    };
    json pattern = {
        { "kind",                     kSkillId },
        { "is_compound",              true },
        { "subfeature_count",         5 },
        { "latch_pocket_size_mm",     in.latch_pocket_size_mm },
        { "screw_count",              2 },
        { "plunger_dia_mm",           in.plunger_bore_dia_mm },
        { "horizontal_cyl_count",     1 },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill;drill";
    tooling.tool_dia_mm       = std::max(in.latch_pocket_size_mm * 0.3,
                                         in.plunger_bore_dia_mm);
    tooling.tool_length_mm    = std::max(in.latch_pocket_depth_mm,
                                         in.plunger_bore_depth_mm) + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.03;
    const double plR = in.plunger_bore_dia_mm / 2.0;
    tooling.stock_removed_mm3 =
        in.latch_pocket_size_mm * in.latch_pocket_size_mm * in.latch_pocket_depth_mm
        + 2.0 * M_PI * screwR * screwR * in.screw_hole_depth_mm
        + M_PI * plR * plR
              * (in.plunger_bore_depth_mm + in.latch_pocket_size_mm)
        + in.receiver_slot_depth_mm * in.receiver_slot_width_mm
              * in.receiver_slot_height_mm;
    tooling.est_cycle_time_s = std::max(15.0,
        in.latch_pocket_depth_mm * 0.5
        + in.plunger_bore_depth_mm * 0.3 + 5.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "end_mill" },
              { "tool_dia_mm", in.latch_pocket_size_mm * 0.3 },
              { "depth_mm", in.latch_pocket_depth_mm },
              { "purpose", "latch body pocket (ANSI/BHMA A156.16)" } },
            { { "tool_type", "drill" },
              { "tool_dia_mm", in.screw_hole_dia_mm },
              { "pass_count", 2 },
              { "purpose", "M3 retention screws (DIN 7991 countersunk)" } },
            { { "tool_type", "drill" },
              { "tool_dia_mm", in.plunger_bore_dia_mm },
              { "depth_mm", in.plunger_bore_depth_mm + in.latch_pocket_size_mm },
              { "purpose", "horizontal plunger bore" } },
            { { "tool_type", "end_mill" },
              { "tool_dia_mm", in.receiver_slot_height_mm },
              { "purpose", "receiver/strike slot" } },
        } },
        { "standard_ref", "ANSI BHMA A156.16 + DIN 7991" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::spring_loaded_door_latch applied: pocket={}x{}x{} faces {}→{}",
                  in.latch_pocket_size_mm, in.latch_pocket_size_mm,
                  in.latch_pocket_depth_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // PRIMARY: look for small Z-axis cylinders (screws) + at least 1
    // X-axis cylinder (plunger bore) + a top-face pocket.
    int zSmallCyls = 0;
    int xCyls      = 0;
    double xCylRadius = 0.0;
    gp_Pnt firstZ(0, 0, 0);

    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        const gp_Cylinder c = surf.Cylinder();
        const gp_Dir d = c.Axis().Direction();
        if (std::abs(std::abs(d.Z()) - 1.0) < 1e-2 && c.Radius() < 3.0) {
            if (zSmallCyls == 0) firstZ = c.Axis().Location();
            zSmallCyls++;
        }
        if (std::abs(std::abs(d.X()) - 1.0) < 1e-2) {
            xCyls++;
            xCylRadius = c.Radius();
        }
    }

    if (zSmallCyls >= 2 && xCyls >= 1) {
        double xMin, yMin, zMin, xMax, yMax, zMax;
        wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
        json recovered = {
            { "position_x_mm",       firstZ.X() },
            { "position_y_mm",       firstZ.Y() },
            { "axis_dir",            { 0.0, 0.0, -1.0 } },
            { "plunger_bore_dia_mm", 2.0 * xCylRadius },
        };
        json matched = {
            { "small_z_cyl_count", zSmallCyls },
            { "x_axis_cyl_count",  xCyls },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.65, matched });
    }

    // FALLBACK: metadata replay
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        out.push_back(RecognizedFeature{
            kSkillId, f.params, 1.0,
            { { "via", "metadata_replay" }, { "subfeature_count", 5 } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::spring_loaded_door_latch
