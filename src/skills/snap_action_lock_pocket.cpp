// @lat: [[engine/skills#snap_action_lock_pocket]]

#include "snap_action_lock_pocket.hpp"

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

namespace koocadcam::skill::snap_action_lock_pocket {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.lock_body_dia_mm < 6.0) {
        r.add("DFM-SNAPLOCK-DIA", "error",
              "snap_action_lock_pocket: lock_body_dia " +
              std::to_string(in.lock_body_dia_mm) +
              " mm < 6 mm (smallest commercial cam-lock)");
    }
    if (in.lock_body_depth_mm <= 0.0 || in.key_counterbore_depth_mm <= 0.0
        || in.cam_slot_depth_offset_mm <= 0.0
        || in.spring_pocket_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "snap_action_lock_pocket: all depths must be > 0");
    }
    if (in.key_counterbore_dia_mm <= in.lock_body_dia_mm) {
        r.add("DFM-SNAPLOCK-KEY", "error",
              "snap_action_lock_pocket: key_counterbore_dia (" +
              std::to_string(in.key_counterbore_dia_mm) +
              ") must be > lock_body_dia (" +
              std::to_string(in.lock_body_dia_mm) + ")");
    }
    if (in.key_counterbore_depth_mm >= in.lock_body_depth_mm) {
        r.add("DFM-SNAPLOCK-KEY", "error",
              "snap_action_lock_pocket: key_counterbore_depth >= "
              "lock_body_depth (would make a through-bore)");
    }
    if (in.cam_slot_width_mm < 0.8) {
        r.add("DFM-002", "error",
              "snap_action_lock_pocket: cam_slot_width " +
              std::to_string(in.cam_slot_width_mm) + " mm < 0.8 mm");
    }
    if (in.spring_pocket_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "snap_action_lock_pocket: spring_pocket_dia " +
              std::to_string(in.spring_pocket_dia_mm) + " mm < 0.8 mm");
    }
    // Spring pocket must NOT break through the lock body bore wall on the
    // wrong side.  Required: |offset - body_r| > spring_pocket_r + 0.5 mm.
    // Allow the pocket to TOUCH the wall (offset == body_r) which is the
    // intended detent geometry, but warn if it goes past the wall.
    const double bodyR  = in.lock_body_dia_mm / 2.0;
    const double springR = in.spring_pocket_dia_mm / 2.0;
    if (in.spring_pocket_offset_mm + springR > bodyR + 1.5) {
        r.add("DFM-SNAPLOCK-SPRING", "error",
              "snap_action_lock_pocket: spring pocket center+radius (" +
              std::to_string(in.spring_pocket_offset_mm + springR) +
              " mm) > body_r + 1.5 mm — breaks through bore wall");
    }
    if (in.cam_slot_depth_offset_mm > in.lock_body_depth_mm) {
        r.add("DFM-SNAPLOCK-CAM", "error",
              "snap_action_lock_pocket: cam_slot_depth_offset > "
              "lock_body_depth (slot below cavity floor)");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "snap_action_lock_pocket DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("snap_action_lock_pocket: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    if (std::abs(adir.X()) > 1e-6 || std::abs(adir.Y()) > 1e-6
        || adir.Z() >= 0) {
        throw SkillError("snap_action_lock_pocket: only axis_dir = -Z supported in slice 1");
    }

    const double zTop = zMax;
    const gp_Pnt entryPnt(in.position_x_mm, in.position_y_mm, zTop + kOverhang);
    const gp_Ax2 mainAx(entryPnt, adir);

    // ── Sub-feature 1: lock body bore ──────────────────────────────────
    const TopoDS_Shape lockBody = pr::cylinder(
        mainAx, in.lock_body_dia_mm / 2.0,
        in.lock_body_depth_mm + kOverhang);

    // ── Sub-feature 2: key counterbore (concentric) ────────────────────
    const TopoDS_Shape keyCBore = pr::cylinder(
        mainAx, in.key_counterbore_dia_mm / 2.0,
        in.key_counterbore_depth_mm + kOverhang);

    // ── Sub-feature 3: cam slot — rectangle perpendicular to bore axis,
    //                  at Z = zTop - cam_slot_depth_offset
    const double slotZ = zTop - in.cam_slot_depth_offset_mm;
    const gp_Pnt slotOrigin(
        in.position_x_mm - in.cam_slot_length_mm / 2.0,
        in.position_y_mm - in.cam_slot_width_mm / 2.0,
        slotZ);
    const gp_Ax2 slotAx(slotOrigin, gp::DZ());
    const TopoDS_Shape camSlot = pr::box(
        slotAx,
        in.cam_slot_length_mm,
        in.cam_slot_width_mm,
        in.cam_slot_depth_offset_mm + kOverhang);

    // ── Sub-feature 4: spring retention pocket (off-axis cylinder) ─────
    const gp_Pnt springStart(
        in.position_x_mm + in.spring_pocket_offset_mm,
        in.position_y_mm,
        zTop + kOverhang);
    const gp_Ax2 springAx(springStart, gp_Dir(0.0, 0.0, -1.0));
    const TopoDS_Shape springPocket = pr::cylinder(
        springAx, in.spring_pocket_dia_mm / 2.0,
        in.spring_pocket_depth_mm + 2.0 * kOverhang);

    // Fuse lock body + counterbore + cam slot + spring pocket → single cut
    TopoDS_Shape fused = pr::fuse(lockBody, keyCBore);
    fused = pr::fuse(fused, camSlot);
    fused = pr::fuse(fused, springPocket);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    json params = {
        { "entry_face_id",             *entryId },
        { "position_x_mm",             in.position_x_mm },
        { "position_y_mm",             in.position_y_mm },
        { "axis_dir",                  { adir.X(), adir.Y(), adir.Z() } },
        { "lock_body_dia_mm",          in.lock_body_dia_mm },
        { "lock_body_depth_mm",        in.lock_body_depth_mm },
        { "key_counterbore_dia_mm",    in.key_counterbore_dia_mm },
        { "key_counterbore_depth_mm",  in.key_counterbore_depth_mm },
        { "cam_slot_length_mm",        in.cam_slot_length_mm },
        { "cam_slot_width_mm",         in.cam_slot_width_mm },
        { "cam_slot_depth_offset_mm",  in.cam_slot_depth_offset_mm },
        { "spring_pocket_dia_mm",      in.spring_pocket_dia_mm },
        { "spring_pocket_offset_mm",   in.spring_pocket_offset_mm },
        { "spring_pocket_depth_mm",    in.spring_pocket_depth_mm },
    };
    json pattern = {
        { "kind",                      kSkillId },
        { "is_compound",               true },
        { "subfeature_count",          4 },
        { "lock_body_dia_mm",          in.lock_body_dia_mm },
        { "key_counterbore_dia_mm",    in.key_counterbore_dia_mm },
        { "concentric_cyl_count",      2 },
        { "off_axis_cyl_count",        1 },
        { "rect_slot_count",           1 },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill;end_mill;counterbore";
    tooling.tool_dia_mm       = in.key_counterbore_dia_mm;
    tooling.tool_length_mm    = in.lock_body_depth_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 280.0;
    tooling.feed_per_tooth_mm = 0.04;
    const double bodyR = in.lock_body_dia_mm / 2.0;
    const double kbR   = in.key_counterbore_dia_mm / 2.0;
    const double sprR  = in.spring_pocket_dia_mm / 2.0;
    tooling.stock_removed_mm3 =
        M_PI * bodyR * bodyR * in.lock_body_depth_mm
        + M_PI * (kbR * kbR - bodyR * bodyR) * in.key_counterbore_depth_mm
        + in.cam_slot_length_mm * in.cam_slot_width_mm
              * in.cam_slot_depth_offset_mm
        + M_PI * sprR * sprR * in.spring_pocket_depth_mm;
    tooling.est_cycle_time_s = std::max(20.0,
        in.lock_body_depth_mm * 0.4 + in.cam_slot_length_mm * 0.2 + 5.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },
              { "tool_dia_mm", in.lock_body_dia_mm },
              { "depth_mm", in.lock_body_depth_mm },
              { "purpose", "lock body bore (DIN 18254)" } },
            { { "tool_type", "counterbore" },
              { "tool_dia_mm", in.key_counterbore_dia_mm },
              { "depth_mm", in.key_counterbore_depth_mm },
              { "purpose", "key escutcheon seat (ANSI/BHMA A156.5)" } },
            { { "tool_type", "end_mill" },
              { "tool_dia_mm", in.cam_slot_width_mm },
              { "purpose", "cam-arm swing slot" } },
            { { "tool_type", "drill" },
              { "tool_dia_mm", in.spring_pocket_dia_mm },
              { "depth_mm", in.spring_pocket_depth_mm },
              { "purpose", "detent spring pocket" } },
        } },
        { "standard_ref", "DIN 18254 / ANSI BHMA A156.5" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::snap_action_lock_pocket applied: body={} kb={} cam={}x{} faces {}→{}",
                  in.lock_body_dia_mm, in.key_counterbore_dia_mm,
                  in.cam_slot_length_mm, in.cam_slot_width_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // PRIMARY: geometric.
    // Look for 2+ coaxial Z-axis cylinders (lock body + counterbore) and at
    // least one off-axis small Z cylinder (spring pocket).
    struct CylZ {
        double radius;
        gp_Pnt center;
    };
    std::vector<CylZ> cyls;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        const gp_Cylinder c = surf.Cylinder();
        const gp_Dir d = c.Axis().Direction();
        if (std::abs(std::abs(d.Z()) - 1.0) > 1e-2) continue;
        cyls.push_back({ c.Radius(), c.Axis().Location() });
    }

    // Find a coaxial pair with different radii (lock body + counterbore)
    int   bestI = -1, bestJ = -1;
    double smallR = 0.0, largeR = 0.0;
    gp_Pnt axisCenter(0, 0, 0);
    for (size_t i = 0; i < cyls.size(); ++i) {
        for (size_t j = i + 1; j < cyls.size(); ++j) {
            const double dx = cyls[i].center.X() - cyls[j].center.X();
            const double dy = cyls[i].center.Y() - cyls[j].center.Y();
            if (std::hypot(dx, dy) > 1e-2) continue;
            if (std::abs(cyls[i].radius - cyls[j].radius) < 1e-3) continue;
            const double s = std::min(cyls[i].radius, cyls[j].radius);
            const double l = std::max(cyls[i].radius, cyls[j].radius);
            if (s < 3.0) continue;
            if (s > smallR) {
                bestI = static_cast<int>(i);
                bestJ = static_cast<int>(j);
                smallR = s;
                largeR = l;
                axisCenter = cyls[i].center;
            }
        }
    }
    if (bestI >= 0) {
        // Look for an off-axis small cylinder (spring pocket)
        int offAxis = 0;
        for (size_t k = 0; k < cyls.size(); ++k) {
            if (static_cast<int>(k) == bestI || static_cast<int>(k) == bestJ) continue;
            const double dx = cyls[k].center.X() - axisCenter.X();
            const double dy = cyls[k].center.Y() - axisCenter.Y();
            if (std::hypot(dx, dy) > smallR * 0.5
                && cyls[k].radius < smallR * 0.5) {
                offAxis++;
            }
        }

        json recovered = {
            { "position_x_mm",            axisCenter.X() },
            { "position_y_mm",            axisCenter.Y() },
            { "axis_dir",                 { 0.0, 0.0, -1.0 } },
            { "lock_body_dia_mm",         2.0 * smallR },
            { "key_counterbore_dia_mm",   2.0 * largeR },
        };
        json matched = {
            { "concentric_pair",       { bestI, bestJ } },
            { "off_axis_cyl_count",    offAxis },
        };
        const double conf = (offAxis > 0) ? 0.75 : 0.6;
        out.push_back(RecognizedFeature{ kSkillId, recovered, conf, matched });
    }

    // FALLBACK: metadata replay
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        out.push_back(RecognizedFeature{
            kSkillId, f.params, 1.0,
            { { "via", "metadata_replay" }, { "subfeature_count", 4 } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::snap_action_lock_pocket
