// @lat: [[engine/skills#set_screw_anti_rotation_pocket]]

#include "set_screw_anti_rotation_pocket.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <array>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::set_screw_anti_rotation_pocket {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

struct SetScrewEntry {
    const char* size;
    double      pilot_dia_mm;
};

// ISO 261 metric coarse pilot table (subset for set-screws).
constexpr std::array<SetScrewEntry, 5> kSSTable {{
    { "M3", 2.50 },
    { "M4", 3.30 },
    { "M5", 4.20 },
    { "M6", 5.00 },
    { "M8", 6.80 },
}};

}  // namespace

double setScrewPilotDiameterFor(const std::string& set_screw_M)
{
    for (const auto& e : kSSTable)
        if (set_screw_M == e.size) return e.pilot_dia_mm;
    return 0.0;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    const double pilotDia = setScrewPilotDiameterFor(in.set_screw_M);
    if (in.set_screw_M.empty() || pilotDia <= 0.0) {
        r.add("DFM-SS-SIZE", "error",
              "set_screw_anti_rotation_pocket: unknown set_screw_M '" +
              in.set_screw_M + "' (supported: M3/M4/M5/M6/M8)");
        return r;
    }
    if (in.shaft_intersect_dia <= 0.0) {
        r.add("DFM-INPUT", "error",
              "set_screw_anti_rotation_pocket: shaft_intersect_dia must be > 0");
    }
    if (in.shaft_intersect_dia <= pilotDia) {
        r.add("DFM-SS-GEOM", "error",
              "set_screw_anti_rotation_pocket: shaft_intersect_dia " +
              std::to_string(in.shaft_intersect_dia) +
              " mm must be > set-screw pilot dia " +
              std::to_string(pilotDia) + " mm");
    }
    if (in.boss_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "set_screw_anti_rotation_pocket: boss_thickness_mm must be > 0");
    }
    if (in.chamfer_size_mm < 0.1) {
        r.add("DFM-SS-CHAMFER", "warning",
              "set_screw_anti_rotation_pocket: chamfer size " +
              std::to_string(in.chamfer_size_mm) +
              " mm < 0.1 mm (ISO 4029 recommends ≥ 0.1 mm entry chamfer)");
    }
    // Axis perpendicularity gate — DFM bake-in: shaft axis must be
    // approximately perpendicular to set-screw axis (dot ≈ 0).
    const double dotAxes = std::abs(in.axis_dir.Dot(in.shaft_axis_dir));
    if (dotAxes > 0.05) {
        r.add("DFM-SS-PERP", "warning",
              "set_screw_anti_rotation_pocket: shaft and set-screw axes not "
              "perpendicular (|cos θ| = " + std::to_string(dotAxes) + " > 0.05)");
    }

    // Workpiece bbox vs cross-drill diameter sanity.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double minorDim = std::min({ xMax - xMin, yMax - yMin, zMax - zMin });
    if (in.shaft_intersect_dia + 2.0 > minorDim) {
        r.add("DFM-SS-WALL", "warning",
              "set_screw_anti_rotation_pocket: cross-drill (" +
              std::to_string(in.shaft_intersect_dia) +
              " mm) leaves < 2 mm wall on one side of the workpiece");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "set_screw_anti_rotation_pocket DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError(
        "set_screw_anti_rotation_pocket: entry_face datum unresolved");

    const double pilotDia = setScrewPilotDiameterFor(in.set_screw_M);

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    // ── Sub-feature 1: vertical set-screw pilot hole ───────────────────
    gp_Pnt entryPnt;
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        if (adir.Z() < 0)
            entryPnt = gp_Pnt(in.position_x_mm, in.position_y_mm, zMax + kOverhang);
        else
            entryPnt = gp_Pnt(in.position_x_mm, in.position_y_mm, zMin - kOverhang);
    } else {
        entryPnt = gp_Pnt(in.position_x_mm, in.position_y_mm, (zMin + zMax) / 2.0);
    }
    const gp_Ax2 setScrewAx(entryPnt, adir);
    const TopoDS_Shape pilotCutter =
        pr::cylinder(setScrewAx, pilotDia / 2.0, in.boss_thickness_mm + kOverhang);

    // ── Sub-feature 2: perpendicular shaft cross-drill ─────────────────
    // Build a long cylinder along shaft_axis_dir passing through
    // (position_x, position_y, midZ).  Length ≥ 1.5 × bbox diag.
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));
    const double crossLen = bboxDiag * 2.0;

    // The cross-drill axis passes through the same XY but at a Z below the
    // top face by roughly boss_thickness_mm / 2 (where the set-screw tip
    // reaches the shaft).  For a top-face entry (adir = -Z) and a -Z
    // boss_thickness, we plant the cross-drill at zMax - boss_thickness/2.
    const double crossZ = (adir.Z() < 0) ? (zMax - in.boss_thickness_mm * 0.5)
                                         : (zMin + in.boss_thickness_mm * 0.5);
    // Start the cylinder OUTSIDE the workpiece on one side, then it spans
    // through.  We push the start point back by crossLen/2 along
    // -shaft_axis_dir from (position_x, position_y, crossZ).
    const gp_Pnt crossCenter(in.position_x_mm, in.position_y_mm, crossZ);
    const gp_Dir sdir = in.shaft_axis_dir;
    const gp_Pnt crossStart(
        crossCenter.X() - sdir.X() * crossLen * 0.5,
        crossCenter.Y() - sdir.Y() * crossLen * 0.5,
        crossCenter.Z() - sdir.Z() * crossLen * 0.5);
    const gp_Ax2 crossAx(crossStart, sdir);
    const TopoDS_Shape crossCutter =
        pr::cylinder(crossAx, in.shaft_intersect_dia / 2.0, crossLen);

    // ── Sub-feature 3: entry chamfer (cone frustum) ────────────────────
    // Cone has small radius = pilotDia/2, large radius = pilotDia/2 +
    // chamfer_size, height = chamfer_size.
    const double pilotR     = pilotDia / 2.0;
    const double chamferOR  = pilotR + in.chamfer_size_mm;
    // Cone is short; place it from entry plane upward (against adir) by
    // chamfer_size.  We extrude along -adir so the small end sits at the
    // entry plane and the large end sticks above.  Realised via
    // coneFrustum(axis, r_big, r_small, height).
    gp_Pnt chamferStart = entryPnt;
    // shift back by chamfer_size along -adir so when the cone extrudes by
    // chamfer_size, its small end meets the entry plane.
    chamferStart.SetX(chamferStart.X() - adir.X() * in.chamfer_size_mm);
    chamferStart.SetY(chamferStart.Y() - adir.Y() * in.chamfer_size_mm);
    chamferStart.SetZ(chamferStart.Z() - adir.Z() * in.chamfer_size_mm);
    const gp_Ax2 chamferAx(chamferStart, adir);
    const TopoDS_Shape chamferCutter =
        pr::coneFrustum(chamferAx, chamferOR, pilotR, in.chamfer_size_mm);

    // Pilot + chamfer are concentric → fuse, then cross-drill (separate
    // axis) is fused separately.
    const TopoDS_Shape fused1 = pr::fuse(pilotCutter, chamferCutter);
    const TopoDS_Shape fused2 = pr::fuse(fused1, crossCutter);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused2);

    json params = {
        { "entry_face_id",       *entryId },
        { "position_x_mm",       in.position_x_mm },
        { "position_y_mm",       in.position_y_mm },
        { "axis_dir",            { adir.X(), adir.Y(), adir.Z() } },
        { "shaft_axis_dir",      { sdir.X(), sdir.Y(), sdir.Z() } },
        { "set_screw_M",         in.set_screw_M },
        { "shaft_intersect_dia", in.shaft_intersect_dia },
        { "chamfer_size_mm",     in.chamfer_size_mm },
        { "boss_thickness_mm",   in.boss_thickness_mm },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "subfeature_count",      3 },
        { "set_screw_M",           in.set_screw_M },
        { "pilot_dia_mm",          pilotDia },
        { "shaft_intersect_dia",   in.shaft_intersect_dia },
        { "boss_thickness_mm",     in.boss_thickness_mm },
        { "chamfer_size_mm",       in.chamfer_size_mm },
        { "chamfer_outer_r_mm",    chamferOR },
        { "axis_dir",              { adir.X(), adir.Y(), adir.Z() } },
        { "shaft_axis_dir",        { sdir.X(), sdir.Y(), sdir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "drill;tap;chamfer;drill_cross";
    tooling.tool_dia_mm       = pilotDia;
    tooling.tool_length_mm    = in.boss_thickness_mm * 1.5 + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 280.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 =
        M_PI * pilotR * pilotR * in.boss_thickness_mm +
        M_PI * (in.shaft_intersect_dia/2.0) * (in.shaft_intersect_dia/2.0)
             * bboxDiag * 0.5;   // approx (cross-drill stays inside material)
    tooling.est_cycle_time_s = std::max(4.0,
        in.boss_thickness_mm * 0.2 + 3.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },
              { "tool_dia_mm", pilotDia },
              { "depth_mm",    in.boss_thickness_mm },
              { "purpose",     "set-screw pilot" } },
            { { "tool_type", "tap" },
              { "thread_size", in.set_screw_M },
              { "tap_depth_mm", in.boss_thickness_mm } },
            { { "tool_type", "chamfer_tool" },
              { "tool_dia_mm", 2.0 * chamferOR },
              { "depth_mm",    in.chamfer_size_mm },
              { "angle_deg",   45.0 },
              { "purpose",     "ISO 4029 entry chamfer" } },
            { { "tool_type", "drill_cross" },
              { "tool_dia_mm", in.shaft_intersect_dia },
              { "purpose",     "shaft pilot cross-drill (perpendicular)" } },
        } },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::set_screw_anti_rotation_pocket applied: ss={} cross={}",
                  in.set_screw_M, in.shaft_intersect_dia);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition — metadata replay ────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        out.push_back(RecognizedFeature{
            kSkillId, f.params, 1.0,
            { { "via", "metadata_replay" },
              { "subfeature_count", 3 } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::set_screw_anti_rotation_pocket
