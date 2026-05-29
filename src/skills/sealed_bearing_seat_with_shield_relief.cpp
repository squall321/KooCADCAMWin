// @lat: [[engine/skills#sealed_bearing_seat_with_shield_relief]]

#include "sealed_bearing_seat_with_shield_relief.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::sealed_bearing_seat_with_shield_relief {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

constexpr double kReliefAxialWidth_mm = 1.5;    // SKF standard
constexpr double kChamferAxial_mm     = 0.5;
constexpr double kChamferAngleDeg     = 30.0;

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.outer_dia_mm <= 0.0)
        r.add("DFM-INPUT", "error", std::string(kSkillId) +
              ": outer_dia_mm must be > 0");
    if (in.shield_relief_dia_mm <= 0.0)
        r.add("DFM-INPUT", "error", std::string(kSkillId) +
              ": shield_relief_dia_mm must be > 0");
    if (in.seat_depth_mm <= 0.0)
        r.add("DFM-INPUT", "error", std::string(kSkillId) +
              ": seat_depth_mm must be > 0");

    if (in.outer_dia_mm > 0.0 && in.shield_relief_dia_mm > 0.0 &&
        in.shield_relief_dia_mm <= in.outer_dia_mm) {
        r.add("DFM-SHIELD-DIA", "error", std::string(kSkillId) +
              ": shield_relief_dia (" + std::to_string(in.shield_relief_dia_mm) +
              ") must be > outer_dia (" + std::to_string(in.outer_dia_mm) +
              ") to provide a relief annulus");
    }
    if (in.outer_dia_mm > 0.0 && in.shield_relief_dia_mm > 0.0 &&
        in.shield_relief_dia_mm > in.outer_dia_mm + 2.0) {
        r.add("DFM-SHIELD-RELIEF-MAX", "error", std::string(kSkillId) +
              ": shield_relief_dia exceeds outer_dia + 2.0 mm — bearing "
              "would lose support");
    }
    if (in.outer_dia_mm > 0.0 && in.shield_relief_dia_mm > 0.0) {
        const double reliefDepth =
            (in.shield_relief_dia_mm - in.outer_dia_mm) / 2.0;
        if (reliefDepth > 0.0 && reliefDepth < 0.3) {
            r.add("DFM-002", "error", std::string(kSkillId) +
                  ": relief radial depth " + std::to_string(reliefDepth) +
                  " mm < useful minimum 0.3 mm");
        }
    }
    // Need enough seat depth to fit 2 reliefs + a centre seat band.
    if (in.seat_depth_mm > 0.0 && in.seat_depth_mm < 2.0 * kReliefAxialWidth_mm + 1.0) {
        r.add("DFM-INPUT", "warning", std::string(kSkillId) +
              ": seat_depth " + std::to_string(in.seat_depth_mm) +
              " mm leaves little race-bearing surface between reliefs");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = std::string(kSkillId) + " DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError(std::string(kSkillId) +
                                   ": entry_face datum unresolved");

    const double outerR  = in.outer_dia_mm / 2.0;
    const double reliefR = in.shield_relief_dia_mm / 2.0;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    constexpr double kEntryOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;
    gp_Pnt entry(in.position_x_mm, in.position_y_mm, zMax + kEntryOverhang);
    if (!(std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6 && adir.Z() < 0)) {
        const double bboxDiag = std::sqrt(
            (xMax - xMin) * (xMax - xMin) +
            (yMax - yMin) * (yMax - yMin) +
            (zMax - zMin) * (zMax - zMin));
        entry = gp_Pnt(
            in.position_x_mm - adir.X() * (bboxDiag + kEntryOverhang),
            in.position_y_mm - adir.Y() * (bboxDiag + kEntryOverhang),
            (zMin + zMax) / 2.0 - adir.Z() * (bboxDiag + kEntryOverhang));
    }

    // Sub 1: bore at outer_dia, full seat depth.
    const gp_Ax2 axOuter(entry, adir);
    const TopoDS_Shape outerBore =
        pr::cylinder(axOuter, outerR, in.seat_depth_mm + kEntryOverhang);

    // Sub 3: front shield-relief groove — annular ring near entry face.
    const gp_Ax2 axFrontRelief(entry, adir);
    const TopoDS_Shape frontRelief =
        pr::annularRing(axFrontRelief,
                        reliefR,
                        outerR + 1e-4,
                        kReliefAxialWidth_mm);

    // Sub 4: back shield-relief groove — annular ring at the far end.
    const double backReliefAxialPos =
        std::max(0.0, in.seat_depth_mm - kReliefAxialWidth_mm);
    const gp_Pnt backReliefOrigin(
        entry.X() + adir.X() * backReliefAxialPos,
        entry.Y() + adir.Y() * backReliefAxialPos,
        entry.Z() + adir.Z() * backReliefAxialPos);
    const gp_Ax2 axBackRelief(backReliefOrigin, adir);
    const TopoDS_Shape backRelief =
        pr::annularRing(axBackRelief,
                        reliefR,
                        outerR + 1e-4,
                        kReliefAxialWidth_mm);

    // Sub 5: 0.5 × 30° chamfer.
    const double chamferTopR = outerR + kChamferAxial_mm * std::tan(
        kChamferAngleDeg * M_PI / 180.0);
    const gp_Ax2 chamferAx(entry, adir);
    const TopoDS_Shape chamfer =
        pr::coneFrustum(chamferAx, chamferTopR, outerR, kChamferAxial_mm);

    // Sub 2 (outer-race seat band) is emergent — the surviving cylindrical
    // wall between the two reliefs.

    const TopoDS_Shape fused1 = pr::fuse(outerBore,  frontRelief);
    const TopoDS_Shape fused2 = pr::fuse(fused1,     backRelief);
    const TopoDS_Shape fused  = pr::fuse(fused2,     chamfer);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    json params = {
        { "entry_face_id",          *entryId },
        { "position_x_mm",          in.position_x_mm },
        { "position_y_mm",          in.position_y_mm },
        { "axis_dir",               { adir.X(), adir.Y(), adir.Z() } },
        { "outer_dia_mm",           in.outer_dia_mm },
        { "shield_relief_dia_mm",   in.shield_relief_dia_mm },
        { "seat_depth_mm",          in.seat_depth_mm },
    };
    const double reliefRadialDepth =
        (in.shield_relief_dia_mm - in.outer_dia_mm) / 2.0;
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "subfeature_count",           5 },
        { "outer_dia_mm",               in.outer_dia_mm },
        { "shield_relief_dia_mm",       in.shield_relief_dia_mm },
        { "seat_depth_mm",              in.seat_depth_mm },
        { "relief_radial_depth_mm",     reliefRadialDepth },
        { "relief_axial_width_mm",      kReliefAxialWidth_mm },
        { "back_relief_axial_pos_mm",   backReliefAxialPos },
        { "chamfer_axial_mm",           kChamferAxial_mm },
        { "chamfer_angle_deg",          kChamferAngleDeg },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type     = "boring_bar;groove_tool;chamfer_tool";
    tooling.tool_dia_mm   = in.outer_dia_mm;
    tooling.tool_material = "carbide";
    tooling.flute_count   = 1;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 =
        M_PI * outerR * outerR * in.seat_depth_mm +
        2.0 * M_PI * (reliefR * reliefR - outerR * outerR) * kReliefAxialWidth_mm;
    tooling.est_cycle_time_s = std::max(4.0, in.seat_depth_mm / 20.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "boring_bar" },
              { "dia_mm", in.outer_dia_mm },
              { "depth_mm", in.seat_depth_mm } },
            { { "tool_type", "groove_tool" },
              { "groove_outer_dia_mm", in.shield_relief_dia_mm },
              { "width_mm", kReliefAxialWidth_mm },
              { "axial_pos_mm", 0.0 },
              { "note", "front shield relief" } },
            { { "tool_type", "groove_tool" },
              { "groove_outer_dia_mm", in.shield_relief_dia_mm },
              { "width_mm", kReliefAxialWidth_mm },
              { "axial_pos_mm", backReliefAxialPos },
              { "note", "back shield relief" } },
            { { "tool_type", "chamfer_tool" },
              { "axial_mm", kChamferAxial_mm },
              { "angle_deg", kChamferAngleDeg } },
        } },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::sealed_bearing_seat_with_shield_relief applied: od={} relief_od={}",
                  in.outer_dia_mm, in.shield_relief_dia_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::sealed_bearing_seat_with_shield_relief
