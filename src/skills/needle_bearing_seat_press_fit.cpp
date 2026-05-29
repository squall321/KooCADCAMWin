// @lat: [[engine/skills#needle_bearing_seat_press_fit]]

#include "needle_bearing_seat_press_fit.hpp"

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

namespace koocadcam::skill::needle_bearing_seat_press_fit {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

constexpr double kChamferAxial_mm = 0.5;
constexpr double kChamferAngleDeg = 30.0;
constexpr const char* kFitClass   = "H7";

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.outer_dia_mm <= 0.0)
        r.add("DFM-INPUT", "error", std::string(kSkillId) +
              ": outer_dia_mm must be > 0");
    if (in.depth_mm <= 0.0)
        r.add("DFM-INPUT", "error", std::string(kSkillId) +
              ": depth_mm must be > 0");
    if (in.retention_dia_mm <= 0.0)
        r.add("DFM-INPUT", "error", std::string(kSkillId) +
              ": retention_dia_mm must be > 0");

    if (in.outer_dia_mm > 0.0 && in.retention_dia_mm > 0.0 &&
        in.retention_dia_mm >= in.outer_dia_mm) {
        r.add("DFM-SEAT-GEOM", "error", std::string(kSkillId) +
              ": retention_dia (" + std::to_string(in.retention_dia_mm) +
              ") must be < outer_dia (" + std::to_string(in.outer_dia_mm) +
              ") to act as a stop");
    }
    if (in.retention_dia_mm > 0.0 && in.retention_dia_mm < 0.8) {
        r.add("DFM-002", "error", std::string(kSkillId) +
              ": retention_dia " + std::to_string(in.retention_dia_mm) +
              " mm < min 0.8 mm");
    }
    if (in.outer_dia_mm > 0.0 && in.depth_mm > 0.0) {
        const double ratio = in.depth_mm / in.outer_dia_mm;
        if (ratio > 4.0) {
            r.add("DFM-BORE-RATIO", "warning", std::string(kSkillId) +
                  ": depth/outer_dia " + std::to_string(ratio) +
                  " > 4 — deep press fit; bar deflection may degrade fit");
        }
    }
    if (in.outer_dia_mm > 0.0 && in.outer_dia_mm < 3.0) {
        r.add("DFM-PRESS-FIT-MIN", "error", std::string(kSkillId) +
              ": outer_dia " + std::to_string(in.outer_dia_mm) +
              " mm < 3.0 mm — press fits unreliable on this OD");
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

    const double outerR     = in.outer_dia_mm / 2.0;
    const double retentionR = in.retention_dia_mm / 2.0;
    const double retentionLen = in.outer_dia_mm / 8.0;

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

    // Sub 1: precision bore at outer_dia for the press-fit needle cup.
    const gp_Ax2 axOuter(entry, adir);
    const TopoDS_Shape outerBore =
        pr::cylinder(axOuter, outerR, in.depth_mm + kEntryOverhang);

    // Sub 2: retention shoulder — smaller bore extending beyond the press
    // fit depth, leaving a backstop step.
    const TopoDS_Shape retentionBore =
        pr::cylinder(axOuter, retentionR,
                     in.depth_mm + retentionLen + kEntryOverhang);

    // Sub 3: 0.5 × 30° lead-in chamfer.
    const double chamferTopR = outerR + kChamferAxial_mm * std::tan(
        kChamferAngleDeg * M_PI / 180.0);
    const gp_Ax2 chamferAx(entry, adir);
    const TopoDS_Shape chamfer =
        pr::coneFrustum(chamferAx, chamferTopR, outerR, kChamferAxial_mm);

    const TopoDS_Shape fused1 = pr::fuse(outerBore, retentionBore);
    const TopoDS_Shape fused  = pr::fuse(fused1,    chamfer);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    json params = {
        { "entry_face_id",     *entryId },
        { "position_x_mm",     in.position_x_mm },
        { "position_y_mm",     in.position_y_mm },
        { "axis_dir",          { adir.X(), adir.Y(), adir.Z() } },
        { "outer_dia_mm",      in.outer_dia_mm },
        { "depth_mm",          in.depth_mm },
        { "retention_dia_mm",  in.retention_dia_mm },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "subfeature_count",      3 },
        { "outer_dia_mm",          in.outer_dia_mm },
        { "depth_mm",              in.depth_mm },
        { "retention_dia_mm",      in.retention_dia_mm },
        { "retention_length_mm",   retentionLen },
        { "fit_class",             kFitClass },
        { "chamfer_axial_mm",      kChamferAxial_mm },
        { "chamfer_angle_deg",     kChamferAngleDeg },
        { "axis_dir",              { adir.X(), adir.Y(), adir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type     = "reamer;boring_bar;chamfer_tool";
    tooling.tool_dia_mm   = in.outer_dia_mm;
    tooling.tool_material = "carbide";
    tooling.flute_count   = 4;
    tooling.cutting_speed_sfm = 150.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 =
        M_PI * outerR * outerR * in.depth_mm +
        M_PI * retentionR * retentionR * retentionLen;
    tooling.est_cycle_time_s = std::max(5.0, in.depth_mm / 15.0);
    tooling.extra = {
        { "fit_class",       kFitClass },
        { "tool_sequence", {
            { { "tool_type", "reamer" },
              { "dia_mm", in.outer_dia_mm },
              { "depth_mm", in.depth_mm },
              { "fit_class", kFitClass },
              { "note", "precision H7 bore for needle bearing OD" } },
            { { "tool_type", "boring_bar" },
              { "dia_mm", in.retention_dia_mm },
              { "depth_mm", in.depth_mm + retentionLen },
              { "note", "retention shoulder backstop" } },
            { { "tool_type", "chamfer_tool" },
              { "axial_mm", kChamferAxial_mm },
              { "angle_deg", kChamferAngleDeg },
              { "note", "0.5 × 30° SKF press-fit lead-in" } },
        } },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::needle_bearing_seat_press_fit applied: od={} depth={} ret={}",
                  in.outer_dia_mm, in.depth_mm, in.retention_dia_mm);

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

}  // namespace koocadcam::skill::needle_bearing_seat_press_fit
