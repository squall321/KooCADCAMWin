// @lat: [[engine/skills#nas_bolt_pattern_circular]]

#include "nas_bolt_pattern_circular.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepBuilderAPI_Transform.hxx>
#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::nas_bolt_pattern_circular {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

double clearanceExpansionFor(const std::string& grade) {
    if (grade == "close")  return 0.0;
    if (grade == "medium") return 0.1;
    if (grade == "free")   return 0.2;
    return -1.0;
}

bool isValidCount(int n) {
    return n == 4 || n == 6 || n == 8;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "nas_bolt_pattern_circular: face_id out of range");
    }
    if (!(in.pcd_mm > 0.0) || !(in.bolt_dia_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "nas_bolt_pattern_circular: pcd and bolt_dia must be > 0");
        return r;
    }
    if (!isValidCount(in.bolt_count)) {
        r.add("DFM-BOLT-COUNT", "error",
              "nas_bolt_pattern_circular: bolt_count " +
              std::to_string(in.bolt_count) + " not in {4, 6, 8}");
    }
    if (clearanceExpansionFor(in.bolt_hole_clearance_grade) < 0.0) {
        r.add("DFM-CLEARANCE-GRADE", "error",
              "nas_bolt_pattern_circular: clearance grade '" +
              in.bolt_hole_clearance_grade +
              "' not in {close, medium, free}");
    }
    // Circumferential spacing check: at least 2 × bolt_dia chord between holes
    if (in.bolt_count > 0 && in.pcd_mm > 0.0 && in.bolt_dia_mm > 0.0) {
        const double chord = 2.0 * (in.pcd_mm / 2.0) *
                             std::sin(M_PI / static_cast<double>(in.bolt_count));
        if (chord < in.bolt_dia_mm * 2.0) {
            r.add("DFM-PCD-SPACING", "warning",
                  "nas_bolt_pattern_circular: chord spacing " +
                  std::to_string(chord) +
                  " mm < 2 × bolt_dia — holes too close");
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "nas_bolt_pattern_circular DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const gp_Dir n     = wp.faceNormal(in.face_id);
    const gp_Dir drillDir(-n.X(), -n.Y(), -n.Z());

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    constexpr double kOverhang = 0.05;
    const double clearanceDia = in.bolt_dia_mm + clearanceExpansionFor(in.bolt_hole_clearance_grade);
    const double holeR        = clearanceDia / 2.0;
    const double holeHt       = bboxDiag + 2.0 * kOverhang;
    const double pcdRadius    = in.pcd_mm / 2.0;

    // Rotation axis = face normal direction passing through pattern center.
    const gp_Pnt patternCenter(in.center_x_mm, in.center_y_mm, faceC.Z());
    const gp_Ax1 rotAxis(patternCenter, n);

    // Build the "first" cutter at angle 0 (along +X from pattern center).
    const gp_Pnt firstHoleCenter(
        patternCenter.X() + pcdRadius,
        patternCenter.Y(),
        patternCenter.Z());
    const gp_Pnt firstToolStart(
        firstHoleCenter.X() + n.X() * kOverhang,
        firstHoleCenter.Y() + n.Y() * kOverhang,
        firstHoleCenter.Z() + n.Z() * kOverhang);
    const gp_Ax2 firstAx(firstToolStart, drillDir);
    const TopoDS_Shape firstCutter = pr::cylinder(firstAx, holeR, holeHt);

    // ── Sequential pr::cut loop — N cutters via gp_Trsf::SetRotation ────
    TopoDS_Shape current = wp.shape();
    for (int i = 0; i < in.bolt_count; ++i) {
        const double theta = 2.0 * M_PI * static_cast<double>(i) /
                             static_cast<double>(in.bolt_count);
        gp_Trsf t;
        t.SetRotation(rotAxis, theta);
        BRepBuilderAPI_Transform xform(firstCutter, t, true);
        if (!xform.IsDone()) {
            throw SkillError("nas_bolt_pattern_circular: rotation transform failed");
        }
        current = pr::cut(current, xform.Shape());
    }
    const TopoDS_Shape newShape = current;

    // ── Derived volume ──────────────────────────────────────────────────
    const double oneHoleVol = M_PI * holeR * holeR * (zMax - zMin);
    const double removedVol = oneHoleVol * in.bolt_count;

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "face_id",                   in.face_id },
        { "center_x_mm",               in.center_x_mm },
        { "center_y_mm",               in.center_y_mm },
        { "pcd_mm",                    in.pcd_mm },
        { "bolt_count",                in.bolt_count },
        { "bolt_dia_mm",               in.bolt_dia_mm },
        { "bolt_hole_clearance_grade", in.bolt_hole_clearance_grade },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "aerospace_feature_type",     "nas_bolt_circle" },
        { "subfeature_count",           in.bolt_count },
        { "fastener_dia",               in.bolt_dia_mm },
        { "clearance_dia_mm",           clearanceDia },
        { "bolt_count",                 in.bolt_count },
        { "pcd_mm",                     in.pcd_mm },
        { "derived_volume_removed_mm3", removedVol },
        { "standard_ref",               "NAS6000 / NAS618 (clearance grade)" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill";
    tooling.tool_dia_mm       = clearanceDia;
    tooling.tool_length_mm    = bboxDiag + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 280.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  = std::max(2.0, in.bolt_count * (zMax - zMin) / 30.0);
    tooling.extra = {
        { "aerospace_feature_type", "nas_bolt_circle" },
        { "bolt_count",             in.bolt_count },
        { "pcd_mm",                 in.pcd_mm },
        { "clearance_grade",        in.bolt_hole_clearance_grade },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::nas_bolt_pattern_circular applied: N={} PCD={} dia={}",
                  in.bolt_count, in.pcd_mm, in.bolt_dia_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition: metadata replay ─────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = {
            { "source",                 "metadata_replay" },
            { "is_compound",            true },
            { "aerospace_feature_type", "nas_bolt_circle" },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::nas_bolt_pattern_circular
