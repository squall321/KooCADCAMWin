// @lat: [[engine/skills#rivet_dimple_compound]]

#include "rivet_dimple_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::rivet_dimple_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

bool isStandardRivetDia(double d) {
    return std::abs(d - 3.2) < 1e-3 ||
           std::abs(d - 4.0) < 1e-3 ||
           std::abs(d - 4.8) < 1e-3;
}

bool isStandardAngle(double a) {
    return std::abs(a - 82.0) < 1e-3 ||
           std::abs(a - 100.0) < 1e-3 ||
           std::abs(a - 120.0) < 1e-3;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "rivet_dimple_compound: face_id out of range");
    }
    if (!(in.rivet_dia_mm > 0.0) || !(in.sheet_thickness_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "rivet_dimple_compound: rivet_dia and sheet_thickness must be > 0");
        return r;
    }
    if (!isStandardRivetDia(in.rivet_dia_mm)) {
        r.add("DFM-RIVET-DIA", "error",
              "rivet_dimple_compound: rivet_dia " +
              std::to_string(in.rivet_dia_mm) +
              " mm not in MS20426 set {3.2, 4.0, 4.8}");
    }
    if (!isStandardAngle(in.dimple_angle_deg)) {
        r.add("DFM-DIMPLE-ANGLE", "error",
              "rivet_dimple_compound: dimple_angle " +
              std::to_string(in.dimple_angle_deg) +
              " deg not in MS33558 set {82, 100, 120}");
    }
    if (in.sheet_thickness_mm < in.rivet_dia_mm * 0.3) {
        r.add("DFM-SHEET-THICKNESS", "error",
              "rivet_dimple_compound: sheet_thickness " +
              std::to_string(in.sheet_thickness_mm) +
              " mm < rivet_dia × 0.3 (" +
              std::to_string(in.rivet_dia_mm * 0.3) + " mm)");
    }
    const double dimpleDepth = in.sheet_thickness_mm * 0.7;
    if (dimpleDepth > in.sheet_thickness_mm * 0.8) {
        // Defensive: 0.7 ≤ 0.8 always but flag if rule changed
        r.add("DFM-DIMPLE-DEPTH", "error",
              "rivet_dimple_compound: derived dimple_depth exceeds 0.8 × sheet_thickness");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "rivet_dimple_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const gp_Dir n     = wp.faceNormal(in.face_id);
    // Drill direction is opposite face normal (into the body).
    const gp_Dir drillDir(-n.X(), -n.Y(), -n.Z());

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    constexpr double kOverhang = 0.05;

    // Position on the face plane (use face center Z as reference; XY supplied)
    const gp_Pnt entryPlanePoint(in.center_x_mm, in.center_y_mm, faceC.Z());

    // Tool start sits just outside the face along +n
    const gp_Pnt toolStart(
        entryPlanePoint.X() + n.X() * kOverhang,
        entryPlanePoint.Y() + n.Y() * kOverhang,
        entryPlanePoint.Z() + n.Z() * kOverhang);

    // ── Sub-feature 1: through-hole pilot ───────────────────────────────
    const double pilotR     = in.rivet_dia_mm / 2.0;
    const double pilotHt    = bboxDiag + 2.0 * kOverhang;
    const gp_Ax2 pilotAx(toolStart, drillDir);
    const TopoDS_Shape pilotTool = pr::cylinder(pilotAx, pilotR, pilotHt);

    // ── Sub-feature 2: conical dimple at entry face ─────────────────────
    // included angle = dimple_angle_deg; half-angle drives cone geometry.
    // dimple depth = sheet_thickness × 0.7 (per MS33558 envelope).
    // Top opening dia = pilot_dia + 2 × depth × tan(half_angle).
    const double halfAngleRad = in.dimple_angle_deg * 0.5 * M_PI / 180.0;
    const double dimpleDepth  = in.sheet_thickness_mm * 0.7;
    const double topRadius    = pilotR + dimpleDepth * std::tan(halfAngleRad);

    // Cone frustum: top at entry plane (rTop = topRadius),
    // bottom at +drillDir × dimpleDepth (rBot = pilotR).
    const gp_Ax2 coneAx(entryPlanePoint, drillDir);
    const TopoDS_Shape coneTool =
        pr::coneFrustum(coneAx, topRadius, pilotR, dimpleDepth);

    // ── Sequential pr::cut loop (slice-14 guard) ────────────────────────
    TopoDS_Shape current = wp.shape();
    current = pr::cut(current, pilotTool);
    current = pr::cut(current, coneTool);
    const TopoDS_Shape newShape = current;

    // ── Derived volume ──────────────────────────────────────────────────
    const double pilotVol = M_PI * pilotR * pilotR * in.sheet_thickness_mm;
    const double rTop = topRadius;
    const double rBot = pilotR;
    const double frustumVol = (M_PI * dimpleDepth / 3.0) *
                              (rTop * rTop + rTop * rBot + rBot * rBot);
    const double pilotInCone = M_PI * rBot * rBot * dimpleDepth;
    const double dimpleVol   = std::max(0.0, frustumVol - pilotInCone);
    const double removedVol  = pilotVol + dimpleVol;

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "face_id",            in.face_id },
        { "center_x_mm",        in.center_x_mm },
        { "center_y_mm",        in.center_y_mm },
        { "rivet_dia_mm",       in.rivet_dia_mm },
        { "sheet_thickness_mm", in.sheet_thickness_mm },
        { "dimple_angle_deg",   in.dimple_angle_deg },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "aerospace_feature_type",     "flush_rivet_dimple" },
        { "subfeature_count",           2 },
        { "fastener_dia",               in.rivet_dia_mm },
        { "sheet_thickness_mm",         in.sheet_thickness_mm },
        { "dimple_angle_deg",           in.dimple_angle_deg },
        { "dimple_depth_mm",            dimpleDepth },
        { "dimple_top_dia_mm",          2.0 * topRadius },
        { "derived_volume_removed_mm3", removedVol },
        { "standard_ref",               "MS33558 / MS20426" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill;countersink";
    tooling.tool_dia_mm       = 2.0 * topRadius;
    tooling.tool_length_mm    = bboxDiag + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  = std::max(1.0, in.sheet_thickness_mm / 30.0);
    tooling.extra = {
        { "aerospace_feature_type", "flush_rivet_dimple" },
        { "standard",               "MS33558" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::rivet_dimple_compound applied: dia={} t={} angle={}",
                  in.rivet_dia_mm, in.sheet_thickness_mm, in.dimple_angle_deg);

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
            { "source",                "metadata_replay" },
            { "is_compound",           true },
            { "aerospace_feature_type", "flush_rivet_dimple" },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::rivet_dimple_compound
