// @lat: [[engine/skills#hi_lok_install_hole]]

#include "hi_lok_install_hole.hpp"

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

namespace koocadcam::skill::hi_lok_install_hole {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

bool isStandardFastenerDia(double d) {
    return std::abs(d - 4.0) < 1e-3 ||
           std::abs(d - 4.8) < 1e-3 ||
           std::abs(d - 5.6) < 1e-3 ||
           std::abs(d - 6.4) < 1e-3;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "hi_lok_install_hole: face_id out of range");
    }
    if (!(in.fastener_dia_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "hi_lok_install_hole: fastener_dia must be > 0");
        return r;
    }
    if (!isStandardFastenerDia(in.fastener_dia_mm)) {
        r.add("DFM-FAST-DIA", "error",
              "hi_lok_install_hole: fastener_dia " +
              std::to_string(in.fastener_dia_mm) +
              " mm not in NAS1769 set {4.0, 4.8, 5.6, 6.4}");
    }
    if (in.countersink_style != "100" &&
        in.countersink_style != "130" &&
        in.countersink_style != "none") {
        r.add("DFM-CSK-STYLE", "error",
              "hi_lok_install_hole: countersink_style '" +
              in.countersink_style + "' not in {100, 130, none}");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "hi_lok_install_hole DFM failed:";
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
    const gp_Pnt entryPlanePoint(in.center_x_mm, in.center_y_mm, faceC.Z());
    const gp_Pnt toolStart(
        entryPlanePoint.X() + n.X() * kOverhang,
        entryPlanePoint.Y() + n.Y() * kOverhang,
        entryPlanePoint.Z() + n.Z() * kOverhang);

    // ── Sub-feature 1: primary drill (through clearance hole) ──────────
    const double drillR  = in.fastener_dia_mm / 2.0;
    const double drillHt = bboxDiag + 2.0 * kOverhang;
    const gp_Ax2 drillAx(toolStart, drillDir);
    const TopoDS_Shape drillTool = pr::cylinder(drillAx, drillR, drillHt);

    // ── Sub-feature 2: optional countersink ────────────────────────────
    int subCount = 1;
    double cskDepth = 0.0;
    double cskTopR  = 0.0;
    bool   hasCsk   = false;
    TopoDS_Shape coneTool;
    if (in.countersink_style == "100" || in.countersink_style == "130") {
        hasCsk = true;
        const double angleDeg = (in.countersink_style == "100") ? 100.0 : 130.0;
        const double halfRad  = angleDeg * 0.5 * M_PI / 180.0;
        // Conventional CSK depth = 0.6 × fastener_dia (NAS5238 envelope).
        cskDepth = in.fastener_dia_mm * 0.6;
        cskTopR  = drillR + cskDepth * std::tan(halfRad);
        const gp_Ax2 coneAx(entryPlanePoint, drillDir);
        coneTool = pr::coneFrustum(coneAx, cskTopR, drillR, cskDepth);
        subCount = 2;
    }

    // ── Sequential pr::cut loop ────────────────────────────────────────
    TopoDS_Shape current = wp.shape();
    current = pr::cut(current, drillTool);
    if (hasCsk) {
        current = pr::cut(current, coneTool);
    }
    const TopoDS_Shape newShape = current;

    // ── Derived volume ──────────────────────────────────────────────────
    const double drillVol = M_PI * drillR * drillR * (zMax - zMin);
    double cskVol = 0.0;
    if (hasCsk) {
        const double rTop = cskTopR, rBot = drillR;
        const double frustVol = (M_PI * cskDepth / 3.0) *
                                (rTop * rTop + rTop * rBot + rBot * rBot);
        cskVol = std::max(0.0, frustVol - M_PI * rBot * rBot * cskDepth);
    }
    const double removedVol = drillVol + cskVol;

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "face_id",            in.face_id },
        { "center_x_mm",        in.center_x_mm },
        { "center_y_mm",        in.center_y_mm },
        { "fastener_dia_mm",    in.fastener_dia_mm },
        { "countersink_style",  in.countersink_style },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                hasCsk },
        { "aerospace_feature_type",     "hi_lok_install" },
        { "subfeature_count",           subCount },
        { "fastener_dia",               in.fastener_dia_mm },
        { "countersink_style",          in.countersink_style },
        { "csk_top_dia_mm",             2.0 * cskTopR },
        { "csk_depth_mm",               cskDepth },
        { "derived_volume_removed_mm3", removedVol },
        { "standard_ref",               "NAS1769 / NAS1097 / NAS5238" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = hasCsk ? "drill;countersink" : "drill";
    tooling.tool_dia_mm       = hasCsk ? 2.0 * cskTopR : in.fastener_dia_mm;
    tooling.tool_length_mm    = bboxDiag + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 280.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  = std::max(2.0, (zMax - zMin) / 30.0);
    tooling.extra = {
        { "aerospace_feature_type", "hi_lok_install" },
        { "fastener_family",        "Hi-Lok" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::hi_lok_install_hole applied: dia={} csk={}",
                  in.fastener_dia_mm, in.countersink_style);

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
            { "aerospace_feature_type", "hi_lok_install" },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::hi_lok_install_hole
