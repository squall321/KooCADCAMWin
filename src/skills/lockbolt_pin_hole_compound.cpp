// @lat: [[engine/skills#lockbolt_pin_hole_compound]]

#include "lockbolt_pin_hole_compound.hpp"

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

namespace koocadcam::skill::lockbolt_pin_hole_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

bool isStandardPinDia(double d) {
    return std::abs(d - 4.78) < 1e-2 ||
           std::abs(d - 6.35) < 1e-2 ||
           std::abs(d - 7.94) < 1e-2;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "lockbolt_pin_hole_compound: face_id out of range");
    }
    if (!(in.pin_dia_mm > 0.0) || !(in.grip_length_mm > 0.0) ||
        !(in.collar_clearance_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "lockbolt_pin_hole_compound: all dimensions must be > 0");
        return r;
    }
    if (!isStandardPinDia(in.pin_dia_mm)) {
        r.add("DFM-PIN-DIA", "error",
              "lockbolt_pin_hole_compound: pin_dia " +
              std::to_string(in.pin_dia_mm) +
              " mm not in NAS1396 set {4.78, 6.35, 7.94}");
    }
    if (in.grip_length_mm < in.pin_dia_mm * 0.6) {
        r.add("DFM-GRIP-LENGTH", "error",
              "lockbolt_pin_hole_compound: grip_length " +
              std::to_string(in.grip_length_mm) +
              " mm < pin_dia × 0.6 minimum stack-up");
    }
    if (in.collar_clearance_mm >= in.pin_dia_mm * 0.25) {
        r.add("DFM-COLLAR-CLEAR", "error",
              "lockbolt_pin_hole_compound: collar_clearance " +
              std::to_string(in.collar_clearance_mm) +
              " mm exceeds pin_dia × 0.25 (too sloppy)");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "lockbolt_pin_hole_compound DFM failed:";
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

    // ── Sub-feature 1: pin through-bore ────────────────────────────────
    const double pinR  = in.pin_dia_mm / 2.0;
    const double pinHt = bboxDiag + 2.0 * kOverhang;
    const gp_Ax2 pinAx(toolStart, drillDir);
    const TopoDS_Shape pinTool = pr::cylinder(pinAx, pinR, pinHt);

    // ── Sub-feature 2: collar-side relief counterbore ──────────────────
    // 1.0 mm deep × (pin_dia + 2×clearance) wide at the entry face on the
    // collar side (which is the entry face by convention).
    const double reliefR     = pinR + in.collar_clearance_mm;
    const double reliefDepth = 1.0;
    const gp_Ax2 reliefAx(entryPlanePoint, drillDir);
    const TopoDS_Shape reliefTool =
        pr::cylinder(reliefAx, reliefR, reliefDepth);

    // ── Sequential pr::cut loop (slice-14 guard) ────────────────────────
    TopoDS_Shape current = wp.shape();
    current = pr::cut(current, pinTool);
    current = pr::cut(current, reliefTool);
    const TopoDS_Shape newShape = current;

    // ── Derived volume ──────────────────────────────────────────────────
    const double pinVol    = M_PI * pinR * pinR * in.grip_length_mm;
    const double reliefVol = M_PI * (reliefR * reliefR - pinR * pinR) * reliefDepth;
    const double removedVol = pinVol + reliefVol;

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "face_id",             in.face_id },
        { "center_x_mm",         in.center_x_mm },
        { "center_y_mm",         in.center_y_mm },
        { "pin_dia_mm",          in.pin_dia_mm },
        { "grip_length_mm",      in.grip_length_mm },
        { "collar_clearance_mm", in.collar_clearance_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "aerospace_feature_type",     "lockbolt_pin_hole" },
        { "subfeature_count",           2 },
        { "fastener_dia",               in.pin_dia_mm },
        { "grip_length_mm",             in.grip_length_mm },
        { "collar_relief_dia_mm",       2.0 * reliefR },
        { "collar_relief_depth_mm",     reliefDepth },
        { "derived_volume_removed_mm3", removedVol },
        { "standard_ref",               "NAS1396 / MIL-B-23086" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill;counterbore";
    tooling.tool_dia_mm       = 2.0 * reliefR;
    tooling.tool_length_mm    = bboxDiag + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 280.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  = std::max(2.0, in.grip_length_mm / 30.0);
    tooling.extra = {
        { "aerospace_feature_type", "lockbolt_pin_hole" },
        { "fastener_family",        "Huck_Lockbolt" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::lockbolt_pin_hole_compound applied: pin={} grip={}",
                  in.pin_dia_mm, in.grip_length_mm);

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
            { "aerospace_feature_type", "lockbolt_pin_hole" },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::lockbolt_pin_hole_compound
