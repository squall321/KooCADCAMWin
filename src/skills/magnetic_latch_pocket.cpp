// @lat: [[engine/skills#magnetic_latch_pocket]]

#include "magnetic_latch_pocket.hpp"

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

namespace koocadcam::skill::magnetic_latch_pocket {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Catalog constants ───────────────────────────────────────────────────

namespace {

constexpr double kMagnetNominalDia = 8.0;     // Ø8 mm rare-earth disc magnet
constexpr double kMagnetSlipFitDia = 8.05;    // H10 slip-fit bore
constexpr double kMagnetDepth      = 5.0;
constexpr double kChamferRimTop    = 9.0;     // 45° chamfer top dia
constexpr double kChamferDepth     = 0.5;
constexpr double kMountPitch       = 30.0;
constexpr double kMountPilotDia    = 2.5;     // M3 pilot
constexpr double kMountPilotDepth  = 6.0;

constexpr double kMinThickness     = kMagnetDepth + 2.0;

}  // namespace

// ── DFM gates ────────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!std::isfinite(in.position_x_mm) || !std::isfinite(in.position_y_mm)) {
        r.add("DFM-INPUT", "error",
              "magnetic_latch_pocket: position_xy must be finite");
        return r;
    }
    constexpr double slipFitClearance = kMagnetSlipFitDia - kMagnetNominalDia;
    if constexpr (slipFitClearance < 0.02 || slipFitClearance > 0.10) {
        r.add("DFM-MAG-FIT", "error",
              "magnetic_latch_pocket: slip-fit clearance " +
              std::to_string(slipFitClearance) +
              " mm outside H10 range [0.02, 0.10] mm");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness = zMax - zMin;
    if (thickness < kMinThickness) {
        r.add("DFM-MAG-DEPTH", "error",
              "magnetic_latch_pocket: panel thickness " +
              std::to_string(thickness) +
              " mm < required " + std::to_string(kMinThickness) + " mm");
    }

    // Pocket and mount-pilot bbox fit.
    const double pocketR = kChamferRimTop / 2.0;
    if (in.position_x_mm - pocketR < xMin || in.position_x_mm + pocketR > xMax ||
        in.position_y_mm - pocketR < yMin || in.position_y_mm + pocketR > yMax) {
        r.add("DFM-MAG-DEPTH", "error",
              "magnetic_latch_pocket: pocket exceeds workpiece bbox");
    }
    const double mountLX = in.position_x_mm - kMountPitch / 2.0;
    const double mountRX = in.position_x_mm + kMountPitch / 2.0;
    if (mountLX < xMin || mountRX > xMax) {
        r.add("DFM-MAG-DEPTH", "error",
              "magnetic_latch_pocket: 30 mm mount pitch exceeds workpiece bbox");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "magnetic_latch_pocket DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double zTop = zMax;
    constexpr double kOverhang = 0.05;

    const gp_Pnt centerTop(in.position_x_mm, in.position_y_mm, zTop + kOverhang);
    const gp_Ax2 centerAx(centerTop, gp_Dir(0, 0, -1));

    // Sub-feature 1: magnet pocket cylinder (Ø8.05 mm × 5 mm)
    const TopoDS_Shape pocketTool = pr::cylinder(
        centerAx, kMagnetSlipFitDia / 2.0, kMagnetDepth + kOverhang);

    // Sub-feature 2: 45° retention chamfer (cone frustum)
    // Cone goes from kChamferRimTop dia at zTop to kMagnetSlipFitDia at
    // zTop − chamfer_depth.
    const gp_Pnt chamferTop(in.position_x_mm, in.position_y_mm, zTop);
    const gp_Ax2 chamferAx(chamferTop, gp_Dir(0, 0, -1));
    const TopoDS_Shape chamferTool = pr::coneFrustum(
        chamferAx,
        kChamferRimTop    / 2.0,     // r1 at axis origin (top)
        kMagnetSlipFitDia / 2.0,     // r2 at +chamferDepth (deeper)
        kChamferDepth);

    // Sub-features 3 & 4: mounting screw pilots
    auto mountTool = [&](double mx) {
        const gp_Pnt p(mx, in.position_y_mm, zTop + kOverhang);
        const gp_Ax2 ax(p, gp_Dir(0, 0, -1));
        return pr::cylinder(ax, kMountPilotDia / 2.0,
                            kMountPilotDepth + kOverhang);
    };
    const double mountLX = in.position_x_mm - kMountPitch / 2.0;
    const double mountRX = in.position_x_mm + kMountPitch / 2.0;
    const TopoDS_Shape mountL = mountTool(mountLX);
    const TopoDS_Shape mountR = mountTool(mountRX);

    // Fuse → single cut
    const TopoDS_Shape f1 = pr::fuse(pocketTool, chamferTool);
    const TopoDS_Shape f2 = pr::fuse(f1, mountL);
    const TopoDS_Shape fusedCutter = pr::fuse(f2, mountR);
    const TopoDS_Shape newShape    = pr::cut(wp.shape(), fusedCutter);

    // ── Signature ──────────────────────────────────────────────────────
    json params = {
        { "position_x_mm", in.position_x_mm },
        { "position_y_mm", in.position_y_mm },
        { "axis_dir",      { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "subfeature_count",      4 },
        { "magnet_dia_mm",         kMagnetNominalDia },
        { "magnet_slipfit_dia_mm", kMagnetSlipFitDia },
        { "magnet_depth_mm",       kMagnetDepth },
        { "chamfer_rim_top_mm",    kChamferRimTop },
        { "chamfer_depth_mm",      kChamferDepth },
        { "mount_pitch_mm",        kMountPitch },
        { "mount_pilot_dia_mm",    kMountPilotDia },
        { "mount_positions",       json::array({
            json::array({ mountLX, in.position_y_mm }),
            json::array({ mountRX, in.position_y_mm }),
        }) },
        { "axis_dir",              { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type   = "drill;chamfer_mill";
    tooling.tool_dia_mm = kMagnetSlipFitDia;
    tooling.tool_material = "carbide";
    const double pocketV =
        M_PI * (kMagnetSlipFitDia / 2.0) * (kMagnetSlipFitDia / 2.0) * kMagnetDepth;
    const double oneMountV =
        M_PI * (kMountPilotDia / 2.0) * (kMountPilotDia / 2.0) * kMountPilotDepth;
    // Chamfer cone-frustum extra (above the pocket cylinder).
    const double R = kChamferRimTop / 2.0, rs = kMagnetSlipFitDia / 2.0;
    const double frustV  = (M_PI * kChamferDepth / 3.0) * (R * R + R * rs + rs * rs);
    const double cylSegV = M_PI * rs * rs * kChamferDepth;
    const double chamferExtra = std::max(0.0, frustV - cylSegV);
    tooling.stock_removed_mm3 = pocketV + chamferExtra + 2.0 * oneMountV;
    tooling.est_cycle_time_s  = std::max(2.0, pocketV / 200.0);
    tooling.extra = {
        { "tool_sequence", json::array({
            { { "tool_type", "drill" },        { "tool_dia_mm", kMagnetSlipFitDia },
              { "depth_mm", kMagnetDepth },    { "purpose", "magnet_pocket" } },
            { { "tool_type", "chamfer_mill" }, { "tool_dia_mm", kChamferRimTop },
              { "chamfer_size_mm", kChamferDepth }, { "purpose", "retention_chamfer" } },
            { { "tool_type", "drill" },        { "tool_dia_mm", kMountPilotDia },
              { "pass_count", 2 },             { "purpose", "mount_screw_pilot" } },
        }) },
        { "magnet_grade",   "N48" },
        { "fit_class",      "H10_slip" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::magnetic_latch_pocket applied: Ø{} pocket + chamfer + 2 mounts",
                  kMagnetSlipFitDia);

    return SkillOutput{ wpNew, sig };
}

// ── Recognize: metadata replay ──────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::magnetic_latch_pocket
