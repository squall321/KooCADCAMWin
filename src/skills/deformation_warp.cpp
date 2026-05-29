// @lat: [[engine/skills#deformation_warp]]

#include "deformation_warp.hpp"

#include "Workpiece.hpp"

#include <BRepBuilderAPI_GTransform.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax1.hxx>
#include <gp_GTrsf.hxx>
#include <gp_Mat.hxx>
#include <gp_Trsf.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::deformation_warp {

using nlohmann::json;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    if (s.IsNull()) return 0.0;
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ── validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.scale_x <= 0.0 || in.scale_y <= 0.0 || in.scale_z <= 0.0) {
        r.add("DFM-INPUT", "error",
              "deformation_warp: scale factors must be > 0 "
              "(sx=" + std::to_string(in.scale_x) +
              " sy=" + std::to_string(in.scale_y) +
              " sz=" + std::to_string(in.scale_z) + ")");
        return r;
    }
    auto checkRange = [&](const char* name, double s) {
        if (s < 0.1 || s > 10.0) {
            r.add("DFM-WARP-LIMIT", "error",
                  std::string("deformation_warp: ") + name + " = " +
                  std::to_string(s) +
                  " outside [0.1, 10.0] envelope (ISO 286 limit)");
        }
    };
    checkRange("scale_x", in.scale_x);
    checkRange("scale_y", in.scale_y);
    checkRange("scale_z", in.scale_z);

    if (in.angle_deg < -720.0 || in.angle_deg > 720.0) {
        r.add("DFM-INPUT", "warning",
              "deformation_warp: angle_deg " + std::to_string(in.angle_deg) +
              " is > 2 full rotations — likely a unit error");
    }
    return r;
}

// ── synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "deformation_warp DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double startVol = volumeOf(wp.shape());

    // ── Step 1: non-uniform scale (gp_GTrsf diagonal matrix) ─────────────
    gp_GTrsf gt;
    gp_Mat mat(in.scale_x, 0.0,         0.0,
               0.0,         in.scale_y, 0.0,
               0.0,         0.0,         in.scale_z);
    gt.SetVectorialPart(mat);
    // Translation part stays zero — scale about world origin.

    TopoDS_Shape afterScale;
    try {
        BRepBuilderAPI_GTransform gtBuilder(wp.shape(), gt, /*copy=*/true);
        if (!gtBuilder.IsDone())
            throw Standard_Failure(
                "deformation_warp: GTransform (scale) failed");
        afterScale = gtBuilder.Shape();
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("deformation_warp: ") + ex.what());
    }

    const double midVol = volumeOf(afterScale);

    // ── Step 2: rigid rotation ───────────────────────────────────────────
    gp_Trsf rotTrsf;
    rotTrsf.SetRotation(gp_Ax1(in.axis_origin, in.axis_dir),
                        in.angle_deg * M_PI / 180.0);
    TopoDS_Shape afterRot;
    try {
        BRepBuilderAPI_Transform rotBuilder(afterScale, rotTrsf, /*copy=*/true);
        if (!rotBuilder.IsDone())
            throw Standard_Failure(
                "deformation_warp: Transform (rotation) failed");
        afterRot = rotBuilder.Shape();
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("deformation_warp: ") + ex.what());
    }

    const double endVol = volumeOf(afterRot);
    const double scaleProd = in.scale_x * in.scale_y * in.scale_z;

    // ── Signature ────────────────────────────────────────────────────────
    json deformMatrix = json::array({
        json::array({ in.scale_x, 0.0,         0.0,         0.0 }),
        json::array({ 0.0,         in.scale_y, 0.0,         0.0 }),
        json::array({ 0.0,         0.0,         in.scale_z, 0.0 }),
        json::array({ 0.0,         0.0,         0.0,         1.0 }),
    });

    json params = {
        { "scale_x",     in.scale_x },
        { "scale_y",     in.scale_y },
        { "scale_z",     in.scale_z },
        { "axis_origin", { in.axis_origin.X(), in.axis_origin.Y(), in.axis_origin.Z() } },
        { "axis_dir",    { in.axis_dir.X(),    in.axis_dir.Y(),    in.axis_dir.Z()    } },
        { "angle_deg",   in.angle_deg },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    2 },     // scale + rotate
        { "deformation_matrix",  deformMatrix },
        { "scale_determinant",   scaleProd },
        { "start_volume_mm3",    startVol },
        { "scaled_volume_mm3",   midVol },
        { "end_volume_mm3",      endVol },
        { "is_uniform_scale",
              std::abs(in.scale_x - in.scale_y) < 1e-9 &&
              std::abs(in.scale_y - in.scale_z) < 1e-9 },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "warp_compound";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = std::max(0.0, startVol - endVol);
    tooling.est_cycle_time_s  = 0.5;        // CPU-only op
    tooling.extra = {
        { "operation_chain", {
            "non-uniform GTransform scale " +
                std::to_string(in.scale_x) + "x" +
                std::to_string(in.scale_y) + "x" +
                std::to_string(in.scale_z),
            "rigid Transform rotation " +
                std::to_string(in.angle_deg) + "°",
        } },
        { "deformation_matrix",  deformMatrix },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(afterRot, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::deformation_warp applied: scale=({},{},{}) rot={}° "
                  "vol {:.2f}→{:.2f}",
                  in.scale_x, in.scale_y, in.scale_z, in.angle_deg,
                  startVol, endVol);

    return SkillOutput{ wpNew, sig };
}

// ── recognition ──────────────────────────────────────────────────────────
//
// A warp leaves no analytic signature on geometry alone — every face has
// simply been transformed.  Strategy: history replay (1.0).  Geometric
// fallback is impossible without a "before" reference, so we only return
// candidates when the feature-history records the warp.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rp = {
            { "scale_x",     f.params.value("scale_x", 1.0) },
            { "scale_y",     f.params.value("scale_y", 1.0) },
            { "scale_z",     f.params.value("scale_z", 1.0) },
            { "axis_origin", f.params.value("axis_origin",
                                            json::array({0,0,0})) },
            { "axis_dir",    f.params.value("axis_dir",
                                            json::array({0,0,1})) },
            { "angle_deg",   f.params.value("angle_deg", 0.0) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rp, 1.0,
            json{ { "source", "feature_history" } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::deformation_warp
