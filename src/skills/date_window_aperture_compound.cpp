// @lat: [[engine/skills#date_window_aperture_compound]]

#include "date_window_aperture_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>

namespace koocadcam::skill::date_window_aperture_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;

// Axis-aligned box cut tool whose XY footprint is centred at (cx, cy) at a
// given bottom Z, extruding +Z by `depth`.
TopoDS_Shape centredBox(double cx, double cy, double bottomZ,
                        double sx, double sy, double depth)
{
    const gp_Pnt origin(cx - sx / 2.0, cy - sy / 2.0, bottomZ);
    const gp_Ax2 ax(origin, gp::DZ());
    return pr::box(ax, sx, sy, depth);
}
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.window_len_mm <= 0.0 || in.window_wid_mm <= 0.0 ||
        in.glass_step_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "date_window_aperture_compound: window/glass dims must be > 0");
        return r;
    }

    if (in.bevel_mm <= 0.0 || in.bevel_mm >= in.window_wid_mm / 2.0) {
        r.add("DFM-BEVEL", "error",
              "date_window_aperture_compound: bevel " +
              std::to_string(in.bevel_mm) +
              " mm must be in (0, window_wid/2 = " +
              std::to_string(in.window_wid_mm / 2.0) + ")");
    }

    if (in.glass_step_margin_mm <= in.bevel_mm) {
        r.add("DFM-MARGIN", "error",
              "date_window_aperture_compound: glass_step_margin " +
              std::to_string(in.glass_step_margin_mm) +
              " mm must exceed bevel " + std::to_string(in.bevel_mm) + " mm");
    }

    if (!wp.shape().IsNull()) {
        double xMin, yMin, zMin, xMax, yMax, zMax;
        wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
        const double thickness = zMax - zMin;
        if (in.glass_step_depth_mm >= thickness) {
            r.add("DFM-STEP", "error",
                  "date_window_aperture_compound: glass_step_depth " +
                  std::to_string(in.glass_step_depth_mm) +
                  " mm >= stock thickness " + std::to_string(thickness) + " mm");
        }
    }
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "date_window_aperture_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double thickness = zMax - zMin;
    const double cx = in.center_xy.X();
    const double cy = in.center_xy.Y();

    // ── 1) Window aperture (rectangular through-cut DOWN from top) ───────
    const double throughDepth = thickness + 2.0 * kOver;
    const TopoDS_Shape apertureTool = centredBox(
        cx, cy, zMin - kOver,
        in.window_len_mm, in.window_wid_mm, throughDepth);
    TopoDS_Shape current = pr::cut(wp.shape(), apertureTool);

    // ── 2) Beveled frame (wider shallow box step around the aperture) ────
    // Approximated as a wider shallow rebate (window + 2*bevel) just below the
    // surface — gives the chamfered surround / printed frame seat.
    const double bevelDepth = std::max(0.3, in.glass_step_depth_mm * 0.5);
    const double frameLen   = in.window_len_mm + 2.0 * in.bevel_mm;
    const double frameWid   = in.window_wid_mm + 2.0 * in.bevel_mm;
    const TopoDS_Shape frameTool = centredBox(
        cx, cy, topZ - bevelDepth,
        frameLen, frameWid, bevelDepth + kOver);
    current = pr::cut(current, frameTool);

    // ── 3) Glass step recess (widest shallow rebate at the surface) ──────
    const double glassLen = in.window_len_mm + 2.0 * in.glass_step_margin_mm;
    const double glassWid = in.window_wid_mm + 2.0 * in.glass_step_margin_mm;
    const TopoDS_Shape glassTool = centredBox(
        cx, cy, topZ - in.glass_step_depth_mm,
        glassLen, glassWid, in.glass_step_depth_mm + kOver);
    current = pr::cut(current, glassTool);

    const double vAperture = in.window_len_mm * in.window_wid_mm * thickness;
    const double vFrame    = (frameLen * frameWid - in.window_len_mm * in.window_wid_mm)
                             * bevelDepth;
    const double vGlass    = (glassLen * glassWid - frameLen * frameWid)
                             * in.glass_step_depth_mm;
    const double volRemoved = vAperture + std::max(0.0, vFrame) + std::max(0.0, vGlass);

    json params = {
        { "center_xy",            { in.center_xy.X(),
                                    in.center_xy.Y(),
                                    in.center_xy.Z() } },
        { "window_len_mm",        in.window_len_mm },
        { "window_wid_mm",        in.window_wid_mm },
        { "bevel_mm",             in.bevel_mm },
        { "glass_step_depth_mm",  in.glass_step_depth_mm },
        { "glass_step_margin_mm", in.glass_step_margin_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "watch_feature_type",         "date_window_aperture" },
        { "subfeature_count",           3 },
        { "derived_window_len_mm",      in.window_len_mm },
        { "derived_window_wid_mm",      in.window_wid_mm },
        { "derived_frame_len_mm",       frameLen },
        { "derived_frame_wid_mm",       frameWid },
        { "derived_glass_len_mm",       glassLen },
        { "derived_glass_wid_mm",       glassWid },
        { "derived_volume_removed_mm3", volRemoved },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill;chamfer_mill;end_mill";
    tooling.tool_dia_mm       = std::min(in.window_len_mm, in.window_wid_mm);
    tooling.tool_length_mm    = thickness + 2.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(15.0, volRemoved / 40.0);
    tooling.extra = {
        { "watch_application", "date_window_aperture" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::date_window_aperture_compound: len={} wid={} bevel={}",
                  in.window_len_mm, in.window_wid_mm, in.bevel_mm);

    return SkillOutput{ wpNew, sig };
}

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
    if (!out.empty()) return out;

    // Geometric fallback: presence of stacked planar steps (more planar faces
    // than a plain block) hints at a stepped rectangular aperture.
    int planarCount = 0;
    for (int i = 0; i < wp.faceCount(); ++i)
        if (wp.isFacePlanar(i)) ++planarCount;
    if (planarCount >= 12) {
        json recovered = { { "window_len_mm",        4.0 },
                           { "window_wid_mm",         3.0 },
                           { "bevel_mm",              0.4 },
                           { "glass_step_depth_mm",   0.6 },
                           { "glass_step_margin_mm",  1.0 } };
        json matched   = { { "source",        "geometric_stepped_aperture" },
                           { "planar_faces",  planarCount } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::date_window_aperture_compound
