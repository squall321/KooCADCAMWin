// @lat: [[engine/skills#zif_connector_window_compound]]

#include "zif_connector_window_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Shape.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <string>
#include <vector>

namespace koocadcam::skill::zif_connector_window_compound {

namespace pr = koocadcam::engine::prim;
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

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "zif_connector_window_compound: face_id out of range");
    }
    if (!(in.window_length_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "zif_connector_window_compound: window_length_mm must be > 0");
    }
    if (!(in.window_width_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "zif_connector_window_compound: window_width_mm must be > 0");
    }
    if (!(in.window_depth_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "zif_connector_window_compound: window_depth_mm must be > 0");
    }
    if (!(in.locking_tab_width_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "zif_connector_window_compound: locking_tab_width_mm must be > 0");
    }
    if (!(in.locking_tab_depth_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "zif_connector_window_compound: locking_tab_depth_mm must be > 0");
    }
    if (in.window_width_mm > 0.0 && in.window_width_mm < 0.3) {
        r.add("DFM-MIN-HOLE", "error",
              "zif_connector_window_compound: window_width " +
              std::to_string(in.window_width_mm) + " < 0.3 mm minimum");
    }
    if (in.locking_tab_depth_mm > 0.0 && in.locking_tab_depth_mm < 0.4) {
        r.add("DFM-MIN-WALL", "error",
              "zif_connector_window_compound: locking_tab_depth " +
              std::to_string(in.locking_tab_depth_mm) + " < 0.4 mm minimum");
    }
    if (in.window_length_mm > 0.0 &&
        (in.window_length_mm < 4.0 || in.window_length_mm > 25.0)) {
        r.add("DFM-PHONE-RNG", "warning",
              "window_length " + std::to_string(in.window_length_mm) +
              " outside typical phone range [4, 25] mm");
    }
    if (in.window_width_mm > 0.0 &&
        (in.window_width_mm < 0.8 || in.window_width_mm > 2.5)) {
        r.add("DFM-PHONE-RNG", "warning",
              "window_width " + std::to_string(in.window_width_mm) +
              " outside typical phone range [0.8, 2.5] mm");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "zif_connector_window_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const double baseZ = faceC.Z();

    const double L  = in.window_length_mm;
    const double W  = in.window_width_mm;
    const double D  = in.window_depth_mm;
    const double tw = in.locking_tab_width_mm;
    const double td = in.locking_tab_depth_mm;
    const double cx = in.center_x_mm;
    const double cy = in.center_y_mm;
    constexpr double kEmbed = 0.05;

    const double volBefore = volumeOf(wp.shape());
    std::vector<TopoDS_Shape> cutters;

    // ── Op 1: central through-window ────────────────────────────────────
    // If user-specified depth_mm exceeds the local thickness, push a
    // generous through depth so the cut clears the bottom (we use
    // D + 4 mm guard so a 1 mm phone frame is cleanly cut through).
    {
        const double cutDepth = D + 4.0;
        const gp_Pnt origin(cx - L / 2.0, cy - W / 2.0, baseZ - cutDepth + kEmbed);
        const gp_Ax2 ax(origin, gp::DZ());
        cutters.push_back(pr::box(ax, L, W, cutDepth + kEmbed));
    }

    // ── Op 2: left  locking-tab recess ──────────────────────────────────
    {
        const double lx = cx - L / 2.0 - tw;
        const double ly = cy - (W + tw) / 2.0;   // a bit wider than window
        const gp_Pnt origin(lx, ly, baseZ - td);
        const gp_Ax2 ax(origin, gp::DZ());
        cutters.push_back(pr::box(ax, tw, W + tw, td + kEmbed));
    }
    // ── Op 3: right locking-tab recess ──────────────────────────────────
    {
        const double rx = cx + L / 2.0;
        const double ry = cy - (W + tw) / 2.0;
        const gp_Pnt origin(rx, ry, baseZ - td);
        const gp_Ax2 ax(origin, gp::DZ());
        cutters.push_back(pr::box(ax, tw, W + tw, td + kEmbed));
    }

    TopoDS_Shape newShape = pr::cutMany(wp.shape(), cutters);
    if (newShape.IsNull()) {
        throw SkillError("zif_connector_window_compound: cutMany returned null");
    }
    const double volAfter   = volumeOf(newShape);
    const double volRemoved = volBefore - volAfter;

    json params = {
        { "face_id",              in.face_id },
        { "center_x_mm",          in.center_x_mm },
        { "center_y_mm",          in.center_y_mm },
        { "window_length_mm",     in.window_length_mm },
        { "window_width_mm",      in.window_width_mm },
        { "window_depth_mm",      in.window_depth_mm },
        { "locking_tab_width_mm", in.locking_tab_width_mm },
        { "locking_tab_depth_mm", in.locking_tab_depth_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "is_phone_feature",           true },
        { "phone_feature_type",         "zif_window" },
        { "subfeature_count",           static_cast<int>(cutters.size()) },
        { "window_length_mm",           in.window_length_mm },
        { "window_width_mm",            in.window_width_mm },
        { "window_depth_mm",            in.window_depth_mm },
        { "locking_tab_count",          2 },
        { "locking_tab_width_mm",       in.locking_tab_width_mm },
        { "locking_tab_depth_mm",       in.locking_tab_depth_mm },
        { "derived_volume_removed_mm3", volRemoved },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = std::max(0.3, std::min(W, tw) * 0.5);
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = volRemoved / 80.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::zif_connector_window_compound: L={} W={} D={} vol-removed={}",
        L, W, D, volRemoved);

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
        rf.confidence       = 0.92;
        rf.matched_geometry = f.pattern;
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::zif_connector_window_compound
