// @lat: [[engine/skills#access_panel_cutout_radiused]]

#include "access_panel_cutout_radiused.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <array>
#include <cmath>
#include <memory>
#include <utility>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::access_panel_cutout_radiused {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.panel_w_mm <= 0.0 || in.panel_h_mm <= 0.0 ||
        in.corner_radius_mm <= 0.0 || in.rivet_dia_mm <= 0.0 ||
        in.nutplate_count <= 0) {
        r.add("DFM-INPUT", "error",
              "access_panel_cutout_radiused: all dims/count must be > 0");
        return r;
    }

    const double minHalf = std::min(in.panel_w_mm, in.panel_h_mm) / 2.0;
    if (in.corner_radius_mm >= minHalf) {
        r.add("DFM-CORNER", "error",
              "access_panel_cutout_radiused: corner_radius_mm (" +
              std::to_string(in.corner_radius_mm) +
              ") must be < min(panel_w,panel_h)/2 (" +
              std::to_string(minHalf) + ")");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "access_panel_cutout_radiused DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double thickness = zMax - zMin;
    const double cutDepth  = thickness + 2.0 * kOver;    // full-depth opening

    const double cx = in.center_xy.X();
    const double cy = in.center_xy.Y();
    const double hw = in.panel_w_mm / 2.0;
    const double hh = in.panel_h_mm / 2.0;
    const double r  = in.corner_radius_mm;

    // ── 1) Rectangular panel cutout (box, full web depth) ────────────────
    const gp_Pnt boxOrigin(cx - hw, cy - hh, topZ + kOver - cutDepth);
    const TopoDS_Shape boxTool = pr::box(
        gp_Ax2(boxOrigin, gp::DZ()),
        in.panel_w_mm, in.panel_h_mm, cutDepth);
    TopoDS_Shape current = pr::cut(wp.shape(), boxTool);

    // ── 1b) 4 corner-relief cylinders (radius the corners), sequential ───
    const std::array<std::pair<double, double>, 4> corners {{
        { cx - hw, cy - hh },
        { cx + hw, cy - hh },
        { cx - hw, cy + hh },
        { cx + hw, cy + hh },
    }};
    for (const auto& [px, py] : corners) {
        const gp_Pnt cStart(px, py, topZ + kOver - cutDepth);
        const gp_Ax2 cAx(cStart, gp::DZ());
        current = pr::cut(current, pr::cylinder(cAx, r, cutDepth));
    }

    // ── 2) Perimeter nutplate rivet holes (sequential, one each) ─────────
    const double rivetR = in.rivet_dia_mm / 2.0;
    const double ringW  = hw + 2.0 * r;          // ring just outside the panel
    const double ringH  = hh + 2.0 * r;
    const double rivetDepth = thickness + 2.0 * kOver;
    json rivets_json = json::array();
    for (int i = 0; i < in.nutplate_count; ++i) {
        const double theta = 2.0 * M_PI * static_cast<double>(i) /
                             static_cast<double>(in.nutplate_count);
        const double rx = cx + ringW * std::cos(theta);
        const double ry = cy + ringH * std::sin(theta);
        const gp_Pnt rStart(rx, ry, topZ + kOver - rivetDepth);
        const gp_Ax2 rAx(rStart, gp::DZ());
        current = pr::cut(current, pr::cylinder(rAx, rivetR, rivetDepth));
        rivets_json.push_back({ { "x_mm", rx }, { "y_mm", ry } });
    }

    // Analytic removed volume: box + 4 corner-relief (~quarter each) +
    // nutplate rivets.
    const double vBox    = in.panel_w_mm * in.panel_h_mm * thickness;
    const double vCorner = 4.0 * 0.25 * M_PI * r * r * thickness;
    const double vRivets = static_cast<double>(in.nutplate_count) *
                           M_PI * rivetR * rivetR * thickness;
    const double volRemoved = vBox + vCorner + vRivets;

    json params = {
        { "center_xy",        { in.center_xy.X(),
                                in.center_xy.Y(),
                                in.center_xy.Z() } },
        { "panel_w_mm",       in.panel_w_mm },
        { "panel_h_mm",       in.panel_h_mm },
        { "corner_radius_mm", in.corner_radius_mm },
        { "nutplate_count",   in.nutplate_count },
        { "rivet_dia_mm",     in.rivet_dia_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "aerostruct_feature_type",    "access_panel_cutout_radiused" },
        { "subfeature_count",           1 + in.nutplate_count },
        { "nutplate_count",             in.nutplate_count },
        { "rivet_positions",            rivets_json },
        { "derived_corner_radius_mm",   in.corner_radius_mm },
        { "derived_rivet_dia_mm",       in.rivet_dia_mm },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "radiused access-panel cutout" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill;drill";
    tooling.tool_dia_mm       = 2.0 * in.corner_radius_mm;
    tooling.tool_length_mm    = cutDepth + 10.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(60.0, 8.0 * in.nutplate_count);
    tooling.extra = {
        { "aerostruct_feature_type", "access_panel_cutout_radiused" },
        { "nutplate_count",          in.nutplate_count },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::access_panel_cutout_radiused: {}x{} r={} nutplates={}",
                  in.panel_w_mm, in.panel_h_mm, in.corner_radius_mm,
                  in.nutplate_count);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

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
            { "source",                  "metadata_replay" },
            { "is_compound",             true },
            { "aerostruct_feature_type", "access_panel_cutout_radiused" },
        };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric fallback: many small +Z cylinders (nutplate rivet ring).
    int rivetCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Dir d = s.Cylinder().Axis().Direction();
            if (std::abs(d.Z()) > 0.9 && s.Cylinder().Radius() <= 4.0) ++rivetCyls;
        } catch (...) {}
    }
    if (rivetCyls >= 4) {
        json recovered = { { "nutplate_count", rivetCyls },
                           { "rivet_dia_mm",   3.2 } };
        json matched   = { { "source",     "geometric_nutplate_ring" },
                           { "rivet_cyls", rivetCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::access_panel_cutout_radiused
