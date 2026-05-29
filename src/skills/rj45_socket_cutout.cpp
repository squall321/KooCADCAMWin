// @lat: [[engine/skills#rj45_socket_cutout]]

#include "rj45_socket_cutout.hpp"

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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::rj45_socket_cutout {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── 8P8C panel-mount jack dimensions (TIA/EIA-568 / Stewart SS-39000) ───
namespace {
constexpr double kPocketWidth_mm   = 13.0;
constexpr double kPocketHeight_mm  = 14.5;
constexpr double kPocketCornerR_mm =  0.5;
constexpr double kTabReliefW_mm    =  3.0;
constexpr double kTabReliefH_mm    =  1.0;   // y-extent below pocket
constexpr double kTabReliefD_mm    =  2.0;   // depth into panel
constexpr double kChamfer_mm       =  0.5;
constexpr double kMaxPanelThk_mm   =  5.0;   // jack thread engagement limit
constexpr double kMinSpacing_mm    = 14.0;   // TIA/EIA-568 pitch min
}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.port_count < 1 || in.port_count > 24) {
        r.add("DFM-INPUT", "error",
              "rj45_socket_cutout: port_count " +
              std::to_string(in.port_count) + " outside [1, 24]");
    }
    if (in.port_count > 1 && in.spacing_mm < kMinSpacing_mm) {
        r.add("DFM-RJ45-PITCH", "error",
              "rj45_socket_cutout: spacing_mm " +
              std::to_string(in.spacing_mm) +
              " < TIA/EIA minimum " + std::to_string(kMinSpacing_mm) + " mm");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness = zMax - zMin;
    if (thickness > kMaxPanelThk_mm) {
        r.add("DFM-RJ45-PANEL", "error",
              "rj45_socket_cutout: panel thickness " +
              std::to_string(thickness) + " mm > " +
              std::to_string(kMaxPanelThk_mm) +
              " mm (jack thread engagement limit)");
    }

    if constexpr (kChamfer_mm < 0.3) {
        r.add("DFM-002", "error",
              "rj45_socket_cutout: chamfer < 0.3 mm (not machinable)");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "rj45_socket_cutout DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.face);
    if (!entryId) throw SkillError("rj45_socket_cutout: face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness = zMax - zMin;

    constexpr double kOverhang = 0.05;
    const double topZ = zMax;
    const double botZ = zMin;

    const double rad = in.direction_deg * M_PI / 180.0;
    const gp_Dir rowDir(std::cos(rad), std::sin(rad), 0.0);

    std::vector<TopoDS_Shape> allCutters;
    allCutters.reserve(in.port_count * 3);

    for (int i = 0; i < in.port_count; ++i) {
        const double cx = in.position_x_mm + i * in.spacing_mm * rowDir.X();
        const double cy = in.position_y_mm + i * in.spacing_mm * rowDir.Y();

        // (1) Main through pocket — rounded rect, full-thickness cut.
        const gp_Pnt mainBottom(cx, cy, botZ - kOverhang);
        const TopoDS_Shape mainTool = pr::roundedRectPocketTool(
            mainBottom, kPocketWidth_mm, kPocketHeight_mm,
            thickness + 2 * kOverhang, kPocketCornerR_mm);

        // (2) Locking-tab relief — small box on bottom edge of main pocket.
        const gp_Pnt tabOrigin(
            cx - kTabReliefW_mm / 2.0,
            cy - kPocketHeight_mm / 2.0 - kTabReliefH_mm,
            topZ - kTabReliefD_mm);
        const gp_Ax2 tabAx(tabOrigin, gp_Dir(0.0, 0.0, 1.0));
        const TopoDS_Shape tabTool = pr::box(tabAx,
            kTabReliefW_mm, kTabReliefH_mm, kTabReliefD_mm + kOverhang);

        // (3) Edge chamfer — a 0.5 × 0.5 mm ring around the pocket on top.
        //                    Modelled as a slightly larger rounded-rect
        //                    pocket of depth = chamfer, fused on top.
        const gp_Pnt chamferBottom(cx, cy, topZ - kChamfer_mm);
        const TopoDS_Shape chamferTool = pr::roundedRectPocketTool(
            chamferBottom,
            kPocketWidth_mm  + 2 * kChamfer_mm,
            kPocketHeight_mm + 2 * kChamfer_mm,
            kChamfer_mm + kOverhang,
            kPocketCornerR_mm + kChamfer_mm);

        // Fuse the three sub-cuts (concentric overlap on top face).
        const TopoDS_Shape portFused = pr::fuseMany(mainTool, { tabTool, chamferTool });
        allCutters.push_back(portFused);
    }

    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), allCutters);

    // ── Signature ───────────────────────────────────────────────────────
    json positions = json::array();
    for (int i = 0; i < in.port_count; ++i) {
        positions.push_back({
            { "x", in.position_x_mm + i * in.spacing_mm * rowDir.X() },
            { "y", in.position_y_mm + i * in.spacing_mm * rowDir.Y() },
        });
    }

    json params = {
        { "entry_face_id",  *entryId },
        { "position_x_mm",  in.position_x_mm },
        { "position_y_mm",  in.position_y_mm },
        { "axis_dir",       { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "port_count",     in.port_count },
        { "spacing_mm",     in.spacing_mm },
        { "direction_deg",  in.direction_deg },
    };

    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    3 },
        { "port_count",          in.port_count },
        { "spacing_mm",          in.spacing_mm },
        { "pocket_width_mm",     kPocketWidth_mm },
        { "pocket_height_mm",    kPocketHeight_mm },
        { "pocket_corner_r_mm",  kPocketCornerR_mm },
        { "tab_relief_w_mm",     kTabReliefW_mm },
        { "tab_relief_h_mm",     kTabReliefH_mm },
        { "tab_relief_d_mm",     kTabReliefD_mm },
        { "chamfer_mm",          kChamfer_mm },
        { "positions",           positions },
        { "axis_dir",            { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "subfeatures",         {
            { { "kind", "main_through_pocket" },
              { "size_mm", { kPocketWidth_mm, kPocketHeight_mm } } },
            { { "kind", "tab_relief_slot" },
              { "size_mm", { kTabReliefW_mm, kTabReliefH_mm, kTabReliefD_mm } } },
            { { "kind", "front_chamfer" },
              { "break_mm", kChamfer_mm } },
        } },
    };

    ToolingMeta tooling;
    tooling.tool_type        = "end_mill;chamfer_mill";
    tooling.tool_dia_mm      = 3.0;
    tooling.tool_length_mm   = thickness * 1.5 + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 4;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.04;
    const double pocketArea  = kPocketWidth_mm * kPocketHeight_mm;
    const double tabVol      = kTabReliefW_mm * kTabReliefH_mm * kTabReliefD_mm;
    const double chamferVol  = kChamfer_mm *
        (2 * (kPocketWidth_mm + kPocketHeight_mm) + 4 * kChamfer_mm) * kChamfer_mm;
    const double perPortVol  = pocketArea * thickness + tabVol + chamferVol;
    tooling.stock_removed_mm3 = perPortVol * in.port_count;
    tooling.est_cycle_time_s  = std::max(2.0, in.port_count * 4.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "end_mill" },     { "tool_dia_mm", 3.0 },
              { "note", "main rounded-rect pocket" } },
            { { "tool_type", "end_mill" },     { "tool_dia_mm", 1.0 },
              { "note", "locking-tab relief slot" } },
            { { "tool_type", "chamfer_mill" }, { "tool_dia_mm", 3.0 },
              { "note", "0.5 mm × 45° break on front edge" } },
        } },
        { "standard", "TIA/EIA-568 8P8C" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::rj45_socket_cutout applied: ports={} pitch={} faces {}→{}",
                  in.port_count, in.spacing_mm, wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        const auto& p = f.pattern;
        json recovered = {
            { "position_x_mm", f.params.value("position_x_mm", 0.0) },
            { "position_y_mm", f.params.value("position_y_mm", 0.0) },
            { "port_count",    f.params.value("port_count",    1) },
            { "spacing_mm",    f.params.value("spacing_mm",    16.0) },
            { "direction_deg", f.params.value("direction_deg", 0.0) },
            { "axis_dir",      f.params.value("axis_dir",      json::array({0.0,0.0,-1.0})) },
        };
        json matched = {
            { "source",           "feature_history_replay" },
            { "subfeature_count", p.value("subfeature_count", 0) },
            { "port_count",       p.value("port_count",       0) },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 1.0, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::rj45_socket_cutout
