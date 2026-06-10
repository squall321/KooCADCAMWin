// @lat: [[engine/skills#rest_milling]]

#include "rest_milling.hpp"

#include "Workpiece.hpp"
#include "drill_hole.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <utility>

namespace koocadcam::skill::rest_milling {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.length_mm <= 0.0 || in.width_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "rest_milling: length_mm and width_mm must be > 0");
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "rest_milling: depth_mm must be > 0");
    }
    if (in.cleanup_tool_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "rest_milling: cleanup_tool_dia_mm " +
              std::to_string(in.cleanup_tool_dia_mm) +
              " < min 0.8 mm (end-mill min size)");
    }
    if (in.previous_tool_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "rest_milling: previous_tool_dia_mm must be > 0 "
              "(rest milling cleans up after a previous tool)");
    }
    if (in.cleanup_tool_dia_mm > 0.0 && in.previous_tool_dia_mm > 0.0 &&
        in.cleanup_tool_dia_mm >= in.previous_tool_dia_mm - 1e-9) {
        r.add("DFM-REST-DIA", "error",
              "rest_milling: cleanup_tool_dia (" +
              std::to_string(in.cleanup_tool_dia_mm) +
              ") must be < previous_tool_dia (" +
              std::to_string(in.previous_tool_dia_mm) +
              ") — rest milling needs a smaller tool");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Strategy: place 4 drill_hole calls at the corners of the input rectangle,
// each at cleanup_tool_dia and reaching pocket-bottom depth.  This models
// the cleanup tool reaching into each corner without re-cutting the bulk
// pocket material.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "rest_milling DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double halfL = in.length_mm / 2.0;
    const double halfW = in.width_mm  / 2.0;

    // Position the cleanup tool one tool radius INWARD from each rectangle
    // corner so the tool fits geometrically.  At slice-1 fidelity this is a
    // visual stand-in for the "tool reaches into the corner" effect.
    const double r = in.cleanup_tool_dia_mm / 2.0;
    const std::array<std::pair<double, double>, 4> corners {{
        { in.center_x_mm - halfL + r, in.center_y_mm - halfW + r },
        { in.center_x_mm + halfL - r, in.center_y_mm - halfW + r },
        { in.center_x_mm - halfL + r, in.center_y_mm + halfW - r },
        { in.center_x_mm + halfL - r, in.center_y_mm + halfW - r },
    }};

    std::shared_ptr<Workpiece> cur =
        std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) cur->addFeature(prev);  // keep full chain history

    for (const auto& [cx, cy] : corners) {
        drill_hole::Input dh;
        dh.entry_face    = in.entry_face;
        dh.position_x_mm = cx;
        dh.position_y_mm = cy;
        dh.axis_dir      = in.axis_dir;
        dh.diameter_mm   = in.cleanup_tool_dia_mm;
        dh.depth_mm      = in.depth_mm;
        dh.through_hole  = false;
        SkillOutput out  = drill_hole::apply(*cur, dh);
        cur = out.workpiece;
    }

    // ── Build the rest_milling signature ──────────────────────────────
    json params = {
        { "center_x_mm",            in.center_x_mm },
        { "center_y_mm",            in.center_y_mm },
        { "axis_dir",               { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "length_mm",              in.length_mm },
        { "width_mm",               in.width_mm },
        { "depth_mm",               in.depth_mm },
        { "previous_tool_dia_mm",   in.previous_tool_dia_mm },
        { "cleanup_tool_dia_mm",    in.cleanup_tool_dia_mm },
    };

    json cornerPositions = json::array();
    for (const auto& [cx, cy] : corners)
        cornerPositions.push_back({ { "x", cx }, { "y", cy } });

    json pattern = {
        { "kind",                       kSkillId },
        { "corner_cylinder_count",      4 },
        { "length_mm",                  in.length_mm },
        { "width_mm",                   in.width_mm },
        { "depth_mm",                   in.depth_mm },
        { "previous_tool_dia_mm",       in.previous_tool_dia_mm },
        { "cleanup_tool_dia_mm",        in.cleanup_tool_dia_mm },
        { "corner_positions",           cornerPositions },
        { "axis_dir",                   { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type                       = "end_mill_rest";
    tooling.tool_dia_mm                     = in.cleanup_tool_dia_mm;
    tooling.tool_length_mm                  = in.depth_mm * 1.5 + 5.0;
    tooling.tool_material                   = "carbide";
    tooling.flute_count                     = 2;
    tooling.cutting_speed_sfm               = 500.0;   // slower — small tool
    tooling.feed_per_tooth_mm               = 0.010;
    tooling.stock_removed_mm3               = 4.0 * M_PI * r * r * in.depth_mm;
    tooling.est_cycle_time_s                = std::max(1.0,
        4.0 * in.depth_mm * 0.5 + 5.0);
    tooling.extra["strategy"]               = "rest_milling";
    tooling.extra["previous_tool_dia_mm"]   = in.previous_tool_dia_mm;
    tooling.extra["cleanup_tool_dia_mm"]    = in.cleanup_tool_dia_mm;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = cur;
    wpNew->addFeature(sig);

    spdlog::debug("skill::rest_milling applied: {}x{}x{} prev_tool={} cleanup_tool={}",
                  in.length_mm, in.width_mm, in.depth_mm,
                  in.previous_tool_dia_mm, in.cleanup_tool_dia_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Feature-history first (high confidence).  For geometric recognition we
// look at drill_hole candidates and group them into sets of 4 that form a
// rectangle — same shape as pocket_with_corner_relief, but with no
// connecting pocket material.  Confidence 0.70 — the topology is the same
// as a 4-hole drill pattern.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // ── A. Feature history ──────────────────────────────────────────
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 0.90;
        rf.matched_geometry = {
            { "source", "feature_history" },
        };
        out.push_back(rf);
    }
    if (!out.empty()) return out;

    // ── B. Geometric heuristic: 4 holes in a rectangle ───────────────
    auto holes = drill_hole::recognize(wp);
    if (holes.size() < 4) return out;

    // Cluster on (diameter, depth) — rest_milling produces 4 identical-spec
    // holes.  Only consider holes whose recovered_params contain all the
    // keys we need.
    struct H { double x, y, d, dep; };
    std::vector<H> hs;
    for (const auto& h : holes) {
        const auto ix = h.recovered_params.find("position_x_mm");
        const auto iy = h.recovered_params.find("position_y_mm");
        const auto id = h.recovered_params.find("diameter_mm");
        const auto ip = h.recovered_params.find("depth_mm");
        if (ix == h.recovered_params.end() ||
            iy == h.recovered_params.end() ||
            id == h.recovered_params.end() ||
            ip == h.recovered_params.end()) continue;
        hs.push_back({
            ix->get<double>(), iy->get<double>(),
            id->get<double>(), ip->get<double>()
        });
    }
    if (hs.size() < 4) return out;

    // Try the first 4 — if they form a rectangle, emit.
    // (A full O(n⁴) search is unnecessary at slice-1 fidelity.)
    auto formsRect = [](const std::vector<H>& a) {
        if (a.size() < 4) return false;
        // Same diameter + depth, 4 corners forming a rectangle.
        for (size_t i = 1; i < 4; ++i) {
            if (std::abs(a[i].d   - a[0].d  ) > 0.05) return false;
            if (std::abs(a[i].dep - a[0].dep) > 0.05) return false;
        }
        std::vector<double> xs{ a[0].x, a[1].x, a[2].x, a[3].x };
        std::vector<double> ys{ a[0].y, a[1].y, a[2].y, a[3].y };
        std::sort(xs.begin(), xs.end());
        std::sort(ys.begin(), ys.end());
        // Two pairs of equal X, two pairs of equal Y.
        return std::abs(xs[0] - xs[1]) < 0.1 && std::abs(xs[2] - xs[3]) < 0.1 &&
               std::abs(ys[0] - ys[1]) < 0.1 && std::abs(ys[2] - ys[3]) < 0.1;
    };

    std::vector<H> first4(hs.begin(), hs.begin() + 4);
    if (!formsRect(first4)) return out;

    // Recover input from the rectangle of corners.
    double cx = 0.0, cy = 0.0;
    for (const auto& h : first4) { cx += h.x; cy += h.y; }
    cx /= 4.0; cy /= 4.0;
    double maxDX = 0.0, maxDY = 0.0;
    for (const auto& h : first4) {
        maxDX = std::max(maxDX, std::abs(h.x - cx));
        maxDY = std::max(maxDY, std::abs(h.y - cy));
    }
    // The rest-milling synthesizer offset each hole inward by tool radius —
    // back it out.
    const double cleanupDia  = first4[0].d;
    const double cleanupR    = cleanupDia / 2.0;
    const double length      = 2.0 * (maxDX + cleanupR);
    const double width       = 2.0 * (maxDY + cleanupR);
    const double depth       = first4[0].dep;

    json rp = {
        { "center_x_mm",            cx },
        { "center_y_mm",            cy },
        { "axis_dir",               { 0.0, 0.0, -1.0 } },
        { "length_mm",              length },
        { "width_mm",               width },
        { "depth_mm",               depth },
        { "previous_tool_dia_mm",   0.0 },   // unknown from geometry alone
        { "cleanup_tool_dia_mm",    cleanupDia },
    };
    RecognizedFeature rf;
    rf.skill_id         = kSkillId;
    rf.recovered_params = rp;
    rf.confidence       = 0.70;
    rf.matched_geometry = {
        { "source", "geometric_heuristic" },
        { "corner_count", 4 },
        { "note",
          "4 identical-spec corner drills in a rectangle pattern; "
          "previous_tool_dia not recoverable from geometry" },
    };
    out.push_back(rf);
    return out;
}

}  // namespace koocadcam::skill::rest_milling
