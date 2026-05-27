// @lat: [[engine/skills#pocket_with_corner_relief]]

#include "pocket_with_corner_relief.hpp"

#include "Workpiece.hpp"
#include "drill_hole.hpp"
#include "mill_rect_pocket.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <utility>

namespace koocadcam::skill::pocket_with_corner_relief {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.length_mm <= 0.0 || in.width_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "pocket_with_corner_relief: length and width must be > 0");
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "pocket_with_corner_relief: depth must be > 0");
    }
    if (in.corner_relief_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "pocket_with_corner_relief: corner_relief_dia_mm must be > 0");
    }
    if (in.corner_relief_dia_mm > 0.0 && in.corner_relief_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "pocket_with_corner_relief: corner_relief_dia " +
              std::to_string(in.corner_relief_dia_mm) +
              " mm < min 0.8 mm");
    }
    const double cornerR = in.corner_relief_dia_mm / 2.0;
    if (in.corner_relief_dia_mm > 0.0 && cornerR < 0.2) {
        r.add("DFM-004", "error",
              "pocket_with_corner_relief: derived corner_r " +
              std::to_string(cornerR) + " mm < min R 0.2 mm");
    }

    // Geometry sanity: the relief drills must fit inside the pocket
    // footprint (corner_r < min(length, width)/2 — same as mill_rect_pocket).
    const double minDim = std::min(in.length_mm, in.width_mm);
    if (cornerR > 0.0 && cornerR >= minDim / 2.0) {
        r.add("DFM-INPUT", "error",
              "pocket_with_corner_relief: relief drills would overlap "
              "(corner_r >= min(length, width)/2)");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "pocket_with_corner_relief DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double halfL = in.length_mm / 2.0;
    const double halfW = in.width_mm  / 2.0;

    // Corner positions (XY of each drilled relief).
    const std::array<std::pair<double, double>, 4> corners {{
        { in.center_x_mm - halfL, in.center_y_mm - halfW },
        { in.center_x_mm + halfL, in.center_y_mm - halfW },
        { in.center_x_mm - halfL, in.center_y_mm + halfW },
        { in.center_x_mm + halfL, in.center_y_mm + halfW },
    }};

    constexpr double kReliefExtraDepth = 0.5;   // drills go 0.5 mm past floor
    const double reliefDepth = in.depth_mm + kReliefExtraDepth;

    // ── Step 1: drill 4 corner-relief holes sequentially ──────────────
    //
    // Each drill_hole::apply produces a new Workpiece (drill_hole's apply
    // re-constructs Workpiece without carrying feature history, just like
    // all the other slice-1 skills — see drill_hole.cpp ~L178).  Our
    // outer signature (appended at the end) is what CAPP cares about.
    // The intermediate per-corner signatures are not preserved; that's
    // consistent with the slice-1 skill contract.
    std::shared_ptr<Workpiece> cur =
        std::make_shared<Workpiece>(wp.shape(), wp.material());

    for (const auto& [cx, cy] : corners) {
        drill_hole::Input dh;
        dh.entry_face    = in.entry_face;
        dh.position_x_mm = cx;
        dh.position_y_mm = cy;
        dh.axis_dir      = in.axis_dir;
        dh.diameter_mm   = in.corner_relief_dia_mm;
        dh.depth_mm      = reliefDepth;
        dh.through_hole  = false;
        SkillOutput out = drill_hole::apply(*cur, dh);
        cur = out.workpiece;
    }

    // ── Step 2: mill the rectangular pocket on the drilled workpiece ──
    mill_rect_pocket::Input mp;
    mp.entry_face  = in.entry_face;
    mp.center_x_mm = in.center_x_mm;
    mp.center_y_mm = in.center_y_mm;
    mp.axis_dir    = in.axis_dir;
    mp.length_mm   = in.length_mm;
    mp.width_mm    = in.width_mm;
    mp.depth_mm    = in.depth_mm;
    mp.corner_r_mm = in.corner_relief_dia_mm / 2.0;   // exactly inscribes the relief
    SkillOutput pocketOut = mill_rect_pocket::apply(*cur, mp);

    // ── Re-stamp the signature for the macro ──────────────────────────
    FeatureSignature sig = pocketOut.signature;
    sig.skill_id = kSkillId;

    sig.params = {
        { "center_x_mm",          in.center_x_mm },
        { "center_y_mm",          in.center_y_mm },
        { "axis_dir",             { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "length_mm",            in.length_mm },
        { "width_mm",             in.width_mm },
        { "depth_mm",             in.depth_mm },
        { "corner_relief_dia_mm", in.corner_relief_dia_mm },
    };

    json cornerPositions = json::array();
    for (const auto& [cx, cy] : corners)
        cornerPositions.push_back({ { "x", cx }, { "y", cy } });

    sig.pattern = {
        { "kind",                       kSkillId },
        { "planar_wall_count",          4 },
        { "corner_cylinder_count",      4 },   // the relief drills
        { "bottom_planar_face_present", true },
        { "length_mm",                  in.length_mm },
        { "width_mm",                   in.width_mm },
        { "depth_mm",                   in.depth_mm },
        { "corner_relief_dia_mm",       in.corner_relief_dia_mm },
        { "corner_relief_depth_mm",     reliefDepth },
        { "corner_positions",           cornerPositions },
        { "axis_dir",                   { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
    };

    ToolingMeta tooling = sig.tooling;
    tooling.tool_type = "drill;end_mill";
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type",  "drill" },
              { "tool_dia_mm", in.corner_relief_dia_mm },
              { "depth_mm",    reliefDepth },
              { "pass_count",  4 },
              { "note",        "4 corner relief drills" } },
            { { "tool_type", "end_mill" },
              { "tool_dia_mm", in.corner_relief_dia_mm },
              { "depth_mm",    in.depth_mm },
              { "note",        "rect pocket; corner_r matches relief" } },
        } },
    };
    sig.tooling = tooling;

    auto wpNew = pocketOut.workpiece;
    wpNew->addFeature(sig);

    spdlog::debug("skill::pocket_with_corner_relief applied: {}x{}x{} relief={}",
                  in.length_mm, in.width_mm, in.depth_mm, in.corner_relief_dia_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Strategy (slice-1, geometric heuristic):
//   For each rect pocket recognized by mill_rect_pocket, look at all
//   drill_hole candidates and see if at least 4 of them sit at the pocket
//   corners (within tolerance ≈ corner_r) AND extend DEEPER than the pocket
//   bottom.  When that's true, this is a pocket_with_corner_relief.
//
// We accept the standard caveats around mill_rect_pocket's recognize() —
// see the TODO in mill_rect_pocket_test.cpp about STEP round-trip identity
// matching.  When mill_rect_pocket finds no candidates (post-STEP-import),
// this skill's recognize() will also return empty; that's expected slice-1
// behaviour, not a bug here.

namespace {

bool nearPoint(double x1, double y1, double x2, double y2, double tolMm)
{
    const double dx = x1 - x2;
    const double dy = y1 - y2;
    return std::sqrt(dx*dx + dy*dy) < tolMm;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    auto pockets = mill_rect_pocket::recognize(wp);
    auto holes   = drill_hole::recognize(wp);
    if (pockets.empty() || holes.empty()) return out;

    for (const auto& p : pockets) {
        // Recover pocket geometry.
        const auto itCx = p.recovered_params.find("center_x_mm");
        const auto itCy = p.recovered_params.find("center_y_mm");
        const auto itL  = p.recovered_params.find("length_mm");
        const auto itW  = p.recovered_params.find("width_mm");
        const auto itD  = p.recovered_params.find("depth_mm");
        if (itCx == p.recovered_params.end() ||
            itCy == p.recovered_params.end() ||
            itL  == p.recovered_params.end() ||
            itW  == p.recovered_params.end() ||
            itD  == p.recovered_params.end()) continue;

        const double cx = itCx->get<double>();
        const double cy = itCy->get<double>();
        const double lx = itL->get<double>();
        const double wy = itW->get<double>();
        const double dz = itD->get<double>();

        const double halfL = lx / 2.0;
        const double halfW = wy / 2.0;
        const std::array<std::pair<double, double>, 4> corners {{
            { cx - halfL, cy - halfW },
            { cx + halfL, cy - halfW },
            { cx - halfL, cy + halfW },
            { cx + halfL, cy + halfW },
        }};

        // Scan drill candidates: keep those (a) near a corner and (b)
        // deeper than the pocket floor.
        std::array<bool, 4>   cornerHit{ false, false, false, false };
        std::array<double, 4> cornerDia{ 0.0, 0.0, 0.0, 0.0 };
        // Tolerance for "at-corner" — we accept up to half of the smaller
        // pocket dim, but no smaller than 1mm.
        const double posTol = std::max(1.0, std::min(halfL, halfW) * 0.5);

        for (const auto& h : holes) {
            const auto ix = h.recovered_params.find("position_x_mm");
            const auto iy = h.recovered_params.find("position_y_mm");
            const auto id = h.recovered_params.find("depth_mm");
            const auto iD = h.recovered_params.find("diameter_mm");
            if (ix == h.recovered_params.end() ||
                iy == h.recovered_params.end() ||
                id == h.recovered_params.end() ||
                iD == h.recovered_params.end()) continue;
            const double hx    = ix->get<double>();
            const double hy    = iy->get<double>();
            const double hDep  = id->get<double>();
            const double hDia  = iD->get<double>();
            if (hDep <= dz + 1e-3) continue;  // must be deeper than floor
            for (int k = 0; k < 4; ++k) {
                if (cornerHit[k]) continue;
                if (nearPoint(hx, hy, corners[k].first, corners[k].second, posTol)) {
                    cornerHit[k] = true;
                    cornerDia[k] = hDia;
                    break;
                }
            }
        }

        int hits = 0;
        for (bool h : cornerHit) if (h) ++hits;
        if (hits < 4) continue;

        // All four corners have a relief.  Reliefs should be roughly the
        // same diameter; take their mean.
        double diaMean = 0.0;
        for (double d : cornerDia) diaMean += d;
        diaMean /= 4.0;

        // axis_dir is not guaranteed in mill_rect_pocket recovered_params;
        // default to a -Z assumption when absent.
        nlohmann::json axisJson = json::array({ 0.0, 0.0, -1.0 });
        const auto itAx = p.recovered_params.find("axis_dir");
        if (itAx != p.recovered_params.end()) axisJson = *itAx;

        json rp = {
            { "center_x_mm",          cx },
            { "center_y_mm",          cy },
            { "axis_dir",             axisJson },
            { "length_mm",            lx },
            { "width_mm",             wy },
            { "depth_mm",             dz },
            { "corner_relief_dia_mm", diaMean },
        };
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = rp;
        r.confidence       = 0.85;
        r.matched_geometry = {
            { "rect_pocket",  p.matched_geometry },
            { "corner_count", hits },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::pocket_with_corner_relief
