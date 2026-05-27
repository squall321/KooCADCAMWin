// @lat: [[engine/skills#bore_and_finish]]

#include "bore_and_finish.hpp"

#include "Workpiece.hpp"
#include "bore_cylindrical.hpp"
#include "drill_hole.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::bore_and_finish {

using nlohmann::json;

// ── Tunables for the multi-step finishing sequence ───────────────────────

namespace {

constexpr double kRoughUndersize_mm     = 1.5;   // drill is 1.5 mm under final
constexpr double kSemifinishUndersize_mm = 0.2;  // semi-finish bore is 0.2 mm under final

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.final_diameter_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "bore_and_finish: final_diameter_mm must be > 0");
        return r;
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "bore_and_finish: depth_mm must be > 0");
    }

    // We need enough margin for drill (final-1.5), semi-finish (final-0.2),
    // and the finishing pass.  Final >= 1.5 keeps the drill at >= 0 mm
    // (sub-min 0.8 mm is caught further down via DFM-002).
    if (in.final_diameter_mm < 1.5) {
        r.add("DFM-MARGIN", "error",
              "bore_and_finish: final_diameter " +
              std::to_string(in.final_diameter_mm) +
              " mm < 1.5 mm (not enough margin for multi-step finish)");
    }

    const double drillDia = in.final_diameter_mm - kRoughUndersize_mm;
    if (drillDia < 0.8) {
        r.add("DFM-002", "error",
              "bore_and_finish: derived drill diameter " +
              std::to_string(drillDia) + " mm < 0.8 mm");
    }
    const double semiFinDia = in.final_diameter_mm - kSemifinishUndersize_mm;
    if (semiFinDia < 0.8) {
        r.add("DFM-002", "error",
              "bore_and_finish: derived semi-finish bore diameter " +
              std::to_string(semiFinDia) + " mm < 0.8 mm");
    }

    // Bore-bar deflection check (inherited from bore_cylindrical).
    const double ratio = (in.final_diameter_mm > 0.0)
        ? (in.depth_mm / in.final_diameter_mm) : 0.0;
    if (ratio > 4.0) {
        r.add("DFM-BORE-RATIO", "warning",
              "bore_and_finish depth/dia ratio " + std::to_string(ratio) +
              " > 4 — boring bar deflection may degrade tolerance");
    }

    if (!in.tolerance_class.empty() &&
        in.tolerance_class != "H7" &&
        in.tolerance_class != "H8" &&
        in.tolerance_class != "H9") {
        r.add("DFM-INPUT", "warning",
              "bore_and_finish: unknown tolerance_class '" +
              in.tolerance_class + "' — expected H7/H8/H9");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Sequence: drill → semi-finish bore → finish bore.  Each step builds on
// the previous step's Workpiece.  We use Skill outputs as our chain so
// inner DFM checks fire correctly (we ignore inner warnings; only errors
// throw via the inner apply()).

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "bore_and_finish DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double drillDia    = in.final_diameter_mm - kRoughUndersize_mm;
    const double semiFinDia  = in.final_diameter_mm - kSemifinishUndersize_mm;
    const double finalDia    = in.final_diameter_mm;

    // ── Step 1: rough drill ─────────────────────────────────────────────
    drill_hole::Input dh;
    dh.entry_face    = in.entry_face;
    dh.position_x_mm = in.position_x_mm;
    dh.position_y_mm = in.position_y_mm;
    dh.axis_dir      = in.axis_dir;
    dh.diameter_mm   = drillDia;
    dh.depth_mm      = in.depth_mm;
    dh.through_hole  = false;
    SkillOutput step1 = drill_hole::apply(wp, dh);

    // ── Step 2: semi-finish bore ────────────────────────────────────────
    //
    // bore_cylindrical's own DFM warns when dia < 6 mm (consider drill+ream
    // path).  For this macro we accept that warning silently — semi-finish
    // bores on small holes are precisely the case where drill+ream is the
    // textbook answer, but slice-1 doesn't have a ream skill yet.
    bore_cylindrical::Input bc;
    bc.entry_face      = in.entry_face;
    bc.position_x_mm   = in.position_x_mm;
    bc.position_y_mm   = in.position_y_mm;
    bc.axis_dir        = in.axis_dir;
    bc.diameter_mm     = semiFinDia;
    bc.depth_mm        = in.depth_mm;
    bc.tolerance_class = in.tolerance_class;
    SkillOutput step2 = bore_cylindrical::apply(*step1.workpiece, bc);

    // ── Step 3: finish ─────────────────────────────────────────────────
    //
    // FALLBACK: ream::apply is not committed yet.  We model the finishing
    // pass as a tiny final bore_cylindrical at the exact final_diameter.
    // When the ream skill becomes available, replace this block with a
    // ream::Input + ream::apply call (signature stamping below stays the
    // same).
    bore_cylindrical::Input finalPass = bc;
    finalPass.diameter_mm = finalDia;
    SkillOutput step3 = bore_cylindrical::apply(*step2.workpiece, finalPass);

    // ── Re-stamp the signature for the macro ───────────────────────────
    FeatureSignature sig = step3.signature;
    sig.skill_id = kSkillId;

    sig.params = {
        { "position_x_mm",    in.position_x_mm },
        { "position_y_mm",    in.position_y_mm },
        { "axis_dir",         { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "final_diameter_mm", finalDia },
        { "depth_mm",         in.depth_mm },
        { "tolerance_class",  in.tolerance_class },
    };
    sig.pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     1 },           // visible after all passes
        { "circular_edge_count",        2 },
        { "bottom_planar_face_present", true },
        { "final_diameter_mm",          finalDia },
        { "depth_mm",                   in.depth_mm },
        { "tolerance_class",            in.tolerance_class },
        { "axis_dir",                   { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
    };

    // Tooling sequence: drill + boring bar + (placeholder) reamer.
    ToolingMeta tooling = sig.tooling;
    tooling.tool_type = "drill;boring_bar;reamer";
    tooling.tool_dia_mm = finalDia;
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },
              { "tool_dia_mm", drillDia },
              { "depth_mm",    in.depth_mm } },
            { { "tool_type", "boring_bar" },
              { "tool_dia_mm", semiFinDia },
              { "depth_mm",    in.depth_mm },
              { "tolerance_class", in.tolerance_class } },
            { { "tool_type", "reamer" },
              { "tool_dia_mm", finalDia },
              { "depth_mm",    in.depth_mm },
              { "tolerance_class", in.tolerance_class },
              { "note", "ream skill not yet available — modelled as final bore pass" } },
        } },
    };
    sig.tooling = tooling;

    auto wpNew = step3.workpiece;
    wpNew->addFeature(sig);

    spdlog::debug("skill::bore_and_finish applied: drill={} semi={} final={} depth={}",
                  drillDia, semiFinDia, finalDia, in.depth_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// We can't distinguish "drilled-then-bored-then-reamed" from a plain
// drill_hole on geometry alone — both leave the same cylindrical pattern.
// We return a "bore_and_finish" candidate when:
//   - the candidate diameter ≥ 1.5 mm
//   - depth/dia ≤ 4 (in-range for boring-bar finishing)
//   - confidence ≈ 0.7 (deliberately low, overlapping with bore_cylindrical
//     and drill_hole).

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    auto drillCands = drill_hole::recognize(wp);
    for (const auto& c : drillCands) {
        const auto itDia = c.recovered_params.find("diameter_mm");
        if (itDia == c.recovered_params.end()) continue;
        const double dia = itDia->get<double>();
        if (dia < 1.5) continue;

        const auto itDepth = c.recovered_params.find("depth_mm");
        const double depth =
            (itDepth != c.recovered_params.end()) ? itDepth->get<double>() : 0.0;

        const auto itThrough = c.recovered_params.find("through_hole");
        if (itThrough != c.recovered_params.end() && itThrough->get<bool>())
            continue;  // through holes are unlikely precision bores in slice-1

        const double ratio = (dia > 0.0) ? (depth / dia) : 0.0;
        if (ratio > 4.0) continue;
        if (depth < 1e-3)  continue;

        json rp = {
            { "position_x_mm",    c.recovered_params["position_x_mm"] },
            { "position_y_mm",    c.recovered_params["position_y_mm"] },
            { "axis_dir",         c.recovered_params["axis_dir"] },
            { "final_diameter_mm", dia },
            { "depth_mm",         depth },
            { "tolerance_class",  "H7" },
        };
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = rp;
        r.confidence       = 0.7;
        r.matched_geometry = c.matched_geometry;
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::bore_and_finish
