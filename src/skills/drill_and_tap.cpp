// @lat: [[engine/skills#drill_and_tap]]

#include "drill_and_tap.hpp"

#include "Workpiece.hpp"
#include "_iso_thread_table.hpp"
#include "drill_hole.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <array>
#include <cmath>

namespace koocadcam::skill::drill_and_tap {

namespace tt = koocadcam::skill::thread_table;
using nlohmann::json;

// ── Thread-size lookup ───────────────────────────────────────────────────
//
// Central thread_table covers M3..M30.  Small consumer sizes M1.6/M2/M2.5
// are not in the central table (it starts at M3), so we keep ONLY those
// three non-overlapping rows here.

namespace {

struct SmallPilot {
    const char* size;
    double      pilot_dia_mm;
};

constexpr std::array<SmallPilot, 3> kSmallSizes {{
    { "M1.6", 1.25 },
    { "M2",   1.60 },
    { "M2.5", 2.05 },
}};

}  // namespace

double pilotDiameterFor(const std::string& thread_size)
{
    for (const auto& e : kSmallSizes) {
        if (thread_size == e.size) return e.pilot_dia_mm;
    }
    if (const auto* s = tt::findMetric(thread_size)) return s->tap_pilot_dia_mm;
    return 0.0;
}

std::string threadSizeFor(double pilot_dia_mm, double tol_mm)
{
    const char* bestSize = nullptr;
    double bestErr = tol_mm;
    for (const auto& e : kSmallSizes) {
        const double err = std::abs(e.pilot_dia_mm - pilot_dia_mm);
        if (err < bestErr) { bestErr = err; bestSize = e.size; }
    }
    // drill_and_tap only supports up to M6 in its public contract.
    for (const auto& e : tt::kMetricThreads) {
        const std::string k = e.size_key;
        if (k != "M3" && k != "M4" && k != "M5" && k != "M6") continue;
        const double err = std::abs(e.tap_pilot_dia_mm - pilot_dia_mm);
        if (err < bestErr) { bestErr = err; bestSize = e.size_key; }
    }
    return bestSize ? std::string(bestSize) : std::string();
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.thread_size.empty()) {
        r.add("DFM-TAP-SIZE", "error", "drill_and_tap: thread_size is empty");
        return r;
    }
    const double pilotDia = pilotDiameterFor(in.thread_size);
    if (pilotDia <= 0.0) {
        r.add("DFM-TAP-SIZE", "error",
              "drill_and_tap: unknown thread_size '" + in.thread_size +
              "' (supported: M1.6/M2/M2.5/M3/M4/M5/M6)");
        return r;
    }

    if (in.tap_depth_mm <= 0.0) {
        r.add("DFM-TAP-DEPTH", "error",
              "drill_and_tap: tap_depth_mm must be > 0");
    }
    if (in.pilot_extra_depth_mm < 0.0) {
        r.add("DFM-INPUT", "error",
              "drill_and_tap: pilot_extra_depth_mm must be >= 0");
    }

    // Compose the inner drill check (covers DFM-002 + peck-warning).
    drill_hole::Input dh;
    dh.entry_face    = in.entry_face;
    dh.position_x_mm = in.position_x_mm;
    dh.position_y_mm = in.position_y_mm;
    dh.axis_dir      = in.axis_dir;
    dh.diameter_mm   = pilotDia;
    dh.depth_mm      = in.tap_depth_mm + in.pilot_extra_depth_mm;
    dh.through_hole  = in.through_hole;
    DFMReport inner = drill_hole::validate(wp, dh);
    for (const auto& f : inner.findings) r.add(f.code, f.severity, f.message);

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "drill_and_tap DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double pilotDia    = pilotDiameterFor(in.thread_size);
    const double pilotDepth  = in.tap_depth_mm + in.pilot_extra_depth_mm;

    // Compose: call drill_hole with the resolved pilot diameter & depth.
    drill_hole::Input dh;
    dh.entry_face    = in.entry_face;
    dh.position_x_mm = in.position_x_mm;
    dh.position_y_mm = in.position_y_mm;
    dh.axis_dir      = in.axis_dir;
    dh.diameter_mm   = pilotDia;
    dh.depth_mm      = pilotDepth;
    dh.through_hole  = in.through_hole;

    SkillOutput drillOut = drill_hole::apply(wp, dh);

    // Re-stamp the signature for the outer skill.
    FeatureSignature sig = drillOut.signature;
    sig.skill_id = kSkillId;

    sig.params = {
        { "position_x_mm",        in.position_x_mm },
        { "position_y_mm",        in.position_y_mm },
        { "axis_dir",             { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "thread_size",          in.thread_size },
        { "tap_depth_mm",         in.tap_depth_mm },
        { "pilot_extra_depth_mm", in.pilot_extra_depth_mm },
        { "pilot_dia_mm",         pilotDia },
        { "through_hole",         in.through_hole },
    };
    sig.pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     1 },
        { "circular_edge_count",        2 },
        { "bottom_planar_face_present", !in.through_hole },
        { "thread_size",                in.thread_size },
        { "tap_depth_mm",               in.tap_depth_mm },
        { "pilot_dia_mm",               pilotDia },
        { "axis_dir",                   { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
    };

    // Tooling: this is a two-tool sequence.  Keep the drill tooling as the
    // primary (matches the underlying geometry), but record the full
    // sequence in `extra` so CAPP can schedule the tap pass.
    ToolingMeta tooling = sig.tooling;
    tooling.tool_type = "drill;tap";
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },
              { "tool_dia_mm", pilotDia },
              { "depth_mm",    pilotDepth } },
            { { "tool_type", "tap" },
              { "thread_size", in.thread_size },
              { "tap_depth_mm", in.tap_depth_mm } },
        } },
        { "note",
          "tap thread geometry not modelled in slice-1; tap operation"
          " carried as metadata for CAPP" }
    };
    sig.tooling = tooling;

    auto wpNew = drillOut.workpiece;
    wpNew->addFeature(sig);

    spdlog::debug("skill::drill_and_tap applied: {} (pilot {}×{})",
                  in.thread_size, pilotDia, pilotDepth);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// We cannot recover the tap_depth from B-rep alone (the thread is not in
// geometry).  We can however nominate a drill_hole candidate as a possible
// "drill_and_tap" if its recovered diameter matches one of the standard
// pilots — recognize() returns these as alternative candidates with a
// lower confidence so CAPP can choose.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    auto drillCands = drill_hole::recognize(wp);
    for (const auto& c : drillCands) {
        const auto itDia = c.recovered_params.find("diameter_mm");
        if (itDia == c.recovered_params.end()) continue;
        const double dia = itDia->get<double>();

        const std::string size = threadSizeFor(dia, 0.05);
        if (size.empty()) continue;

        const auto itDepth = c.recovered_params.find("depth_mm");
        const double recoveredDepth =
            (itDepth != c.recovered_params.end()) ? itDepth->get<double>() : 0.0;

        const auto itThrough = c.recovered_params.find("through_hole");
        const bool through =
            (itThrough != c.recovered_params.end()) && itThrough->get<bool>();

        // We can't separate tap_depth from pilot_extra_depth from a STEP
        // round-trip — choose a conservative default: tap_depth_mm ≈
        // recovered_depth - 1mm (the standard pilot_extra_depth default).
        const double tapDepthGuess =
            through ? 0.0 : std::max(0.0, recoveredDepth - 1.0);

        json rp = {
            { "position_x_mm", c.recovered_params["position_x_mm"] },
            { "position_y_mm", c.recovered_params["position_y_mm"] },
            { "axis_dir",      c.recovered_params["axis_dir"] },
            { "thread_size",   size },
            { "tap_depth_mm",  tapDepthGuess },
            { "pilot_extra_depth_mm", through ? 0.0 : 1.0 },
            { "pilot_dia_mm",  dia },
            { "through_hole",  through },
        };
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = rp;
        // Overlaps with plain drill_hole, so confidence is modest.
        r.confidence       = 0.6;
        r.matched_geometry = c.matched_geometry;
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::drill_and_tap
