// @lat: [[engine/skills#abrasive_flow]]

#include "abrasive_flow.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::abrasive_flow {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.cycles <= 0) {
        r.add("DFM-INPUT", "error",
              "abrasive_flow cycles must be > 0 (got " +
              std::to_string(in.cycles) + ")");
    }
    if (in.ra_before_um <= 0.0) {
        r.add("DFM-INPUT", "error",
              "abrasive_flow ra_before_um must be > 0 (got " +
              std::to_string(in.ra_before_um) + ")");
    }
    if (in.ra_after_um <= 0.0) {
        r.add("DFM-INPUT", "error",
              "abrasive_flow ra_after_um must be > 0");
    }
    if (in.ra_after_um >= in.ra_before_um) {
        r.add("DFM-AFM-RA", "error",
              "abrasive_flow ra_after_um " + std::to_string(in.ra_after_um) +
              " >= ra_before_um " + std::to_string(in.ra_before_um) +
              " — AFM must reduce surface roughness");
    }
    if (in.ra_after_um < 0.05) {
        r.add("DFM-AFM-RA-FLOOR", "warning",
              "abrasive_flow ra_after_um " + std::to_string(in.ra_after_um) +
              " < 0.05 μm — below AFM practical floor; consider lapping");
    }
    if (in.cycles > 200) {
        r.add("DFM-AFM-CYCLES", "info",
              "abrasive_flow cycles " + std::to_string(in.cycles) +
              " > 200 — diminishing returns; consider increasing pressure");
    }
    if (in.pressure_mpa <= 0.0) {
        r.add("DFM-INPUT", "error",
              "abrasive_flow pressure_mpa must be > 0");
    }
    (void)wp;
    return r;
}

// ── Synthesis (metadata-only — no Boolean op) ────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "abrasive_flow DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double raReduction = in.ra_before_um - in.ra_after_um;
    const double raReductionPct = (in.ra_before_um > 0.0)
        ? (raReduction / in.ra_before_um * 100.0) : 0.0;

    json params = {
        { "channel_label",   in.channel_label },
        { "ra_before_um",    in.ra_before_um },
        { "ra_after_um",     in.ra_after_um },
        { "cycles",          in.cycles },
        { "abrasive_media",  in.abrasive_media },
        { "pressure_mpa",    in.pressure_mpa },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "channel_label",       in.channel_label },
        { "ra_before_um",        in.ra_before_um },
        { "ra_after_um",         in.ra_after_um },
        { "ra_reduction_um",     raReduction },
        { "ra_reduction_pct",    raReductionPct },
        { "cycles",              in.cycles },
        { "geometry_changed",    false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "afm_putty_extruder";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = in.abrasive_media;
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;       // microns × surface area — negligible
    // Cycle time ≈ 30 s per stroke pair.
    tooling.est_cycle_time_s  = std::max(60.0, in.cycles * 30.0);
    tooling.extra["dfm_findings"]        = findings;
    tooling.extra["ra_reduction_um"]     = raReduction;
    tooling.extra["ra_reduction_pct"]    = raReductionPct;
    tooling.extra["pressure_mpa"]        = in.pressure_mpa;
    tooling.extra["geometric_no_op"]     = true;
    tooling.extra["machining_constraint"] =
        "Abrasive flow machining — internal channel polish; viscoelastic "
        "abrasive putty extruded back-and-forth; geometry unchanged within "
        "measurement tolerance (micron-scale removal)";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    // Metadata-only: clone the workpiece shape + material, copy prior
    // features, and append this signature.  Same pattern as anneal /
    // heat_treatment_common::cloneWithSignature.
    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::abrasive_flow applied: Ra {:.3f}→{:.3f} μm "
                  "({} cycles, {} MPa)",
                  in.ra_before_um, in.ra_after_um,
                  in.cycles, in.pressure_mpa);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// AFM leaves no geometric trace: a polished channel is the same B-rep as
// an unpolished channel.  Recognize() returns ONLY metadata-replay
// candidates from wp.features().  Pattern follows src/skills/anneal.cpp.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::abrasive_flow
