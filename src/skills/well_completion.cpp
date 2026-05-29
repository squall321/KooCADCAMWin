// @lat: [[engine/skills#well_completion]]

#include "well_completion.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::well_completion {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.well_id.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": well_id must be non-empty");
    }

    if (in.depth_m <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": depth_m must be > 0 (got " +
              std::to_string(in.depth_m) + ")");
    }

    if (in.casing_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": casing_dia_mm must be > 0 (got " +
              std::to_string(in.casing_dia_mm) + ")");
    } else if (in.casing_dia_mm < 114.3 || in.casing_dia_mm > 339.7) {
        r.add("DFM-WC-CASING", "info",
              std::string(kSkillId) + ": casing_dia_mm " +
              std::to_string(in.casing_dia_mm) +
              " outside typical 4.5\"–13.375\" production casing range");
    }

    if (in.perforation_count <= 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": perforation_count must be > 0 (got " +
              std::to_string(in.perforation_count) + ")");
    } else if (in.perforation_count > 200) {
        r.add("DFM-WC-PERF", "info",
              std::string(kSkillId) + ": perforation_count " +
              std::to_string(in.perforation_count) +
              " is unusually heavy shot density — verify gun spacing");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "well_completion DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "well_id",           in.well_id },
        { "depth_m",           in.depth_m },
        { "casing_dia_mm",     in.casing_dia_mm },
        { "perforation_count", in.perforation_count },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "well_id",           in.well_id },
        { "depth_m",           in.depth_m },
        { "casing_dia_mm",     in.casing_dia_mm },
        { "perforation_count", in.perforation_count },
        { "geometry_changed",  false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "perforating_gun";
    tooling.tool_dia_mm       = in.casing_dia_mm;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Rig-up + run-in + perforation + run-out, ~0.5 s per shot + 600 s baseline.
    tooling.est_cycle_time_s  = 600.0 + 0.5 * static_cast<double>(in.perforation_count);
    tooling.extra = {
        { "process",           "oilgas_well_completion" },
        { "well_id",           in.well_id },
        { "depth_m",           in.depth_m },
        { "casing_dia_mm",     in.casing_dia_mm },
        { "perforation_count", in.perforation_count },
        { "process_category",  "downhole_installation" },
        { "geometric_no_op",   true },
        { "dfm_findings",      findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::well_completion applied: well={} depth={}m casing={}mm perfs={}",
        in.well_id, in.depth_m, in.casing_dia_mm, in.perforation_count);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Metadata-only: downhole completion has no surface-model fingerprint.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != std::string(kSkillId)) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::well_completion
