// @lat: [[engine/skills#regrind_polymer]]

#include "regrind_polymer.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::regrind_polymer {

using nlohmann::json;

namespace {

bool isKnownPolymer(const std::string& p)
{
    return p == "PET" || p == "PP"  || p == "HDPE" || p == "LDPE" ||
           p == "PS"  || p == "ABS" || p == "PVC"  || p == "PC"   ||
           p == "PMMA";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.polymer_type.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": polymer_type must be non-empty");
    }
    if (in.grind_size_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": grind_size_mm must be > 0 (got " +
              std::to_string(in.grind_size_mm) + ")");
    }
    if (in.contamination_pct < 0.0 || in.contamination_pct >= 100.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": contamination_pct must be in [0, 100) (got " +
              std::to_string(in.contamination_pct) + ")");
    }

    if (!in.polymer_type.empty() && !isKnownPolymer(in.polymer_type)) {
        r.add("DFM-REGRIND-TYPE", "info",
              std::string(kSkillId) + ": polymer_type '" + in.polymer_type +
              "' not in standard list (PET | PP | HDPE | LDPE | PS | ABS | "
              "PVC | PC | PMMA) — will be treated as custom resin");
    }

    if (in.grind_size_mm > 0.0 && in.grind_size_mm < 1.0) {
        r.add("DFM-REGRIND-FINE", "info",
              std::string(kSkillId) + ": grind_size_mm " +
              std::to_string(in.grind_size_mm) +
              " < 1 mm — very fine powder; cryogenic grinding may be required");
    }
    if (in.grind_size_mm > 25.0) {
        r.add("DFM-REGRIND-COARSE", "info",
              std::string(kSkillId) + ": grind_size_mm " +
              std::to_string(in.grind_size_mm) +
              " > 25 mm — exceeds feed-throat of typical extruder, "
              "consider secondary grind before re-melt");
    }
    if (in.contamination_pct > 5.0 && in.contamination_pct < 100.0) {
        r.add("DFM-REGRIND-CONT", "warning",
              std::string(kSkillId) + ": contamination_pct " +
              std::to_string(in.contamination_pct) +
              " > 5% — high foreign-material fraction; downstream re-melt "
              "quality (clarity, strength) will be degraded");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "regrind_polymer DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "polymer_type",      in.polymer_type },
        { "grind_size_mm",     in.grind_size_mm },
        { "contamination_pct", in.contamination_pct },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "polymer_type",      in.polymer_type },
        { "grind_size_mm",     in.grind_size_mm },
        { "contamination_pct", in.contamination_pct },
        { "process_category",  "eol_polymer_regrind" },
        { "geometry_changed",  false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "granulator";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "hardened_steel_blades";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 60.0;   // per-kg approximation
    tooling.extra = {
        { "process",           "regrind_polymer" },
        { "polymer_type",      in.polymer_type },
        { "grind_size_mm",     in.grind_size_mm },
        { "contamination_pct", in.contamination_pct },
        { "process_category",  "eol_polymer_regrind" },
        { "geometric_no_op",   true },
        { "dfm_findings",      findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::regrind_polymer applied: type={} size={}mm contamination={}%",
                  in.polymer_type, in.grind_size_mm, in.contamination_pct);

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
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::regrind_polymer
