// @lat: [[engine/skills#flexo_print]]

#include "flexo_print.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>
#include <unordered_set>

namespace koocadcam::skill::flexo_print {

using nlohmann::json;

namespace {

const std::unordered_set<std::string>& knownInkTypes()
{
    static const std::unordered_set<std::string> s = {
        "water", "solvent", "uv",
    };
    return s;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.anilox_lpi <= 0) {
        r.add("DFM-INPUT", "error",
              "flexo_print anilox_lpi must be > 0 (got " +
              std::to_string(in.anilox_lpi) + ")");
    }
    if (in.dot_size_um <= 0.0) {
        r.add("DFM-INPUT", "error",
              "flexo_print dot_size_um must be > 0 (got " +
              std::to_string(in.dot_size_um) + ")");
    }
    if (in.ink_type.empty() || knownInkTypes().count(in.ink_type) == 0) {
        r.add("DFM-INPUT", "error",
              "flexo_print unknown ink_type '" + in.ink_type +
              "' (expected water | solvent | uv)");
    }

    if (in.anilox_lpi > 0 && in.anilox_lpi < 200) {
        r.add("DFM-ANILOX-COARSE", "info",
              "flexo_print anilox_lpi " + std::to_string(in.anilox_lpi) +
              " < 200 — coarse anilox; heavy ink lay-down");
    }
    if (in.anilox_lpi > 1400) {
        r.add("DFM-ANILOX-FINE", "info",
              "flexo_print anilox_lpi " + std::to_string(in.anilox_lpi) +
              " > 1400 — fine anilox; verify dot gain compensation");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "flexo_print DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "anilox_lpi",  in.anilox_lpi },
        { "dot_size_um", in.dot_size_um },
        { "ink_type",    in.ink_type },
    };

    json pattern = {
        { "kind",             kSkillId },
        { "anilox_lpi",       in.anilox_lpi },
        { "dot_size_um",      in.dot_size_um },
        { "ink_type",         in.ink_type },
        { "process_family",   "flexography" },
        { "geometry_changed", false },
        { "is_process_only",  true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = (in.ink_type == "uv") ? "flexo_uv_press"
                                                       : "flexo_press";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "photopolymer_plate";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 60.0;
    tooling.extra = {
        { "process",          "flexographic_printing" },
        { "anilox_lpi",       in.anilox_lpi },
        { "dot_size_um",      in.dot_size_um },
        { "ink_type",         in.ink_type },
        { "process_category", "printing" },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::flexo_print applied: lpi={} dot={}um ink={}",
                  in.anilox_lpi, in.dot_size_um, in.ink_type);

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
        rf.matched_geometry = { { "source", "feature_history" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::flexo_print
