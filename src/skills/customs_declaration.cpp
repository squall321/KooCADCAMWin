// @lat: [[engine/skills#customs_declaration]]

#include "customs_declaration.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cctype>

namespace koocadcam::skill::customs_declaration {

using nlohmann::json;

namespace {

bool isAllDigits(const std::string& s)
{
    if (s.empty()) return false;
    for (char c : s) {
        if (!std::isdigit(static_cast<unsigned char>(c))) return false;
    }
    return true;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.hs_code.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": hs_code must not be empty");
    } else {
        const std::size_t n = in.hs_code.size();
        if (n < 6 || n > 10 || !isAllDigits(in.hs_code)) {
            r.add("DFM-CUS-HSCODE", "error",
                  std::string(kSkillId) + ": hs_code '" + in.hs_code +
                  "' invalid — must be 6 to 10 digits");
        }
    }

    if (in.country_origin.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": country_origin must not be empty");
    } else if (in.country_origin.size() != 2 && in.country_origin.size() != 3) {
        r.add("DFM-CUS-ORIGIN", "error",
              std::string(kSkillId) + ": country_origin '" + in.country_origin +
              "' invalid — expected ISO 3166 alpha-2 or alpha-3");
    }

    if (in.value_usd <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": value_usd must be > 0 (got " +
              std::to_string(in.value_usd) + ")");
    } else {
        if (in.value_usd > 1000000.0) {
            r.add("DFM-CUS-VALUE-HIGH", "info",
                  std::string(kSkillId) + ": value_usd " +
                  std::to_string(in.value_usd) +
                  " > 1,000,000 — formal entry required");
        }
        if (in.value_usd < 800.0) {
            r.add("DFM-CUS-VALUE-LOW", "info",
                  std::string(kSkillId) + ": value_usd " +
                  std::to_string(in.value_usd) +
                  " < 800 — within de minimis threshold");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "customs_declaration DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    std::string origin_uc = in.country_origin;
    std::transform(origin_uc.begin(), origin_uc.end(), origin_uc.begin(),
                   [](unsigned char c) { return std::toupper(c); });

    json params = {
        { "hs_code",        in.hs_code },
        { "country_origin", in.country_origin },
        { "value_usd",      in.value_usd },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_inspection_only",   true },
        { "geometric_change",     false },
        { "hs_code",              in.hs_code },
        { "country_origin",       origin_uc },
        { "value_usd",            in.value_usd },
        { "hs_code_length",       static_cast<int>(in.hs_code.size()) },
        { "entry_type",           in.value_usd > 1000000.0 ? "formal"
                                  : in.value_usd < 800.0   ? "de_minimis"
                                                           : "informal" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "customs_record";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Customs filing — broker submission ~ 15 min nominal.
    tooling.est_cycle_time_s  = 900.0;
    tooling.extra["inspection_type"] = "customs_declaration";
    tooling.extra["hs_code"]         = in.hs_code;
    tooling.extra["country_origin"]  = origin_uc;
    tooling.extra["value_usd"]       = in.value_usd;
    tooling.extra["dfm_findings"]    = findings;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::customs_declaration applied: hs={} origin={} value=${}",
        in.hs_code, origin_uc, in.value_usd);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

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

}  // namespace koocadcam::skill::customs_declaration
