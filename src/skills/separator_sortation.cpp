// @lat: [[engine/skills#separator_sortation]]

#include "separator_sortation.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::separator_sortation {

using nlohmann::json;

namespace {

bool isKnownMethod(const std::string& m)
{
    return m == "magnetic" || m == "density" || m == "eddy_current";
}

bool isKnownFraction(const std::string& f)
{
    return f == "ferrous" || f == "non_ferrous" || f == "polymer" ||
           f == "glass"   || f == "aluminum"    || f == "copper";
}

// Returns true when the method/target pairing is the textbook-recommended one.
bool isCompatiblePair(const std::string& method, const std::string& fraction)
{
    if (method == "magnetic"     && fraction == "ferrous")     return true;
    if (method == "eddy_current" && fraction == "non_ferrous") return true;
    if (method == "eddy_current" && fraction == "aluminum")    return true;
    if (method == "eddy_current" && fraction == "copper")      return true;
    if (method == "density")                                   return true;
    return false;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.target_fraction.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": target_fraction must be non-empty");
    }
    if (in.purity_pct <= 0.0 || in.purity_pct > 100.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": purity_pct must be in (0, 100] (got " +
              std::to_string(in.purity_pct) + ")");
    }

    if (!isKnownMethod(in.method)) {
        r.add("DFM-SORT-METHOD", "info",
              std::string(kSkillId) + ": method '" + in.method +
              "' unrecognised — expected magnetic | density | eddy_current");
    }
    if (!in.target_fraction.empty() && !isKnownFraction(in.target_fraction)) {
        r.add("DFM-SORT-FRACTION", "info",
              std::string(kSkillId) + ": target_fraction '" + in.target_fraction +
              "' not in standard list (ferrous | non_ferrous | polymer | glass | "
              "aluminum | copper) — will be treated as custom stream");
    }

    if (in.purity_pct > 0.0 && in.purity_pct < 90.0) {
        r.add("DFM-SORT-PURITY", "warning",
              std::string(kSkillId) + ": purity_pct " +
              std::to_string(in.purity_pct) +
              " < 90% — downstream re-melt / re-grind may require a "
              "second pass to meet spec");
    }

    if (isKnownMethod(in.method) && !in.target_fraction.empty() &&
        !isCompatiblePair(in.method, in.target_fraction)) {
        r.add("DFM-SORT-COMPAT", "info",
              std::string(kSkillId) + ": method '" + in.method +
              "' is suboptimal for target_fraction '" + in.target_fraction +
              "' — consider a more standard pairing");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "separator_sortation DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "method",          in.method },
        { "target_fraction", in.target_fraction },
        { "purity_pct",      in.purity_pct },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "method",           in.method },
        { "target_fraction",  in.target_fraction },
        { "purity_pct",       in.purity_pct },
        { "process_category", "eol_sortation" },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "separator";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = in.method == "magnetic"     ? "permanent_magnet"
                              : in.method == "eddy_current" ? "rotating_pole_drum"
                              : in.method == "density"      ? "fluidized_bed"
                              :                               "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 60.0;   // per-kg approximation
    tooling.extra = {
        { "process",          "sortation" },
        { "method",           in.method },
        { "target_fraction",  in.target_fraction },
        { "purity_pct",       in.purity_pct },
        { "process_category", "eol_sortation" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::separator_sortation applied: method={} frac={} purity={}%",
                  in.method, in.target_fraction, in.purity_pct);

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

}  // namespace koocadcam::skill::separator_sortation
