// @lat: [[engine/skills#racket_string]]

#include "racket_string.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cstdlib>
#include <string>

namespace koocadcam::skill::racket_string {

using nlohmann::json;

namespace {

// Parse "16x19" → (16, 19).  Returns true on success.
bool parsePattern(const std::string& pat, int& mainCount, int& crossCount)
{
    auto x = pat.find('x');
    if (x == std::string::npos || x == 0 || x + 1 >= pat.size()) {
        // Also accept upper-case 'X'.
        x = pat.find('X');
        if (x == std::string::npos || x == 0 || x + 1 >= pat.size())
            return false;
    }
    try {
        size_t p1 = 0, p2 = 0;
        const std::string a = pat.substr(0, x);
        const std::string b = pat.substr(x + 1);
        mainCount  = std::stoi(a, &p1);
        crossCount = std::stoi(b, &p2);
        if (p1 != a.size() || p2 != b.size()) return false;
        return mainCount > 0 && crossCount > 0;
    } catch (const std::exception&) {
        return false;
    }
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.tension_lbf < 30.0 || in.tension_lbf > 80.0) {
        r.add("DFM-RACKET-TENSION", "error",
              "racket_string tension_lbf " + std::to_string(in.tension_lbf) +
              " outside playable range [30, 80] lbf");
    }

    int mainC = 0, crossC = 0;
    if (!parsePattern(in.string_pattern, mainC, crossC)) {
        r.add("DFM-RACKET-PATTERN", "error",
              "racket_string string_pattern '" + in.string_pattern +
              "' malformed — expected '<mains>x<crosses>' (e.g. '16x19')");
    } else if (mainC < 12 || mainC > 22) {
        r.add("DFM-RACKET-PATTERN", "info",
              "racket_string main_count " + std::to_string(mainC) +
              " outside common range [12, 22]");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "racket_string DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    int mainC = 0, crossC = 0;
    parsePattern(in.string_pattern, mainC, crossC);

    json params = {
        { "tension_lbf",   in.tension_lbf },
        { "string_pattern", in.string_pattern },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "tension_lbf",      in.tension_lbf },
        { "string_pattern",   in.string_pattern },
        { "main_count",       mainC },
        { "cross_count",      crossC },
        { "total_strings",    mainC + crossC },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "stringing_machine";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "none";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ~25 s per string (mains + crosses) on a constant-pull machine.
    tooling.est_cycle_time_s  = 25.0 * static_cast<double>(mainC + crossC);
    tooling.extra = {
        { "process",        "racket_stringing" },
        { "tension_lbf",    in.tension_lbf },
        { "string_pattern", in.string_pattern },
        { "main_count",     mainC },
        { "cross_count",    crossC },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::racket_string applied: tension={} lbf pattern={}",
                  in.tension_lbf, in.string_pattern);

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

}  // namespace koocadcam::skill::racket_string
