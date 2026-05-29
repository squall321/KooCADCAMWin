// @lat: [[engine/skills#club_balance]]

#include "club_balance.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::club_balance {

using nlohmann::json;

namespace {

// Returns true if s matches /^[A-F][0-9]$/ (golf swing-weight code).
bool isValidSwingWeight(const std::string& s)
{
    if (s.size() != 2) return false;
    const char a = s[0];
    const char b = s[1];
    return a >= 'A' && a <= 'F' && b >= '0' && b <= '9';
}

// Map "D2" → numeric scale: A=0..F=5, digit 0..9 → 6*A + digit (lexical order).
int swingWeightToScale(const std::string& s)
{
    if (!isValidSwingWeight(s)) return -1;
    return static_cast<int>(s[0] - 'A') * 10 + static_cast<int>(s[1] - '0');
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!isValidSwingWeight(in.swing_weight)) {
        r.add("DFM-CLUB-SW", "error",
              "club_balance swing_weight '" + in.swing_weight +
              "' malformed — expected <A-F><0-9> (e.g. 'D2')");
    }

    if (in.head_mass_g < 150.0 || in.head_mass_g > 350.0) {
        r.add("DFM-CLUB-MASS", "error",
              "club_balance head_mass_g " + std::to_string(in.head_mass_g) +
              " outside physical range [150, 350] g");
    } else if (in.head_mass_g < 180.0 || in.head_mass_g > 230.0) {
        r.add("DFM-CLUB-MASS", "info",
              "club_balance head_mass_g " + std::to_string(in.head_mass_g) +
              " outside common driver range [180, 230] g");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "club_balance DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const int swScale = swingWeightToScale(in.swing_weight);

    json params = {
        { "swing_weight", in.swing_weight },
        { "head_mass_g",  in.head_mass_g },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "swing_weight",        in.swing_weight },
        { "swing_weight_scale",  swScale },
        { "head_mass_g",         in.head_mass_g },
        { "geometry_changed",    false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "swing_weight_scale";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "none";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ~45 s for measure + tip-weight adjust + remeasure.
    tooling.est_cycle_time_s  = 45.0;
    tooling.extra = {
        { "process",      "club_swing_balance" },
        { "swing_weight", in.swing_weight },
        { "head_mass_g",  in.head_mass_g },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::club_balance applied: sw={} head={}g",
                  in.swing_weight, in.head_mass_g);

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

}  // namespace koocadcam::skill::club_balance
