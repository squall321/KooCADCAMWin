// @lat: [[engine/skills#fixture_setup]]

#include "fixture_setup.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <regex>

namespace koocadcam::skill::fixture_setup {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.fixture_type.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": fixture_type must be non-empty");
    }
    if (in.clamp_locations.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": clamp_locations must have ≥ 1 entry");
    }
    if (in.work_offsets.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": work_offsets must have ≥ 1 entry");
    }
    static const std::regex offsetRe("G5[4-9]");
    for (const auto& off : in.work_offsets) {
        if (!std::regex_match(off, offsetRe)) {
            r.add("FX-OFFSET-FORM", "warning",
                  std::string(kSkillId) + ": work_offset '" + off +
                  "' is non-standard (expected G54..G59)");
        }
    }
    (void)wp;
    return r;
}

// ── Synthesis (metadata-only, geometric no-op) ───────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = std::string(kSkillId) + " DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json clamps = json::array();
    for (const auto& xyz : in.clamp_locations) {
        clamps.push_back({ xyz[0], xyz[1], xyz[2] });
    }
    json offsets = json::array();
    for (const auto& o : in.work_offsets) offsets.push_back(o);

    json params = {
        { "fixture_type",     in.fixture_type },
        { "clamp_locations",  clamps },
        { "work_offsets",     offsets },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "fixture_type",      in.fixture_type },
        { "clamp_locations",   clamps },
        { "work_offsets",      offsets },
        { "geometry_changed",  false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "fixture";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Manual fixture-setup cycle time: ~ 60 s per clamp + 30 s base.
    tooling.est_cycle_time_s  = 30.0 + 60.0 * static_cast<double>(in.clamp_locations.size());
    tooling.extra["fixture_type"]     = in.fixture_type;
    tooling.extra["clamp_count"]      = in.clamp_locations.size();
    tooling.extra["work_offsets"]     = offsets;
    tooling.extra["geometric_no_op"]  = true;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::fixture_setup applied: type={} clamps={} offsets={}",
                  in.fixture_type, in.clamp_locations.size(), in.work_offsets.size());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition (metadata replay) ────────────────────────────────────────

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

}  // namespace koocadcam::skill::fixture_setup
