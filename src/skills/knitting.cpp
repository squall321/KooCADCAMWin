// @lat: [[engine/skills#knitting]]

#include "knitting.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::knitting {

using nlohmann::json;

namespace {

bool isKnownYarn(const std::string& s)
{
    return s == "cotton" || s == "wool" || s == "polyester" ||
           s == "nylon" || s == "blend";
}

bool isKnownStitch(const std::string& s)
{
    return s == "jersey" || s == "rib" || s == "interlock" ||
           s == "purl" || s == "jacquard";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.gauge <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": gauge must be > 0 (got " +
              std::to_string(in.gauge) + ")");
    }

    if (!isKnownYarn(in.yarn_type)) {
        r.add("DFM-TEX-YARN", "info",
              std::string(kSkillId) + ": yarn_type '" + in.yarn_type +
              "' unrecognised — expected cotton | wool | polyester | nylon | blend");
    }

    if (!isKnownStitch(in.stitch_pattern)) {
        r.add("DFM-TEX-STITCH", "info",
              std::string(kSkillId) + ": stitch_pattern '" + in.stitch_pattern +
              "' unrecognised — expected jersey | rib | interlock | purl | jacquard");
    }

    if (in.gauge > 0.0 && (in.gauge < 4.0 || in.gauge > 60.0)) {
        r.add("DFM-TEX-GAUGE", "info",
              std::string(kSkillId) + ": gauge " + std::to_string(in.gauge) +
              " outside typical industrial range [4, 60] stitches/inch");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "knitting DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double stitches_per_cm = in.gauge / 2.54;  // gauge is per inch

    json params = {
        { "gauge",          in.gauge },
        { "yarn_type",      in.yarn_type },
        { "stitch_pattern", in.stitch_pattern },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "gauge",             in.gauge },
        { "yarn_type",         in.yarn_type },
        { "stitch_pattern",    in.stitch_pattern },
        { "stitches_per_cm",   stitches_per_cm },
        { "geometry_changed",  false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "knitting_machine";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "latched_needle";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Finer gauges → more stitches → longer cycle.
    tooling.est_cycle_time_s  = in.gauge > 0.0 ? in.gauge * 5.0 : 0.0;
    tooling.extra = {
        { "process",          "knitting" },
        { "stitch_pattern",   in.stitch_pattern },
        { "yarn_type",        in.yarn_type },
        { "stitches_per_cm",  stitches_per_cm },
        { "process_category", "textile" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::knitting applied: gauge={} yarn={} stitch={}",
        in.gauge, in.yarn_type, in.stitch_pattern);

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

}  // namespace koocadcam::skill::knitting
