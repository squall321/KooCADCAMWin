// @lat: [[engine/skills#dye_penetrant_test]]

#include "dye_penetrant_test.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <string>
#include <vector>

namespace koocadcam::skill::dye_penetrant_test {

using nlohmann::json;

namespace {

bool isAllowedSensitivity(const std::string& s)
{
    return s == "level_1_low"  || s == "level_2_med" ||
           s == "level_3_high" || s == "level_4_ultra";
}

bool isPorousMaterial(const std::string& m)
{
    return m.find("sintered")       != std::string::npos ||
           m.find("powder_metal")   != std::string::npos ||
           m.find("foam")           != std::string::npos ||
           m.find("porous")         != std::string::npos;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.dwell_time_min <= 0.0) {
        r.add("DFM-INPUT", "error",
              "dye_penetrant_test dwell_time_min must be > 0 (got " +
              std::to_string(in.dwell_time_min) + ")");
    } else if (in.dwell_time_min < 5.0) {
        r.add("DFM-PT-DWELL-SHORT", "info",
              "dye_penetrant_test dwell_time_min " +
              std::to_string(in.dwell_time_min) +
              " < 5 — under-soak; tight cracks may not fill");
    } else if (in.dwell_time_min > 60.0) {
        r.add("DFM-PT-DWELL-LONG", "info",
              "dye_penetrant_test dwell_time_min " +
              std::to_string(in.dwell_time_min) +
              " > 60 — over-soak; excess penetrant may obscure indications");
    }

    if (!isAllowedSensitivity(in.sensitivity_class)) {
        r.add("DFM-INPUT", "error",
              "dye_penetrant_test sensitivity_class '" + in.sensitivity_class +
              "' not in {level_1_low, level_2_med, level_3_high, level_4_ultra}");
    }

    auto faceId = wp.resolve(in.target_face_datum);
    if (!faceId) {
        r.add("DFM-PT-FACE", "error",
              "dye_penetrant_test target_face_datum did not resolve");
    } else if (wp.faceArea(*faceId) <= 0.0) {
        r.add("DFM-PT-AREA", "error",
              "dye_penetrant_test: target face has non-positive area");
    }

    if (isPorousMaterial(wp.material())) {
        r.add("DFM-PT-MATERIAL", "info",
              "dye_penetrant_test on porous material '" + wp.material() +
              "' — penetrant absorption may obscure surface crack indications");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "dye_penetrant_test DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto faceId = wp.resolve(in.target_face_datum);
    if (!faceId) throw SkillError("dye_penetrant_test: face datum unresolved");

    json params = {
        { "target_face_id",    *faceId },
        { "dwell_time_min",    in.dwell_time_min },
        { "sensitivity_class", in.sensitivity_class },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_inspection_only",  true },
        { "geometric_change",    false },
        { "target_face_id",      *faceId },
        { "dwell_time_min",      in.dwell_time_min },
        { "sensitivity_class",   in.sensitivity_class },
        { "inspection_method",   "liquid_penetrant_visible_red" },
        { "ndt_category",        "PT" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "penetrant_kit";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "penetrant_dye";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Apply (~ 30 s) + dwell + wipe (~ 30 s) + developer (~ 60 s) + inspect.
    tooling.est_cycle_time_s  = (in.dwell_time_min * 60.0) + 120.0;
    tooling.extra["dfm_findings"]         = findings;
    tooling.extra["machining_constraint"] =
        "inspection — liquid penetrant, no material removal";
    tooling.extra["ndt_category"]         = "PT";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::dye_penetrant_test applied: face={} dwell={}min sens={}",
                  *faceId, in.dwell_time_min, in.sensitivity_class);

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

}  // namespace koocadcam::skill::dye_penetrant_test
