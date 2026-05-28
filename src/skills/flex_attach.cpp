// @lat: [[engine/skills#flex_attach]]

#include "flex_attach.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::flex_attach {

using nlohmann::json;

namespace {

bool isKnownMethod(const std::string& m)
{
    return m == "ZIF"    || m == "ACF" ||
           m == "LIF"    || m == "solder";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.contact_count <= 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": contact_count must be > 0 (got " +
              std::to_string(in.contact_count) + ")");
    }
    if (in.retention_force_n <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": retention_force_n must be > 0 (got " +
              std::to_string(in.retention_force_n) + ")");
    } else {
        if (in.retention_force_n < 1.0) {
            r.add("DFM-FLEX-FORCE-LOW", "error",
                  std::string(kSkillId) + ": retention_force_n " +
                  std::to_string(in.retention_force_n) +
                  " < 1.0 N — cable will fall out under vibration");
        } else if (in.retention_force_n > 80.0) {
            r.add("DFM-FLEX-FORCE-HIGH", "info",
                  std::string(kSkillId) + ": retention_force_n " +
                  std::to_string(in.retention_force_n) +
                  " > 80 N — over-spec, copper-trace tearing risk on rework");
        }
    }

    if (!isKnownMethod(in.attach_method)) {
        r.add("DFM-FLEX-METHOD", "error",
              std::string(kSkillId) + ": attach_method '" + in.attach_method +
              "' not in catalog (expected ZIF | ACF | LIF | solder)");
    }

    if (in.attach_method == "ZIF" && in.contact_count > 100) {
        r.add("DFM-FLEX-PITCH", "info",
              std::string(kSkillId) + ": contact_count " +
              std::to_string(in.contact_count) +
              " on ZIF — sub-0.3 mm pitch territory; verify connector spec");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

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

    json params = {
        { "attach_method",     in.attach_method },
        { "contact_count",     in.contact_count },
        { "retention_force_n", in.retention_force_n },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "attach_method",     in.attach_method },
        { "contact_count",     in.contact_count },
        { "retention_force_n", in.retention_force_n },
        { "geometry_changed",  false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = (in.attach_method == "ACF") ? "hot_bar_bonder"
                                                            : "manual_or_robot_inserter";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = (in.attach_method == "ACF") ? "ACF_film"
                                                            : "connector_assembly";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ZIF/LIF: ~5 s of manual insertion.  ACF: 15 s hot-bar dwell.
    tooling.est_cycle_time_s  = (in.attach_method == "ACF") ? 15.0 : 5.0;
    tooling.extra = {
        { "process",           "flex_cable_attach" },
        { "process_category",  "electronics_assembly" },
        { "attach_method",     in.attach_method },
        { "contact_count",     in.contact_count },
        { "retention_force_n", in.retention_force_n },
        { "geometric_no_op",   true },
        { "dfm_findings",      findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::flex_attach applied: method={} contacts={} F={} N",
                  in.attach_method, in.contact_count, in.retention_force_n);

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

}  // namespace koocadcam::skill::flex_attach
