// @lat: [[engine/skills#catenary_install]]

#include "catenary_install.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::catenary_install {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.line_voltage_kv <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": line_voltage_kv must be > 0 (got " +
              std::to_string(in.line_voltage_kv) + ")");
    } else {
        if (in.line_voltage_kv < 0.6) {
            r.add("DFM-CT-VOLT", "error",
                  std::string(kSkillId) + ": line_voltage_kv " +
                  std::to_string(in.line_voltage_kv) +
                  " < 0.6 kV — below traction system minimum");
        }
        if (in.line_voltage_kv > 50.0) {
            r.add("DFM-CT-VOLT", "error",
                  std::string(kSkillId) + ": line_voltage_kv " +
                  std::to_string(in.line_voltage_kv) +
                  " > 50 kV — exceeds standard traction system range");
        }
    }

    if (in.span_m <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": span_m must be > 0 (got " +
              std::to_string(in.span_m) + ")");
    } else {
        if (in.span_m < 30.0) {
            r.add("DFM-CT-SPAN", "error",
                  std::string(kSkillId) + ": span_m " +
                  std::to_string(in.span_m) +
                  " < 30 m — uneconomic mast density");
        }
        if (in.span_m > 80.0) {
            r.add("DFM-CT-SPAN", "error",
                  std::string(kSkillId) + ": span_m " +
                  std::to_string(in.span_m) +
                  " > 80 m — excessive sag/swing under wind load");
        }
    }

    if (in.height_m <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": height_m must be > 0 (got " +
              std::to_string(in.height_m) + ")");
    } else {
        if (in.height_m < 4.7) {
            r.add("DFM-CT-HEIGHT", "error",
                  std::string(kSkillId) + ": height_m " +
                  std::to_string(in.height_m) +
                  " < 4.7 m — below standard contact-wire height ATR");
        }
        if (in.height_m > 6.5) {
            r.add("DFM-CT-HEIGHT", "error",
                  std::string(kSkillId) + ": height_m " +
                  std::to_string(in.height_m) +
                  " > 6.5 m — pantograph reach exceeded");
        }
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "catenary_install DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string current_type =
        (in.line_voltage_kv >= 10.0) ? "AC" : "DC";

    json params = {
        { "line_voltage_kv", in.line_voltage_kv },
        { "span_m",          in.span_m },
        { "height_m",        in.height_m },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "line_voltage_kv",  in.line_voltage_kv },
        { "span_m",           in.span_m },
        { "height_m",         in.height_m },
        { "current_type",     current_type },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "catenary_stringing_train";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = in.span_m * 1000.0;
    tooling.tool_material     = "copper_contact_wire";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ~ 1800 s per span for full system stringing + tensioning.
    tooling.est_cycle_time_s  = 1800.0;
    tooling.extra = {
        { "process",         "catenary_install" },
        { "line_voltage_kv", in.line_voltage_kv },
        { "span_m",          in.span_m },
        { "height_m",        in.height_m },
        { "current_type",    current_type },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::catenary_install applied: voltage={} span={} height={}",
                  in.line_voltage_kv, in.span_m, in.height_m);

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

}  // namespace koocadcam::skill::catenary_install
