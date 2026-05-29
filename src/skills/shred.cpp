// @lat: [[engine/skills#shred]]

#include "shred.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <memory>
#include <string>

namespace koocadcam::skill::shred {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.throughput_kg_h <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": throughput_kg_h must be > 0 (got " +
              std::to_string(in.throughput_kg_h) + ")");
    }
    if (in.final_particle_size_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": final_particle_size_mm must be > 0 (got " +
              std::to_string(in.final_particle_size_mm) + ")");
    }

    if (in.final_particle_size_mm > 0.0 && in.final_particle_size_mm < 1.0) {
        r.add("DFM-SHRED-FINE", "info",
              std::string(kSkillId) + ": final_particle_size_mm " +
              std::to_string(in.final_particle_size_mm) +
              " < 1 mm — very fine grind, consider cryogenic shredding "
              "and dust-explosion safeguards");
    }
    if (in.final_particle_size_mm > 200.0) {
        r.add("DFM-SHRED-COARSE", "info",
              std::string(kSkillId) + ": final_particle_size_mm " +
              std::to_string(in.final_particle_size_mm) +
              " > 200 mm — coarse primary shred only; downstream "
              "sortation may need a secondary grind");
    }
    if (in.throughput_kg_h > 5000.0) {
        r.add("DFM-SHRED-THRU", "info",
              std::string(kSkillId) + ": throughput_kg_h " +
              std::to_string(in.throughput_kg_h) +
              " > 5000 — exceeds typical single-shaft commercial shredder");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "shred DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "throughput_kg_h",        in.throughput_kg_h },
        { "final_particle_size_mm", in.final_particle_size_mm },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "throughput_kg_h",        in.throughput_kg_h },
        { "final_particle_size_mm", in.final_particle_size_mm },
        { "process_category",       "eol_size_reduction" },
        { "geometry_changed",       false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "shredder";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "hardened_steel_rotor";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Assume 1 kg of part processed at the stated throughput: cycle ≈ 3600 / rate s.
    const double rate = std::max(1.0, in.throughput_kg_h);
    tooling.est_cycle_time_s  = 3600.0 / rate;
    tooling.extra = {
        { "process",                 "shred" },
        { "throughput_kg_h",         in.throughput_kg_h },
        { "final_particle_size_mm",  in.final_particle_size_mm },
        { "process_category",        "eol_size_reduction" },
        { "geometric_no_op",         true },
        { "dfm_findings",            findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::shred applied: thru={}kg/h size={}mm",
                  in.throughput_kg_h, in.final_particle_size_mm);

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

}  // namespace koocadcam::skill::shred
