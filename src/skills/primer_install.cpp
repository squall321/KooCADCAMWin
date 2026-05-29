// @lat: [[engine/skills#primer_install]]

#include "primer_install.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::primer_install {

using nlohmann::json;

namespace {

bool isKnownPrimerSize(const std::string& s)
{
    return s == "small_pistol" || s == "large_pistol" ||
           s == "small_rifle"  || s == "large_rifle"  ||
           s == "magnum";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.primer_size.empty()) {
        r.add("DFM-INPUT", "error",
              "primer_install primer_size must not be empty");
    } else if (!isKnownPrimerSize(in.primer_size)) {
        r.add("DFM-AMMO-PRIMER", "info",
              "primer_install primer_size '" + in.primer_size +
              "' unrecognised — expected small_pistol/large_pistol/"
              "small_rifle/large_rifle/magnum");
    }

    if (in.primer_brand.empty()) {
        r.add("DFM-AMMO-BRAND", "error",
              "primer_install primer_brand must not be empty (traceability required)");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "primer_install DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "primer_size",  in.primer_size },
        { "primer_brand", in.primer_brand },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "primer_size",      in.primer_size },
        { "primer_brand",     in.primer_brand },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "primer_seating_punch";
    tooling.tool_dia_mm       = (in.primer_size.find("large") != std::string::npos)
                                    ? 5.33 : 4.45;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 1.0;
    tooling.extra = {
        { "process",      "primer_seat" },
        { "primer_size",  in.primer_size },
        { "primer_brand", in.primer_brand },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::primer_install applied: size={} brand={}",
                  in.primer_size, in.primer_brand);

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

}  // namespace koocadcam::skill::primer_install
