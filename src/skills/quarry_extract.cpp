// @lat: [[engine/skills#quarry_extract]]

#include "quarry_extract.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::quarry_extract {

using nlohmann::json;

namespace {

bool isKnownMethod(const std::string& m)
{
    return m == "drill_blast" || m == "diamond_wire" ||
           m == "channeling"  || m == "ripping";
}

bool isDimensionStone(const std::string& r)
{
    return r == "marble" || r == "granite" || r == "limestone" ||
           r == "sandstone";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.volume_m3 <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": volume_m3 must be > 0 (got " +
              std::to_string(in.volume_m3) + ")");
    }

    if (in.rock_type.empty()) {
        r.add("DFM-INPUT", "warning",
              std::string(kSkillId) + ": rock_type empty — defaulting to 'granite'");
    }

    if (!isKnownMethod(in.extraction_method)) {
        r.add("DFM-QUARRY-METHOD", "info",
              std::string(kSkillId) + ": extraction_method '" +
              in.extraction_method +
              "' not in {drill_blast, diamond_wire, channeling, ripping}");
    }

    if (in.extraction_method == "drill_blast" &&
        (in.rock_type == "marble" || in.rock_type == "limestone")) {
        r.add("DFM-QUARRY-METHOD-FIT", "warning",
              std::string(kSkillId) + ": drill_blast on '" + in.rock_type +
              "' induces microfracture — prefer diamond_wire/channeling for dimension stone");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "quarry_extract DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string rock = in.rock_type.empty() ? "granite" : in.rock_type;

    json params = {
        { "rock_type",         rock },
        { "volume_m3",         in.volume_m3 },
        { "extraction_method", in.extraction_method },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "rock_type",           rock },
        { "volume_m3",           in.volume_m3 },
        { "extraction_method",   in.extraction_method },
        { "is_dimension_stone",  isDimensionStone(rock) },
        { "geometry_changed",    false },
    };

    ToolingMeta tooling;
    tooling.tool_type         =
        (in.extraction_method == "diamond_wire") ? "diamond_wire_saw"
      : (in.extraction_method == "channeling")   ? "channeling_machine"
      : (in.extraction_method == "drill_blast")  ? "rock_drill_explosives"
      : (in.extraction_method == "ripping")      ? "ripper_dozer"
                                                 : "quarry_generic";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "diamond_wire";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Rough cycle: diamond wire ~2 m^2/h ⇒ ~0.5 h/m^3 on a 2 m cut, etc.
    tooling.est_cycle_time_s  = in.volume_m3 * 1800.0;
    tooling.extra = {
        { "process",           "quarry_extraction" },
        { "rock_type",         rock },
        { "volume_m3",         in.volume_m3 },
        { "extraction_method", in.extraction_method },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    // No geometric change — clone shape + material.
    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::quarry_extract applied: rock={} vol_m3={} method={}",
                  rock, in.volume_m3, in.extraction_method);

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

}  // namespace koocadcam::skill::quarry_extract
