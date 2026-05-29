// @lat: [[engine/skills#hull_lay]]

#include "hull_lay.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::hull_lay {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.hull_section_id.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": hull_section_id must be non-empty");
    }

    if (in.plate_thick_mm < 4.0 || in.plate_thick_mm > 60.0) {
        r.add("DFM-HULL-PLATE", "error",
              std::string(kSkillId) + ": plate_thick_mm " +
              std::to_string(in.plate_thick_mm) +
              " outside [4, 60] mm — outside shipyard shell-plate gauge range");
    }

    if (in.length_m <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": length_m must be > 0 (got " +
              std::to_string(in.length_m) + ")");
    }

    if (in.beam_m <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": beam_m must be > 0 (got " +
              std::to_string(in.beam_m) + ")");
    }

    if (in.length_m > 30.0) {
        r.add("DFM-HULL-LEN", "info",
              std::string(kSkillId) + ": length_m " +
              std::to_string(in.length_m) +
              " > 30 — mega-block, plan multi-crane synchronous lift");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "hull_lay DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "hull_section_id", in.hull_section_id },
        { "plate_thick_mm",  in.plate_thick_mm },
        { "length_m",        in.length_m },
        { "beam_m",          in.beam_m },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "hull_section_id",   in.hull_section_id },
        { "plate_thick_mm",    in.plate_thick_mm },
        { "length_m",          in.length_m },
        { "beam_m",            in.beam_m },
        { "footprint_m2",      in.length_m * in.beam_m },
        { "geometry_changed",  false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "gantry_crane";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = in.length_m * 1000.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Allow ~ 8 h to land + tack a full hull section onto the keel blocks.
    tooling.est_cycle_time_s  = 8.0 * 3600.0;
    tooling.extra = {
        { "process",          "hull_section_lay" },
        { "hull_section_id",  in.hull_section_id },
        { "plate_thick_mm",   in.plate_thick_mm },
        { "length_m",         in.length_m },
        { "beam_m",           in.beam_m },
        { "code_reference",   "IACS_PR2_block_erection" },
        { "process_category", "shipbuilding_hull" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::hull_lay applied: section={} plate={}mm L={}m B={}m",
        in.hull_section_id, in.plate_thick_mm, in.length_m, in.beam_m);

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

}  // namespace koocadcam::skill::hull_lay
