// @lat: [[engine/skills#mems_dicing]]

#include "mems_dicing.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::mems_dicing {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.kerf_um <= 0.0) {
        r.add("DFM-INPUT", "error",
              "mems_dicing kerf_um must be > 0");
    } else {
        if (in.kerf_um < 10.0) {
            r.add("DFM-KERF", "error",
                  "mems_dicing kerf_um " + std::to_string(in.kerf_um) +
                  " < 10 um — below blade / laser kerf tolerance");
        }
        if (in.kerf_um > 200.0) {
            r.add("DFM-KERF", "error",
                  "mems_dicing kerf_um " + std::to_string(in.kerf_um) +
                  " > 200 um — wastes wafer area and risks chipping");
        }
    }

    if (in.die_size_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "mems_dicing die_size_mm must be > 0");
    } else {
        if (in.die_size_mm < 0.3) {
            r.add("DFM-DIE-SIZE", "error",
                  "mems_dicing die_size_mm " + std::to_string(in.die_size_mm) +
                  " < 0.3 mm — die too small to handle post-dicing");
        }
        if (in.die_size_mm > 30.0) {
            r.add("DFM-DIE-SIZE", "error",
                  "mems_dicing die_size_mm " + std::to_string(in.die_size_mm) +
                  " > 30 mm — exceeds reasonable wafer-die size");
        }
    }

    if (in.blade_type != "diamond_resin" && in.blade_type != "Ni_bonded" &&
        in.blade_type != "stealth_laser") {
        r.add("DFM-BLADE", "info",
              "mems_dicing blade_type '" + in.blade_type +
              "' unrecognised — expected 'diamond_resin' / 'Ni_bonded' / 'stealth_laser'");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "mems_dicing DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Stealth laser → no kerf rinse needed; resin / Ni-bonded blades → DI rinse.
    const bool needs_rinse = (in.blade_type != "stealth_laser");
    // Per-street linear speed: laser is ~ 4×  blade for same kerf quality.
    const double feed_mm_per_s = (in.blade_type == "stealth_laser") ? 200.0 : 50.0;

    json params = {
        { "kerf_um",     in.kerf_um },
        { "die_size_mm", in.die_size_mm },
        { "blade_type",  in.blade_type },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "kerf_um",          in.kerf_um },
        { "die_size_mm",      in.die_size_mm },
        { "blade_type",       in.blade_type },
        { "feed_mm_per_s",    feed_mm_per_s },
        { "rinse_required",   needs_rinse },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = (in.blade_type == "stealth_laser")
                                    ? "stealth_dicing_laser"
                                    : "wafer_dicing_saw";
    tooling.tool_dia_mm       = in.kerf_um / 1000.0;       // um → mm
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     =
        (in.blade_type == "Ni_bonded")     ? "Ni_bonded_diamond" :
        (in.blade_type == "stealth_laser") ? "Nd_YAG_laser" :
                                             "resin_bonded_diamond";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Rough: 300 mm wafer with ~2000 streets / die_size, scaled by feed.
    tooling.est_cycle_time_s  = 300.0 / feed_mm_per_s * 30.0;
    tooling.extra = {
        { "process",        "mems_dicing" },
        { "kerf_um",        in.kerf_um },
        { "die_size_mm",    in.die_size_mm },
        { "blade_type",     in.blade_type },
        { "rinse_required", needs_rinse },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::mems_dicing applied: kerf={}um die={}mm blade={}",
                  in.kerf_um, in.die_size_mm, in.blade_type);

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

}  // namespace koocadcam::skill::mems_dicing
