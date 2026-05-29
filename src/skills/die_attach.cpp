// @lat: [[engine/skills#die_attach]]

#include "die_attach.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::die_attach {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.die_size_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "die_attach die_size_mm must be > 0");
    } else {
        if (in.die_size_mm < 0.3) {
            r.add("DFM-DIE-SIZE", "error",
                  "die_attach die_size_mm " + std::to_string(in.die_size_mm) +
                  " < 0.3 mm — below practical singulation limit");
        }
        if (in.die_size_mm > 25.0) {
            r.add("DFM-DIE-SIZE", "error",
                  "die_attach die_size_mm " + std::to_string(in.die_size_mm) +
                  " > 25.0 mm — die too large for standard pick-and-place");
        }
    }

    if (in.bond_force_n <= 0.0) {
        r.add("DFM-INPUT", "error",
              "die_attach bond_force_n must be > 0");
    } else {
        if (in.bond_force_n < 0.05) {
            r.add("DFM-BOND-FORCE", "error",
                  "die_attach bond_force_n " + std::to_string(in.bond_force_n) +
                  " < 0.05 N — insufficient wetting of adhesive");
        }
        if (in.bond_force_n > 30.0) {
            r.add("DFM-BOND-FORCE", "error",
                  "die_attach bond_force_n " + std::to_string(in.bond_force_n) +
                  " > 30 N — risk of die cracking");
        }
    }

    if (in.adhesive_type != "epoxy" && in.adhesive_type != "AgPaste" &&
        in.adhesive_type != "solder") {
        r.add("DFM-ADHESIVE", "info",
              "die_attach adhesive_type '" + in.adhesive_type +
              "' unrecognised — expected 'epoxy' / 'AgPaste' / 'solder'");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "die_attach DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "adhesive_type", in.adhesive_type },
        { "die_size_mm",   in.die_size_mm },
        { "bond_force_n",  in.bond_force_n },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "adhesive_type",    in.adhesive_type },
        { "die_size_mm",      in.die_size_mm },
        { "bond_force_n",     in.bond_force_n },
        { "blt_um",
          in.adhesive_type == "solder" ? 25.0 :
          (in.adhesive_type == "AgPaste" ? 30.0 : 40.0) },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = (in.adhesive_type == "solder")
                                    ? "eutectic_die_bonder"
                                    : "epoxy_die_bonder";
    tooling.tool_dia_mm       = in.die_size_mm;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "vacuum_collet";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = (in.adhesive_type == "solder") ? 3.5 : 2.0;
    tooling.extra = {
        { "process",       "die_attach" },
        { "adhesive_type", in.adhesive_type },
        { "die_size_mm",   in.die_size_mm },
        { "bond_force_n",  in.bond_force_n },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::die_attach applied: adhesive={} die_size={} force={}",
                  in.adhesive_type, in.die_size_mm, in.bond_force_n);

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

}  // namespace koocadcam::skill::die_attach
