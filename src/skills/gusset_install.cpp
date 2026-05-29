// @lat: [[engine/skills#gusset_install]]

#include "gusset_install.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::gusset_install {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.gusset_size_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": gusset_size_mm must be > 0 (got " +
              std::to_string(in.gusset_size_mm) + ")");
    } else {
        if (in.gusset_size_mm < 100.0) {
            r.add("DFM-GUS-SIZE", "error",
                  std::string(kSkillId) + ": gusset_size_mm " +
                  std::to_string(in.gusset_size_mm) +
                  " < 100 mm — undersized for structural joint");
        }
        if (in.gusset_size_mm > 1200.0) {
            r.add("DFM-GUS-SIZE", "error",
                  std::string(kSkillId) + ": gusset_size_mm " +
                  std::to_string(in.gusset_size_mm) +
                  " > 1200 mm — exceeds typical plate stock");
        }
    }

    if (in.weld_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": weld_length_mm must be > 0 (got " +
              std::to_string(in.weld_length_mm) + ")");
    } else if (in.gusset_size_mm > 0.0) {
        if (in.weld_length_mm < 2.0 * in.gusset_size_mm) {
            r.add("DFM-GUS-WELD", "error",
                  std::string(kSkillId) + ": weld_length_mm " +
                  std::to_string(in.weld_length_mm) +
                  " < 2 × gusset_size (" +
                  std::to_string(2.0 * in.gusset_size_mm) +
                  ") — insufficient weld for load transfer");
        }
        if (in.weld_length_mm > 4.0 * in.gusset_size_mm) {
            r.add("DFM-GUS-WELD", "info",
                  std::string(kSkillId) + ": weld_length_mm " +
                  std::to_string(in.weld_length_mm) +
                  " > 4 × gusset_size — likely overspec");
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
        std::string msg = "gusset_install DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "gusset_size_mm", in.gusset_size_mm },
        { "weld_length_mm", in.weld_length_mm },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "gusset_size_mm",   in.gusset_size_mm },
        { "weld_length_mm",   in.weld_length_mm },
        { "weld_ratio",       in.gusset_size_mm > 0.0
                                  ? in.weld_length_mm / in.gusset_size_mm
                                  : 0.0 },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "stick_welder";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = in.weld_length_mm;
    tooling.tool_material     = "low_hydrogen_electrode";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Manual SMAW ~ 150 mm/min effective.
    tooling.est_cycle_time_s  = in.weld_length_mm * 60.0 / 150.0;
    tooling.extra = {
        { "process",        "gusset_plate_install" },
        { "gusset_size_mm", in.gusset_size_mm },
        { "weld_length_mm", in.weld_length_mm },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::gusset_install applied: size={} weld={}",
                  in.gusset_size_mm, in.weld_length_mm);

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

}  // namespace koocadcam::skill::gusset_install
