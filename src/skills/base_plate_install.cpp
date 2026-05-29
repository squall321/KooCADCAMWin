// @lat: [[engine/skills#base_plate_install]]

#include "base_plate_install.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::base_plate_install {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.plate_thick_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": plate_thick_mm must be > 0 (got " +
              std::to_string(in.plate_thick_mm) + ")");
    } else {
        if (in.plate_thick_mm < 10.0) {
            r.add("DFM-BP-THK", "error",
                  std::string(kSkillId) + ": plate_thick_mm " +
                  std::to_string(in.plate_thick_mm) +
                  " < 10 mm — too thin to distribute column load");
        }
        if (in.plate_thick_mm > 80.0) {
            r.add("DFM-BP-THK", "error",
                  std::string(kSkillId) + ": plate_thick_mm " +
                  std::to_string(in.plate_thick_mm) +
                  " > 80 mm — exceeds typical hot-rolled plate stock");
        }
    }

    if (in.bolt_count < 2) {
        r.add("DFM-BP-COUNT", "error",
              std::string(kSkillId) + ": bolt_count " +
              std::to_string(in.bolt_count) +
              " < 2 — at least 2 anchors required for stability");
    } else if (in.bolt_count > 16) {
        r.add("DFM-BP-COUNT", "error",
              std::string(kSkillId) + ": bolt_count " +
              std::to_string(in.bolt_count) +
              " > 16 — exceeds typical column base plate layout");
    } else if (in.bolt_count % 2 != 0) {
        r.add("DFM-BP-COUNT", "info",
              std::string(kSkillId) + ": bolt_count " +
              std::to_string(in.bolt_count) +
              " is odd — even count preferred for symmetric load");
    }

    if (in.anchor_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": anchor_dia_mm must be > 0 (got " +
              std::to_string(in.anchor_dia_mm) + ")");
    } else {
        if (in.anchor_dia_mm < 12.0) {
            r.add("DFM-BP-ANCH", "error",
                  std::string(kSkillId) + ": anchor_dia_mm " +
                  std::to_string(in.anchor_dia_mm) +
                  " < 12 mm — below M12 minimum structural anchor");
        }
        if (in.anchor_dia_mm > 64.0) {
            r.add("DFM-BP-ANCH", "error",
                  std::string(kSkillId) + ": anchor_dia_mm " +
                  std::to_string(in.anchor_dia_mm) +
                  " > 64 mm — exceeds typical anchor bolt range");
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
        std::string msg = "base_plate_install DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "plate_thick_mm", in.plate_thick_mm },
        { "bolt_count",     in.bolt_count },
        { "anchor_dia_mm",  in.anchor_dia_mm },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "plate_thick_mm",   in.plate_thick_mm },
        { "bolt_count",       in.bolt_count },
        { "anchor_dia_mm",    in.anchor_dia_mm },
        { "symmetric",        in.bolt_count % 2 == 0 },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "torque_wrench";
    tooling.tool_dia_mm       = in.anchor_dia_mm;
    tooling.tool_length_mm    = in.plate_thick_mm;
    tooling.tool_material     = "alloy_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Plate seat + torque sweep ~ 30 s per anchor.
    tooling.est_cycle_time_s  = 30.0 * static_cast<double>(in.bolt_count);
    tooling.extra = {
        { "process",        "base_plate_install" },
        { "plate_thick_mm", in.plate_thick_mm },
        { "bolt_count",     in.bolt_count },
        { "anchor_dia_mm",  in.anchor_dia_mm },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::base_plate_install applied: t={} bolts={} dia={}",
                  in.plate_thick_mm, in.bolt_count, in.anchor_dia_mm);

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

}  // namespace koocadcam::skill::base_plate_install
