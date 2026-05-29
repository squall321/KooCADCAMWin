// @lat: [[engine/skills#anchor_bolt_install]]

#include "anchor_bolt_install.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::anchor_bolt_install {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.bolt_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": bolt_dia_mm must be > 0 (got " +
              std::to_string(in.bolt_dia_mm) + ")");
    } else {
        if (in.bolt_dia_mm < 12.0) {
            r.add("DFM-AB-DIA", "error",
                  std::string(kSkillId) + ": bolt_dia_mm " +
                  std::to_string(in.bolt_dia_mm) +
                  " < 12 mm — below M12 structural anchor minimum");
        }
        if (in.bolt_dia_mm > 64.0) {
            r.add("DFM-AB-DIA", "error",
                  std::string(kSkillId) + ": bolt_dia_mm " +
                  std::to_string(in.bolt_dia_mm) +
                  " > 64 mm — exceeds typical anchor bolt range");
        }
    }

    if (in.embedment_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": embedment_mm must be > 0 (got " +
              std::to_string(in.embedment_mm) + ")");
    } else if (in.bolt_dia_mm > 0.0) {
        if (in.embedment_mm < 8.0 * in.bolt_dia_mm) {
            r.add("DFM-AB-EMBED", "error",
                  std::string(kSkillId) + ": embedment_mm " +
                  std::to_string(in.embedment_mm) +
                  " < 8 × bolt_dia (" +
                  std::to_string(8.0 * in.bolt_dia_mm) +
                  ") — insufficient pull-out capacity");
        }
        if (in.embedment_mm > 30.0 * in.bolt_dia_mm) {
            r.add("DFM-AB-EMBED", "info",
                  std::string(kSkillId) + ": embedment_mm " +
                  std::to_string(in.embedment_mm) +
                  " > 30 × bolt_dia — likely overspec");
        }
    }

    if (in.torque_nm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": torque_nm must be > 0 (got " +
              std::to_string(in.torque_nm) + ")");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "anchor_bolt_install DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double embedRatio = in.bolt_dia_mm > 0.0
                                  ? in.embedment_mm / in.bolt_dia_mm
                                  : 0.0;

    json params = {
        { "bolt_dia_mm",  in.bolt_dia_mm },
        { "embedment_mm", in.embedment_mm },
        { "torque_nm",    in.torque_nm },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "bolt_dia_mm",      in.bolt_dia_mm },
        { "embedment_mm",     in.embedment_mm },
        { "torque_nm",        in.torque_nm },
        { "embed_ratio",      embedRatio },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "torque_wrench";
    tooling.tool_dia_mm       = in.bolt_dia_mm;
    tooling.tool_length_mm    = in.embedment_mm;
    tooling.tool_material     = "alloy_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Setting + curing inspection: cast-in-place set ~ 60 s per bolt.
    tooling.est_cycle_time_s  = 60.0;
    tooling.extra = {
        { "process",      "anchor_bolt_install" },
        { "bolt_dia_mm",  in.bolt_dia_mm },
        { "embedment_mm", in.embedment_mm },
        { "torque_nm",    in.torque_nm },
        { "embed_ratio",  embedRatio },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::anchor_bolt_install applied: dia={} embed={} torque={}",
                  in.bolt_dia_mm, in.embedment_mm, in.torque_nm);

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

}  // namespace koocadcam::skill::anchor_bolt_install
