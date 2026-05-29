// @lat: [[engine/skills#bms_install]]

#include "bms_install.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::bms_install {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.bms_id.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": bms_id must be non-empty");
    }

    if (in.channel_count <= 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": channel_count must be > 0 (got " +
              std::to_string(in.channel_count) + ")");
    } else if (in.channel_count < 4 || in.channel_count > 192) {
        r.add("DFM-BMS-CH", "error",
              std::string(kSkillId) + ": channel_count " +
              std::to_string(in.channel_count) +
              " outside supported range [4, 192]");
    }

    if (in.communication_protocol != "CAN" &&
        in.communication_protocol != "LIN") {
        r.add("DFM-BMS-COMM", "error",
              std::string(kSkillId) + ": communication_protocol '" +
              in.communication_protocol + "' not in {CAN, LIN}");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "bms_install DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "bms_id",                 in.bms_id },
        { "channel_count",          in.channel_count },
        { "communication_protocol", in.communication_protocol },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "bms_id",                 in.bms_id },
        { "channel_count",          in.channel_count },
        { "communication_protocol", in.communication_protocol },
        { "bus_speed_kbps",
          in.communication_protocol == "CAN" ? 500 : 20 },
        { "geometry_changed",       false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "bms_install_station";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "pcb_fr4";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ~ 90 s for mounting + 0.8 s per harness channel terminate.
    tooling.est_cycle_time_s  = 90.0
                                + 0.8 * static_cast<double>(in.channel_count);
    tooling.extra = {
        { "process",                "ev_bms_install" },
        { "bms_id",                 in.bms_id },
        { "channel_count",          in.channel_count },
        { "communication_protocol", in.communication_protocol },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::bms_install applied: bms={} channels={} protocol={}",
                  in.bms_id, in.channel_count, in.communication_protocol);

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

}  // namespace koocadcam::skill::bms_install
