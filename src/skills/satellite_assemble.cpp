// @lat: [[engine/skills#satellite_assemble]]

#include "satellite_assemble.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::satellite_assemble {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.bus_id.empty()) {
        r.add("DFM-INPUT", "error",
              "satellite_assemble bus_id must not be empty");
    }

    if (in.mass_kg <= 0.0) {
        r.add("DFM-SPACE-MASS", "error",
              "satellite_assemble mass_kg " + std::to_string(in.mass_kg) +
              " must be > 0");
    } else if (in.mass_kg > 25000.0) {
        r.add("DFM-SPACE-MASS", "error",
              "satellite_assemble mass_kg " + std::to_string(in.mass_kg) +
              " > 25000 kg — exceeds heaviest commercial GEO bus envelope");
    }

    if (in.instrument_count < 1) {
        r.add("DFM-SPACE-PAYLOAD", "error",
              "satellite_assemble instrument_count " +
              std::to_string(in.instrument_count) +
              " < 1 — a bus must carry at least one payload");
    } else if (in.instrument_count > 64) {
        r.add("DFM-SPACE-PAYLOAD", "error",
              "satellite_assemble instrument_count " +
              std::to_string(in.instrument_count) +
              " > 64 — exceeds practical payload deck mounting envelope");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "satellite_assemble DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "bus_id",           in.bus_id },
        { "mass_kg",          in.mass_kg },
        { "instrument_count", in.instrument_count },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "bus_id",            in.bus_id },
        { "mass_kg",           in.mass_kg },
        { "instrument_count",  in.instrument_count },
        { "mass_per_instrument_kg",
          in.instrument_count > 0
              ? (in.mass_kg / static_cast<double>(in.instrument_count))
              : 0.0 },
        { "geometry_changed",  false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "clean_room_fixture";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "anodized_aluminum";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Rough estimate: 8 person-hours per instrument integration.
    tooling.est_cycle_time_s  =
        in.instrument_count * 8.0 * 3600.0;
    tooling.extra = {
        { "process",          "satellite_AIT_integration" },
        { "bus_id",           in.bus_id },
        { "mass_kg",          in.mass_kg },
        { "instrument_count", in.instrument_count },
        { "clean_room_class", "ISO-7" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::satellite_assemble applied: bus={} mass={}kg instruments={}",
                  in.bus_id, in.mass_kg, in.instrument_count);

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

}  // namespace koocadcam::skill::satellite_assemble
