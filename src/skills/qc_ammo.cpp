// @lat: [[engine/skills#qc_ammo]]

#include "qc_ammo.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::qc_ammo {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.cartridge_oal_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "qc_ammo cartridge_oal_mm must be > 0");
    } else {
        if (in.cartridge_oal_mm < 5.0) {
            r.add("DFM-AMMO-OAL", "error",
                  "qc_ammo cartridge_oal_mm " +
                  std::to_string(in.cartridge_oal_mm) +
                  " < 5.0 mm — below smallest small-arms cartridge");
        }
        if (in.cartridge_oal_mm > 150.0) {
            r.add("DFM-AMMO-OAL", "error",
                  "qc_ammo cartridge_oal_mm " +
                  std::to_string(in.cartridge_oal_mm) +
                  " > 150 mm — exceeds small-arms classification");
        }
    }

    if (in.accept_count < 0) {
        r.add("DFM-INPUT", "error",
              "qc_ammo accept_count must be >= 0");
    }
    if (in.reject_count < 0) {
        r.add("DFM-INPUT", "error",
              "qc_ammo reject_count must be >= 0");
    }

    const int total = in.accept_count + in.reject_count;
    if (total <= 0) {
        r.add("DFM-AMMO-QCSAMPLE", "error",
              "qc_ammo accept_count + reject_count must be > 0 (need a sample)");
    } else {
        const double yieldPct =
            100.0 * static_cast<double>(in.accept_count) /
            static_cast<double>(total);
        if (yieldPct < 95.0) {
            r.add("DFM-AMMO-YIELD", "info",
                  "qc_ammo yield " + std::to_string(yieldPct) +
                  "% < 95% — lot review recommended");
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
        std::string msg = "qc_ammo DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const int total = in.accept_count + in.reject_count;
    const double yieldPct =
        total > 0
            ? (100.0 * static_cast<double>(in.accept_count) /
               static_cast<double>(total))
            : 0.0;

    json params = {
        { "cartridge_oal_mm", in.cartridge_oal_mm },
        { "accept_count",     in.accept_count },
        { "reject_count",     in.reject_count },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "cartridge_oal_mm", in.cartridge_oal_mm },
        { "accept_count",     in.accept_count },
        { "reject_count",     in.reject_count },
        { "yield_pct",        yieldPct },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "qc_gauge_set";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = in.cartridge_oal_mm;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 0.8 * static_cast<double>(total);
    tooling.extra = {
        { "process",          "ammo_final_qc" },
        { "cartridge_oal_mm", in.cartridge_oal_mm },
        { "accept_count",     in.accept_count },
        { "reject_count",     in.reject_count },
        { "yield_pct",        yieldPct },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::qc_ammo applied: oal={} accept={} reject={} yield={}",
                  in.cartridge_oal_mm, in.accept_count, in.reject_count,
                  yieldPct);

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

}  // namespace koocadcam::skill::qc_ammo
