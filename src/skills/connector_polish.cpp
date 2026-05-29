// @lat: [[engine/skills#connector_polish]]

#include "connector_polish.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <string>

namespace koocadcam::skill::connector_polish {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.return_loss_db >= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": return_loss_db must be < 0 (got " +
              std::to_string(in.return_loss_db) +
              ") — return loss is reported as a NEGATIVE dB value");
    }

    const bool knownClass =
        (in.polish_class == "UPC" ||
         in.polish_class == "APC" ||
         in.polish_class == "PC");
    if (!knownClass) {
        r.add("DFM-POLISH-CLASS", "error",
              std::string(kSkillId) + ": polish_class '" + in.polish_class +
              "' invalid — expected UPC | APC | PC");
    }

    if (in.return_loss_db > -20.0 && in.return_loss_db < 0.0) {
        r.add("DFM-POLISH-RL-WEAK", "error",
              std::string(kSkillId) + ": return_loss_db " +
              std::to_string(in.return_loss_db) +
              " too weak — > −20 dB indicates failed polish");
    }

    // Class-specific realism checks (info-level).
    if (knownClass && in.polish_class == "UPC" && in.return_loss_db < -55.0) {
        r.add("DFM-POLISH-RL-UPC", "info",
              std::string(kSkillId) + ": UPC return_loss_db " +
              std::to_string(in.return_loss_db) +
              " stronger than typical (~ −50 dB) — verify measurement");
    }
    if (knownClass && in.polish_class == "APC" && in.return_loss_db > -50.0) {
        r.add("DFM-POLISH-RL-APC", "info",
              std::string(kSkillId) + ": APC return_loss_db " +
              std::to_string(in.return_loss_db) +
              " weak for APC (typ. ≤ −60 dB) — re-check endface angle");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "connector_polish DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double endface_angle_deg = (in.polish_class == "APC") ? 8.0 : 0.0;

    json params = {
        { "polish_class",    in.polish_class },
        { "return_loss_db",  in.return_loss_db },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "polish_class",      in.polish_class },
        { "return_loss_db",    in.return_loss_db },
        { "endface_angle_deg", endface_angle_deg },
        { "geometry_changed",  false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "ferrule_polisher";
    tooling.tool_dia_mm       = 12.0;       // typical jig diameter
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "lapping_film_stack";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;        // sub-µm removal — negligible
    // Multi-stage polish: typical 4 × 30 s film changes + final.
    tooling.est_cycle_time_s  = (in.polish_class == "APC") ? 180.0 : 150.0;
    tooling.extra = {
        { "process",            "connector_polish" },
        { "polish_class",       in.polish_class },
        { "endface_angle_deg",  endface_angle_deg },
        { "return_loss_db",     in.return_loss_db },
        { "dfm_findings",       findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::connector_polish applied: class={} RL={}dB angle={}°",
                  in.polish_class, in.return_loss_db, endface_angle_deg);

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

}  // namespace koocadcam::skill::connector_polish
