// @lat: [[engine/skills#ic_bond]]

#include "ic_bond.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::ic_bond {

using nlohmann::json;

namespace {

bool isKnownBondType(const std::string& t)
{
    return t == "ball"   || t == "wedge" ||
           t == "stitch" || t == "ribbon";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.bond_count <= 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": bond_count must be > 0 (got " +
              std::to_string(in.bond_count) + ")");
    }
    if (in.ultrasonic_power_w < 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": ultrasonic_power_w must be >= 0 (got " +
              std::to_string(in.ultrasonic_power_w) + ")");
    } else {
        if (in.ultrasonic_power_w > 5.0) {
            r.add("DFM-BOND-USPOWER-HIGH", "error",
                  std::string(kSkillId) + ": ultrasonic_power_w " +
                  std::to_string(in.ultrasonic_power_w) +
                  " W > 5.0 — die crack / pad-lift risk");
        } else if (in.ultrasonic_power_w < 0.05 && in.ultrasonic_power_w > 0.0) {
            r.add("DFM-BOND-USPOWER-LOW", "info",
                  std::string(kSkillId) + ": ultrasonic_power_w " +
                  std::to_string(in.ultrasonic_power_w) +
                  " W < 0.05 — non-stick / lifted-ball risk");
        }
    }

    if (!isKnownBondType(in.bond_type)) {
        r.add("DFM-BOND-TYPE", "error",
              std::string(kSkillId) + ": bond_type '" + in.bond_type +
              "' not in catalog (expected ball | wedge | stitch | ribbon)");
    }

    if (in.bond_count > 5000) {
        r.add("DFM-BOND-COUNT-HIGH", "info",
              std::string(kSkillId) + ": bond_count " +
              std::to_string(in.bond_count) +
              " > 5000 — long cycle time, consider flip-chip");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = std::string(kSkillId) + " DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "bond_type",          in.bond_type },
        { "bond_count",         in.bond_count },
        { "ultrasonic_power_w", in.ultrasonic_power_w },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "bond_type",           in.bond_type },
        { "bond_count",          in.bond_count },
        { "ultrasonic_power_w",  in.ultrasonic_power_w },
        { "geometry_changed",    false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "wire_bonder";
    tooling.tool_dia_mm       = 0.0;       // wire dia handled separately
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = (in.bond_type == "wedge") ? "aluminum_wire" : "gold_wire";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;       // additive (wire deposit)
    // Modern wire bonders ≈ 8 bonds/second.  Cycle = count / 8.
    tooling.est_cycle_time_s  = static_cast<double>(in.bond_count) / 8.0;
    tooling.extra = {
        { "process",            "ic_wire_bond" },
        { "process_category",   "electronics_assembly" },
        { "bond_type",          in.bond_type },
        { "bond_count",         in.bond_count },
        { "ultrasonic_power_w", in.ultrasonic_power_w },
        { "geometric_no_op",    true },
        { "dfm_findings",       findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::ic_bond applied: type={} count={} usp={}W",
                  in.bond_type, in.bond_count, in.ultrasonic_power_w);

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

}  // namespace koocadcam::skill::ic_bond
