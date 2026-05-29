// @lat: [[engine/skills#fuel_cell_stack_assemble]]

#include "fuel_cell_stack_assemble.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>
#include <string>

namespace koocadcam::skill::fuel_cell_stack_assemble {

using nlohmann::json;

namespace {

double perCellOcv(const std::string& mea)
{
    if (mea == "PEM")  return 1.00;
    if (mea == "SOFC") return 0.90;
    if (mea == "AFC")  return 0.95;
    if (mea == "PAFC") return 0.85;
    if (mea == "MCFC") return 0.80;
    return 1.00;
}

bool isKnownMea(const std::string& m)
{
    return m == "PEM" || m == "SOFC" || m == "AFC" || m == "PAFC" || m == "MCFC";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.cell_count <= 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": cell_count must be ≥ 1 (got " +
              std::to_string(in.cell_count) + ")");
    } else if (in.cell_count > 600) {
        r.add("DFM-FC-COUNT", "info",
              std::string(kSkillId) + ": cell_count " +
              std::to_string(in.cell_count) +
              " > 600 — unusually large stack, verify cooling plan");
    }

    if (in.mea_type.empty()) {
        r.add("DFM-INPUT", "warning",
              std::string(kSkillId) + ": mea_type empty — defaulting to PEM");
    } else if (!isKnownMea(in.mea_type)) {
        r.add("DFM-FC-MEA", "info",
              std::string(kSkillId) + ": mea_type '" + in.mea_type +
              "' not in {PEM, SOFC, AFC, PAFC, MCFC}");
    }

    if (in.stack_voltage_v <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": stack_voltage_v must be > 0 (got " +
              std::to_string(in.stack_voltage_v) + ")");
    } else if (in.cell_count > 0) {
        const double expected = in.cell_count * perCellOcv(in.mea_type);
        if (expected > 0.0) {
            const double err = std::abs(in.stack_voltage_v - expected) / expected;
            if (err > 0.20) {
                r.add("DFM-FC-VOLT", "info",
                      std::string(kSkillId) + ": stack_voltage_v " +
                      std::to_string(in.stack_voltage_v) +
                      " differs from expected (cell_count × per-cell OCV ≈ " +
                      std::to_string(expected) + ") by > 20 %");
            }
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "fuel_cell_stack_assemble DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string mea = in.mea_type.empty() ? "PEM" : in.mea_type;
    const double expected = in.cell_count * perCellOcv(mea);

    json params = {
        { "cell_count",      in.cell_count },
        { "mea_type",        mea },
        { "stack_voltage_v", in.stack_voltage_v },
    };
    json pattern = {
        { "kind",                     kSkillId },
        { "cell_count",               in.cell_count },
        { "mea_type",                 mea },
        { "stack_voltage_v",          in.stack_voltage_v },
        { "expected_stack_voltage_v", expected },
        { "geometry_changed",         false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "stack_press";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "stainless_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ~15 s per cell to align + place; +60 s for end-plate / tie-rod torque.
    tooling.est_cycle_time_s  = 60.0 + 15.0 * in.cell_count;
    tooling.extra = {
        { "process",                 "fuel_cell_stack_assemble" },
        { "mea_type",                mea },
        { "cell_count",              in.cell_count },
        { "expected_stack_voltage_v", expected },
        { "process_category",        "fuel_cell_assembly" },
        { "geometric_no_op",         true },
        { "dfm_findings",            findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::fuel_cell_stack_assemble applied: N={} mea={} V={}",
        in.cell_count, mea, in.stack_voltage_v);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != std::string(kSkillId)) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::fuel_cell_stack_assemble
