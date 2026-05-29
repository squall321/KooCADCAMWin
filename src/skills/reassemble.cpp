// @lat: [[engine/skills#reassemble]]

#include "reassemble.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <unordered_set>

namespace koocadcam::skill::reassemble {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.reassembly_order.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": reassembly_order must be non-empty");
    }
    if (in.torque_specs_count < 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": torque_specs_count must be >= 0 (got " +
              std::to_string(in.torque_specs_count) + ")");
    }

    std::unordered_set<std::string> seen;
    for (const auto& id : in.reassembly_order) {
        if (!seen.insert(id).second) {
            r.add("DFM-RA-DUP", "info",
                  std::string(kSkillId) + ": duplicated id '" + id +
                  "' in reassembly_order — verify operator log");
            break;
        }
    }

    const int partCount = static_cast<int>(in.reassembly_order.size());
    if (partCount > 0 && in.torque_specs_count > partCount) {
        r.add("DFM-RA-TORQUE", "info",
              std::string(kSkillId) + ": torque_specs_count " +
              std::to_string(in.torque_specs_count) +
              " > part_count " + std::to_string(partCount) +
              " — more torqued fasteners than parts; verify spec");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "reassemble DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json order = json::array();
    for (const auto& id : in.reassembly_order) order.push_back(id);
    const int partCount = static_cast<int>(in.reassembly_order.size());

    json params = {
        { "reassembly_order",   order },
        { "torque_specs_count", in.torque_specs_count },
    };
    json pattern = {
        { "kind",               kSkillId },
        { "reassembly_order",   order },
        { "part_count",         partCount },
        { "torque_specs_count", in.torque_specs_count },
        { "geometry_changed",   false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "torque_wrench;hand_tools";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // 60 s base + 30 s per part — rough reassembly cycle estimate.
    tooling.est_cycle_time_s  = 60.0 + 30.0 * partCount;
    tooling.extra = {
        { "process",            "reassemble" },
        { "process_category",   "maintenance" },
        { "geometric_no_op",    true },
        { "part_count",         partCount },
        { "torque_specs_count", in.torque_specs_count },
        { "dfm_findings",       findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::reassemble applied: parts={} torque_specs={}",
        partCount, in.torque_specs_count);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Reassembly leaves no B-rep trace.  Metadata replay only.

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

}  // namespace koocadcam::skill::reassemble
