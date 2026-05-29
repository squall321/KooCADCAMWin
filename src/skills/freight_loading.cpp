// @lat: [[engine/skills#freight_loading]]

#include "freight_loading.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace koocadcam::skill::freight_loading {

using nlohmann::json;

namespace {

// Approximate gross payload capacity (kg) by container type.
// Values are conservative industry norms for max-pay weight.
double containerPayloadKg(const std::string& t)
{
    if (t == "20ft")        return 28200.0;   // 20' standard dry
    if (t == "40ft")        return 28800.0;   // 40' standard dry
    if (t == "40HC")        return 28560.0;   // 40' high-cube
    if (t == "53ft_truck")  return 20000.0;   // US dry van trailer
    if (t == "LTL_pallet")  return 1500.0;    // single LTL pallet slot
    return -1.0;                              // unknown
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.freight_class.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": freight_class must not be empty");
    }
    if (in.weight_kg <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": weight_kg must be > 0 (got " +
              std::to_string(in.weight_kg) + ")");
    }
    if (in.count <= 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": count must be > 0 (got " +
              std::to_string(in.count) + ")");
    }

    const double payload = containerPayloadKg(in.container_type);
    if (payload < 0.0) {
        r.add("DFM-FL-CONTAINER", "error",
              std::string(kSkillId) + ": container_type '" + in.container_type +
              "' unrecognized — expected 20ft|40ft|40HC|53ft_truck|LTL_pallet");
    } else {
        if (in.weight_kg > payload) {
            r.add("DFM-FL-OVERLOAD", "error",
                  std::string(kSkillId) + ": weight_kg " +
                  std::to_string(in.weight_kg) +
                  " exceeds " + in.container_type + " payload " +
                  std::to_string(payload) + " kg");
        } else if (in.weight_kg > 0.0 && in.weight_kg < 0.30 * payload) {
            r.add("DFM-FL-UNDERLOAD", "info",
                  std::string(kSkillId) + ": weight_kg " +
                  std::to_string(in.weight_kg) + " is < 30% of " +
                  in.container_type + " payload (" +
                  std::to_string(payload) + " kg) — poor utilization");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "freight_loading DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double payload  = containerPayloadKg(in.container_type);
    const double util_pct = (payload > 0.0)
                            ? (100.0 * in.weight_kg / payload)
                            : 0.0;

    json params = {
        { "freight_class",  in.freight_class },
        { "weight_kg",      in.weight_kg },
        { "count",          in.count },
        { "container_type", in.container_type },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_inspection_only",    true },
        { "geometric_change",      false },
        { "freight_class",         in.freight_class },
        { "weight_kg",             in.weight_kg },
        { "count",                 in.count },
        { "container_type",        in.container_type },
        { "container_payload_kg",  payload },
        { "utilization_pct",       util_pct },
        { "avg_unit_weight_kg",
          (in.count > 0) ? (in.weight_kg / static_cast<double>(in.count)) : 0.0 },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "loading_record";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Forklift loading ~ 90 s per pallet-equivalent.
    tooling.est_cycle_time_s  = 90.0 * static_cast<double>(in.count);
    tooling.extra["inspection_type"] = "freight_loading";
    tooling.extra["container_type"]  = in.container_type;
    tooling.extra["utilization_pct"] = util_pct;
    tooling.extra["dfm_findings"]    = findings;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::freight_loading applied: class={} weight={}kg count={} container={}",
        in.freight_class, in.weight_kg, in.count, in.container_type);

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

}  // namespace koocadcam::skill::freight_loading
