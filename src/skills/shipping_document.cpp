// @lat: [[engine/skills#shipping_document]]

#include "shipping_document.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace koocadcam::skill::shipping_document {

using nlohmann::json;

namespace {

bool isKnownDocType(const std::string& d)
{
    return d == "BOL" || d == "CMR" || d == "AWB" ||
           d == "SWB" || d == "INV";
}

// Free-flow / generic carrier label per doc_type — used as the
// nominal recordkeeping tag for downstream brokers.
const char* carrierLabelFor(const std::string& d)
{
    if (d == "BOL") return "road_rail_carrier";
    if (d == "CMR") return "intl_road_carrier";
    if (d == "AWB") return "air_carrier";
    if (d == "SWB") return "sea_carrier";
    if (d == "INV") return "commercial_invoice";
    return "unknown_carrier";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.doc_type.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": doc_type must not be empty");
    } else if (!isKnownDocType(in.doc_type)) {
        r.add("DFM-SD-DOCTYPE", "error",
              std::string(kSkillId) + ": doc_type '" + in.doc_type +
              "' invalid — expected BOL|CMR|AWB|SWB|INV");
    }

    if (in.recipient.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": recipient must not be empty");
    }

    if (in.weight_kg <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": weight_kg must be > 0 (got " +
              std::to_string(in.weight_kg) + ")");
    } else {
        if (in.weight_kg > 30000.0) {
            r.add("DFM-SD-WEIGHT-HIGH", "info",
                  std::string(kSkillId) + ": weight_kg " +
                  std::to_string(in.weight_kg) +
                  " > 30000 — heavy-lift, requires special handling");
        }
        if (in.doc_type == "AWB" && in.weight_kg > 7000.0) {
            r.add("DFM-SD-WEIGHT-AIR", "info",
                  std::string(kSkillId) +
                  ": AWB weight_kg " + std::to_string(in.weight_kg) +
                  " > 7000 — exceeds typical ULD payload");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "shipping_document DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string carrier = carrierLabelFor(in.doc_type);

    json params = {
        { "doc_type",  in.doc_type },
        { "recipient", in.recipient },
        { "weight_kg", in.weight_kg },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_inspection_only",   true },
        { "geometric_change",     false },
        { "doc_type",             in.doc_type },
        { "recipient",            in.recipient },
        { "weight_kg",            in.weight_kg },
        { "carrier_label",        carrier },
        { "mode",                 (in.doc_type == "AWB") ? "air"
                                  : (in.doc_type == "SWB") ? "sea"
                                  : (in.doc_type == "CMR") ? "intl_road"
                                  : (in.doc_type == "BOL") ? "road_rail"
                                                            : "invoice" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "shipping_record";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Document issue: ~ 10 minutes for full set.
    tooling.est_cycle_time_s  = 600.0;
    tooling.extra["inspection_type"] = "shipping_document";
    tooling.extra["doc_type"]        = in.doc_type;
    tooling.extra["carrier_label"]   = carrier;
    tooling.extra["dfm_findings"]    = findings;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::shipping_document applied: doc={} recipient={} weight={}kg",
        in.doc_type, in.recipient, in.weight_kg);

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

}  // namespace koocadcam::skill::shipping_document
