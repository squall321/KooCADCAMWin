// @lat: [[engine/skills#automated_storage]]

#include "automated_storage.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::automated_storage {

using nlohmann::json;

namespace {

bool isKnownClass(const std::string& c)
{
    return c == "light" || c == "medium" || c == "heavy";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.bin_id.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": bin_id must be non-empty");
    }

    if (in.storage_height_m <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": storage_height_m must be > 0 (got " +
              std::to_string(in.storage_height_m) + ")");
    } else if (in.storage_height_m > 30.0) {
        r.add("DFM-ASRS-HEIGHT", "info",
              std::string(kSkillId) + ": storage_height_m " +
              std::to_string(in.storage_height_m) +
              " exceeds typical AS/RS rack height (30 m) — verify mast spec");
    }

    if (!isKnownClass(in.load_class)) {
        r.add("DFM-ASRS-CLASS", "info",
              std::string(kSkillId) + ": load_class '" + in.load_class +
              "' unrecognised — expected light | medium | heavy");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "automated_storage DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "bin_id",           in.bin_id },
        { "storage_height_m", in.storage_height_m },
        { "load_class",       in.load_class },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "bin_id",           in.bin_id },
        { "storage_height_m", in.storage_height_m },
        { "load_class",       in.load_class },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "asrs_shuttle";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Mast travel at 1 m/s + 4 s for fork put/pick.
    tooling.est_cycle_time_s  = 4.0 + in.storage_height_m;
    tooling.extra = {
        { "process",          "automated_storage" },
        { "bin_id",           in.bin_id },
        { "load_class",       in.load_class },
        { "process_category", "material_handling" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::automated_storage applied: bin={} height={}m class={}",
        in.bin_id, in.storage_height_m, in.load_class);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Metadata-only: leaves no geometric trace on the workpiece.

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

}  // namespace koocadcam::skill::automated_storage
