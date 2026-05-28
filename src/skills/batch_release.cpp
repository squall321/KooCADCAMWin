// @lat: [[engine/skills#batch_release]]

#include "batch_release.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace koocadcam::skill::batch_release {

using nlohmann::json;

namespace {

bool isReleaseClass(const std::string& s)
{
    return s == "A" || s == "B" || s == "C" || s == "scrap";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.lot_size <= 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": lot_size must be > 0 (got " +
              std::to_string(in.lot_size) + ")");
    }
    if (in.accept_count < 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": accept_count must be >= 0 (got " +
              std::to_string(in.accept_count) + ")");
    }
    if (in.lot_size > 0 && in.accept_count > in.lot_size) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": accept_count (" +
              std::to_string(in.accept_count) +
              ") cannot exceed lot_size (" +
              std::to_string(in.lot_size) + ")");
    }
    if (in.lot_id.empty()) {
        r.add("DFM-BR-LOTID", "error",
              std::string(kSkillId) +
              ": lot_id must not be empty (traceability requirement)");
    }
    if (!isReleaseClass(in.release_class)) {
        r.add("DFM-BR-CLASS", "error",
              std::string(kSkillId) + ": release_class '" + in.release_class +
              "' invalid (expected A | B | C | scrap)");
    }

    // Info-level: yield checks
    if (in.lot_size > 0) {
        const double yield_ = static_cast<double>(in.accept_count) /
                              static_cast<double>(in.lot_size);
        if (yield_ < 0.95) {
            r.add("DFM-BR-YIELD-LOW", "info",
                  std::string(kSkillId) + ": yield " + std::to_string(yield_) +
                  " < 0.95 — investigate process drift");
        }
        if (yield_ < 0.95 && in.release_class == "A") {
            r.add("DFM-BR-CLASS-MISMATCH", "info",
                  std::string(kSkillId) +
                  ": release_class 'A' with yield < 0.95 — suspicious, "
                  "class A typically requires high yield");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "batch_release DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double yield_ = static_cast<double>(in.accept_count) /
                          static_cast<double>(in.lot_size);

    json params = {
        { "lot_id",        in.lot_id },
        { "lot_size",      in.lot_size },
        { "accept_count",  in.accept_count },
        { "release_class", in.release_class },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_inspection_only",   true },
        { "geometric_change",     false },
        { "lot_id",               in.lot_id },
        { "lot_size",             in.lot_size },
        { "accept_count",         in.accept_count },
        { "release_class",        in.release_class },
        { "yield",                yield_ },
        { "inspection_method",    "lot_release_disposition" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "release_record";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Release is administrative — ~5 minutes fixed.
    tooling.est_cycle_time_s  = 300.0;
    tooling.extra["inspection_type"] = "batch_release";
    tooling.extra["yield"]           = yield_;
    tooling.extra["release_class"]   = in.release_class;
    tooling.extra["dfm_findings"]    = findings;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::batch_release applied: lot={} size={} accept={} class={}",
        in.lot_id, in.lot_size, in.accept_count, in.release_class);

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

}  // namespace koocadcam::skill::batch_release
