// @lat: [[engine/skills#radiographic_test]]

#include "radiographic_test.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <string>
#include <vector>

namespace koocadcam::skill::radiographic_test {

using nlohmann::json;

namespace {

bool isAllowedFilmClass(const std::string& c)
{
    // ISO 11699-1 classes: C1 (highest detail) … C6 (lowest detail / fastest).
    return c == "C1" || c == "C2" || c == "C3" ||
           c == "C4" || c == "C5" || c == "C6";
}

bool isHighDensityMaterial(const std::string& m)
{
    // Penetration is poor for thick dense metals at low kV.  This is an
    // approximate hint based on the alloy family.
    return m.find("steel")          != std::string::npos ||
           m.find("iron")           != std::string::npos ||
           m.find("inconel")        != std::string::npos ||
           m.find("nickel")         != std::string::npos ||
           m.find("tungsten")       != std::string::npos;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.source_kv <= 0.0) {
        r.add("DFM-INPUT", "error",
              "radiographic_test source_kv must be > 0 (got " +
              std::to_string(in.source_kv) + ")");
    } else if (in.source_kv < 40.0) {
        r.add("DFM-RT-KV-LOW", "info",
              "radiographic_test source_kv " + std::to_string(in.source_kv) +
              " < 40 kV — soft X-ray regime, suitable only for thin / light samples");
    } else if (in.source_kv > 500.0) {
        r.add("DFM-RT-KV-HIGH", "info",
              "radiographic_test source_kv " + std::to_string(in.source_kv) +
              " > 500 kV — high-energy regime; may need linac source");
    }

    if (in.exposure_min <= 0.0) {
        r.add("DFM-INPUT", "error",
              "radiographic_test exposure_min must be > 0 (got " +
              std::to_string(in.exposure_min) + ")");
    } else if (in.exposure_min > 60.0) {
        r.add("DFM-RT-EXP-LONG", "info",
              "radiographic_test exposure_min " +
              std::to_string(in.exposure_min) +
              " > 60 — very long exposure; consider higher kV or faster film");
    }

    if (!isAllowedFilmClass(in.film_class)) {
        r.add("DFM-INPUT", "error",
              "radiographic_test film_class '" + in.film_class +
              "' not in ISO 11699-1 {C1..C6}");
    }

    auto faceId = wp.resolve(in.target_face_datum);
    if (!faceId) {
        r.add("DFM-RT-FACE", "error",
              "radiographic_test target_face_datum did not resolve");
    } else if (wp.faceArea(*faceId) <= 0.0) {
        r.add("DFM-RT-AREA", "error",
              "radiographic_test: target face has non-positive area");
    }

    // Cross-check kV vs. material: thick steel under-penetrates at low kV.
    if (isHighDensityMaterial(wp.material()) && in.source_kv > 0.0 && in.source_kv < 150.0) {
        r.add("DFM-RT-PEN", "info",
              "radiographic_test source_kv " + std::to_string(in.source_kv) +
              " kV on dense material '" + wp.material() +
              "' — marginal penetration; consider ≥ 200 kV");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "radiographic_test DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto faceId = wp.resolve(in.target_face_datum);
    if (!faceId) throw SkillError("radiographic_test: face datum unresolved");

    json params = {
        { "target_face_id", *faceId },
        { "source_kv",      in.source_kv },
        { "exposure_min",   in.exposure_min },
        { "film_class",     in.film_class },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_inspection_only",  true },
        { "geometric_change",    false },
        { "target_face_id",      *faceId },
        { "source_kv",           in.source_kv },
        { "exposure_min",        in.exposure_min },
        { "film_class",          in.film_class },
        { "inspection_method",   "x_ray_film_radiography" },
        { "iso_standard",        "ISO_11699-1" },
        { "ndt_category",        "RT" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "x_ray_tube";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "tungsten_target";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Setup + warmup + exposure + processing ≈ exposure + 600 s overhead.
    tooling.est_cycle_time_s  = (in.exposure_min * 60.0) + 600.0;
    tooling.extra["dfm_findings"]         = findings;
    tooling.extra["machining_constraint"] =
        "inspection — X-ray radiography, no material removal";
    tooling.extra["radiation_safety"]     = "controlled_area_required";
    tooling.extra["ndt_category"]         = "RT";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::radiographic_test applied: face={} kV={} exp={}min film={}",
                  *faceId, in.source_kv, in.exposure_min, in.film_class);

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

}  // namespace koocadcam::skill::radiographic_test
