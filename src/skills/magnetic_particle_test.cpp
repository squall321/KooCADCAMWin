// @lat: [[engine/skills#magnetic_particle_test]]

#include "magnetic_particle_test.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <string>
#include <vector>

namespace koocadcam::skill::magnetic_particle_test {

using nlohmann::json;

namespace {

bool isFerromagnetic(const std::string& m)
{
    // Carbon + alloy + tool + 400-series stainless are ferromagnetic.
    // Aluminum, brass, copper, titanium, austenitic stainless (3xx) are NOT.
    if (m.find("carbon_steel")      != std::string::npos) return true;
    if (m.find("alloy_steel")       != std::string::npos) return true;
    if (m.find("tool_steel")        != std::string::npos) return true;
    if (m.find("cast_iron")         != std::string::npos) return true;
    if (m.find("ductile_iron")      != std::string::npos) return true;
    if (m.find("stainless_400")     != std::string::npos) return true;
    if (m.find("stainless_410")     != std::string::npos) return true;
    if (m.find("stainless_416")     != std::string::npos) return true;
    if (m.find("stainless_420")     != std::string::npos) return true;
    if (m.find("stainless_440")     != std::string::npos) return true;
    return false;
}

bool isAllowedTechnique(const std::string& t)
{
    return t == "wet" || t == "dry";
}

bool isAllowedSeverity(const std::string& s)
{
    return s == "none" || s == "linear" || s == "rounded" || s == "cluster";
}

bool isAllowedSurfacePrep(const std::string& p)
{
    return p == "as_machined" || p == "ground" || p == "polished";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!isFerromagnetic(wp.material())) {
        r.add("DFM-MT-MATERIAL", "error",
              "magnetic_particle_test requires a ferromagnetic material; "
              "'" + wp.material() + "' is non-ferromagnetic — use PT or ET instead");
    }
    if (!isAllowedTechnique(in.technique)) {
        r.add("DFM-INPUT", "error",
              "magnetic_particle_test technique '" + in.technique +
              "' not in {wet, dry}");
    }
    if (!isAllowedSeverity(in.severity_class)) {
        r.add("DFM-INPUT", "error",
              "magnetic_particle_test severity_class '" + in.severity_class +
              "' not in {none, linear, rounded, cluster}");
    }
    if (!isAllowedSurfacePrep(in.surface_prep)) {
        r.add("DFM-INPUT", "error",
              "magnetic_particle_test surface_prep '" + in.surface_prep +
              "' not in {as_machined, ground, polished}");
    }

    auto faceId = wp.resolve(in.target_face_datum);
    if (!faceId) {
        r.add("DFM-MT-FACE", "error",
              "magnetic_particle_test target_face_datum did not resolve");
    } else if (wp.faceArea(*faceId) <= 0.0) {
        r.add("DFM-MT-AREA", "error",
              "magnetic_particle_test: target face has non-positive area");
    }

    if (in.severity_class == "linear") {
        r.add("DFM-MT-SEVERITY", "warning",
              "magnetic_particle_test recorded a linear indication — "
              "linear flux-leakage patterns typically indicate cracks");
    } else if (in.severity_class == "cluster") {
        r.add("DFM-MT-SEVERITY", "warning",
              "magnetic_particle_test recorded a cluster indication — "
              "may indicate porosity / inclusion field");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "magnetic_particle_test DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto faceId = wp.resolve(in.target_face_datum);
    if (!faceId) throw SkillError("magnetic_particle_test: face datum unresolved");

    const bool defectDetected = in.severity_class != "none";

    json params = {
        { "target_face_id", *faceId },
        { "technique",      in.technique },
        { "severity_class", in.severity_class },
        { "surface_prep",   in.surface_prep },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_inspection_only",  true },
        { "geometric_change",    false },
        { "target_face_id",      *faceId },
        { "technique",           in.technique },
        { "severity_class",      in.severity_class },
        { "surface_prep",        in.surface_prep },
        { "defect_detected",     defectDetected },
        { "inspection_method",   "magnetic_particle_yoke" },
        { "ndt_category",        "MT" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "magnetic_yoke";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 200.0;       // typical AC yoke leg span
    tooling.tool_material     = "soft_iron_core";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Setup + magnetize + apply + dwell + inspect ~ 90 s
    tooling.est_cycle_time_s  = 90.0;
    tooling.extra["dfm_findings"]         = findings;
    tooling.extra["machining_constraint"] =
        "inspection — magnetic particle yoke, no material removal";
    tooling.extra["defect_detected"]      = defectDetected;
    tooling.extra["ndt_category"]         = "MT";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::magnetic_particle_test applied: face={} tech={} sev={} prep={}",
                  *faceId, in.technique, in.severity_class, in.surface_prep);

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

}  // namespace koocadcam::skill::magnetic_particle_test
