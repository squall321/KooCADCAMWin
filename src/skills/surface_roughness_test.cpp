// @lat: [[engine/skills#surface_roughness_test]]

#include "surface_roughness_test.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <string>
#include <vector>

namespace koocadcam::skill::surface_roughness_test {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.target_ra_um <= 0.0) {
        r.add("DFM-INPUT", "error",
              "surface_roughness_test target_ra_um must be > 0 (got " +
              std::to_string(in.target_ra_um) + ")");
    }
    if (in.cutoff_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "surface_roughness_test cutoff_length_mm must be > 0 (got " +
              std::to_string(in.cutoff_length_mm) + ")");
    }
    if (in.measured_ra_um < 0.0) {
        r.add("DFM-INPUT", "error",
              "surface_roughness_test measured_ra_um cannot be negative (got " +
              std::to_string(in.measured_ra_um) + ")");
    }

    auto faceId = wp.resolve(in.target_face_datum);
    if (!faceId) {
        r.add("DFM-PROBE-FACE", "error",
              "surface_roughness_test target_face_datum did not resolve");
    } else {
        if (wp.faceArea(*faceId) <= 0.0) {
            r.add("DFM-PROBE-AREA", "error",
                  "surface_roughness_test: target face has non-positive area");
        }
    }

    if (in.target_ra_um > 0.0 && in.target_ra_um < 0.01) {
        r.add("DFM-RA-RANGE", "info",
              "surface_roughness_test target_ra_um " +
              std::to_string(in.target_ra_um) +
              " < 0.01 — sub-10nm Ra, requires optical interferometer");
    }
    if (in.target_ra_um > 50.0) {
        r.add("DFM-RA-RANGE", "info",
              "surface_roughness_test target_ra_um " +
              std::to_string(in.target_ra_um) +
              " > 50 — very rough; Ra rarely the right characteristic");
    }

    if (in.measured_ra_um > 0.0 && in.measured_ra_um > in.target_ra_um) {
        r.add("DFM-RA-OUTSPEC", "warning",
              "surface_roughness_test: measured Ra " +
              std::to_string(in.measured_ra_um) +
              " µm exceeds target " +
              std::to_string(in.target_ra_um) + " µm");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "surface_roughness_test DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto faceId = wp.resolve(in.target_face_datum);
    if (!faceId) throw SkillError("surface_roughness_test: face datum unresolved");

    const bool hasMeas  = in.measured_ra_um > 0.0;
    const bool conforms = hasMeas && in.measured_ra_um <= in.target_ra_um;

    json params = {
        { "target_face_id",   *faceId },
        { "target_ra_um",     in.target_ra_um },
        { "measured_ra_um",   in.measured_ra_um },
        { "cutoff_length_mm", in.cutoff_length_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_inspection_only",  true },
        { "geometric_change",    false },
        { "target_face_id",      *faceId },
        { "target_ra_um",        in.target_ra_um },
        { "measured_ra_um",      in.measured_ra_um },
        { "has_measurement",     hasMeas },
        { "conformance",         conforms },
        { "cutoff_length_mm",    in.cutoff_length_mm },
        { "iso_standard",        "ISO_4288" },
        { "inspection_method",   "stylus_profilometry" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "profilometer_stylus";
    tooling.tool_dia_mm       = 0.002;        // 2 µm diamond stylus tip
    tooling.tool_length_mm    = 15.0;
    tooling.tool_material     = "diamond";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Cycle: 5 × cutoff trace + setup ≈ 30 s.
    tooling.est_cycle_time_s  = 30.0 + 5.0 * in.cutoff_length_mm;
    tooling.extra["dfm_findings"]         = findings;
    tooling.extra["machining_constraint"] =
        "inspection — Ra / Rz stylus measurement, no material removal";
    tooling.extra["conformance"]          = conforms;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::surface_roughness_test applied: face={} tgt_Ra={} um meas_Ra={} um conform={}",
                  *faceId, in.target_ra_um, in.measured_ra_um, conforms);

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

}  // namespace koocadcam::skill::surface_roughness_test
