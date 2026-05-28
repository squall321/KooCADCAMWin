// @lat: [[engine/skills#ultrasonic_test]]

#include "ultrasonic_test.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <string>
#include <vector>

namespace koocadcam::skill::ultrasonic_test {

using nlohmann::json;

namespace {

bool isAllowedDefectClass(const std::string& c)
{
    return c == "none" || c == "minor" || c == "major" || c == "critical";
}

// UT works well on wrought metals; cast or non-metallic materials are
// attenuative.  Emit info only.
bool isHighAttenuationMaterial(const std::string& m)
{
    return m.find("cast")     != std::string::npos ||
           m.find("polymer")  != std::string::npos ||
           m.find("composite")!= std::string::npos ||
           m.find("ceramic")  != std::string::npos;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.freq_mhz <= 0.0 || in.freq_mhz > 100.0) {
        r.add("DFM-INPUT", "error",
              "ultrasonic_test freq_mhz must be in (0, 100] (got " +
              std::to_string(in.freq_mhz) + ")");
    }
    if (in.probe_dia_mm <= 0.0 || in.probe_dia_mm > 100.0) {
        r.add("DFM-INPUT", "error",
              "ultrasonic_test probe_dia_mm must be in (0, 100] (got " +
              std::to_string(in.probe_dia_mm) + ")");
    }
    if (in.area_mm2 <= 0.0) {
        r.add("DFM-INPUT", "error",
              "ultrasonic_test area_mm2 must be > 0 (got " +
              std::to_string(in.area_mm2) + ")");
    }
    if (!isAllowedDefectClass(in.defect_class)) {
        r.add("DFM-INPUT", "error",
              "ultrasonic_test defect_class '" + in.defect_class +
              "' not in {none, minor, major, critical}");
    }

    auto faceId = wp.resolve(in.target_face_datum);
    if (!faceId) {
        r.add("DFM-UT-FACE", "error",
              "ultrasonic_test target_face_datum did not resolve");
    } else {
        const double area = wp.faceArea(*faceId);
        if (area <= 0.0) {
            r.add("DFM-UT-AREA", "error",
                  "ultrasonic_test: target face has non-positive area");
        } else if (in.area_mm2 > area * 1.05) {
            r.add("DFM-UT-AREA", "warning",
                  "ultrasonic_test scan area " + std::to_string(in.area_mm2) +
                  " mm^2 exceeds target face area " + std::to_string(area) +
                  " mm^2 (+5%)");
        }
    }

    if (isHighAttenuationMaterial(wp.material())) {
        r.add("DFM-UT-MATERIAL", "info",
              "ultrasonic_test on '" + wp.material() +
              "' — high acoustic attenuation may reduce sensitivity");
    }

    if (in.defect_class == "critical") {
        r.add("DFM-UT-DEFECT", "warning",
              "ultrasonic_test recorded a 'critical' defect indication — "
              "part likely scrap pending engineering review");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "ultrasonic_test DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto faceId = wp.resolve(in.target_face_datum);
    if (!faceId) throw SkillError("ultrasonic_test: face datum unresolved");

    const bool defectDetected = in.defect_class != "none";

    json params = {
        { "target_face_id", *faceId },
        { "freq_mhz",       in.freq_mhz },
        { "probe_dia_mm",   in.probe_dia_mm },
        { "area_mm2",       in.area_mm2 },
        { "defect_class",   in.defect_class },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_inspection_only",  true },
        { "geometric_change",    false },
        { "target_face_id",      *faceId },
        { "freq_mhz",            in.freq_mhz },
        { "probe_dia_mm",        in.probe_dia_mm },
        { "area_mm2",            in.area_mm2 },
        { "defect_class",        in.defect_class },
        { "defect_detected",     defectDetected },
        { "inspection_method",   "ultrasonic_pulse_echo" },
        { "ndt_category",        "UT" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "ut_transducer";
    tooling.tool_dia_mm       = in.probe_dia_mm;
    tooling.tool_length_mm    = 25.0;       // typical probe body height
    tooling.tool_material     = "piezo_ceramic";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ~0.5 s per mm² + 30 s setup, capped sensibly.
    tooling.est_cycle_time_s  = 30.0 + 0.5 * in.area_mm2;
    tooling.extra["dfm_findings"]         = findings;
    tooling.extra["machining_constraint"] =
        "inspection — ultrasonic pulse-echo, no material removal";
    tooling.extra["defect_detected"]      = defectDetected;
    tooling.extra["ndt_category"]         = "UT";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::ultrasonic_test applied: face={} f={}MHz dia={}mm A={}mm² class={}",
                  *faceId, in.freq_mhz, in.probe_dia_mm,
                  in.area_mm2, in.defect_class);

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

}  // namespace koocadcam::skill::ultrasonic_test
