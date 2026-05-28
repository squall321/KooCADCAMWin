// @lat: [[engine/skills#hardness_test]]

#include "hardness_test.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace koocadcam::skill::hardness_test {

using nlohmann::json;

namespace {

bool isAllowedScale(const std::string& s)
{
    return s == "HRC" || s == "HRB" || s == "HV" || s == "HB";
}

bool isAllowedIndenter(const std::string& i)
{
    return i == "diamond_brale"       || i == "diamond_pyramid" ||
           i == "tungsten_carbide_ball"|| i == "steel_ball";
}

// Conventional scale/indenter pairing per ASTM E18 / E92 / E10.
bool indenterMatchesScale(const std::string& scale, const std::string& indenter)
{
    if (scale == "HRC") return indenter == "diamond_brale";
    if (scale == "HRB") return indenter == "tungsten_carbide_ball" ||
                               indenter == "steel_ball";
    if (scale == "HV")  return indenter == "diamond_pyramid";
    if (scale == "HB")  return indenter == "tungsten_carbide_ball" ||
                               indenter == "steel_ball";
    return false;
}

// Typical load envelopes per scale.  Outside ⇒ info-only.
//   HRC : 150 kgf (major); minor 10 kgf included in std but reported as 150.
//   HRB : 100 kgf.
//   HV  : 0.001 - 100 kgf (microhardness to macro).
//   HB  : 500 - 3000 kgf.
std::pair<double, double> loadRangeFor(const std::string& scale)
{
    if (scale == "HRC") return { 150.0, 150.0 };
    if (scale == "HRB") return { 100.0, 100.0 };
    if (scale == "HV")  return {   0.001, 100.0 };
    if (scale == "HB")  return { 500.0, 3000.0 };
    return { 0.0, 0.0 };
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!isAllowedScale(in.scale)) {
        r.add("DFM-INPUT", "error",
              "hardness_test scale '" + in.scale +
              "' not in {HRC, HRB, HV, HB}");
    }
    if (!isAllowedIndenter(in.indenter_type)) {
        r.add("DFM-INPUT", "error",
              "hardness_test indenter_type '" + in.indenter_type +
              "' not in {diamond_brale, diamond_pyramid, "
              "tungsten_carbide_ball, steel_ball}");
    }
    if (in.load_kgf <= 0.0) {
        r.add("DFM-INPUT", "error",
              "hardness_test load_kgf must be > 0 (got " +
              std::to_string(in.load_kgf) + ")");
    }
    if (in.measured_hardness < 0.0) {
        r.add("DFM-INPUT", "error",
              "hardness_test measured_hardness cannot be negative (got " +
              std::to_string(in.measured_hardness) + ")");
    }

    auto faceId = wp.resolve(in.target_face_datum);
    if (!faceId) {
        r.add("DFM-HT-FACE", "error",
              "hardness_test target_face_datum did not resolve");
    } else if (wp.faceArea(*faceId) <= 0.0) {
        r.add("DFM-HT-AREA", "error",
              "hardness_test: target face has non-positive area");
    }

    // Scale ↔ indenter compatibility (info-level: lab practice can deviate).
    if (isAllowedScale(in.scale) && isAllowedIndenter(in.indenter_type) &&
        !indenterMatchesScale(in.scale, in.indenter_type)) {
        r.add("DFM-HARD-MATCH", "info",
              "hardness_test indenter '" + in.indenter_type +
              "' not standard for scale '" + in.scale +
              "' — verify ASTM E18/E92/E10 mapping");
    }

    // Load envelope for the chosen scale (info-level).
    if (isAllowedScale(in.scale) && in.load_kgf > 0.0) {
        const auto [lo, hi] = loadRangeFor(in.scale);
        if (in.load_kgf < lo || in.load_kgf > hi) {
            r.add("DFM-HARD-LOAD", "info",
                  "hardness_test load_kgf " + std::to_string(in.load_kgf) +
                  " outside typical range [" + std::to_string(lo) + ", " +
                  std::to_string(hi) + "] kgf for scale '" + in.scale + "'");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "hardness_test DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto faceId = wp.resolve(in.target_face_datum);
    if (!faceId) throw SkillError("hardness_test: face datum unresolved");

    const bool hasMeas = in.measured_hardness > 0.0;

    json params = {
        { "target_face_id",     *faceId },
        { "scale",              in.scale },
        { "indenter_type",      in.indenter_type },
        { "load_kgf",           in.load_kgf },
        { "measured_hardness",  in.measured_hardness },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_inspection_only",  true },
        { "geometric_change",    false },
        { "target_face_id",      *faceId },
        { "scale",               in.scale },
        { "indenter_type",       in.indenter_type },
        { "load_kgf",            in.load_kgf },
        { "measured_hardness",   in.measured_hardness },
        { "has_measurement",     hasMeas },
        { "measurement_mode",    "single_point_indentation" },
        { "inspection_method",   "indentation_hardness" },
        { "ndt_category",        "hardness" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "hardness_tester";
    tooling.tool_dia_mm       = (in.indenter_type.find("ball") != std::string::npos)
                                    ? 1.5875       // 1/16 in steel ball (HRB)
                                    : 0.2;         // diamond tip footprint
    tooling.tool_length_mm    = 30.0;
    tooling.tool_material     = (in.indenter_type.find("diamond") != std::string::npos)
                                    ? "diamond"
                                    : "tungsten_carbide";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ASTM E18 indentation cycle: preload + main load + dwell + read ≈ 15 s.
    tooling.est_cycle_time_s  = 15.0;
    tooling.extra["dfm_findings"]         = findings;
    tooling.extra["machining_constraint"] =
        "inspection — single-point indentation hardness, no material removal";
    tooling.extra["measurement_mode"]     = "single_point_indentation";
    tooling.extra["ndt_category"]         = "hardness";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::hardness_test applied: face={} scale={} ind={} L={}kgf H={}",
                  *faceId, in.scale, in.indenter_type,
                  in.load_kgf, in.measured_hardness);

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

}  // namespace koocadcam::skill::hardness_test
