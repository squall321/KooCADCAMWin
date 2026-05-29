// @lat: [[engine/skills#rotor_balancing]]

#include "rotor_balancing.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

namespace koocadcam::skill::rotor_balancing {

using nlohmann::json;

// ── ISO 1940/1 balance-quality grades ────────────────────────────────────
//
// Each grade G_N is defined by the product ω · e_per ≤ N (mm/s).  Common
// aerospace / industrial reference set; finer grades (G0.4 / G1) cover
// gyroscopes and precision spindles, G2.5 covers gas-turbine rotors, G6.3
// covers general machinery, G16/G40 cover slower-speed shafts.
namespace {

constexpr std::array<const char*, 7> kIsoGrades = {
    "G0.4", "G1", "G2.5", "G6.3", "G16", "G40", "G100",
};

bool isKnownIsoGrade(const std::string& g)
{
    for (const auto* k : kIsoGrades) if (g == k) return true;
    return false;
}

// Return the numeric envelope for grade (mm/s); 0.0 if unknown.
double gradeEnvelopeMmPerS(const std::string& g)
{
    if (g == "G0.4")  return 0.4;
    if (g == "G1")    return 1.0;
    if (g == "G2.5")  return 2.5;
    if (g == "G6.3")  return 6.3;
    if (g == "G16")   return 16.0;
    if (g == "G40")   return 40.0;
    if (g == "G100")  return 100.0;
    return 0.0;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.mass_imbalance_g_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "rotor_balancing mass_imbalance_g_mm must be > 0");
    }
    if (in.rotation_speed_rpm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "rotor_balancing rotation_speed_rpm must be > 0");
    }
    if (in.rotor_mass_kg <= 0.0) {
        r.add("DFM-INPUT", "error",
              "rotor_balancing rotor_mass_kg must be > 0");
    }

    if (!isKnownIsoGrade(in.balance_grade_iso_1940)) {
        r.add("DFM-BAL-GRADE", "warning",
              "rotor_balancing balance_grade_iso_1940 '" +
              in.balance_grade_iso_1940 +
              "' not in ISO 1940/1 set {G0.4, G1, G2.5, G6.3, G16, G40, G100}");
    }

    if (in.correction_plane_count != 1 && in.correction_plane_count != 2) {
        r.add("DFM-BAL-PLANE", "warning",
              "rotor_balancing correction_plane_count " +
              std::to_string(in.correction_plane_count) +
              " not in {1 static, 2 dynamic}");
    }

    // Consistency check: imbalance × ω should be ≤ grade envelope × rotor mass.
    // U_per = mass_imbalance_g_mm / rotor_mass_kg / 1000.0  (g·mm / kg ⇒ um)
    // ω = 2π · rpm/60   (rad/s)
    // v = U_per × ω × 1e-3 (mm/s)
    if (in.rotor_mass_kg > 0.0 && in.rotation_speed_rpm > 0.0 &&
        in.mass_imbalance_g_mm > 0.0) {
        const double envelope = gradeEnvelopeMmPerS(in.balance_grade_iso_1940);
        if (envelope > 0.0) {
            const double omega = 2.0 * M_PI * in.rotation_speed_rpm / 60.0;
            const double v_mm_s =
                (in.mass_imbalance_g_mm / in.rotor_mass_kg / 1000.0) * omega * 1e-3;
            if (v_mm_s > envelope * 1.1) {
                r.add("DFM-BAL-GRADE-MISMATCH", "info",
                      "rotor_balancing residual v ≈ " + std::to_string(v_mm_s) +
                      " mm/s exceeds " + in.balance_grade_iso_1940 +
                      " envelope " + std::to_string(envelope) + " mm/s");
            }
        }
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "rotor_balancing DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double envelope = gradeEnvelopeMmPerS(in.balance_grade_iso_1940);

    json params = {
        { "mass_imbalance_g_mm",     in.mass_imbalance_g_mm },
        { "balance_grade_iso_1940",  in.balance_grade_iso_1940 },
        { "rotation_speed_rpm",      in.rotation_speed_rpm },
        { "correction_plane_count",  in.correction_plane_count },
        { "rotor_mass_kg",           in.rotor_mass_kg },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "mass_imbalance_g_mm",        in.mass_imbalance_g_mm },
        { "balance_grade_iso_1940",     in.balance_grade_iso_1940 },
        { "rotation_speed_rpm",         in.rotation_speed_rpm },
        { "correction_plane_count",     in.correction_plane_count },
        { "rotor_mass_kg",              in.rotor_mass_kg },
        { "grade_envelope_mm_s",        envelope },
        { "balance_type",
          (in.correction_plane_count == 1) ? "static" : "dynamic" },
        { "geometry_changed",           false },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "balancing_machine";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;       // QA record only
    // Typical balance cycle: setup 10 min + 2 runs (~ 30 s each) per plane.
    tooling.est_cycle_time_s  = 600.0 + in.correction_plane_count * 60.0;
    tooling.extra = {
        { "process",                  "dynamic_balancing" },
        { "balance_grade_iso_1940",   in.balance_grade_iso_1940 },
        { "mass_imbalance_g_mm",      in.mass_imbalance_g_mm },
        { "rotation_speed_rpm",       in.rotation_speed_rpm },
        { "correction_plane_count",   in.correction_plane_count },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::rotor_balancing applied: imbalance={} g·mm grade={} rpm={} planes={}",
                  in.mass_imbalance_g_mm, in.balance_grade_iso_1940,
                  in.rotation_speed_rpm, in.correction_plane_count);

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

}  // namespace koocadcam::skill::rotor_balancing
