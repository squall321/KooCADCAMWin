#pragma once
// @lat: [[engine/skills#heat treatment — shared material tables]]
//
// **Shared internal helpers, not a public primitive.**
//
// Used ONLY by the five HEAT TREATMENT skills (anneal, harden, temper,
// normalize, quench).  All five operations are *geometric no-ops* — they
// change the material's microstructure (hardness, grain size, residual
// stress) without altering the workpiece shape.  The shared concerns are:
//
//   1. Material → process-temperature lookup tables.  Each material has a
//      published range for anneal, austenitize (harden), and normalize
//      temperatures.  Skills emit *info-level* DFM findings when the user
//      supplies a temperature outside the typical range — they DO NOT
//      reject the input (heat-treatment recipes are routinely tuned outside
//      the textbook nominal).
//
//   2. Quench-medium compatibility.  Some material / quench pairs are
//      well-known antipatterns (water-quenching aluminum cracks; oil-
//      quenching stainless degrades corrosion resistance).  Emit info.
//
//   3. Expected post-process hardness estimates.  Used to populate the
//      FeatureSignature so downstream consumers (process planners, QA)
//      know what hardness to verify after the heat-treat cycle.
//
// The lookup is intentionally small (7 commonly-used industrial materials).
// Unknown materials produce a "material_unknown" info finding and skip the
// range check — they DO NOT block the operation.  This is policy: a CAD/CAM
// tool should not refuse to plan an operation on an unrecognised material;
// it should warn and proceed.
//
// Hardness scales follow industry convention:
//   - HB  (Brinell)        — soft / annealed states (~ 50-300 HB).
//   - HRC (Rockwell C)     — hardened steels (~ 20-65 HRC).
//   - HRB (Rockwell B)     — non-ferrous + soft steels (~ 50-100 HRB).
//
// Anyone tempted to reach for this table from outside the heat-treatment
// family should instead propose a real material catalog (perhaps a JSON
// file backed by an XSD).  These helpers are NOT a stabilised contract.

#include "DFM.hpp"
#include "Skill.hpp"
#include "Workpiece.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace koocadcam::skill::heat_treatment_common {

// ── Material × heat-treatment data ───────────────────────────────────────

struct MaterialHTData
{
    std::string  material_id;

    // Anneal cycle (soft + relieve stress).
    double       anneal_temp_min_c = 0.0;
    double       anneal_temp_max_c = 0.0;
    double       anneal_hardness_hb = 0.0;     // typical post-anneal hardness

    // Austenitize (heat above critical temp before quench) — for harden.
    double       austenitize_temp_min_c = 0.0;
    double       austenitize_temp_max_c = 0.0;
    double       hardened_hrc_max       = 0.0; // peak hardness after quench
    double       martensite_pct_typical = 0.0; // typical martensite fraction

    // Normalize (heat + air cool — uniform grain, slightly higher than anneal).
    double       normalize_temp_min_c = 0.0;
    double       normalize_temp_max_c = 0.0;

    // Solution heat-treat (for precipitation-hardening alloys) — used by
    // stand-alone `quench`.  Zero for materials that don't solution-treat.
    double       solution_temp_min_c = 0.0;
    double       solution_temp_max_c = 0.0;

    // Quench medium compatibility — flag combinations that are known to
    // cause cracking / distortion / surface degradation.  Each entry is
    // ( medium, severity, message ).  Severity is always "info" for the
    // heat-treatment family — these are advisory, not blocking.
    std::vector<std::tuple<std::string, std::string, std::string>>
        quench_incompat;

    // "ferrous" | "non_ferrous" — affects whether HRC or HRB is the
    // natural hardness scale.
    std::string  family = "ferrous";

    // Free-form notes (e.g. precipitation-hardening sequence requirements).
    std::string  note;
};

// Central lookup table.  Sorted by material_id alphabetically for stability.
inline const std::vector<MaterialHTData>& materialTable()
{
    // Values sourced from ASM Handbook Vol. 4 + manufacturer datasheets;
    // ranges are conservative typical-practice envelopes, NOT min-max safe
    // operating limits — emit info-only findings when outside.
    static const std::vector<MaterialHTData> table = {
        // ── alloy_steel_4140 ─────────────────────────────────────────────
        {
            "alloy_steel_4140",
            /* anneal     */ 815.0, 870.0, 197.0,
            /* austenitize*/ 830.0, 870.0, /*HRCmax*/ 58.0, /*mart%*/ 90.0,
            /* normalize  */ 870.0, 900.0,
            /* solution   */   0.0,   0.0,
            /* incompat   */ {
                {"water", "info",
                 "alloy_steel_4140 + water quench → severe distortion risk; "
                 "oil quench preferred"},
            },
            "ferrous",
            "Chromoly steel; oil quench standard, water reserved for thin sections",
        },
        // ── aluminum_6061 ────────────────────────────────────────────────
        {
            "aluminum_6061",
            /* anneal     */ 410.0, 415.0, 30.0,
            /* austenitize*/   0.0,   0.0, /*HRCmax*/  0.0, /*mart%*/  0.0,
            /* normalize  */   0.0,   0.0,
            /* solution   */ 525.0, 540.0,
            /* incompat   */ {
                {"water", "info",
                 "aluminum_6061 water-quench OK from solution temp, but rapid "
                 "section-thickness changes can warp — verify fixturing"},
                {"oil",   "info",
                 "aluminum_6061 + oil quench: cooling rate too slow to retain "
                 "supersaturated solid solution — use water or polymer"},
            },
            "non_ferrous",
            "Precipitation-hardening (T6 = solution quench + age 175 °C / 8 h)",
        },
        // ── aluminum_7075 ────────────────────────────────────────────────
        {
            "aluminum_7075",
            /* anneal     */ 410.0, 415.0, 60.0,
            /* austenitize*/   0.0,   0.0, /*HRCmax*/  0.0, /*mart%*/  0.0,
            /* normalize  */   0.0,   0.0,
            /* solution   */ 465.0, 485.0,
            /* incompat   */ {
                {"water", "info",
                 "aluminum_7075 cold-water quench risks quench cracking on "
                 "thick sections — use 60-80 °C water or polymer for >25 mm"},
                {"oil",   "info",
                 "aluminum_7075 + oil quench: insufficient cooling rate for T6"},
            },
            "non_ferrous",
            "High-strength Al; T6 = solution + age 120 °C / 24 h or 175 °C / 8 h",
        },
        // ── brass_360 ────────────────────────────────────────────────────
        {
            "brass_360",
            /* anneal     */ 425.0, 600.0, 60.0,
            /* austenitize*/   0.0,   0.0, /*HRCmax*/  0.0, /*mart%*/  0.0,
            /* normalize  */   0.0,   0.0,
            /* solution   */   0.0,   0.0,
            /* incompat   */ {
                {"water", "info",
                 "brass_360 + water quench: acceptable for relieving stress, "
                 "but air cool is the production-standard practice"},
            },
            "non_ferrous",
            "Free-machining brass; cannot be hardened by heat treat (work-harden only)",
        },
        // ── carbon_steel_1045 ────────────────────────────────────────────
        {
            "carbon_steel_1045",
            /* anneal     */ 760.0, 840.0, 170.0,
            /* austenitize*/ 820.0, 870.0, /*HRCmax*/ 55.0, /*mart%*/ 90.0,
            /* normalize  */ 845.0, 900.0,
            /* solution   */   0.0,   0.0,
            /* incompat   */ {},
            "ferrous",
            "Medium-carbon steel; water or oil quench from austenitize temp",
        },
        // ── stainless_316 ────────────────────────────────────────────────
        {
            "stainless_316",
            /* anneal     */ 1010.0, 1120.0, 217.0,
            /* austenitize*/    0.0,    0.0, /*HRCmax*/  0.0, /*mart%*/  0.0,
            /* normalize  */    0.0,    0.0,
            /* solution   */ 1040.0, 1120.0,
            /* incompat   */ {
                {"oil", "info",
                 "stainless_316 + oil quench: residual oil film impairs "
                 "passive-layer regeneration — use water or air"},
            },
            "ferrous",
            "Austenitic SS; CANNOT be hardened by quench (no martensite); "
            "solution-anneal + water quench restores corrosion resistance",
        },
        // ── titanium_grade5 ──────────────────────────────────────────────
        {
            "titanium_grade5",
            /* anneal     */ 705.0, 790.0, 334.0,
            /* austenitize*/ 880.0, 950.0, /*HRCmax*/ 36.0, /*mart%*/ 60.0,
            /* normalize  */   0.0,   0.0,
            /* solution   */ 925.0, 970.0,
            /* incompat   */ {
                {"water", "info",
                 "titanium_grade5 cold-water quench risks distortion on "
                 "complex geometries — verify support fixturing"},
            },
            "non_ferrous",
            "Ti-6Al-4V; β-transus ≈ 995 °C — keep below to retain α-β duplex",
        },
    };
    return table;
}

// Lookup by material_id; returns nullptr if material is unknown.
inline const MaterialHTData* lookupMaterial(const std::string& id)
{
    const auto& tbl = materialTable();
    auto it = std::find_if(tbl.begin(), tbl.end(),
        [&](const MaterialHTData& m) { return m.material_id == id; });
    return (it == tbl.end()) ? nullptr : &(*it);
}

// ── DFM helpers ──────────────────────────────────────────────────────────

// Add an info-level finding when `material_id` is not in the table.  Returns
// true if material is known.  Callers should still apply() — unknown
// materials skip the range check but do not block.
inline bool gateMaterialKnown(DFMReport& r,
                              const std::string& skill_id,
                              const std::string& material_id)
{
    if (lookupMaterial(material_id) == nullptr) {
        r.add("DFM-HT-MATERIAL", "info",
              skill_id + ": material '" + material_id +
              "' not in heat-treatment lookup table — "
              "temperature/hardness checks skipped");
        return false;
    }
    return true;
}

// Add an info finding if `temp` falls outside [tmin, tmax].  Both endpoints
// are inclusive; if either bound is 0.0 the check is skipped (signals
// "operation not applicable to this material" — e.g. austenitize on
// aluminum).
inline void gateTempRange(DFMReport& r,
                          const std::string& skill_id,
                          const std::string& material_id,
                          const std::string& cycle_name,
                          double temp_c,
                          double tmin_c, double tmax_c)
{
    if (tmin_c == 0.0 && tmax_c == 0.0) {
        r.add("DFM-HT-NA", "info",
              skill_id + ": " + cycle_name + " temperature range unavailable "
              "for material '" + material_id + "' (operation may not apply)");
        return;
    }
    if (temp_c < tmin_c) {
        r.add("DFM-HT-TEMP-LOW", "info",
              skill_id + ": " + cycle_name + " temperature " +
              std::to_string(temp_c) + " °C below typical range [" +
              std::to_string(tmin_c) + ", " + std::to_string(tmax_c) +
              "] °C for material '" + material_id + "'");
    } else if (temp_c > tmax_c) {
        r.add("DFM-HT-TEMP-HIGH", "info",
              skill_id + ": " + cycle_name + " temperature " +
              std::to_string(temp_c) + " °C above typical range [" +
              std::to_string(tmin_c) + ", " + std::to_string(tmax_c) +
              "] °C for material '" + material_id + "' — grain growth risk");
    }
}

// Quench-medium / material compatibility check (info-only).
inline void gateQuenchCompat(DFMReport& r,
                             const std::string& skill_id,
                             const std::string& material_id,
                             const std::string& medium)
{
    const auto* m = lookupMaterial(material_id);
    if (!m) return;
    for (const auto& tup : m->quench_incompat) {
        const auto& [bad_medium, sev, msg] = tup;
        if (bad_medium == medium) {
            r.add("DFM-HT-QUENCH", sev, skill_id + ": " + msg);
        }
    }
}

// Hold-time sanity (per-skill loose bound: 1 min ≤ hold ≤ 24 h).
inline void gateHoldTime(DFMReport& r,
                         const std::string& skill_id,
                         double hold_min)
{
    if (hold_min <= 0.0) {
        r.add("DFM-INPUT", "error",
              skill_id + ": hold_time_min must be > 0 (got " +
              std::to_string(hold_min) + ")");
    } else if (hold_min < 1.0) {
        r.add("DFM-HT-HOLD-SHORT", "info",
              skill_id + ": hold_time_min " + std::to_string(hold_min) +
              " < 1 min — soak may be incomplete for thick sections");
    } else if (hold_min > 24.0 * 60.0) {
        r.add("DFM-HT-HOLD-LONG", "info",
              skill_id + ": hold_time_min " + std::to_string(hold_min) +
              " > 24 h — energy / grain-growth concerns");
    }
}

// ── Hardness-estimate helpers ────────────────────────────────────────────

// Annealed hardness (HB).  Returns 0.0 if unknown.
inline double annealedHB(const std::string& material_id)
{
    const auto* m = lookupMaterial(material_id);
    return m ? m->anneal_hardness_hb : 0.0;
}

// Hardened HRC + martensite percentage.
inline std::pair<double, double> hardenedHRC(const std::string& material_id)
{
    const auto* m = lookupMaterial(material_id);
    if (!m) return { 0.0, 0.0 };
    return { m->hardened_hrc_max, m->martensite_pct_typical };
}

// Post-temper hardness estimate.  Tempering reduces hardness monotonically
// with temperature: HRC_after ≈ HRC_max · (1 − (T − 150) / 700).  Clamp to
// [HRC_max · 0.30, HRC_max].
inline double temperedHRC(double hrc_max, double temper_temp_c)
{
    if (hrc_max <= 0.0) return 0.0;
    const double t = std::max(150.0, std::min(700.0, temper_temp_c));
    const double frac = 1.0 - (t - 150.0) / 700.0 * 0.7;
    const double hrc = hrc_max * frac;
    return std::max(hrc_max * 0.30, std::min(hrc_max, hrc));
}

// ── Signature & metadata-replay recognition helper ───────────────────────
//
// Every heat-treatment skill's `recognize()` follows the same pattern:
// scan `workpiece.features()` for entries whose `skill_id` matches and
// return them as RecognizedFeatures with confidence = 1.0.  Geometry
// CANNOT reveal heat-treatment history.

inline std::vector<RecognizedFeature> replayFromFeatures(
    const Workpiece& wp,
    const std::string& target_skill_id)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != target_skill_id) continue;
        RecognizedFeature rf;
        rf.skill_id         = target_skill_id;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

// Construct a tooling metadata block representative of a heat-treatment
// furnace.  Skill-specific extras supplied separately by each apply().
inline ToolingMeta buildFurnaceTooling(const std::string& process_label,
                                       double temp_c,
                                       double hold_min)
{
    ToolingMeta t;
    t.tool_type         = "furnace";
    t.tool_dia_mm       = 0.0;
    t.tool_length_mm    = 0.0;
    t.tool_material     = "refractory_lining";
    t.flute_count       = 0;
    t.cutting_speed_sfm = 0.0;
    t.feed_per_tooth_mm = 0.0;
    t.stock_removed_mm3 = 0.0;             // no material removal
    // Cycle time = ramp (~ 30 min) + hold + cool buffer (~ 30 min).
    t.est_cycle_time_s  = (30.0 + hold_min + 30.0) * 60.0;
    t.extra = {
        { "process",          process_label },
        { "temperature_c",    temp_c },
        { "hold_time_min",    hold_min },
        { "process_category", "heat_treatment" },
        { "geometric_no_op",  true },
    };
    return t;
}

// Append-and-stamp helper: clones `wp` (same shape + material), copies
// prior features through, and appends `sig`.  This is the no-op workpiece
// update used by all five heat-treatment apply()s.
inline std::shared_ptr<Workpiece> cloneWithSignature(
    const Workpiece& wp, const FeatureSignature& sig)
{
    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);
    return wpNew;
}

}  // namespace koocadcam::skill::heat_treatment_common
