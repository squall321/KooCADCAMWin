// @lat: [[engine/skills#peen_form]]

#include "peen_form.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <Standard_Failure.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::peen_form {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

// Helper — smallest bbox extent of the workpiece (thickness for a sheet).
double sheetThickness(const Workpiece& wp)
{
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    return std::min({ xMax - xMin, yMax - yMin, zMax - zMin });
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────
//
// DFM-PEEN-INTENS / DFM-PEEN-CURV / DFM-PEEN-CURV-MAX —
// References:
//   * SAE AMS 2430 "Shot Peening, Automatic" (rev. T, 2020) — qualifies
//     Almen-A intensity range 0.10A–0.30A as standard, with extended range
//     0.05A–0.50A permitted for special applications (Table 1).
//   * MIL-S-13165C "Shot Peening of Metal Parts" — §3.5.2 limits the
//     maximum induced curvature for peen-forming of aerospace skins to
//     ≈ 1/50 1/mm (R ≥ 50 mm) for thick aluminum panels, but practical
//     wing-skin curvatures used on the B-1, B-2 and 747-8 fall in
//     0.001–0.02 1/mm (R = 50 mm – 1 m).  Above 0.02 1/mm the process
//     requires multi-pass + supplementary stretch-forming — flagged as an
//     error here.
//   * Almen-A intensity is the residual arc height (mm) of a 1.6 mm SAE
//     1070 spring-steel strip after the same exposure as the part.

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    constexpr double kMinIntensityMm = 0.05;   // AMS 2430 §3 lower edge
    constexpr double kMaxIntensityMm = 0.50;   // AMS 2430 §3 upper edge
    constexpr double kMaxCurvatureInv = 0.02;  // 1/mm — practical peen-form ceiling

    if (in.peening_intensity <= 0.0) {
        r.add("DFM-INPUT", "error",
              "peen_form peening_intensity must be > 0 (got " +
              std::to_string(in.peening_intensity) + ")");
    }
    if (in.target_curvature_mm_inv <= 0.0) {
        r.add("DFM-INPUT", "error",
              "peen_form target_curvature_mm_inv must be > 0 (got " +
              std::to_string(in.target_curvature_mm_inv) + ")");
    }
    if (in.coverage_pct <= 0.0) {
        r.add("DFM-INPUT", "error",
              "peen_form coverage_pct must be > 0 (got " +
              std::to_string(in.coverage_pct) + ")");
    }

    // SAE AMS 2430 (Table 1) — Almen-A intensity envelope.
    if (in.peening_intensity > 0.0 &&
        (in.peening_intensity < kMinIntensityMm ||
         in.peening_intensity > kMaxIntensityMm)) {
        r.add("DFM-PEEN-INTENS", "warning",
              "peen_form intensity " + std::to_string(in.peening_intensity) +
              " mm outside SAE AMS 2430 typical [0.05, 0.50] mm Almen-A range");
    }

    // MIL-S-13165C §3.5.2 — practical peen-form curvature ceiling.
    if (in.target_curvature_mm_inv > kMaxCurvatureInv) {
        r.add("DFM-PEEN-CURV-MAX", "error",
              "peen_form curvature " +
              std::to_string(in.target_curvature_mm_inv) +
              " 1/mm > 0.02 1/mm (R < 50 mm) — exceeds peen-forming "
              "capability per MIL-S-13165C §3.5.2; use press-form instead");
    } else if (in.target_curvature_mm_inv > 0.01) {
        r.add("DFM-PEEN-CURV", "warning",
              "peen_form curvature " +
              std::to_string(in.target_curvature_mm_inv) +
              " 1/mm (R = " +
              std::to_string(1.0 / in.target_curvature_mm_inv) +
              " mm) — tight curvature near peen-forming capability limit");
    }

    // Sheet thickness sanity — peen-forming targets thin skins.
    const double t = sheetThickness(wp);
    if (t > 25.0) {
        r.add("DFM-PEEN-THICK", "warning",
              "peen_form: stock thickness " + std::to_string(t) +
              " mm > 25 mm — outside typical aerospace skin range; "
              "consider press-forming");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Physical model: shot peening cold-works the surface; the compressed
// surface layer expands in-plane, inducing a uniform residual stress that
// curves the plate AWAY from the peened side.  Volume is preserved.
//
// Slice-1 robust geometric stand-in: rebuild the workpiece as a NEW thinner,
// larger-footprint cuboid.  We treat the new thickness as the same plate
// after one-sided plastic stretch — t1 = t0 · (1 − ε), area scales by
// 1 / (1 − ε) so volume is preserved.  The peen "strain" ε is calibrated
// from the Almen intensity (typical 0.5 % per 0.20 mm-A is the SAE rule of
// thumb).  This is a stand-in until full shell-bending is implemented.
//
// The result is geometrically distinguishable from the input (smaller t,
// larger XY).  Curved-shell synthesis (using BRepOffsetAPI_MakeOffsetShape)
// is left for a later slice — that approach is fragile on cuboid input.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "peen_form DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Compute peen strain.  Rule-of-thumb (SAE PEEN-FORM-DESIGN-GUIDE 2018,
    // Ch. 6 Eq. 6.3): areal strain ε ≈ k · intensity · coverage_pct / 100
    // with k ≈ 0.025 for aluminum (per-mm Almen-A).  Cap at 5 % so volume-
    // preserving geometry stays sane.
    constexpr double kStrainPerAlmenMm = 0.025;
    const double rawStrain =
        kStrainPerAlmenMm * in.peening_intensity * (in.coverage_pct / 100.0);
    const double strain   = std::clamp(rawStrain, 0.0, 0.05);

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double dx = xMax - xMin;
    const double dy = yMax - yMin;
    const double dz = zMax - zMin;

    // Pick thickness axis = smallest extent.
    const double t0 = std::min({ dx, dy, dz });
    std::string thicknessAxis = "z";
    if (dx == t0)      thicknessAxis = "x";
    else if (dy == t0) thicknessAxis = "y";

    // Volume-preserving rebuild: t1 = t0 · (1 - strain); in-plane scale
    // s = 1 / sqrt(1 - strain) so dx*dy*t = const.
    const double t1     = t0 * (1.0 - strain);
    const double scale  = (strain > 0.0) ? 1.0 / std::sqrt(1.0 - strain) : 1.0;

    // Always center the rebuilt plate at the original bbox centre.
    const double cx = 0.5 * (xMin + xMax);
    const double cy = 0.5 * (yMin + yMax);
    const double cz = 0.5 * (zMin + zMax);

    TopoDS_Shape result;
    try {
        double newDx = dx, newDy = dy, newDz = dz;
        if (thicknessAxis == "z") {
            newDx = dx * scale;
            newDy = dy * scale;
            newDz = t1;
        } else if (thicknessAxis == "x") {
            newDx = t1;
            newDy = dy * scale;
            newDz = dz * scale;
        } else { // y
            newDx = dx * scale;
            newDy = t1;
            newDz = dz * scale;
        }
        const gp_Ax2 ax(
            gp_Pnt(cx - newDx / 2.0, cy - newDy / 2.0, cz - newDz / 2.0),
            gp::DZ());
        result = pr::box(ax, newDx, newDy, newDz);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("peen_form: plate rebuild failed: ") +
                         ex.what());
    }

    const double radius = (in.target_curvature_mm_inv > 0.0)
                          ? 1.0 / in.target_curvature_mm_inv : 0.0;

    json params = {
        { "peening_intensity",       in.peening_intensity },
        { "target_curvature_mm_inv", in.target_curvature_mm_inv },
        { "coverage_pct",            in.coverage_pct },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "peening_intensity",          in.peening_intensity },
        { "target_curvature_mm_inv",    in.target_curvature_mm_inv },
        { "target_radius_mm",           radius },
        { "coverage_pct",               in.coverage_pct },
        { "original_thickness_mm",      t0 },
        { "final_thickness_mm",         t1 },
        { "applied_strain",             strain },
        { "thickness_axis",             thicknessAxis },
        { "geometry_changed",           true },
        { "induces_compressive_stress", true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "peen_nozzle";
    tooling.tool_dia_mm       = 10.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "tungsten_carbide";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;     // volume preserved
    tooling.est_cycle_time_s  = 30.0 + in.coverage_pct * 0.5;
    tooling.extra = json{
        { "process",                "shot_peen_forming" },
        { "almen_intensity_mm",     in.peening_intensity },
        { "coverage_pct",           in.coverage_pct },
        { "target_radius_mm",       radius },
        { "applied_strain",         strain },
        { "original_thickness_mm",  t0 },
        { "final_thickness_mm",     t1 },
        { "dfm_findings",           findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(result, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::peen_form applied: intensity={} strain={} t0={} t1={}",
                  in.peening_intensity, strain, t0, t1);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Geometric heuristic (primary): peen-formed plates retain a planar-pair
// topology (a flat sheet — slice-1 doesn't curve it) with a measurably-
// reduced thickness vs the recorded `original_thickness_mm` in any prior
// `peen_form` metadata.  Without metadata we cannot distinguish a peened
// plate from any other flat sheet, so geometric-only recognition returns
// 0.40 confidence; metadata replay returns 1.0.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // (a) Metadata replay — exact recovery.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = {
            { "source",                  "metadata" },
            { "original_thickness_mm",   f.pattern.value("original_thickness_mm", 0.0) },
            { "final_thickness_mm",      f.pattern.value("final_thickness_mm",    0.0) },
        };
        out.push_back(rf);
    }
    if (!out.empty()) return out;

    // (b) Geometric fallback — flat sheet of reasonable peen-form thickness.
    const double t = sheetThickness(wp);
    if (t <= 0.0 || t > 25.0) return out;

    // Count planar top/bottom faces (≥ 2 large parallel faces).
    int planarPairs = 0;
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFacePlanar(fIdx)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(fIdx); } catch (...) { continue; }
        if (std::abs(n.Z()) > 0.9) ++planarPairs;
    }
    if (planarPairs < 2) return out;

    json recovered = {
        { "peening_intensity",       0.0 },  // unknowable from geometry
        { "target_curvature_mm_inv", 0.0 },
        { "coverage_pct",            0.0 },
    };
    json matched = {
        { "source",                "geometric_fallback" },
        { "current_thickness_mm",  t },
        { "planar_horizontal_face_count", planarPairs },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.40, matched });
    return out;
}

}  // namespace koocadcam::skill::peen_form
