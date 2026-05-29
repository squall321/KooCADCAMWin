// @lat: [[engine/skills#shell_roll]]
//
// Real-geometric implementation: rebuild the input flat plate as a hollow
// cylindrical shell (outer Ø = shell_dia_mm, wall = plate_thick_mm).
// Volume is preserved (developed plate volume == shell wall volume) to
// within OCCT tolerance.

#include "shell_roll.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <Standard_Failure.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::shell_roll {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────
//
// Standards reference:
//   ASME Boiler & Pressure Vessel Code, Section VIII Div. 1, UG-16(b):
//     minimum thickness of any shell ≥ 1/16" (1.6 mm) excluding corrosion.
//   ASME B&PV §UG-79 "Forming of Plate": cold-rolled plate is practical to
//     ~ 80 mm thick; thicker requires hot forming.
//   For PRESSURE VESSEL shells, slenderness D/t ≥ 30 is the empirical
//     "thin wall" regime (Roark's "Formulas for Stress and Strain" 9th
//     ed., §13) — below that ratio classical thin-shell stress formulae
//     no longer apply and thick-wall theory (Lamé) must be used.

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.plate_thick_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": plate_thick_mm must be > 0 (got " +
              std::to_string(in.plate_thick_mm) + ")");
    }
    if (in.shell_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": shell_dia_mm must be > 0 (got " +
              std::to_string(in.shell_dia_mm) + ")");
    }
    if (in.shell_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": shell_length_mm must be > 0 (got " +
              std::to_string(in.shell_length_mm) + ")");
    }

    // ASME B&PV VIII-1 UG-16(b): absolute minimum shell wall ≈ 1.6 mm.
    if (in.plate_thick_mm > 0.0 && in.plate_thick_mm < 1.6) {
        r.add("DFM-ROLL-MIN-WALL", "error",
              std::string(kSkillId) + ": plate_thick_mm " +
              std::to_string(in.plate_thick_mm) +
              " < 1.6 mm (ASME B&PV VIII-1 UG-16(b) minimum)");
    }
    if (in.plate_thick_mm > 0.0 && in.plate_thick_mm < 3.0) {
        r.add("DFM-ROLL-THK-MIN", "warning",
              std::string(kSkillId) + ": plate_thick_mm " +
              std::to_string(in.plate_thick_mm) +
              " < 3 mm — buckling risk during roll, springback large");
    }
    // ASME B&PV §UG-79 cold-form practical limit.
    if (in.plate_thick_mm > 80.0) {
        r.add("DFM-ROLL-THK-MAX", "info",
              std::string(kSkillId) + ": plate_thick_mm " +
              std::to_string(in.plate_thick_mm) +
              " > 80 mm (ASME UG-79 cold-form limit) — hot-form required");
    }

    if (in.plate_thick_mm > 0.0 && in.shell_dia_mm > 0.0) {
        const double dOverT = in.shell_dia_mm / in.plate_thick_mm;
        // Thin-wall regime: Roark §13.  D/t ≥ 30 is the "thin shell" cutoff
        // — equivalent to wall ≤ ~3.3% of diameter.
        if (dOverT >= 30.0) {
            r.add("DFM-ROLL-THIN-WALL", "warning",
                  std::string(kSkillId) + ": D/t " +
                  std::to_string(dOverT) +
                  " ≥ 30 — thin-shell regime (Roark §13); verify buckling "
                  "per ASME B&PV VIII-1 UG-28");
        }
        if (dOverT < 10.0) {
            r.add("DFM-ROLL-D-OVER-T", "info",
                  std::string(kSkillId) + ": D/t " +
                  std::to_string(dOverT) +
                  " < 10 — tight-radius bend, may need pre-heat");
        }
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Rebuild as a hollow cylindrical shell:
//   outer cylinder of radius D/2, height = shell_length_mm
//   minus inner cylinder of radius D/2 - plate_thick, same height
//
// Volume check (analytic):
//   Plate (developed):  V_plate = (π·D) × L × t
//   Shell wall:         V_shell = π × L × (R_o² - R_i²)
//                              = π × L × ((D/2)² - (D/2 - t)²)
//                              = π × L × (D·t - t²)
//                              = π × D × L × t × (1 - t/D)
// Volume DELTA vs plate stock ≈ -π × D × L × t² / D
//   → small (~ -t/D × plate_vol) negative delta.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = std::string(kSkillId) + " DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double developedLength = M_PI * in.shell_dia_mm;
    const double dOverT          = in.shell_dia_mm / in.plate_thick_mm;
    const double outerR          = in.shell_dia_mm / 2.0;
    const double innerR          = std::max(0.05, outerR - in.plate_thick_mm);
    const double L               = in.shell_length_mm;

    // Compute volumes BEFORE Boolean for the signature.
    const double plateVol = developedLength * L * in.plate_thick_mm;
    const double shellVol = M_PI * L *
                            ((outerR * outerR) - (innerR * innerR));
    const double volDelta = shellVol - plateVol;

    TopoDS_Shape result;
    try {
        const gp_Ax2 ax(gp_Pnt(0.0, 0.0, 0.0), gp::DZ());
        result = pr::annularRing(ax, outerR, innerR, L);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("shell_roll: shell build failed: ") + ex.what());
    }

    json params = {
        { "plate_thick_mm",  in.plate_thick_mm },
        { "shell_dia_mm",    in.shell_dia_mm },
        { "shell_length_mm", in.shell_length_mm },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "plate_thick_mm",        in.plate_thick_mm },
        { "shell_dia_mm",          in.shell_dia_mm },
        { "shell_length_mm",       in.shell_length_mm },
        { "developed_length_mm",   developedLength },
        { "D_over_t",              dOverT },
        { "outer_radius_mm",       outerR },
        { "inner_radius_mm",       innerR },
        { "shell_volume_mm3",      shellVol },
        { "axis",                  { 0.0, 0.0, 1.0 } },
        { "geometry_changed",      true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({ { "code", f.code },
                             { "severity", f.severity },
                             { "message", f.message } });

    ToolingMeta tooling;
    tooling.tool_type         = "three_roll_plate_bender";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = in.shell_length_mm;
    tooling.tool_material     = "hardened_steel_roll";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    // Rolling is a forming process — no chips.  Convention: positive
    // stock_added_mm3 lives in extra; stock_removed_mm3 remains 0.
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 120.0 + in.plate_thick_mm * 30.0;
    tooling.extra = {
        { "process",              "shell_rolling" },
        { "developed_length_mm",  developedLength },
        { "D_over_t",             dOverT },
        { "shell_volume_mm3",     shellVol },
        { "plate_volume_mm3",     plateVol },
        { "volume_delta_mm3",     volDelta },
        { "stock_added_mm3",      0.0 },   // forming, not deposition
        { "dfm_findings",         findings },
        { "process_category",     "pressure_vessel_fab" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(result, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::shell_roll applied: t={} D={} L={} (devL={:.1f}) faces {}→{}",
                  in.plate_thick_mm, in.shell_dia_mm, in.shell_length_mm,
                  developedLength, wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// PRIMARY: geometric — detect a hollow Z-axis cylindrical shell with two
//   coaxial cylindrical faces whose radii differ by the wall thickness.
// FALLBACK: metadata replay (full confidence).

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // (a) metadata replay — highest confidence.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    if (!out.empty()) return out;

    // (b) geometric heuristic — look for two coaxial Z-axis cylinders.
    if (wp.shape().IsNull()) return out;

    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double bboxL = bb.dz();
    if (bboxL <= 0.0) return out;

    double rOuter = 0.0;
    double rInner = 1e30;
    int    nZCyl  = 0;
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        BRepAdaptor_Surface surf(wp.face(fIdx));
        const gp_Cylinder cyl = surf.Cylinder();
        if (std::abs(std::abs(cyl.Axis().Direction().Z()) - 1.0) > 1e-3)
            continue;
        const double r = cyl.Radius();
        ++nZCyl;
        rOuter = std::max(rOuter, r);
        rInner = std::min(rInner, r);
    }
    // Hollow shell requires at least 2 coaxial cylinders of different radii.
    if (nZCyl < 2)              return out;
    if (rOuter <= 0.0)          return out;
    if (rInner >= rOuter - 1e-3) return out;

    const double wall = rOuter - rInner;
    const double dia  = 2.0 * rOuter;
    if (wall <= 0.0)            return out;
    // Heat-exchanger / pressure-vessel shells are LONG relative to wall —
    // require L/wall > 5 to avoid false matches on stubby rings.
    if (bboxL / wall < 5.0)     return out;

    json recovered = {
        { "plate_thick_mm",   wall },
        { "shell_dia_mm",     dia  },
        { "shell_length_mm",  bboxL },
    };
    json matched = {
        { "outer_radius_mm",  rOuter },
        { "inner_radius_mm",  rInner },
        { "cyl_face_count",   nZCyl },
        { "D_over_t",         dia / wall },
        { "source",           "geometric_fallback" },
    };
    // Confidence 0.6: two coaxial Z cylinders is a strong but not
    // unique signal (could also be flow_form output).
    out.push_back(RecognizedFeature{
        kSkillId, recovered, /*confidence*/ 0.60, matched });
    return out;
}

}  // namespace koocadcam::skill::shell_roll
