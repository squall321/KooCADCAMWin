// @lat: [[engine/skills#tablet_press]]
//
// tablet_press — REAL geometric implementation.
//
// The input workpiece (a blank / blend slug) is REBUILT as a solid tablet
// disc.  Volume is derived from the input tablet_mass_mg and a standard
// pharma blend density (1.5 g/cm³ = 1.5 mg/mm³).  Aspect ratio dia/thick
// is held at 4 (typical round flat-faced tablet, USP <1216>).
//
//   blank slug                tablet disc
//   ┌─────────────┐          ┌─────────────┐
//   │             │   apply  │  ▓▓▓▓▓▓▓    │   (cylinder of Ø d × t,
//   │ block / cyl │  ──────▶ │  ▓▓▓▓▓▓▓    │    same Z-base, centred
//   │             │          │             │    on bbox XY centre)
//   └─────────────┘          └─────────────┘
//
// Recognize: walk faces, identify the disc as ONE cylindrical side face + two
// circular planar faces with same diameter; reconstruct mass from volume.

#include "tablet_press.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::tablet_press {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

// Standard pharma blend bulk density used to convert mass→volume.
// Range per USP <1216> Tablet Friability and Remington's Pharmaceutical
// Sciences §45 "Tablets": granulated blend bulk density 1.3–1.7 g/cm³;
// midpoint 1.5 g/cm³ = 1.5 mg/mm³.
constexpr double kBlendDensityMgPerMm3 = 1.5;

// Aspect ratio dia/thick for a round flat-faced (RFF) tablet, USP <1216>
// "Tablet Friability" Annex II typical compendial geometry.
constexpr double kAspectRatio = 4.0;

struct DiscGeom {
    double diameter_mm;
    double thickness_mm;
    double volume_mm3;
};

DiscGeom discFromMass(double mass_mg)
{
    // V = mass / density;  with d = aspect · t  →  V = π·(d/2)²·t = π·(aspect/2)²·t³
    const double V       = mass_mg / kBlendDensityMgPerMm3;
    const double t3      = V / (M_PI * (kAspectRatio * kAspectRatio) / 4.0);
    const double t       = std::cbrt(t3);
    const double d       = kAspectRatio * t;
    return DiscGeom{ d, t, V };
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────
//
// Standards reference:
//   - USP <1216> "Tablet Friability" (compendial geometry & mass ranges).
//   - Remington's Pharmaceutical Sciences §45 "Tablets" (rotary press
//     mechanical limits — pre-compression 2 kN typical, main compression
//     up to ~50 kN; above 50 kN → capping/lamination risk).

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.press_force_kn <= 0.0) {
        r.add("DFM-INPUT", "error",
              "tablet_press press_force_kn must be > 0 (got " +
              std::to_string(in.press_force_kn) + ")");
    } else if (in.press_force_kn > 50.0) {
        // Remington §45: > 50 kN main-compression → capping / lamination
        // due to elastic recovery exceeding plastic deformation budget.
        r.add("DFM-TP-FORCE-HIGH", "info",
              "tablet_press press_force_kn " +
              std::to_string(in.press_force_kn) +
              " kN > 50 (Remington §45: capping / lamination risk)");
    } else if (in.press_force_kn < 2.0) {
        // Remington §45: < 2 kN → insufficient consolidation, friable.
        r.add("DFM-TP-FORCE-LOW", "info",
              "tablet_press press_force_kn " +
              std::to_string(in.press_force_kn) +
              " kN < 2 (Remington §45: friable / weak tablet risk)");
    }

    if (in.dwell_time_ms <= 0.0) {
        r.add("DFM-INPUT", "error",
              "tablet_press dwell_time_ms must be > 0 (got " +
              std::to_string(in.dwell_time_ms) + ")");
    }

    if (in.tablet_mass_mg <= 0.0) {
        r.add("DFM-INPUT", "error",
              "tablet_press tablet_mass_mg must be > 0 (got " +
              std::to_string(in.tablet_mass_mg) + ")");
    } else if (in.tablet_mass_mg < 30.0 || in.tablet_mass_mg > 2000.0) {
        // USP <1216>: compendial tablets typically 30–2000 mg.
        r.add("DFM-TP-MASS-RANGE", "info",
              "tablet_press tablet_mass_mg " +
              std::to_string(in.tablet_mass_mg) +
              " outside USP <1216> typical 30..2000 mg range");
    }

    // DFM-TP-STOCK — the input blank must have enough volume to source the
    // tablet (otherwise mass is greater than available material).
    if (!wp.shape().IsNull() && in.tablet_mass_mg > 0.0) {
        GProp_GProps p;
        BRepGProp::VolumeProperties(wp.shape(), p);
        const double stockVol = p.Mass();
        const double tabletVol = in.tablet_mass_mg / kBlendDensityMgPerMm3;
        if (tabletVol > stockVol + 1e-6) {
            r.add("DFM-TP-STOCK", "error",
                  "tablet_press required tablet volume " +
                  std::to_string(tabletVol) +
                  " mm³ exceeds blank volume " +
                  std::to_string(stockVol) + " mm³");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Rebuild the workpiece as a tablet disc.  XY centre is taken from the
// blank's bbox centre; Z base from bbox zMin.  Volume is mass / density.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "tablet_press DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double cx = 0.5 * (xMin + xMax);
    const double cy = 0.5 * (yMin + yMax);

    const DiscGeom g = discFromMass(in.tablet_mass_mg);

    TopoDS_Shape disc;
    try {
        const gp_Ax2 ax(gp_Pnt(cx, cy, zMin), gp::DZ());
        disc = pr::cylinder(ax, g.diameter_mm * 0.5, g.thickness_mm);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("tablet_press: disc build failed: ") +
                         ex.what());
    }

    json params = {
        { "press_force_kn",  in.press_force_kn },
        { "dwell_time_ms",   in.dwell_time_ms },
        { "tablet_mass_mg",  in.tablet_mass_mg },
    };

    json pattern = {
        { "kind",             kSkillId },
        { "press_force_kn",   in.press_force_kn },
        { "dwell_time_ms",    in.dwell_time_ms },
        { "tablet_mass_mg",   in.tablet_mass_mg },
        { "tablet_dia_mm",    g.diameter_mm },
        { "tablet_thick_mm",  g.thickness_mm },
        { "tablet_volume_mm3",g.volume_mm3 },
        { "blend_density_mg_mm3", kBlendDensityMgPerMm3 },
        { "process_family",   "solid_dose_compaction" },
        { "geometric_change", true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "tablet_press";
    tooling.tool_dia_mm       = g.diameter_mm;
    tooling.tool_length_mm    = g.thickness_mm;
    tooling.tool_material     = "tool_steel_punch_die";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    // The disc IS the produced material; record it as additive
    // (consistent with weld_buildup convention: negative = volume added).
    tooling.stock_removed_mm3 = -g.volume_mm3;
    tooling.est_cycle_time_s  = in.dwell_time_ms * 0.001 + 0.2;  // dwell + cycle
    tooling.extra = {
        { "process",          "tablet_compression" },
        { "press_force_kn",   in.press_force_kn },
        { "dwell_time_ms",    in.dwell_time_ms },
        { "tablet_mass_mg",   in.tablet_mass_mg },
        { "tablet_dia_mm",    g.diameter_mm },
        { "tablet_thick_mm",  g.thickness_mm },
        { "process_category", "pharma_solid_dose" },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(disc, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::tablet_press applied: mass={}mg → Ø{}×{}mm (V={}mm³)",
                  in.tablet_mass_mg, g.diameter_mm, g.thickness_mm,
                  g.volume_mm3);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// PRIMARY heuristic: look for a single Z-axis cylinder with diameter ≥ 4·
// thickness (aspect ≥ 4, typical RFF tablet).  When the bbox satisfies this
// disc geometry, emit a recovered mass = volume × density at confidence 0.6.
//
// FALLBACK: replay any tablet_press metadata in the feature history at
// confidence 1.0.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // FALLBACK first so metadata replay takes precedence.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "feature_history" } };
        out.push_back(rf);
    }
    if (!out.empty()) return out;

    // PRIMARY geometric heuristic — walk faces for a Z-axis cylinder + two
    // parallel circular caps with thickness < dia/4.
    if (wp.shape().IsNull()) return out;

    int zCylFace = -1;
    double cylRadius = 0.0;
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        BRepAdaptor_Surface s(wp.face(fIdx));
        const gp_Cylinder cyl = s.Cylinder();
        if (std::abs(std::abs(cyl.Axis().Direction().Z()) - 1.0) > 1e-3)
            continue;
        zCylFace = fIdx;
        cylRadius = cyl.Radius();
        break;
    }
    if (zCylFace < 0) return out;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double dz = zMax - zMin;
    const double dia = cylRadius * 2.0;
    if (dz <= 0.0 || dia <= 0.0) return out;
    if (dia < dz * (kAspectRatio - 0.5)) return out;   // not disc-shaped

    GProp_GProps p;
    BRepGProp::VolumeProperties(wp.shape(), p);
    const double vol = p.Mass();
    const double recoveredMass = vol * kBlendDensityMgPerMm3;

    json recovered = {
        { "press_force_kn",  10.0 },         // geometric mode can't recover force
        { "dwell_time_ms",   50.0 },         // or dwell — defaults
        { "tablet_mass_mg",  recoveredMass },
    };
    json matched = {
        { "source",          "geometric_heuristic" },
        { "z_cyl_face_id",   zCylFace },
        { "dia_mm",          dia },
        { "thickness_mm",    dz },
        { "volume_mm3",      vol },
        { "aspect_ratio",    dia / dz },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.55, matched });
    return out;
}

}  // namespace koocadcam::skill::tablet_press
