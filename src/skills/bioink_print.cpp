// @lat: [[engine/skills#bioink_print]]
//
// bioink_print — REAL geometric implementation.
//
// Produces a layered cylindrical scaffold approximation.  Total height is
// derived from the input layer count × layer height; diameter is set from
// the input workpiece bbox (we inscribe the largest XY-fitting circle so
// the scaffold lives within the build envelope).
//
//   ┌─────┐
//   │  ▓  │   ← layer N    (each layer is a Δh thin slab,
//   │  ▓  │     ...           visualised as a cylinder of
//   │  ▓  │   ← layer 1      Δh height stacked on the previous)
//   └─────┘
//
// Slice-1 simplification: instead of N stacked cylinders (which makes the
// face count explode), we build ONE solid cylinder of total height
// (layer_count × layer_height_um × 1e-3) and record the layer count in
// metadata so downstream slicer can re-segment.

#include "bioink_print.hpp"

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
#include <unordered_set>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::bioink_print {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

const std::unordered_set<std::string>& knownBioinks()
{
    static const std::unordered_set<std::string> s = {
        "gelma", "alginate", "collagen", "plga", "fibrin",
        "hyaluronic_acid", "decellularized_ecm",
    };
    return s;
}

// Default scaffold layer count when the input cannot prescribe one.
// At default 200 μm layers and a 4 mm-tall scaffold this is 20 layers,
// consistent with Murphy & Atala (Nature Biotech 2014) tissue scaffold studies.
constexpr int kDefaultLayerCount = 20;

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────
//
// Standards reference:
//   - ISO/ASTM 52900:2021 "Additive manufacturing — General principles".
//   - Murphy & Atala, "3D bioprinting of tissues and organs", Nature
//     Biotechnology 32, 773–785 (2014): hydrogel layer 100–400 μm typical,
//     extending to 50 μm (micro-extrusion) and 1000 μm (large constructs).
//   - Cell density practical viable range 1e3 – 1e8 cells/mL — above this
//     shear in the nozzle exceeds the viability threshold (Blaeser et al.,
//     Adv. Healthc. Mater. 2016).

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.bioink_type.empty()) {
        r.add("DFM-INPUT", "error",
              "bioink_print bioink_type must be specified");
    } else if (knownBioinks().count(in.bioink_type) == 0) {
        r.add("DFM-BIOINK-VIABILITY", "info",
              "bioink_print bioink_type '" + in.bioink_type +
              "' not in standard catalog — verify cytocompatibility");
    }

    if (in.layer_height_um < 50.0 || in.layer_height_um > 1000.0) {
        // Murphy & Atala 2014 hydrogel extrusion practical range.
        r.add("DFM-BIOINK-LAYER", "error",
              "bioink_print layer_height_um " +
              std::to_string(in.layer_height_um) +
              " outside [50, 1000] μm range (Murphy & Atala 2014)");
    }

    if (in.cell_density_per_ml < 1.0e3 || in.cell_density_per_ml > 1.0e8) {
        // Blaeser et al. 2016 shear-vs-viability window.
        r.add("DFM-BIOINK-DENSITY", "error",
              "bioink_print cell_density_per_ml " +
              std::to_string(in.cell_density_per_ml) +
              " outside [1e3, 1e8] viable range (Blaeser et al. 2016)");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Rebuild workpiece as a cylindrical scaffold.  Diameter = the largest
// circle inscribed in the input XY bbox; height = layer_count × layer
// height.  This makes the printed scaffold envelope visible in B-Rep while
// keeping topology trivially analysable (one cyl side, two end caps).

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "bioink_print DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double dx = xMax - xMin;
    const double dy = yMax - yMin;
    const double cx = 0.5 * (xMin + xMax);
    const double cy = 0.5 * (yMin + yMax);

    // Scaffold diameter = inscribed circle in XY bbox.
    const double scaffoldDia = std::max(0.5, std::min(dx, dy));
    const double scaffoldR   = scaffoldDia * 0.5;

    // Layer count: choose so total scaffold height ≤ input bbox Z extent
    // (so scaffold fits in build envelope) and ≥ default.
    const double layerHeightMm = in.layer_height_um * 1e-3;
    int layerCount = kDefaultLayerCount;
    const double envZ = zMax - zMin;
    if (envZ > 0.0 && layerHeightMm > 0.0) {
        const int maxByEnv = static_cast<int>(std::floor(envZ / layerHeightMm));
        if (maxByEnv > 1) layerCount = std::min(layerCount, maxByEnv);
        if (layerCount < 1) layerCount = 1;
    }
    const double scaffoldH = layerCount * layerHeightMm;

    TopoDS_Shape scaffold;
    try {
        const gp_Ax2 ax(gp_Pnt(cx, cy, zMin), gp::DZ());
        scaffold = pr::cylinder(ax, scaffoldR, scaffoldH);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("bioink_print: scaffold build failed: ") +
                         ex.what());
    }

    const double scaffoldVolMm3 = M_PI * scaffoldR * scaffoldR * scaffoldH;
    // Total cells = density (cells/mL) × volume (mL).  1 mL = 1000 mm³.
    const double totalCells =
        in.cell_density_per_ml * (scaffoldVolMm3 / 1000.0);

    json params = {
        { "bioink_type",         in.bioink_type },
        { "cell_density_per_ml", in.cell_density_per_ml },
        { "layer_height_um",     in.layer_height_um },
    };

    json pattern = {
        { "kind",                 kSkillId },
        { "bioink_type",          in.bioink_type },
        { "cell_density_per_ml",  in.cell_density_per_ml },
        { "layer_height_um",      in.layer_height_um },
        { "scaffold_dia_mm",      scaffoldDia },
        { "scaffold_height_mm",   scaffoldH },
        { "layer_count",          layerCount },
        { "scaffold_volume_mm3",  scaffoldVolMm3 },
        { "total_cells",          totalCells },
        { "process_family",       "additive_bioprinting" },
        { "geometric_change",     true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "bioprinter_extruder";
    tooling.tool_dia_mm       = scaffoldDia;
    tooling.tool_length_mm    = scaffoldH;
    tooling.tool_material     = "sterile_polymer_nozzle";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = -scaffoldVolMm3;   // additive: hydrogel deposited
    // Rough: bioprinting deposition ~10 mm³/s per nozzle (literature midrange).
    tooling.est_cycle_time_s  = scaffoldVolMm3 / 10.0;
    tooling.extra = {
        { "process",             "bioink_extrusion" },
        { "bioink_type",         in.bioink_type },
        { "cell_density_per_ml", in.cell_density_per_ml },
        { "layer_height_um",     in.layer_height_um },
        { "layer_count",         layerCount },
        { "scaffold_volume_mm3", scaffoldVolMm3 },
        { "total_cells",         totalCells },
        { "process_category",    "bioprinting" },
        { "dfm_findings",        findings },
        { "biosafety_required",  true },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(scaffold, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::bioink_print applied: Ø{} × {} mm ({} layers, V={} mm³)",
                  scaffoldDia, scaffoldH, layerCount, scaffoldVolMm3);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// PRIMARY: replay history (confidence 1.0).
// FALLBACK: any single Z-axis cylinder is a candidate scaffold; we report
// recovered scaffold_dia + height at confidence 0.45.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

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

    if (wp.shape().IsNull()) return out;

    int zCylFace = -1;
    double cylR = 0.0;
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        BRepAdaptor_Surface s(wp.face(fIdx));
        const gp_Cylinder cyl = s.Cylinder();
        if (std::abs(std::abs(cyl.Axis().Direction().Z()) - 1.0) > 1e-3)
            continue;
        zCylFace = fIdx;
        cylR = cyl.Radius();
        break;
    }
    if (zCylFace < 0) return out;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double dz  = zMax - zMin;
    const double dia = cylR * 2.0;
    if (dz <= 0.0 || dia <= 0.0) return out;

    // Default-derive layer count from a 200 μm layer height assumption.
    const double assumedLayerUm = 200.0;
    const int    derivedLayers  =
        std::max(1, static_cast<int>(std::round(dz / (assumedLayerUm * 1e-3))));

    json recovered = {
        { "bioink_type",         "gelma" },           // best-guess default
        { "cell_density_per_ml", 1.0e6 },
        { "layer_height_um",     assumedLayerUm },
    };
    json matched = {
        { "source",              "geometric_heuristic" },
        { "z_cyl_face_id",       zCylFace },
        { "scaffold_dia_mm",     dia },
        { "scaffold_height_mm",  dz },
        { "derived_layer_count", derivedLayers },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.45, matched });
    return out;
}

}  // namespace koocadcam::skill::bioink_print
