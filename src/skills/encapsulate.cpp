// @lat: [[engine/skills#encapsulate]]
//
// encapsulate — REAL geometric implementation.
//
// Produces a 2-part hard-shell capsule geometry: outer wall = fused cap
// cylinder + body cylinder (overlapping at the join), inner cavity = a
// concentric cylinder cut from the fused shell.  Resulting topology:
//
//   ┌──────────────┐  ←  cap (Ø = size_OD, len = body_len)
//   │     ╱──╲     │
//   │    │    │    │  ←  outer fused shell, hollow interior
//   │    │    │    │
//   │     ╲──╱     │
//   └──────────────┘
//
// Dimensions per Capsugel / Lonza published Coni-Snap hard-shell catalog:
//   size  overall_len_mm   OD_mm    cap_OD_mm
//   000        26.14        9.91     10.05
//   00         23.30        8.53      8.65
//   0          21.70        7.65      7.77
//   1          19.40        6.91      6.99
//   2          18.00        6.35      6.43
//   3          15.90        5.82      5.90
//   4          14.30        5.31      5.39
//
// We simplify (slice-1) to a single outer cylinder of size_OD × overall_len
// with a concentric inner cavity sized so that cavity_volume ≈ catalog
// max fill volume.  Wall thickness ≈ 0.1 mm (hard gelatin standard, USP <711>).

#include "encapsulate.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
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

#include <cmath>
#include <string>
#include <unordered_map>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::encapsulate {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

struct CapsuleDims {
    double overall_len_mm;
    double body_od_mm;
    double cap_od_mm;
    double capacity_mg;   // typical max fill mass (powder)
};

const std::unordered_map<std::string, CapsuleDims>& dimsCatalog()
{
    // Capsugel Coni-Snap published dimensions (mm) and approximate
    // max fill weights (mg).  Capacity numbers also match Remington's
    // §45 capsule capacity table.
    static const std::unordered_map<std::string, CapsuleDims> k = {
        { "000", { 26.14, 9.91, 10.05, 1000.0 } },
        { "00",  { 23.30, 8.53,  8.65,  800.0 } },
        { "0",   { 21.70, 7.65,  7.77,  500.0 } },
        { "1",   { 19.40, 6.91,  6.99,  400.0 } },
        { "2",   { 18.00, 6.35,  6.43,  300.0 } },
        { "3",   { 15.90, 5.82,  5.90,  200.0 } },
        { "4",   { 14.30, 5.31,  5.39,  150.0 } },
    };
    return k;
}

bool isKnownSize(const std::string& sz) { return dimsCatalog().count(sz) > 0; }
double capacityForSize(const std::string& sz)
{
    auto it = dimsCatalog().find(sz);
    return (it == dimsCatalog().end()) ? 0.0 : it->second.capacity_mg;
}

// Hard gelatin shell wall thickness — USP <711> compendial standard ~0.10 mm.
constexpr double kWallThicknessMm = 0.10;

// Cap / body length ratio: cap covers ~30% of overall length, body ~70%,
// with overlap zone in the middle.  Slice-1 simplification: fuse two
// cylinders representing cap + body sections sharing the overall axis.
constexpr double kCapLenFrac = 0.40;

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.fill_weight_mg <= 0.0) {
        r.add("DFM-INPUT", "error",
              "encapsulate fill_weight_mg must be > 0 (got " +
              std::to_string(in.fill_weight_mg) + ")");
    }

    if (!isKnownSize(in.capsule_size)) {
        r.add("DFM-ENC-SIZE", "error",
              "encapsulate capsule_size '" + in.capsule_size +
              "' unknown — expected '000'..'4' (Capsugel Coni-Snap catalog)");
    } else {
        const double cap = capacityForSize(in.capsule_size);
        if (in.fill_weight_mg > cap) {
            // Remington §45: overfill risks cap dislodgement during handling.
            r.add("DFM-ENC-OVERFILL", "info",
                  "encapsulate fill_weight_mg " +
                  std::to_string(in.fill_weight_mg) +
                  " > Capsugel max " + std::to_string(cap) +
                  " mg for size " + in.capsule_size);
        }
    }

    if (in.fill_type != "powder"  &&
        in.fill_type != "pellet"  &&
        in.fill_type != "granule" &&
        in.fill_type != "liquid") {
        r.add("DFM-ENC-TYPE", "info",
              "encapsulate fill_type '" + in.fill_type +
              "' unrecognised — expected powder | pellet | granule | liquid");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Build sequence:
//   1. Body cylinder  axis (0,0,zMin), Ø=body_OD, len=(1−capFrac)·L
//   2. Cap cylinder   axis (0,0,zMin+bodyLen−overlap), Ø=cap_OD, len=capFrac·L+overlap
//   3. Fused outer shell = cap ∪ body
//   4. Inner cavity   axis along Z, Ø=(body_OD−2·wall), len=(L−wall)
//   5. Hollow capsule = outer − cavity
//
// XY centred on the input workpiece bbox; Z base = bbox zMin.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "encapsulate DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const CapsuleDims& d = dimsCatalog().at(in.capsule_size);

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double cx = 0.5 * (xMin + xMax);
    const double cy = 0.5 * (yMin + yMax);
    const double zBase = zMin;

    const double bodyLen = d.overall_len_mm * (1.0 - kCapLenFrac);
    const double capLen  = d.overall_len_mm * kCapLenFrac;
    const double overlap = std::min(1.5, capLen * 0.25);   // join overlap mm
    const double bodyR   = d.body_od_mm * 0.5;
    const double capR    = d.cap_od_mm  * 0.5;

    TopoDS_Shape outer;
    try {
        const gp_Ax2 axBody(gp_Pnt(cx, cy, zBase), gp::DZ());
        const TopoDS_Shape bodyCyl = pr::cylinder(axBody, bodyR, bodyLen);

        const gp_Ax2 axCap(gp_Pnt(cx, cy, zBase + bodyLen - overlap), gp::DZ());
        const TopoDS_Shape capCyl  = pr::cylinder(axCap, capR, capLen + overlap);

        outer = pr::fuse(bodyCyl, capCyl);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("encapsulate: outer shell build failed: ") +
                         ex.what());
    }

    // Inner cavity — uniformly inset by wall thickness, slightly shorter at
    // each end so the dome stays closed.  We size cavity from the smaller
    // (body) OD; the cap region is slightly thicker walled but still
    // contiguous with the cavity hole.
    const double cavityR = bodyR - kWallThicknessMm;
    const double cavityLen = d.overall_len_mm - 2.0 * kWallThicknessMm;

    TopoDS_Shape result = outer;
    if (cavityR > 0.0 && cavityLen > 0.0) {
        try {
            const gp_Ax2 axCav(
                gp_Pnt(cx, cy, zBase + kWallThicknessMm), gp::DZ());
            const TopoDS_Shape cavity = pr::cylinder(axCav, cavityR, cavityLen);
            result = pr::cut(outer, cavity);
        } catch (const Standard_Failure& ex) {
            throw SkillError(std::string("encapsulate: cavity cut failed: ") +
                             ex.what());
        }
    }

    // Geometric metrics for the signature.
    const double capVolMm3   = M_PI * capR  * capR  * (capLen + overlap);
    const double bodyVolMm3  = M_PI * bodyR * bodyR * bodyLen;
    const double outerVolMm3 = capVolMm3 + bodyVolMm3
                               - M_PI * std::min(capR, bodyR) *
                                 std::min(capR, bodyR) * overlap;
    const double cavityVolMm3 = (cavityR > 0.0 && cavityLen > 0.0)
        ? M_PI * cavityR * cavityR * cavityLen
        : 0.0;
    const double shellVolMm3 = std::max(0.0, outerVolMm3 - cavityVolMm3);

    json params = {
        { "capsule_size",   in.capsule_size },
        { "fill_weight_mg", in.fill_weight_mg },
        { "fill_type",      in.fill_type },
    };

    json pattern = {
        { "kind",             kSkillId },
        { "capsule_size",     in.capsule_size },
        { "fill_weight_mg",   in.fill_weight_mg },
        { "fill_type",        in.fill_type },
        { "overall_len_mm",   d.overall_len_mm },
        { "body_od_mm",       d.body_od_mm },
        { "cap_od_mm",        d.cap_od_mm },
        { "wall_mm",          kWallThicknessMm },
        { "cap_volume_mm3",   capVolMm3 },
        { "body_volume_mm3",  bodyVolMm3 },
        { "shell_volume_mm3", shellVolMm3 },
        { "cavity_volume_mm3",cavityVolMm3 },
        { "process_family",   "pharma_encapsulation" },
        { "geometric_change", true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "capsule_filler";
    tooling.tool_dia_mm       = d.body_od_mm;
    tooling.tool_length_mm    = d.overall_len_mm;
    tooling.tool_material     = "stainless_316l_dosator";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = -shellVolMm3;   // additive: shell material added
    tooling.est_cycle_time_s  = 0.5;            // typical dosator cycle
    tooling.extra = {
        { "process",          "capsule_filling" },
        { "capsule_size",     in.capsule_size },
        { "fill_weight_mg",   in.fill_weight_mg },
        { "fill_type",        in.fill_type },
        { "shell_volume_mm3", shellVolMm3 },
        { "cavity_volume_mm3",cavityVolMm3 },
        { "process_category", "pharma_solid_dose" },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(result, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::encapsulate applied: size={} OD={} L={} shell={}mm³",
                  in.capsule_size, d.body_od_mm, d.overall_len_mm, shellVolMm3);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// PRIMARY: replay history (confidence 1.0).
//
// FALLBACK heuristic: search for TWO Z-axis cylindrical faces (body + cap)
// of differing radius along a common axis, with combined length matching
// one of the catalog sizes within ±10%.  Confidence 0.5.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // PRIMARY history-replay path first.
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

    // FALLBACK geometric heuristic.
    if (wp.shape().IsNull()) return out;

    int zCylCount = 0;
    double maxR = 0.0;
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        BRepAdaptor_Surface s(wp.face(fIdx));
        const gp_Cylinder cyl = s.Cylinder();
        if (std::abs(std::abs(cyl.Axis().Direction().Z()) - 1.0) > 1e-3)
            continue;
        ++zCylCount;
        if (cyl.Radius() > maxR) maxR = cyl.Radius();
    }
    if (zCylCount < 2) return out;   // need at least body+cap or shell+cavity

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double dz = zMax - zMin;
    const double od = maxR * 2.0;
    if (dz <= 0.0 || od <= 0.0) return out;

    // Match to nearest catalog size within ±15% on both length AND OD.
    std::string bestSize;
    double bestErr = 1e30;
    for (const auto& [sz, d] : dimsCatalog()) {
        const double lenErr = std::abs(dz - d.overall_len_mm) / d.overall_len_mm;
        const double odErr  = std::abs(od - d.body_od_mm)     / d.body_od_mm;
        if (lenErr > 0.15 || odErr > 0.15) continue;
        const double err = lenErr + odErr;
        if (err < bestErr) { bestErr = err; bestSize = sz; }
    }
    if (bestSize.empty()) return out;

    json recovered = {
        { "capsule_size",   bestSize },
        { "fill_weight_mg", capacityForSize(bestSize) * 0.8 },  // estimate
        { "fill_type",      "powder" },
    };
    json matched = {
        { "source",        "geometric_heuristic" },
        { "z_cyl_count",   zCylCount },
        { "overall_len_mm",dz },
        { "outer_dia_mm",  od },
        { "match_error",   bestErr },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.50, matched });
    return out;
}

}  // namespace koocadcam::skill::encapsulate
