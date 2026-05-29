// @lat: [[engine/skills#electroplate]]
//
// Slice-8 upgrade: REAL geometric thin-shell additive synthesis.  Apply
// fuses a slab of plating metal onto the target face footprint with the
// requested thickness.  Volume delta ≈ face_area × thickness_um × 1e-3 mm³.
// Recognize() walks parallel face-pairs in the electroplate range
// (1 – 200 μm) and emits geometric candidates at confidence 0.5.

#include "electroplate.hpp"

#include "Workpiece.hpp"
#include "_coating_common.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::electroplate {

namespace cc = koocadcam::skill::coating_common;
namespace pr = koocadcam::engine::prim;
using nlohmann::json;

static const char* fitToString(cc::PlatingFit f)
{
    switch (f) {
    case cc::PlatingFit::Good:        return "good";
    case cc::PlatingFit::NeedsStrike: return "needs_strike";
    case cc::PlatingFit::Unknown:     return "unknown";
    }
    return "unknown";
}

// ── Validation ───────────────────────────────────────────────────────────
//
// Thickness range per ASTM B374-06(2018) "Standard Terminology Relating to
// Electroplating" + ASTM B689-97 "Electrodeposited Engineering Nickel" Class
// 4 (50 – 75 μm for heavy industrial Ni) + ASTM B650-95 hard chromium
// (industrial Cr 25 – 750 μm but typical decorative 0.3 – 50 μm).  We
// bracket [1, 200] μm covering decorative + most engineering plates;
// > 200 μm is electroforming territory (ASTM B450).
//
// Compatibility: Cr on Al, Au on steel etc. require an intermediate
// strike per ASM Handbook Vol 5 (Surface Engineering), 1994, Table 1 p. 178.
//
// Face-area minimum: a jig-plating rack cannot fixture targets < ~1 cm²
// reliably (current crowding causes burn-marks, see Lowenheim
// "Electroplating", 1978, §11.4).

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.coating_thickness_um <= 0.0) {
        r.add("DFM-INPUT", "error",
              "electroplate coating_thickness_um must be > 0");
    } else {
        if (in.coating_thickness_um < 1.0) {
            r.add("DFM-EP-THK", "error",
                  "electroplate coating_thickness_um " +
                  std::to_string(in.coating_thickness_um) +
                  " < 1 μm — below ASTM B374 practical electroplate floor");
        }
        if (in.coating_thickness_um > 200.0) {
            r.add("DFM-EP-THK", "error",
                  "electroplate coating_thickness_um " +
                  std::to_string(in.coating_thickness_um) +
                  " > 200 μm — exceeds typical ceiling "
                  "(use electroforming per ASTM B450)");
        }
    }

    if (!cc::isKnownPlatingMetal(in.plating_metal)) {
        r.add("DFM-EP-METAL", "info",
              "electroplate plating_metal '" + in.plating_metal +
              "' not in {nickel, chrome, zinc, copper, gold} — process plan "
              "will treat as custom plating chemistry");
    } else {
        const auto* pc = cc::findPlating(in.plating_metal);
        const std::string base = in.base_material.empty()
                                 ? wp.material()
                                 : in.base_material;
        const cc::PlatingFit fit = cc::classifyPlatingFit(*pc, base);
        if (fit == cc::PlatingFit::NeedsStrike) {
            r.add("DFM-EP-FIT", "info",
                  "electroplate " + in.plating_metal + " on '" + base +
                  "' requires intermediate strike-plate "
                  "(typical: zincate or thin Ni underlayer per ASM Vol 5)");
        } else if (fit == cc::PlatingFit::Unknown) {
            r.add("DFM-EP-FIT", "info",
                  "electroplate " + in.plating_metal + " on '" + base +
                  "' compatibility not in table — verify bath chemistry");
        }
    }

    // Face-area minimum
    if (auto fid = wp.resolve(in.entry_face); fid) {
        const double areaMm2 = wp.faceArea(*fid);
        if (areaMm2 < 100.0) {
            r.add("DFM-EP-AREA", "error",
                  "electroplate target face area " + std::to_string(areaMm2) +
                  " mm² < 100 mm² (1 cm²) — Lowenheim §11.4 fixturing floor");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "electroplate DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("electroplate: entry_face datum unresolved");
    if (!wp.isFacePlanar(*entryId))
        throw SkillError("electroplate: entry_face must be planar for slab fuse");

    const std::string base = in.base_material.empty()
                             ? wp.material()
                             : in.base_material;
    const cc::PlatingCompatibility* pc = cc::findPlating(in.plating_metal);
    const cc::PlatingFit fit = pc ? cc::classifyPlatingFit(*pc, base)
                                  : cc::PlatingFit::Unknown;
    const double hardnessHv = pc ? pc->hardness_hv : 0.0;

    const gp_Dir outNorm  = wp.faceNormal(*entryId);
    const gp_Pnt faceCtr  = wp.faceCenter(*entryId);
    const double faceArea = wp.faceArea(*entryId);

    Bnd_Box faceBb;
    BRepBndLib::AddOptimal(wp.face(*entryId), faceBb);
    double fxMin, fyMin, fzMin, fxMax, fyMax, fzMax;
    faceBb.Get(fxMin, fyMin, fzMin, fxMax, fyMax, fzMax);
    const double fdx = fxMax - fxMin;
    const double fdy = fyMax - fyMin;
    const double fdz = fzMax - fzMin;

    gp_Dir refDir = gp::DX();
    if (std::abs(outNorm.Dot(refDir)) > 0.9) refDir = gp::DY();
    {
        gp_Vec rv(refDir);
        const gp_Vec nv(outNorm);
        rv -= nv * rv.Dot(nv);
        if (rv.Magnitude() < 1e-9) rv = gp_Vec(gp::DY());
        rv.Normalize();
        refDir = gp_Dir(rv);
    }
    const gp_Vec outV(outNorm);
    const gp_Vec xV(refDir);
    const gp_Vec yV = outV.Crossed(xV);
    const gp_Dir yLoc(yV);

    auto axisExtent = [&](double ax, double ay, double az) {
        if (ax > 0.9) return fdx;
        if (ay > 0.9) return fdy;
        if (az > 0.9) return fdz;
        return std::sqrt(fdx * fdx + fdy * fdy + fdz * fdz);
    };
    double slabLen = axisExtent(std::abs(refDir.X()), std::abs(refDir.Y()),
                                std::abs(refDir.Z()));
    double slabWid = axisExtent(std::abs(yLoc.X()),   std::abs(yLoc.Y()),
                                std::abs(yLoc.Z()));
    if (slabLen < 1e-6) slabLen = std::sqrt(faceArea);
    if (slabWid < 1e-6) slabWid = std::sqrt(faceArea);

    const double thickness_mm = in.coating_thickness_um * 1.0e-3;
    const double kSinkPct     = 0.01;
    const double sink         = thickness_mm * kSinkPct;
    const double slabH        = thickness_mm + sink;

    const gp_Pnt slabOrigin(
        faceCtr.X() - refDir.X() * (slabLen / 2.0)
                    - yLoc.X()   * (slabWid / 2.0)
                    - outNorm.X() * sink,
        faceCtr.Y() - refDir.Y() * (slabLen / 2.0)
                    - yLoc.Y()   * (slabWid / 2.0)
                    - outNorm.Y() * sink,
        faceCtr.Z() - refDir.Z() * (slabLen / 2.0)
                    - yLoc.Z()   * (slabWid / 2.0)
                    - outNorm.Z() * sink);

    const gp_Ax2 slabAx(slabOrigin, outNorm, refDir);
    const TopoDS_Shape slab     = pr::box(slabAx, slabLen, slabWid, slabH);
    const TopoDS_Shape newShape = pr::fuse(wp.shape(), slab);

    const double added_mm3 = slabLen * slabWid * thickness_mm;

    json params = {
        { "entry_face_id",        *entryId },
        { "plating_metal",        in.plating_metal },
        { "coating_thickness_um", in.coating_thickness_um },
        { "base_material",        base },
        { "entry_face_normal",    { outNorm.X(), outNorm.Y(), outNorm.Z() } },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "target_face_id",       *entryId },
        { "target_face_area_mm2", faceArea },
        { "plating_metal",        in.plating_metal },
        { "coating_thickness_um", in.coating_thickness_um },
        { "base_material",        base },
        { "plating_fit",          fitToString(fit) },
        { "deposit_hardness_hv",  hardnessHv },
        { "additive",             true },
        { "geometry_changed",     true },
        { "slab_len_mm",          slabLen },
        { "slab_wid_mm",          slabWid },
        { "slab_volume_mm3",      added_mm3 },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "electroplate_bath";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = in.plating_metal + "_anode";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = -added_mm3;   // additive convention
    // Plating rate ~ 0.5 μm/min typical; chrome/gold are slower.
    double rate_um_per_min = 0.5;
    if (in.plating_metal == "chrome") rate_um_per_min = 0.2;
    if (in.plating_metal == "gold")   rate_um_per_min = 0.1;
    tooling.est_cycle_time_s  = std::max(30.0,
                                         in.coating_thickness_um
                                       / rate_um_per_min * 60.0);
    tooling.extra = {
        { "process",              "electroplate" },
        { "plating_metal",        in.plating_metal },
        { "coating_thickness_um", in.coating_thickness_um },
        { "base_material",        base },
        { "plating_fit",          fitToString(fit) },
        { "stock_added_mm3",      added_mm3 },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::electroplate applied: face={} metal={} thk={}μm added={}mm³",
        *entryId, in.plating_metal, in.coating_thickness_um, added_mm3);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Primary: planar face-pairs separated by 1 – 200 μm with matching areas.
// Fallback: feature-history replay.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir ni;
        try { ni = wp.faceNormal(i); } catch (...) { continue; }
        const double areaI = wp.faceArea(i);
        if (areaI < 100.0) continue;
        const gp_Pnt ci = wp.faceCenter(i);

        for (int j = i + 1; j < wp.faceCount(); ++j) {
            if (!wp.isFacePlanar(j)) continue;
            gp_Dir nj;
            try { nj = wp.faceNormal(j); } catch (...) { continue; }
            const double dot = ni.Dot(nj);
            if (std::abs(std::abs(dot) - 1.0) > 1e-3) continue;

            const gp_Pnt cj = wp.faceCenter(j);
            const gp_Vec sepVec(ci, cj);
            const double dist = std::abs(sepVec.Dot(gp_Vec(ni)));
            // electroplate range 1 – 200 μm = 0.001 – 0.2 mm.
            if (dist < 0.0009 || dist > 0.21) continue;

            const double areaJ = wp.faceArea(j);
            const double areaRatio = (areaJ > 0.0)
                ? std::min(areaI, areaJ) / std::max(areaI, areaJ) : 0.0;
            if (areaRatio < 0.7) continue;

            json recovered = {
                { "entry_face_id",        i },
                { "plating_metal",        "nickel" },
                { "coating_thickness_um", dist * 1000.0 },
                { "base_material",        wp.material() },
            };
            json matched = {
                { "parent_face_id",  i },
                { "shell_face_id",   j },
                { "area_ratio",      areaRatio },
                { "separation_mm",   dist },
                { "source",          "geometric" },
            };
            out.push_back(RecognizedFeature{
                kSkillId, recovered, 0.5, matched
            });
            break;
        }
    }
    if (!out.empty()) return out;

    return cc::recognizeFromMetadata(wp, kSkillId);
}

}  // namespace koocadcam::skill::electroplate
