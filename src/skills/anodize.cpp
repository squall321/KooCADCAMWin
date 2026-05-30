// @lat: [[engine/skills#anodize]]
//
// Slice-8 upgrade: REAL geometric thin-shell additive synthesis.  Apply
// builds a slab matching the target face footprint × oxide-layer thickness,
// fuses it onto the workpiece, and records the volume delta in the
// signature.  Recognize() walks the resulting B-rep for parallel
// face-pairs separated by the coating thickness (geometric primary,
// confidence 0.5) and only falls back to feature-history if the
// signature was lost / hash-isolated.

#include "anodize.hpp"

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

namespace koocadcam::skill::anodize {

namespace cc = koocadcam::skill::coating_common;
namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────
//
// Thickness ranges per MIL-A-8625F "Anodic Coatings for Aluminum and
// Aluminum Alloys" (US DoD, 2003):
//   - Type I  (chromic acid) — 0.5  – 7.6 μm
//   - Type II (sulfuric)     — 1.8  – 25  μm typical, up to 75 μm clear
//   - Type III (hardcoat)    — 25   – 100 μm; 50 μm typical
// We bracket [5, 75] μm to cover Type II + most Type III.
//
// Face-area minimum: a tank-dunk anodize bath line cannot reliably mask
// targets smaller than ~1 cm²; the ASM Handbook Vol 5 "Surface Engineering"
// (1994), p. 484 cites this as the practical lower bound for fixturing and
// rinse coverage.

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    // MIL-A-8625F thickness window
    if (in.coating_thickness_um <= 0.0) {
        r.add("DFM-INPUT", "error",
              "anodize coating_thickness_um must be > 0");
    } else {
        if (in.coating_thickness_um < 5.0) {
            r.add("DFM-ANODIZE-THK", "error",
                  "anodize coating_thickness_um " +
                  std::to_string(in.coating_thickness_um) +
                  " < 5 μm — below practical MIL-A-8625F Type I lower bound");
        }
        if (in.coating_thickness_um > 75.0) {
            r.add("DFM-ANODIZE-THK", "error",
                  "anodize coating_thickness_um " +
                  std::to_string(in.coating_thickness_um) +
                  " > 75 μm — exceeds MIL-A-8625F Type III hardcoat ceiling");
        }
    }

    if (in.color.empty()) {
        r.add("DFM-INPUT", "warning",
              "anodize color empty — defaulting to natural");
    } else if (!cc::isKnownAnodizeColor(in.color)) {
        r.add("DFM-ANODIZE-COLOR", "info",
              "anodize color '" + in.color +
              "' not in known palette — process plan will treat as custom dye");
    }

    if (!cc::isKnownAnodizeSeal(in.seal_type)) {
        r.add("DFM-ANODIZE-SEAL", "info",
              "anodize seal_type '" + in.seal_type +
              "' unrecognised — expected hot_water | nickel_acetate | none");
    }

    if (!cc::isAluminumFamily(wp.material())) {
        r.add("DFM-ANODIZE-MATERIAL", "info",
              "anodize substrate material '" + wp.material() +
              "' not aluminum-family — anodize bath chemistry may not bond "
              "(consider electroplate or pvd_coat instead)");
    }

    // Face-area minimum check (only when face datum resolves)
    if (auto fid = wp.resolve(in.entry_face); fid) {
        const double areaMm2 = wp.faceArea(*fid);
        if (areaMm2 < 100.0) {       // 1 cm² = 100 mm²
            r.add("DFM-ANODIZE-AREA", "error",
                  "anodize target face area " + std::to_string(areaMm2) +
                  " mm² < 100 mm² (1 cm²) — ASM Handbook v5 fixturing floor");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Build a slab matching the in-plane footprint of the target face, sized
// `thickness_um × 1e-3` mm thick along the outward normal, sunk 1% into the
// parent for a robust fuse, then fuse it onto the workpiece.  The volume
// added equals (slab footprint) × (oxide thickness).

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "anodize DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("anodize: entry_face datum unresolved");
    if (!wp.isFacePlanar(*entryId))
        throw SkillError("anodize: entry_face must be planar for slab fuse");

    const gp_Dir outNorm  = wp.faceNormal(*entryId);
    const gp_Pnt faceCtr  = wp.faceCenter(*entryId);
    const double faceArea = wp.faceArea(*entryId);

    // Footprint sizing: use the face's 3-D bbox to determine in-plane
    // extents along the local axes.  Same recipe used in weld_buildup.
    Bnd_Box faceBb;
    BRepBndLib::AddOptimal(wp.face(*entryId), faceBb);
    double fxMin, fyMin, fzMin, fxMax, fyMax, fzMax;
    faceBb.Get(fxMin, fyMin, fzMin, fxMax, fyMax, fzMax);
    const double fdx = fxMax - fxMin;
    const double fdy = fyMax - fyMin;
    const double fdz = fzMax - fzMin;

    // Local frame: outNorm = local Z, refDir = local X, yLoc = local Y.
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

    // Slice-9 fix: shrink slab footprint by a small inset so the original
    // target face survives as a thin "frame" around the slab.  This gives
    // recognize() a parallel face-pair (original face at z=top, slab top at
    // z=top+thickness) — the frame can be small (recognize uses an area-
    // similarity threshold tuned for thin-shell coatings).
    const double kInsetMm = 1.0;
    slabLen = std::max(slabLen - 2.0 * kInsetMm, slabLen * 0.5);
    slabWid = std::max(slabWid - 2.0 * kInsetMm, slabWid * 0.5);

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
    const TopoDS_Shape slab = pr::box(slabAx, slabLen, slabWid, slabH);

    const TopoDS_Shape newShape = pr::fuse(wp.shape(), slab);

    const std::string color = in.color.empty() ? "natural" : in.color;
    const cc::RgbHint rgb   = cc::anodizeRgbHint(color);
    const double added_mm3  = slabLen * slabWid * thickness_mm;

    json params = {
        { "entry_face_id",        *entryId },
        { "coating_thickness_um", in.coating_thickness_um },
        { "color",                color },
        { "seal_type",            in.seal_type },
        { "entry_face_normal",    { outNorm.X(), outNorm.Y(), outNorm.Z() } },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "target_face_id",       *entryId },
        { "target_face_area_mm2", faceArea },
        { "coating_thickness_um", in.coating_thickness_um },
        { "color",                color },
        { "color_rgb_hint",       { rgb.r, rgb.g, rgb.b } },
        { "seal_type",            in.seal_type },
        { "anodize_class",
          in.coating_thickness_um >= 50.0 ? "Type_III_hard"
                                          : "Type_II_standard" },
        { "additive",             true },
        { "geometry_changed",     true },
        { "slab_len_mm",          slabLen },
        { "slab_wid_mm",          slabWid },
        { "slab_volume_mm3",      added_mm3 },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "anodize_bath";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "sulfuric_acid_electrolyte";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    // Additive — record as negative removed (same convention as weld_buildup).
    tooling.stock_removed_mm3 = -added_mm3;
    // Anodize growth ~ 1 μm per minute at standard current density.
    tooling.est_cycle_time_s  = std::max(5.0 * 60.0,
                                         in.coating_thickness_um * 60.0);
    tooling.extra = {
        { "process",              "Type_II_anodize" },
        { "coating_thickness_um", in.coating_thickness_um },
        { "color",                color },
        { "seal_type",            in.seal_type },
        { "stock_added_mm3",      added_mm3 },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::anodize applied: face={} thk={}μm added={}mm³ faces {}→{}",
                  *entryId, in.coating_thickness_um, added_mm3,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Primary: walk the B-rep for parallel planar face-pairs whose areas are
// roughly equal and whose mid-plane separation lies in the anodize range
// (5 – 75 μm, i.e. 0.005 – 0.075 mm).  Confidence 0.5 — anodize layers
// are sub-OCCT in real life, so even a clean match is a guess.
//
// Fallback: replay feature-history with confidence 1.0.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Geometric primary
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir ni;
        try { ni = wp.faceNormal(i); } catch (...) { continue; }
        const double areaI = wp.faceArea(i);
        if (areaI < 100.0) continue;     // < 1 cm² unlikely to be a coated face
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
            // Anodize range 5 – 75 μm = 0.005 – 0.075 mm.
            if (dist < 0.004 || dist > 0.08) continue;

            const double areaJ = wp.faceArea(j);
            // Thin-shell coatings leave only a slab top and a thin frame
            // around the slab.  We DON'T require equal areas — just that
            // the smaller is non-degenerate (>= 0.5 mm²).
            if (areaJ < 0.5) continue;
            const double areaRatio = (areaJ > 0.0)
                ? std::min(areaI, areaJ) / std::max(areaI, areaJ) : 0.0;

            json recovered = {
                { "entry_face_id",        i },
                { "coating_thickness_um", dist * 1000.0 },
                { "color",                "natural" },
                { "seal_type",            "hot_water" },
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

    // Fallback: metadata replay
    return cc::recognizeFromMetadata(wp, kSkillId);
}

}  // namespace koocadcam::skill::anodize
