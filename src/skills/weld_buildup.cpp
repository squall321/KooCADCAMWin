// @lat: [[engine/skills#weld_buildup]]

#include "weld_buildup.hpp"

#include "Workpiece.hpp"
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

namespace koocadcam::skill::weld_buildup {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.buildup_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "weld_buildup buildup_thickness_mm must be > 0 (got " +
              std::to_string(in.buildup_thickness_mm) + ")");
    } else if (in.buildup_thickness_mm < 0.5) {
        r.add("DFM-BUILDUP-THICK", "error",
              "weld_buildup buildup_thickness " +
              std::to_string(in.buildup_thickness_mm) +
              " mm < 0.5 mm — below practical single-pass bead height");
    } else if (in.buildup_thickness_mm > 10.0) {
        r.add("DFM-BUILDUP-THICK", "error",
              "weld_buildup buildup_thickness " +
              std::to_string(in.buildup_thickness_mm) +
              " mm > 10 mm — multi-pass build with PWHT required; "
              "consider remaking from solid stock instead");
    }

    if (in.material.empty()) {
        r.add("DFM-INPUT", "error", "weld_buildup material must be specified");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Resolve entry face → outward normal → 2-D in-plane bbox of the face →
// build a slab matching that footprint along the normal direction, sunk by
// 1% of thickness for a robust fuse.  Pattern mirrors face_milling but
// reversed (additive instead of subtractive).

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "weld_buildup DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("weld_buildup: entry_face datum unresolved");
    if (!wp.isFacePlanar(*entryId))
        throw SkillError("weld_buildup: entry_face must be planar");

    // Outward normal of the target face (same recipe used by face_milling /
    // spot_weld / mold_rib).
    const gp_Dir outNorm = wp.faceNormal(*entryId);
    const gp_Pnt faceCtr = wp.faceCenter(*entryId);

    // Compute the entry face's 3-D bbox to size the slab footprint.  We
    // pick a slab whose in-plane extent comfortably covers the face's
    // projected footprint.  Slice-1: use the face's own bbox as the
    // footprint.  A more careful version could project the face polygon
    // onto the entry plane and use that polygon as the wire; the bbox
    // approximation is conservative (always covers the face) and simple.
    Bnd_Box faceBb;
    BRepBndLib::AddOptimal(wp.face(*entryId), faceBb);
    double fxMin, fyMin, fzMin, fxMax, fyMax, fzMax;
    faceBb.Get(fxMin, fyMin, fzMin, fxMax, fyMax, fzMax);
    const double fdx = fxMax - fxMin;
    const double fdy = fyMax - fyMin;
    const double fdz = fzMax - fzMin;

    // Build a local frame on the entry plane: xLoc, yLoc ⟂ outNorm.
    // Use Gram-Schmidt starting from gp::DX (or DY if outNorm ≈ ±X).
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
    const gp_Vec yV = outV.Crossed(xV);   // yLoc
    const gp_Dir yLoc(yV);

    // In-plane extents along refDir / yLoc.  For axis-aligned faces these
    // match the bbox dims directly.  For non-aligned faces, the bbox
    // diagonal is a safe over-estimate.
    const double absRx = std::abs(refDir.X()),
                 absRy = std::abs(refDir.Y()),
                 absRz = std::abs(refDir.Z());
    const double absYx = std::abs(yLoc.X()),
                 absYy = std::abs(yLoc.Y()),
                 absYz = std::abs(yLoc.Z());
    // Approximate the in-plane extents by taking the bbox dims weighted by
    // the local axes' contribution.  Conservative: take the largest match.
    auto axisExtent = [&](double ax, double ay, double az) {
        // Pick the bbox dimension most aligned with this axis.
        if (ax > 0.9) return fdx;
        if (ay > 0.9) return fdy;
        if (az > 0.9) return fdz;
        // Skew axis: use the diagonal of the in-plane bbox (over-estimate).
        return std::sqrt(fdx * fdx + fdy * fdy + fdz * fdz);
    };
    double slabLen  = axisExtent(absRx, absRy, absRz);
    double slabWid  = axisExtent(absYx, absYy, absYz);
    if (slabLen < 1e-6) slabLen = 1.0;   // degenerate face — clamp
    if (slabWid < 1e-6) slabWid = 1.0;

    // Sink 1% of buildup_thickness into the parent to keep the fuse robust
    // (same convention as spot_weld).
    const double kSinkPct = 0.01;
    const double sink     = in.buildup_thickness_mm * kSinkPct;
    const double slabH    = in.buildup_thickness_mm + sink;

    // Slab origin: centre on faceCtr, then shift by half-extents inwards in
    // -refDir and -yLoc, and back by `sink` along -outNorm so the slab
    // overlaps the parent face slightly.
    const gp_Pnt slabOrigin(
        faceCtr.X()
            - refDir.X() * (slabLen / 2.0)
            - yLoc.X()   * (slabWid / 2.0)
            - outNorm.X() * sink,
        faceCtr.Y()
            - refDir.Y() * (slabLen / 2.0)
            - yLoc.Y()   * (slabWid / 2.0)
            - outNorm.Y() * sink,
        faceCtr.Z()
            - refDir.Z() * (slabLen / 2.0)
            - yLoc.Z()   * (slabWid / 2.0)
            - outNorm.Z() * sink);

    // gp_Ax2(origin, Z=outNorm, X=refDir):
    //   - DZ extrudes along outNorm   (the slab thickness)
    //   - DX extrudes along refDir    (the slab length)
    //   - DY extrudes along outNorm × refDir = yLoc (the slab width)
    const gp_Ax2 slabAx(slabOrigin, outNorm, refDir);
    const TopoDS_Shape slab = pr::box(slabAx, slabLen, slabWid, slabH);

    // Additive: fuse the slab onto the workpiece.
    const TopoDS_Shape newShape = pr::fuse(wp.shape(), slab);

    json params = {
        { "entry_face_id",          *entryId },
        { "buildup_thickness_mm",   in.buildup_thickness_mm },
        { "material",               in.material },
        { "entry_face_normal",      { outNorm.X(), outNorm.Y(), outNorm.Z() } },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "thickness_mm",           in.buildup_thickness_mm },
        { "material",               in.material },
        { "target_face_id",         *entryId },
        { "entry_face_normal",      { outNorm.X(), outNorm.Y(), outNorm.Z() } },
        { "slab_len_mm",            slabLen },
        { "slab_wid_mm",            slabWid },
        { "additive",               true },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "weld_buildup_torch";
    tooling.tool_dia_mm       = 2.5;          // typical filler wire / MIG nozzle
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "filler_wire";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    // Volume ADDED — negative sign matches spot_weld / mold_rib convention.
    tooling.stock_removed_mm3 = -(slabLen * slabWid * in.buildup_thickness_mm);
    // Rough estimate: 1 cm³ filler ≈ 4 s deposition.
    tooling.est_cycle_time_s  = std::max(5.0,
        slabLen * slabWid * in.buildup_thickness_mm * 0.004);
    tooling.extra = {
        { "process",      "weld_buildup" },
        { "process_family","cladding" },
        { "material",     in.material },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::weld_buildup applied: thick={} face={} slab={}x{} faces {}→{}",
                  in.buildup_thickness_mm, *entryId, slabLen, slabWid,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Primary: replay history (highest confidence).
//
// Fallback heuristic: search for a thin "lid" on top of the workpiece.  A
// buildup slab has a small dz relative to dx/dy and sits adjacent to a
// large planar face.  Because the fuse merges the slab into the parent
// face, the lid face that remains has area comparable to the parent face,
// and its midplane is offset by `thickness` from the parent's original
// plane.  Without a reference "before" shape this is hard to detect with
// high confidence — we emit candidates with confidence 0.4.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // History replay.
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

    // Geometric heuristic: detect thin slab protrusions.  For each pair of
    // parallel planar faces with matching normals, if their separation
    // along that normal is small (< 10 mm) and their areas roughly match,
    // we tentatively report a buildup with confidence 0.4.  This is
    // inherently ambiguous (it can false-positive on any thin block-like
    // workpiece dimension) — high-fidelity recovery comes from the
    // feature-history path above.
    const auto wpBb = pr::optimalBbox(wp.shape());
    if (std::max({ wpBb.dx(), wpBb.dy(), wpBb.dz() }) < 1e-6) return out;

    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir ni;
        try { ni = wp.faceNormal(i); }
        catch (...) { continue; }

        // Skip very small faces — the slab must cover something macro.
        const double areaI = wp.faceArea(i);
        if (areaI < 10.0) continue;

        const gp_Pnt ci = wp.faceCenter(i);

        // Look for a parallel face displaced by a thin gap along ni.
        for (int j = i + 1; j < wp.faceCount(); ++j) {
            if (!wp.isFacePlanar(j)) continue;
            gp_Dir nj;
            try { nj = wp.faceNormal(j); }
            catch (...) { continue; }

            // Parallel or anti-parallel normals accepted.
            const double dot = ni.Dot(nj);
            if (std::abs(std::abs(dot) - 1.0) > 1e-3) continue;

            const gp_Pnt cj = wp.faceCenter(j);
            const gp_Vec sepVec(ci, cj);
            const double dist = std::abs(sepVec.Dot(gp_Vec(ni)));
            if (dist <= 1e-4 || dist > 10.0) continue;     // out of buildup range

            // Sizes roughly match → looks like slab top + bottom.
            const double areaJ = wp.faceArea(j);
            const double areaRatio = (areaJ > 0.0)
                ? std::min(areaI, areaJ) / std::max(areaI, areaJ)
                : 0.0;
            if (areaRatio < 0.5) continue;

            json recovered = {
                { "buildup_thickness_mm",  dist },
                { "material",              "stainless_steel" },
                { "entry_face_id",         i },
                { "entry_face_normal",     { ni.X(), ni.Y(), ni.Z() } },
            };
            json matched = {
                { "parent_face_id",   i },
                { "slab_top_face_id", j },
                { "area_ratio",       areaRatio },
                { "separation_mm",    dist },
            };
            out.push_back(RecognizedFeature{
                kSkillId, recovered, /*confidence*/ 0.40, matched
            });
            break;  // one candidate per parent face is enough
        }
    }
    return out;
}

}  // namespace koocadcam::skill::weld_buildup
