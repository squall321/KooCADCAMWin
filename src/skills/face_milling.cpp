// @lat: [[engine/skills#face_milling]]

#include "face_milling.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::face_milling {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "face_milling depth must be > 0");
    }
    if (in.depth_mm >= 5.0) {
        r.add("FM-DEPTH", "warning",
              "face_milling depth " + std::to_string(in.depth_mm) +
              " mm ≥ 5 mm — single-pass face mill normally limited to ≤ 5 mm");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    // 1) DFM gate
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "face_milling DFM failed:";
        for (const auto& f : dfm.findings) {
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        }
        throw SkillError(msg);
    }

    // 2) Resolve entry face
    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("face_milling: entry_face datum unresolved");
    if (!wp.isFacePlanar(*entryId))
        throw SkillError("face_milling: entry_face must be planar");

    // 3) Compute skim-box geometry
    //
    //    The skim cutter is a box that:
    //      - covers the workpiece's full in-plane footprint with margin,
    //      - extends from (entry_face plane + small overhang) inward by
    //        depth_mm along the inward normal.
    //
    //    Construction: pick refDir perpendicular to outNorm, then compute
    //    refDir2 = outNorm × refDir.  Origin = faceCtr − pad·refDir
    //              − pad·refDir2 + overhang·outNorm.  Then a gp_Ax2 with
    //    main = -outNorm, X = refDir produces a box that cuts inward.
    const gp_Dir outNorm = wp.faceNormal(*entryId);
    const gp_Pnt faceCtr = wp.faceCenter(*entryId);

    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double bboxDiag = std::sqrt(bb.dx() * bb.dx()
                                    + bb.dy() * bb.dy()
                                    + bb.dz() * bb.dz());

    // Box footprint: cover the bbox diagonal to be safe.
    const double pad = bboxDiag * 1.1;

    // Pick local X (refDir) perpendicular to outNorm.
    gp_Dir refDir = gp::DX();
    if (std::abs(outNorm.Dot(refDir)) > 0.9) refDir = gp::DY();
    // Make refDir strictly perpendicular to outNorm via Gram-Schmidt.
    {
        gp_Vec rv(refDir);
        const gp_Vec nv(outNorm);
        rv -= nv * rv.Dot(nv);
        if (rv.Magnitude() < 1e-9) rv = gp_Vec(gp::DY());
        rv.Normalize();
        refDir = gp_Dir(rv);
    }

    // The box cuts INTO the material along -outNorm.  In the box's local
    // frame (Ax2 with main = cutMain, X = refDir), Y = cutMain × refDir.
    // To make the box straddle the workpiece we put its origin at
    //   faceCtr  + overhang·outNorm        (lift slightly above the plane)
    //           − pad·refDir               (shift -X_local by pad)
    //           − pad·(cutMain × refDir)   (shift -Y_local by pad)
    // so the box then extends 2·pad in +X_local, 2·pad in +Y_local and
    // depth+overhang in +Z_local (= -outNorm = into the material).
    const gp_Dir cutMain(-outNorm.X(), -outNorm.Y(), -outNorm.Z());
    const gp_Vec cutMainVec(cutMain);
    const gp_Vec refVec(refDir);
    const gp_Vec yLocalVec = cutMainVec.Crossed(refVec);   // = local-Y direction
    const gp_Dir yLocal(yLocalVec);

    const double overhang = 0.01;
    const gp_Pnt boxOrigin(
        faceCtr.X() + outNorm.X() * overhang
                    - refDir.X() * pad - yLocal.X() * pad,
        faceCtr.Y() + outNorm.Y() * overhang
                    - refDir.Y() * pad - yLocal.Y() * pad,
        faceCtr.Z() + outNorm.Z() * overhang
                    - refDir.Z() * pad - yLocal.Z() * pad);

    // gp_Ax2(origin, Z=cutMain=-outNorm, X=refDir) → DZ extends into material.
    const gp_Ax2 boxAx(boxOrigin, cutMain, refDir);

    const TopoDS_Shape skimBox = pr::box(boxAx,
                                         2.0 * pad,         // along refDir
                                         2.0 * pad,         // along yLocal
                                         in.depth_mm + overhang);  // along cutMain

    // 4) Cut
    const TopoDS_Shape newShape = pr::cut(wp.shape(), skimBox);

    // 5) Signature
    json params = {
        { "entry_face_id", *entryId },
        { "entry_face_normal", { outNorm.X(), outNorm.Y(), outNorm.Z() } },
        { "depth_mm",      in.depth_mm },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "parallel_face_present", true },
        { "depth_mm",              in.depth_mm },
        { "entry_face_normal",     { outNorm.X(), outNorm.Y(), outNorm.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";   // a face-mill is a special end mill
    tooling.tool_dia_mm       = 25.0;         // typical 1" face mill
    tooling.tool_length_mm    = 30.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 5;
    tooling.cutting_speed_sfm = 600.0;        // aggressive aluminum
    tooling.feed_per_tooth_mm = 0.10;
    // Stock removed ≈ entry face area × depth.  We use the entry-face area
    // as a lower bound (true skim removes a thin slab of that area).
    GProp_GProps props;
    BRepGProp::SurfaceProperties(wp.face(*entryId), props);
    tooling.stock_removed_mm3 = props.Mass() * in.depth_mm;
    tooling.est_cycle_time_s  = std::max(2.0, props.Mass() / 1000.0);  // rough

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::face_milling applied: depth={} faces {}→{}",
                  in.depth_mm, wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// face_milling does NOT change the bbox footprint in the in-plane
// directions: the new entry face has the same XY projection as the bbox.
// So the recognition heuristic is "find the largest planar face whose
// projected area matches the bbox cross-section in that face's normal
// direction."  Without a prior reference, depth_mm cannot be recovered;
// we report 0.0 as a sentinel and confidence 0.40.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());

    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        const double area = wp.faceArea(i);
        gp_Dir n(0.0, 0.0, 1.0);
        try { n = wp.faceNormal(i); }
        catch (...) { continue; }

        // Projected bbox area perpendicular to n.  For axis-aligned faces
        // this is dx*dy / dy*dz / dz*dx.
        const double ax = std::abs(n.X());
        const double ay = std::abs(n.Y());
        const double az = std::abs(n.Z());
        double bboxArea = 0.0;
        if (az > 0.9) bboxArea = bb.dx() * bb.dy();
        else if (ay > 0.9) bboxArea = bb.dx() * bb.dz();
        else if (ax > 0.9) bboxArea = bb.dy() * bb.dz();
        else {
            // Skew face — approximate via the largest pair of bbox dims.
            const double largest = std::max({ bb.dx() * bb.dy(),
                                              bb.dx() * bb.dz(),
                                              bb.dy() * bb.dz() });
            bboxArea = largest;
        }
        if (bboxArea < 1e-6) continue;

        // The face's area should match the bbox cross-section within 10 %.
        if (std::abs(area - bboxArea) / bboxArea > 0.10) continue;

        json recovered = {
            { "entry_face_id",     i },
            { "entry_face_normal", { n.X(), n.Y(), n.Z() } },
            // We cannot recover the absolute depth without prior state.
            { "depth_mm",          0.0 },
        };
        json matched = {
            { "face_id",   i },
            { "face_area", area },
            { "bbox_area", bboxArea },
        };
        out.push_back(RecognizedFeature{
            kSkillId, recovered, /*confidence*/ 0.40, matched
        });
    }
    return out;
}

}  // namespace koocadcam::skill::face_milling
