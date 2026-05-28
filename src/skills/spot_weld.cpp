// @lat: [[engine/skills#spot_weld]]

#include "spot_weld.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <gp_Cylinder.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace koocadcam::skill::spot_weld {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.weld_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "spot_weld weld_dia_mm must be > 0");
    }
    if (in.weld_height_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "spot_weld weld_height_mm must be > 0");
    }
    if (in.weld_dia_mm > 0.0 && in.weld_dia_mm < 3.0) {
        r.add("DFM-WELD-DIA", "error",
              "spot_weld weld_dia_mm " + std::to_string(in.weld_dia_mm) +
              " < 3 mm (electrode tip too small for stable nugget)");
    }
    if (in.weld_dia_mm > 12.0) {
        r.add("DFM-WELD-DIA", "error",
              "spot_weld weld_dia_mm " + std::to_string(in.weld_dia_mm) +
              " > 12 mm (use seam_weld for larger joins)");
    }
    if (in.weld_dia_mm > 0.0 && in.weld_height_mm > in.weld_dia_mm * 0.4) {
        r.add("DFM-WELD-RATIO", "error",
              "spot_weld weld_height " + std::to_string(in.weld_height_mm) +
              " > 0.4 × weld_dia " + std::to_string(in.weld_dia_mm) +
              " — multi-pass build-up, not a single spot");
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
        std::string msg = "spot_weld DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // 2) Resolve entry face
    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("spot_weld: entry_face datum unresolved");
    if (!wp.isFacePlanar(*entryId))
        throw SkillError("spot_weld: entry_face must be planar");

    // 3) Build a SHORT cylinder bead.  Outward normal from the face is the
    //    cylinder's main axis; the bead origin sits exactly on the face so
    //    the cylinder starts at the face and extends outward by weld_height.
    //
    //    OCCT note: a Boolean fuse of a body that just touches the workpiece
    //    surface is fragile — even with the cylinder base coincident with the
    //    planar face the union sometimes leaves a sliver edge.  To make the
    //    fuse robust we sink the bead 1% of its height into the workpiece so
    //    there is a real shared volume to merge across.
    const gp_Dir outNorm = wp.faceNormal(*entryId);
    const gp_Pnt faceCtr = wp.faceCenter(*entryId);

    // Project (position_x, position_y) onto the entry face plane along the
    // face normal.  We use the same XY/Z convention as drill_hole / mill_slot
    // (entry_face is typically the +Z top).  For top-face placement we can
    // use the position directly; for arbitrary faces we offset along refDir.
    gp_Pnt bumpBase;
    if (std::abs(outNorm.X()) < 1e-6 && std::abs(outNorm.Y()) < 1e-6) {
        // Common case — top or bottom face, plane Z == faceCtr.Z()
        bumpBase = gp_Pnt(in.position_x_mm, in.position_y_mm, faceCtr.Z());
    } else {
        bumpBase = gp_Pnt(in.position_x_mm, in.position_y_mm, faceCtr.Z());
    }

    const double kSink = in.weld_height_mm * 0.01;  // small overlap for robust fuse
    const double cylH  = in.weld_height_mm + kSink;

    // Cylinder grows along outNorm; the origin sits kSink BELOW the face so
    // the bead emerges through it.
    gp_Pnt cylOrigin(
        bumpBase.X() - outNorm.X() * kSink,
        bumpBase.Y() - outNorm.Y() * kSink,
        bumpBase.Z() - outNorm.Z() * kSink);
    const gp_Ax2 cylAx(cylOrigin, outNorm);

    const TopoDS_Shape bead = pr::cylinder(cylAx, in.weld_dia_mm / 2.0, cylH);

    // 4) Fuse onto workpiece (ADDITIVE)
    const TopoDS_Shape newShape = pr::fuse(wp.shape(), bead);

    // 5) Build signature
    json params = {
        { "entry_face_id",   *entryId },
        { "position_x_mm",   in.position_x_mm },
        { "position_y_mm",   in.position_y_mm },
        { "weld_dia_mm",     in.weld_dia_mm },
        { "weld_height_mm",  in.weld_height_mm },
        { "material",        in.material },
        { "entry_face_normal", { outNorm.X(), outNorm.Y(), outNorm.Z() } },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "cylindrical_face_count", 1 },
        { "circular_edge_count",    2 },
        { "top_planar_disc_present", true },
        { "weld_dia_mm",            in.weld_dia_mm },
        { "weld_height_mm",         in.weld_height_mm },
        { "additive",               true },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "resistance_welder";
    tooling.tool_dia_mm       = in.weld_dia_mm;
    tooling.tool_length_mm    = 0.0;              // no cutting length — electrode tip
    tooling.tool_material     = "copper_alloy";   // typ. electrode
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    // Volume ADDED, not removed — record as negative so accounting still works.
    tooling.stock_removed_mm3 = -(M_PI * (in.weld_dia_mm / 2.0) *
                                  (in.weld_dia_mm / 2.0) * in.weld_height_mm);
    tooling.est_cycle_time_s  = 1.0;              // resistance welds are FAST
    tooling.extra = {
        { "process",    "resistance_spot_weld" },
        { "material",   in.material },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::spot_weld applied: dia={} h={} faces {}→{}",
                  in.weld_dia_mm, in.weld_height_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Pattern: cylindrical face with VERY SMALL height/diameter ratio
//          (< 0.5) bounded by a small disc on top + a large planar face on
//          the bottom.  We discriminate from drill_hole by checking the
//          BOTTOM face area — for a bump, the larger face is at the BASE
//          (sharing the workpiece face).

namespace {

using EdgeFaceMap = NCollection_IndexedDataMap<
    TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>;

EdgeFaceMap buildEdgeFaceMap(const TopoDS_Shape& shape)
{
    EdgeFaceMap m;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, m);
    return m;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // First, metadata replay — full confidence if we already have a signature.
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

    // Geometric recognition: protruding low-aspect cylindrical bumps.
    const auto edgeFaces = buildEdgeFaceMap(wp.shape());

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;

        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double radius = cyl.Radius();
        const double dia    = 2.0 * radius;
        if (dia > 12.0) continue;  // larger than spot-weld max
        if (dia < 2.5)  continue;  // smaller than electrode tip

        const gp_Ax1 axis = cyl.Axis();
        const gp_Dir adir = axis.Direction();

        // Collect the two bounding circles.
        std::vector<gp_Pnt> circleCenters;
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            if (std::abs(std::abs(c.Axis().Direction().Dot(adir)) - 1.0) > 1e-3)
                continue;
            if (std::abs(c.Radius() - radius) > 1e-3) continue;
            circleCenters.push_back(c.Location());
        }
        if (circleCenters.size() < 2) continue;

        auto projOnAxis = [&](const gp_Pnt& p) {
            return (p.X() - axis.Location().X()) * adir.X() +
                   (p.Y() - axis.Location().Y()) * adir.Y() +
                   (p.Z() - axis.Location().Z()) * adir.Z();
        };
        auto cmp = [&](const gp_Pnt& a, const gp_Pnt& b) {
            return projOnAxis(a) < projOnAxis(b);
        };
        const auto minIt = std::min_element(circleCenters.begin(), circleCenters.end(), cmp);
        const auto maxIt = std::max_element(circleCenters.begin(), circleCenters.end(), cmp);
        const gp_Pnt centerLow  = *minIt;
        const gp_Pnt centerHigh = *maxIt;
        const double height = centerHigh.Distance(centerLow);
        if (height <= 1e-6) continue;

        // Aspect ratio gate: spot weld has h/dia < 0.5 (and DFM requires ≤ 0.4).
        if (height / dia > 0.5) continue;

        // Distinguish ADDITIVE bump from drill_hole by examining adjacent
        // planar faces.  For a BUMP: one adjacent face is the LARGE base
        // workpiece face, the other is a SMALL disc (≈ π r²).  For a DRILL:
        // the small disc is the BOTTOM (inside) and the large face is the
        // workpiece TOP (outside).  Both have the same area pattern.  So we
        // additionally check which circle (top vs bottom along axis) is
        // adjacent to the LARGE face: for a bump, it is the LOW one (the
        // base sitting flush with the parent face).
        const double discArea = M_PI * radius * radius;
        double lowCircleAdjArea  = 0.0;
        double highCircleAdjArea = 0.0;

        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            if (!edgeFaces.Contains(e)) continue;
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Pnt cLoc = crv.Circle().Location();
            const bool isLow = (projOnAxis(cLoc) - projOnAxis(centerLow)) <
                               (projOnAxis(centerHigh) - projOnAxis(cLoc));
            double bestArea = 0.0;
            const auto& adj = edgeFaces.FindFromKey(e);
            for (NCollection_List<TopoDS_Shape>::Iterator it(adj); it.More(); it.Next()) {
                const TopoDS_Face& af = TopoDS::Face(it.Value());
                if (af.IsSame(cylFace)) continue;
                if (BRepAdaptor_Surface(af).GetType() != GeomAbs_Plane) continue;
                GProp_GProps gp;
                BRepGProp::SurfaceProperties(af, gp);
                bestArea = std::max(bestArea, gp.Mass());
            }
            if (isLow) lowCircleAdjArea  = std::max(lowCircleAdjArea, bestArea);
            else       highCircleAdjArea = std::max(highCircleAdjArea, bestArea);
        }
        // Bump pattern: LOW circle sees the LARGE workpiece face, HIGH circle
        // sees the small top disc (≈ discArea).
        const bool isBump = (lowCircleAdjArea  > discArea * 1.5) &&
                            (highCircleAdjArea > 0.0 &&
                             highCircleAdjArea < discArea * 1.5);
        if (!isBump) continue;

        // Position: bump base = the LOW circle centre projected to (x,y).
        // Bead axis points from LOW → HIGH (outward).
        gp_Vec dirVec(centerLow, centerHigh);
        if (dirVec.Magnitude() < 1e-9) continue;
        dirVec.Normalize();

        json recovered = {
            { "position_x_mm",  centerLow.X() },
            { "position_y_mm",  centerLow.Y() },
            { "weld_dia_mm",    dia },
            { "weld_height_mm", height },
            { "material",       "carbon_steel" },
            { "entry_face_normal", { dirVec.X(), dirVec.Y(), dirVec.Z() } },
        };
        json matched = {
            { "cylindrical_face_id",  fIdx },
            { "base_center",          { centerLow.X(),  centerLow.Y(),  centerLow.Z()  } },
            { "top_center",           { centerHigh.X(), centerHigh.Y(), centerHigh.Z() } },
            { "aspect_ratio",         height / dia },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.75, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::spot_weld
