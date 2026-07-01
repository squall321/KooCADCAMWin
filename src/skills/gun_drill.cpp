// @lat: [[engine/skills#gun_drill]]

#include "gun_drill.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <gp_Cylinder.hxx>

#include <limits>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::gun_drill {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.diameter_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "gun_drill diameter must be > 0");
    }
    if (!in.through_hole && in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "gun_drill blind depth must be > 0");
    }

    // Gun-drill diameter envelope.
    if (in.diameter_mm > 0.0 && in.diameter_mm < 1.0) {
        r.add("DFM-GUNDRILL-DIAMIN", "error",
              "gun_drill diameter " + std::to_string(in.diameter_mm) +
              " mm < min 1.0 mm — gun drills do not exist below this");
    }
    if (in.diameter_mm > 50.0) {
        r.add("DFM-GUNDRILL-DIAMAX", "error",
              "gun_drill diameter " + std::to_string(in.diameter_mm) +
              " mm > max 50.0 mm — gun drills are not produced this large");
    }

    // depth/dia ratio range — the whole point of the tool.
    const double ratio = (in.diameter_mm > 0.0 && in.depth_mm > 0.0)
                       ? (in.depth_mm / in.diameter_mm)
                       : 0.0;
    if (!in.through_hole && in.depth_mm > 0.0) {
        if (ratio < 10.0) {
            r.add("DFM-GUNDRILL-RATIO", "warning",
                  "depth/dia ratio " + std::to_string(ratio) +
                  " < 10 — gun drill not justified; use standard drill_hole");
        } else if (ratio > 100.0) {
            r.add("DFM-GUNDRILL-RATIO", "warning",
                  "depth/dia ratio " + std::to_string(ratio) +
                  " > 100 — exceeds typical gun drill range; check tool length");
        }
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "gun_drill DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("gun_drill: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    const double kEntryOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    // Entry point = where the axis pierces the resolved entry-face plane (any
    // axis; the shared helper replaces the old bbox-pullback that mis-placed the
    // launch point and stamped no entry Z).
    const gp_Pnt entryPt =
        entryPointOnFacePlane(wp, *entryId, in.position_x_mm, in.position_y_mm,
                              adir, zMin, zMax);
    const gp_Pnt toolStart(entryPt.X() - adir.X() * kEntryOverhang,
                           entryPt.Y() - adir.Y() * kEntryOverhang,
                           entryPt.Z() - adir.Z() * kEntryOverhang);

    const double toolHeight = in.through_hole
        ? (bboxDiag + 2.0 * kEntryOverhang)
        : (in.depth_mm + kEntryOverhang);

    const gp_Ax2 toolAx(toolStart, adir);
    const TopoDS_Shape cutter = pr::cylinder(toolAx, in.diameter_mm / 2.0, toolHeight);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    const double ratio = (in.diameter_mm > 0.0) ? (in.depth_mm / in.diameter_mm) : 0.0;

    json params = {
        { "entry_face_kind",     "resolved_id" },
        { "entry_face_id",       *entryId },
        { "position_x_mm",       in.position_x_mm },
        { "position_y_mm",       in.position_y_mm },
        { "position_z_mm",       entryPt.Z() },
        { "axis_dir",            { adir.X(), adir.Y(), adir.Z() } },
        { "diameter_mm",         in.diameter_mm },
        { "depth_mm",            in.depth_mm },
        { "through_hole",        in.through_hole },
        { "straightness_class",  in.straightness_class },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     1 },
        { "circular_edge_count",        2 },
        { "bottom_planar_face_present", !in.through_hole },
        { "diameter_mm",                in.diameter_mm },
        { "depth_mm",                   in.depth_mm },
        { "depth_dia_ratio",            ratio },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
        { "straightness_class",         in.straightness_class },
    };

    // Gun-drill tooling: single flute, through-tool coolant, slow feed.
    ToolingMeta tooling;
    tooling.tool_type        = "gun_drill";
    tooling.tool_dia_mm      = in.diameter_mm;
    tooling.tool_length_mm   = in.depth_mm * 1.1 + 25.0;   // gun drills are long
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 1;            // single flute by definition
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.015;       // very slow feed for straightness
    tooling.stock_removed_mm3 = M_PI * (in.diameter_mm / 2.0) * (in.diameter_mm / 2.0)
                              * in.depth_mm;
    tooling.est_cycle_time_s  = std::max(5.0, in.depth_mm / 20.0);
    tooling.extra = json{
        { "through_coolant",    true },
        { "straightness_class", in.straightness_class },
        { "depth_dia_ratio",    ratio },
    };

    FeatureSignature sig {
        kSkillId,
        params,
        pattern,
        tooling,
    };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::gun_drill applied: dia={} depth={} ratio={} faces {}→{}",
                  in.diameter_mm, in.depth_mm, ratio,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Strategy: same topology scan as drill_hole, but only emit a candidate
// when the recovered depth/diameter ratio is ≥ 10 (the gun-drill regime).
// Confidence 0.6 — overlapping with drill_hole and deep_hole_peck, which
// can match the same topology.

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
    const TopoDS_Shape& shape = wp.shape();
    const auto edgeFaces = buildEdgeFaceMap(shape);

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;

        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double radius = cyl.Radius();
        const gp_Ax1 axis = cyl.Axis();

        // Collect ring edges + the largest planar face each opens onto.  Entry is
        // resolved by ADJACENCY, never by axis-projection order — OCCT's cylinder
        // axis sign is arbitrary, so the max-projection ring is the blind bottom
        // (not the mouth) for a -Z-stored axis.  Mirrors drill_hole.
        const double radTol = std::max(1e-4, 1e-3 * radius);
        struct Ring { gp_Pnt center; double adjPlanarArea; };
        std::vector<Ring> rings;
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            if (std::abs(std::abs(c.Axis().Direction().Dot(axis.Direction())) - 1.0) > 1e-3)
                continue;
            if (std::abs(c.Radius() - radius) > radTol) continue;
            double adjMax = 0.0;
            if (edgeFaces.Contains(e)) {
                const auto& adj = edgeFaces.FindFromKey(e);
                for (NCollection_List<TopoDS_Shape>::Iterator it(adj); it.More(); it.Next()) {
                    const TopoDS_Face& af = TopoDS::Face(it.Value());
                    if (af.IsSame(cylFace)) continue;
                    if (BRepAdaptor_Surface(af).GetType() != GeomAbs_Plane) continue;
                    GProp_GProps gp; BRepGProp::SurfaceProperties(af, gp);
                    adjMax = std::max(adjMax, gp.Mass());
                }
            }
            rings.push_back({ c.Location(), adjMax });
        }
        if (rings.size() < 2) continue;

        const gp_Dir adir = axis.Direction();
        auto projOnAxis = [&](const gp_Pnt& p) {
            return (p.X() - axis.Location().X()) * adir.X() +
                   (p.Y() - axis.Location().Y()) * adir.Y() +
                   (p.Z() - axis.Location().Z()) * adir.Z();
        };
        const Ring& loRing = *std::min_element(rings.begin(), rings.end(),
            [&](const Ring& a, const Ring& b){ return projOnAxis(a.center) < projOnAxis(b.center); });
        const Ring& hiRing = *std::max_element(rings.begin(), rings.end(),
            [&](const Ring& a, const Ring& b){ return projOnAxis(a.center) < projOnAxis(b.center); });
        const double depth    = hiRing.center.Distance(loRing.center);
        const double diameter = 2.0 * radius;
        if (depth < 1e-6) continue;

        // Entry vs blind-bottom from adjacency (mirrors drill_hole).
        const double drillBottomArea = M_PI * radius * radius;
        const bool loIsBottom = loRing.adjPlanarArea > 0.0 && loRing.adjPlanarArea <= drillBottomArea * 1.5;
        const bool hiIsBottom = hiRing.adjPlanarArea > 0.0 && hiRing.adjPlanarArea <= drillBottomArea * 1.5;
        const bool through    = !loIsBottom && !hiIsBottom;

        const Ring* entry = &hiRing;
        const Ring* other = &loRing;
        if (loIsBottom)                                       { entry = &hiRing; other = &loRing; }
        else if (hiIsBottom)                                  { entry = &loRing; other = &hiRing; }
        else if (loRing.adjPlanarArea > hiRing.adjPlanarArea) { entry = &loRing; other = &hiRing; }

        gp_Vec drillVec(entry->center, other->center);
        if (drillVec.Magnitude() < 1e-6) continue;
        drillVec.Normalize();

        // Gun-drill signature: depth/dia ≥ 10 AND dia in [1, 50] AND blind
        // (gun drills can also produce through-holes, but recognition for
        // through-holes is highly ambiguous with drill_through_hole).
        const double ratio = (diameter > 0.0) ? (depth / diameter) : 0.0;
        const bool ratioOk = ratio >= 10.0;
        const bool diaOk   = diameter >= 1.0 && diameter <= 50.0;
        if (!ratioOk || !diaOk) continue;

        json recovered = {
            { "position_x_mm",       entry->center.X() },
            { "position_y_mm",       entry->center.Y() },
            { "position_z_mm",       entry->center.Z() },   // the mouth — CAM machines from the real surface
            { "axis_dir",            { drillVec.X(), drillVec.Y(), drillVec.Z() } },
            { "diameter_mm",         diameter },
            { "depth_mm",            through ? 0.0 : depth },
            { "through_hole",        through },
            { "straightness_class",  std::string("0.05_per_100mm") },
        };
        json matched = {
            { "cylindrical_face_id", fIdx },
            { "top_center", { entry->center.X(), entry->center.Y(), entry->center.Z() } },
            { "bot_center", { other->center.X(), other->center.Y(), other->center.Z() } },
            { "depth_dia_ratio", ratio },
        };
        out.push_back(RecognizedFeature{
            kSkillId, recovered, /*confidence*/ 0.6, matched
        });
    }
    return out;
}

}  // namespace koocadcam::skill::gun_drill
