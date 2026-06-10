// @lat: [[engine/skills#line_boring]]

#include "line_boring.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRep_Tool.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pln.hxx>

#include <limits>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::line_boring {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "line_boring diameter must be > 0");
    }

    if (in.dia_mm > 0.0 && in.dia_mm < 6.0) {
        r.add("DFM-LINEBORE-DIA", "error",
              "line_boring diameter " + std::to_string(in.dia_mm) +
              " mm < 6 mm — boring bars below this lack rigidity for line boring");
    }

    if (in.bore_positions_z_mm.size() < 2) {
        r.add("DFM-LINEBORE-COUNT", "error",
              "line_boring requires ≥ 2 bore positions (got " +
              std::to_string(in.bore_positions_z_mm.size()) +
              ") — single position degenerates to bore_cylindrical");
    }

    if (in.segment_overhang_mm < 0.0) {
        r.add("DFM-INPUT", "warning",
              "segment_overhang_mm < 0 is unusual; clamping to 0");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "line_boring DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Resolve the boring-bar axis face (the cylindrical face that already
    // exists on the workpiece is optional — line_boring may also be applied
    // to a pristine block).  We attempt resolution but tolerate failure.
    // Wrap FaceCylinderByAxis into the FaceDatum variant so resolve() can
    // dispatch via std::visit.
    auto axisFaceId = wp.resolve(FaceDatum{ in.axis });

    const gp_Ax1 ax       = in.axis.axis;
    const gp_Dir adir     = ax.Direction();
    const gp_Pnt axOrigin = ax.Location();

    // Axis-local Z range = [min(bore_positions) - overhang, max + overhang].
    auto [minIt, maxIt] = std::minmax_element(
        in.bore_positions_z_mm.begin(), in.bore_positions_z_mm.end());
    const double overhang = std::max(0.0, in.segment_overhang_mm);
    const double zLo = *minIt - overhang;
    const double zHi = *maxIt + overhang;
    const double segLen = zHi - zLo;
    if (segLen <= 0.0) {
        throw SkillError("line_boring: span is zero — bore_positions collapsed");
    }

    // Start point in world space: axOrigin + zLo × adir.
    const gp_Pnt cylStart(
        axOrigin.X() + zLo * adir.X(),
        axOrigin.Y() + zLo * adir.Y(),
        axOrigin.Z() + zLo * adir.Z());

    const gp_Ax2 toolAx(cylStart, adir);
    const TopoDS_Shape cutter = pr::cylinder(toolAx, in.dia_mm / 2.0, segLen);

    // Slice-1 simplification: a SINGLE cylinder spanning all positions.
    // This produces 1 cylindrical face on the workpiece, not 3 separated
    // bore segments.  See header comment.
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json bp = json::array();
    for (double z : in.bore_positions_z_mm) bp.push_back(z);

    json params = {
        { "axis_origin",          { axOrigin.X(), axOrigin.Y(), axOrigin.Z() } },
        { "axis_dir",             { adir.X(), adir.Y(), adir.Z() } },
        { "dia_mm",               in.dia_mm },
        { "bore_positions_z_mm",  bp },
        { "segment_overhang_mm",  overhang },
    };
    if (axisFaceId) params["axis_face_id"] = *axisFaceId;

    json pattern = {
        { "kind",                   kSkillId },
        { "cylindrical_face_count", 1 },   // slice-1 single through cylinder
        { "circular_edge_count",    2 },
        { "dia_mm",                 in.dia_mm },
        { "bore_positions",         bp },
        { "span_axial_mm",          segLen },
        { "axis_dir",               { adir.X(), adir.Y(), adir.Z() } },
    };

    // Line-boring tooling: long, rigid boring bar — slow feed, modest SFM.
    ToolingMeta tooling;
    tooling.tool_type        = "boring_bar";
    tooling.tool_dia_mm      = in.dia_mm;
    tooling.tool_length_mm   = segLen + 50.0;       // bar must clear the workpiece
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 1;                   // single-point insert
    tooling.cutting_speed_sfm = 90.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = M_PI * (in.dia_mm / 2.0) * (in.dia_mm / 2.0) * segLen;
    tooling.est_cycle_time_s  = std::max(10.0, segLen / 10.0);
    tooling.extra = json{
        { "bore_positions_z_mm",  bp },
        { "segment_overhang_mm",  overhang },
        { "boss_count",           static_cast<int>(in.bore_positions_z_mm.size()) },
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

    spdlog::debug("skill::line_boring applied: dia={} span={} positions={} faces {}→{}",
                  in.dia_mm, segLen, in.bore_positions_z_mm.size(),
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Strategy: find a cylindrical face whose axial span is ≥ 2 × dia AND
// whose body crosses at least 2 distinct planar walls (the bosses).
//
// "Crossing a planar wall" is detected by counting the planar faces
// perpendicular to the cylinder axis that lie *between* the two extreme
// circular edges of the cylinder.  We treat each such plane as a boss
// signature.
//
// In slice-1 the synthesis cuts a single continuous through-cylinder, so
// for a typical 3-boss workpiece we expect to see 1 cylindrical face and
// at least 2 boss planes (the inner walls).  Confidence 0.5 reflects the
// ambiguity with a plain through-hole when the planar-wall heuristic
// reports < 2 walls (e.g. on a continuous block).

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
        const double diameter = 2.0 * radius;

        if (diameter < 6.0) continue;   // line-boring minimum

        std::vector<gp_Pnt> circleCenters;
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            if (std::abs(std::abs(c.Axis().Direction().Dot(axis.Direction())) - 1.0) > 1e-3)
                continue;
            if (std::abs(c.Radius() - radius) > 1e-3) continue;
            circleCenters.push_back(c.Location());
        }
        if (circleCenters.size() < 2) continue;

        const gp_Dir adir = axis.Direction();
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
        const gp_Pnt cLow  = *minIt;
        const gp_Pnt cHigh = *maxIt;
        const double span = cHigh.Distance(cLow);

        // Need a long cylinder (span ≥ 2 × dia) to plausibly span > 1 boss.
        if (span < 2.0 * diameter) continue;

        // Count distinct planar walls perpendicular to the axis whose plane
        // crosses the axis between cLow and cHigh.  These are the boss
        // partition planes.
        const double tLow  = projOnAxis(cLow);
        const double tHigh = projOnAxis(cHigh);

        std::vector<double> bossPlanes;
        for (int pIdx = 0; pIdx < wp.faceCount(); ++pIdx) {
            if (pIdx == fIdx) continue;
            if (!wp.isFacePlanar(pIdx)) continue;
            BRepAdaptor_Surface ps(wp.face(pIdx));
            const gp_Pln pl = ps.Plane();
            const gp_Dir nrm = pl.Axis().Direction();
            // Perpendicular to cyl axis = parallel to plane normal.
            if (std::abs(std::abs(nrm.Dot(adir)) - 1.0) > 1e-3) continue;
            const gp_Pnt loc = pl.Location();
            // Axial-position of this plane on the cyl axis.
            const double t = (loc.X() - axis.Location().X()) * adir.X() +
                             (loc.Y() - axis.Location().Y()) * adir.Y() +
                             (loc.Z() - axis.Location().Z()) * adir.Z();
            // Inside the span (strictly between extreme circles, with a tol).
            const double pad = 0.5;
            if (t < tLow + pad || t > tHigh - pad) continue;
            // Deduplicate planes at near-identical axial positions.
            bool dup = false;
            for (double existing : bossPlanes) {
                if (std::abs(existing - t) < 0.5) { dup = true; break; }
            }
            if (!dup) bossPlanes.push_back(t);
        }

        // bossPlanes.size() ≥ 1 = at least 2 bosses crossed (the planes are
        // the *interior* walls between bosses).  For 3 bosses we expect 2 or
        // 4 interior planes (front+back of each web).
        const int interiorWalls = static_cast<int>(bossPlanes.size());

        // Reconstruct nominal bore positions for downstream consumers.
        // Without boss thickness info we approximate the positions as evenly
        // spaced across the span; the recovered values match the synthesis
        // convention when the original positions were evenly distributed.
        const int nominalBoreCount = std::max(2, (interiorWalls / 2) + 1);
        json bp = json::array();
        for (int i = 0; i < nominalBoreCount; ++i) {
            const double z = tLow + (span * (i + 0.5) / nominalBoreCount);
            bp.push_back(z);
        }

        // Single-boss / continuous-stock case: keep low confidence; this is
        // basically a through_hole and the recognizer just couldn't tell.
        // Multi-boss case: still confidence 0.5 because the line_boring
        // synthesis in slice-1 is geometrically indistinguishable from a
        // straight through-hole through pre-bossed stock.
        const double confidence = 0.5;

        gp_Vec drillVec(cHigh, cLow);
        if (drillVec.Magnitude() < 1e-6) continue;
        drillVec.Normalize();

        json recovered = {
            { "axis_origin",         { axis.Location().X(), axis.Location().Y(), axis.Location().Z() } },
            { "axis_dir",            { drillVec.X(), drillVec.Y(), drillVec.Z() } },
            { "dia_mm",              diameter },
            { "bore_positions_z_mm", bp },
            { "segment_overhang_mm", 2.0 },
        };
        json matched = {
            { "cylindrical_face_id", fIdx },
            { "span_axial_mm",       span },
            { "interior_walls",      interiorWalls },
            { "top_center",  { cHigh.X(), cHigh.Y(), cHigh.Z() } },
            { "bot_center",  { cLow.X(),  cLow.Y(),  cLow.Z()  } },
        };
        out.push_back(RecognizedFeature{
            kSkillId, recovered, confidence, matched
        });
    }
    return out;
}

}  // namespace koocadcam::skill::line_boring
