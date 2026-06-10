// @lat: [[engine/skills#groove_turn]]

#include "groove_turn.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
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
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::groove_turn {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.width_mm < 0.5) {
        r.add("DFM-INPUT", "error",
              "groove_turn width_mm (" + std::to_string(in.width_mm) +
              ") < min 0.5 mm (insert kerf)");
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "groove_turn depth_mm must be > 0");
    }
    if (!wp.shape().IsNull()) {
        const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
        const double outerR = bb.outerRadiusXY();
        const double maxDepth = outerR - 1.0;   // keep ≥ 1 mm wall at bottom
        if (in.depth_mm >= maxDepth) {
            r.add("DFM-INPUT", "error",
                  "groove_turn depth_mm (" + std::to_string(in.depth_mm) +
                  ") must be < bbox.outerRadiusXY-1.0 (" +
                  std::to_string(maxDepth) + ")");
        }
        // Groove must be within the stock Z range.
        const double zLow  = in.center_z_mm - in.width_mm / 2.0;
        const double zHigh = in.center_z_mm + in.width_mm / 2.0;
        if (zLow < bb.zMin - 1e-3 || zHigh > bb.zMax + 1e-3) {
            r.add("DFM-INPUT", "error",
                  "groove_turn zone [" + std::to_string(zLow) + "," +
                  std::to_string(zHigh) + "] outside stock Z [" +
                  std::to_string(bb.zMin) + "," + std::to_string(bb.zMax) + "]");
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "groove_turn DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double outerR = bb.outerRadiusXY();
    const double grooveR = outerR - in.depth_mm;

    constexpr double kOverhangR = 0.5;
    const double startZ = in.center_z_mm - in.width_mm / 2.0;
    const double height = in.width_mm;
    const gp_Ax2 ax(gp_Pnt(0.0, 0.0, startZ), gp::DZ());
    const TopoDS_Shape cutter = pr::annularRing(ax,
                                                outerR + kOverhangR,
                                                grooveR,
                                                height);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json params = {
        { "center_z_mm", in.center_z_mm },
        { "width_mm",    in.width_mm },
        { "depth_mm",    in.depth_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     1 },     // groove bottom (smaller radius)
        { "annular_planar_face_count",  2 },     // groove walls
        { "axis",                       { 0.0, 0.0, 1.0 } },
        { "center_z_mm",                in.center_z_mm },
        { "width_mm",                   in.width_mm },
        { "depth_mm",                   in.depth_mm },
        { "groove_radius_mm",           grooveR },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "lathe_groove_insert";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = in.depth_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 1;
    tooling.cutting_speed_sfm = 600.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 =
        M_PI * (outerR * outerR - grooveR * grooveR) * in.width_mm;
    tooling.est_cycle_time_s  = std::max(1.0, in.depth_mm / 8.0);

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::groove_turn applied: center_z={} w={} d={} faces {}→{}",
                  in.center_z_mm, in.width_mm, in.depth_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Pattern: a pair of coaxial planar walls (normal ±Z) with a small Z
// separation, connected by a cylindrical face of smaller radius than the
// stock's outer radius.  The radius of that cylindrical face = (outerR − depth).

namespace {

using EdgeFaceMap = NCollection_IndexedDataMap<
    TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>;

EdgeFaceMap buildEdgeFaceMap(const TopoDS_Shape& shape)
{
    EdgeFaceMap m;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, m);
    return m;
}

bool isPlanarPerpZ(const TopoDS_Face& f)
{
    BRepAdaptor_Surface s(f);
    if (s.GetType() != GeomAbs_Plane) return false;
    return std::abs(std::abs(s.Plane().Axis().Direction().Z()) - 1.0) < 1e-3;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    if (wp.shape().IsNull()) return out;

    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double outerR = bb.outerRadiusXY();
    if (outerR <= 1e-6) return out;

    const auto edgeFaces = buildEdgeFaceMap(wp.shape());

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double radius = cyl.Radius();
        // Must be smaller than stock outer (groove bottom) AND axis along Z.
        if (radius >= outerR - 1e-3) continue;
        const gp_Dir adir = cyl.Axis().Direction();
        if (std::abs(std::abs(adir.Z()) - 1.0) > 1e-3) continue;
        // Axis must pass through the part axis (≈0,0,*).
        const gp_Pnt loc = cyl.Axis().Location();
        if (std::sqrt(loc.X() * loc.X() + loc.Y() * loc.Y()) > 0.5) continue;

        // Two circular bounding edges at distinct Z planes.
        std::vector<gp_Pnt> centers;
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            if (std::abs(std::abs(c.Axis().Direction().Z()) - 1.0) > 1e-3) continue;
            if (std::abs(c.Radius() - radius) > 1e-3) continue;
            centers.push_back(c.Location());
        }
        if (centers.size() < 2) continue;

        auto cmpZ = [](const gp_Pnt& a, const gp_Pnt& b) { return a.Z() < b.Z(); };
        const auto minIt = std::min_element(centers.begin(), centers.end(), cmpZ);
        const auto maxIt = std::max_element(centers.begin(), centers.end(), cmpZ);
        const double zLow  = minIt->Z();
        const double zHigh = maxIt->Z();
        const double width = zHigh - zLow;
        if (width < 1e-3) continue;

        // The cylindrical face must be bounded on BOTH ends by planar walls
        // perpendicular to Z (groove signature).
        int wallCount = 0;
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            if (!edgeFaces.Contains(e)) continue;
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const auto& adj = edgeFaces.FindFromKey(e);
            for (NCollection_List<TopoDS_Shape>::Iterator it(adj); it.More(); it.Next()) {
                const TopoDS_Face& af = TopoDS::Face(it.Value());
                if (af.IsSame(cylFace)) continue;
                if (isPlanarPerpZ(af)) { ++wallCount; break; }
            }
        }
        if (wallCount < 2) continue;

        // Reject if zLow or zHigh coincide with the bbox top/bottom (then
        // the cylindrical face is a turn_external section, not a groove).
        if (std::abs(zLow  - bb.zMin) < 0.1) continue;
        if (std::abs(zHigh - bb.zMax) < 0.1) continue;

        const double center_z = (zLow + zHigh) / 2.0;
        const double depth    = outerR - radius;

        json recovered = {
            { "center_z_mm", center_z },
            { "width_mm",    width },
            { "depth_mm",    depth },
        };
        json matched = {
            { "cylindrical_face_id", fIdx },
            { "groove_radius_mm",    radius },
            { "wall_count",          wallCount },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.90, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::groove_turn
