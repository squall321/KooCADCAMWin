// @lat: [[engine/skills#turn_external]]

#include "turn_external.hpp"

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

namespace koocadcam::skill::turn_external {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.final_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "turn_external final_dia_mm must be > 0");
    }
    if (in.final_dia_mm > 0.0 && in.final_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "turn_external final_dia " + std::to_string(in.final_dia_mm) +
              " mm < min 0.8 mm");
    }
    if (in.end_z_mm <= in.start_z_mm) {
        r.add("DFM-INPUT", "error",
              "turn_external end_z_mm (" + std::to_string(in.end_z_mm) +
              ") must be > start_z_mm (" + std::to_string(in.start_z_mm) + ")");
    }
    if (in.roughing_passes < 1) {
        r.add("DFM-INPUT", "error", "roughing_passes must be ≥ 1");
    }

    // Lathe-specific: final diameter must be strictly less than current
    // outer diameter (otherwise nothing to remove).
    if (!wp.shape().IsNull()) {
        const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
        const double currentDia = 2.0 * bb.outerRadiusXY();
        if (in.final_dia_mm >= currentDia) {
            r.add("DFM-LATHE", "error",
                  "final_dia_mm (" + std::to_string(in.final_dia_mm) +
                  ") must be < 2 × bbox.outerRadiusXY (" +
                  std::to_string(currentDia) + ") — nothing to remove");
        }
        // Z range must overlap the stock.
        if (in.start_z_mm > bb.zMax || in.end_z_mm < bb.zMin) {
            r.add("DFM-LATHE", "error",
                  "turn zone [" + std::to_string(in.start_z_mm) + "," +
                  std::to_string(in.end_z_mm) + "] does not overlap stock Z range");
        }
    }

    // Surface-speed sanity (typical aluminum OD turn: ~150–600 m/min equivalent).
    if (in.final_dia_mm > 0.0) {
        // Use rule-of-thumb spindle ~2000 rpm; surface speed v = π·d·rpm/1000 (m/min).
        const double v_m_min = M_PI * in.final_dia_mm * 2000.0 / 1000.0;
        if (v_m_min > 1500.0) {
            r.add("DFM-SURFSP", "warning",
                  "turn_external estimated surface speed " +
                  std::to_string(v_m_min) +
                  " m/min > 1500 — reduce RPM or step diameter");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    // 1) DFM gate
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "turn_external DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // 2) Bbox → current outer radius
    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double currentRadius = bb.outerRadiusXY();
    const double finalRadius   = in.final_dia_mm / 2.0;

    // 3) Build the cutter: an annular ring from finalRadius outward to
    //    (currentRadius + overhang) over the Z range.  We extend the outer
    //    cylinder radius by a small overhang so any minor bbox imprecision
    //    doesn't leave a sliver of un-cut material.
    constexpr double kOverhangR = 0.5;   // mm radial overhang on outer wall
    constexpr double kOverhangZ = 0.01;  // mm axial overhang each side
    const double outerR = currentRadius + kOverhangR;
    const double startZ = in.start_z_mm - kOverhangZ;
    const double height = (in.end_z_mm + kOverhangZ) - startZ;

    const gp_Ax2 ax(gp_Pnt(0.0, 0.0, startZ), gp::DZ());
    const TopoDS_Shape cutter = pr::annularRing(ax, outerR, finalRadius, height);

    // 4) Cut
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    // 5) Signature
    json params = {
        { "start_z_mm",      in.start_z_mm },
        { "end_z_mm",        in.end_z_mm },
        { "final_dia_mm",    in.final_dia_mm },
        { "roughing_passes", in.roughing_passes },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     1 },
        { "annular_planar_face_count",  2 },     // shoulders at start_z and end_z
        { "axis",                       { 0.0, 0.0, 1.0 } },
        { "final_dia_mm",               in.final_dia_mm },
        { "start_z_mm",                 in.start_z_mm },
        { "end_z_mm",                   in.end_z_mm },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "lathe_od_insert";
    tooling.tool_dia_mm       = 0.0;                  // single-point tool, not rotary
    tooling.tool_length_mm    = (in.end_z_mm - in.start_z_mm) + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 1;                    // single-point insert
    tooling.cutting_speed_sfm = 800.0;                // aluminum OD turn
    tooling.feed_per_tooth_mm = 0.15;                 // ipr equivalent
    {
        // Material removed = π·(R_initial² - R_final²)·dz
        const double dz = in.end_z_mm - in.start_z_mm;
        tooling.stock_removed_mm3 =
            M_PI * (currentRadius * currentRadius - finalRadius * finalRadius) * dz;
    }
    tooling.est_cycle_time_s = std::max(1.0,
        in.roughing_passes * (in.end_z_mm - in.start_z_mm) / 60.0);
    tooling.extra = json{
        { "roughing_passes", in.roughing_passes }
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    // 6) New workpiece + history
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::turn_external applied: R {} → {} over Z [{}, {}] faces {}→{}",
                  currentRadius, finalRadius, in.start_z_mm, in.end_z_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Pattern: a turned section appears as a cylindrical face whose axis is the Z
// axis and whose radius is strictly less than the workpiece's bbox outer
// radius.  The cylindrical face is bounded above and below by circular edges
// that lie at the start/end Z planes, and those circular edges are shared
// with planar annular faces (the shoulders) whose normals are along ±Z.

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
    const gp_Dir n = s.Plane().Axis().Direction();
    return std::abs(std::abs(n.Z()) - 1.0) < 1e-3;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const TopoDS_Shape& shape = wp.shape();
    if (shape.IsNull()) return out;

    const pr::Bbox3d bb = pr::optimalBbox(shape);
    const double outerR = bb.outerRadiusXY();
    if (outerR <= 1e-6) return out;

    const auto edgeFaces = buildEdgeFaceMap(shape);

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double radius = cyl.Radius();

        // Reject the stock's outer cylinder (radius ≈ bbox outer).
        if (std::abs(radius - outerR) < 0.05) continue;
        // Reject if radius >= bbox outer (cannot be a turned section).
        if (radius >= outerR - 1e-3) continue;

        // Axis must be parallel to +Z (lathe convention).
        const gp_Dir adir = cyl.Axis().Direction();
        if (std::abs(std::abs(adir.Z()) - 1.0) > 1e-3) continue;

        // Collect circular edges on this face perpendicular to Z.
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
        if (zHigh - zLow < 1e-3) continue;

        // Verify at least one annular planar shoulder face (normal ±Z) shares
        // a circular edge with this cylinder.
        int annularShoulderCount = 0;
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            if (!edgeFaces.Contains(e)) continue;
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const auto& adj = edgeFaces.FindFromKey(e);
            for (NCollection_List<TopoDS_Shape>::Iterator it(adj); it.More(); it.Next()) {
                const TopoDS_Face& af = TopoDS::Face(it.Value());
                if (af.IsSame(cylFace)) continue;
                if (isPlanarPerpZ(af)) { ++annularShoulderCount; break; }
            }
        }
        if (annularShoulderCount < 1) continue;

        json recovered = {
            { "start_z_mm",      zLow },
            { "end_z_mm",        zHigh },
            { "final_dia_mm",    2.0 * radius },
            { "roughing_passes", 1 },
        };
        json matched = {
            { "cylindrical_face_id", fIdx },
            { "radius_mm",           radius },
            { "outer_radius_mm",     outerR },
            { "shoulder_face_count", annularShoulderCount },
        };
        // High confidence if both shoulders present, lower if only one.
        const double conf = (annularShoulderCount >= 2) ? 0.92 : 0.78;
        out.push_back(RecognizedFeature{ kSkillId, recovered, conf, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::turn_external
