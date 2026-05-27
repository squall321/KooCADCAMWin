// @lat: [[engine/skills#bore_with_shelf]]

#include "bore_with_shelf.hpp"

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

namespace koocadcam::skill::bore_with_shelf {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.upper_dia_mm <= 0.0 || in.lower_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "bore_with_shelf diameters must be > 0");
    }
    if (in.upper_depth_mm <= 0.0 || in.lower_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "bore_with_shelf depths must be > 0");
    }

    if (in.upper_dia_mm > 0.0 && in.upper_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "bore_with_shelf upper_dia " + std::to_string(in.upper_dia_mm) +
              " mm < min 0.8 mm");
    }
    if (in.lower_dia_mm > 0.0 && in.lower_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "bore_with_shelf lower_dia " + std::to_string(in.lower_dia_mm) +
              " mm < min 0.8 mm");
    }

    // Step requirement: lower must be wider than upper.
    if (in.upper_dia_mm > 0.0 && in.lower_dia_mm > 0.0 &&
        in.lower_dia_mm <= in.upper_dia_mm) {
        r.add("DFM-STEP", "error",
              "lower_dia (" + std::to_string(in.lower_dia_mm) +
              ") must be > upper_dia (" + std::to_string(in.upper_dia_mm) +
              ") — otherwise this is a plain bore");
    }

    const double total = in.upper_depth_mm + in.lower_depth_mm;
    if (total > 20.0) {
        r.add("DFM-DEPTH", "warning",
              "total step-bore depth " + std::to_string(total) +
              " mm > 20 mm — verify workpiece thickness allows this");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "bore_with_shelf DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("bore_with_shelf: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    const double kEntryOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    // Upper cylinder origin = on/just outside the entry face, along -axis_dir.
    // Lower cylinder origin = upper origin shifted by (upper_depth + overhang) along +axis_dir,
    // i.e. starts at the shelf plane and extends into the material.
    gp_Pnt upperStart;
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        // Common straight-down/up case
        if (adir.Z() < 0) {
            upperStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMax + kEntryOverhang);
        } else {
            upperStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMin - kEntryOverhang);
        }
    } else {
        upperStart = gp_Pnt(
            in.position_x_mm - adir.X() * (bboxDiag + kEntryOverhang),
            in.position_y_mm - adir.Y() * (bboxDiag + kEntryOverhang),
            (zMin + zMax) / 2.0 - adir.Z() * (bboxDiag + kEntryOverhang));
    }

    // Upper cylinder tall enough to pierce just past the shelf depth.
    const double upperHeight = in.upper_depth_mm + kEntryOverhang;

    // Lower cylinder starts where the upper "ends" (at upper_depth below entry).
    // We add a small overlap with the upper to guarantee a clean Boolean step.
    const double lowerOverlap = 0.05;
    const gp_Pnt lowerStart(
        upperStart.X() + adir.X() * (in.upper_depth_mm + kEntryOverhang - lowerOverlap),
        upperStart.Y() + adir.Y() * (in.upper_depth_mm + kEntryOverhang - lowerOverlap),
        upperStart.Z() + adir.Z() * (in.upper_depth_mm + kEntryOverhang - lowerOverlap));
    const double lowerHeight = in.lower_depth_mm + lowerOverlap;

    const gp_Ax2 upperAx(upperStart, adir);
    const gp_Ax2 lowerAx(lowerStart, adir);

    const TopoDS_Shape upperCutter = pr::cylinder(upperAx, in.upper_dia_mm / 2.0, upperHeight);
    const TopoDS_Shape lowerCutter = pr::cylinder(lowerAx, in.lower_dia_mm / 2.0, lowerHeight);

    // cutMany packs both into a single Boolean — faster and topology-cleaner
    // than two sequential cuts.
    const TopoDS_Shape newShape = pr::cutMany(wp.shape(),
        std::vector<TopoDS_Shape>{ upperCutter, lowerCutter });

    json params = {
        { "entry_face_kind", "resolved_id" },
        { "entry_face_id",   *entryId },
        { "position_x_mm",   in.position_x_mm },
        { "position_y_mm",   in.position_y_mm },
        { "axis_dir",        { adir.X(), adir.Y(), adir.Z() } },
        { "upper_dia_mm",    in.upper_dia_mm },
        { "upper_depth_mm",  in.upper_depth_mm },
        { "lower_dia_mm",    in.lower_dia_mm },
        { "lower_depth_mm",  in.lower_depth_mm },
    };
    json pattern = {
        { "kind",                     kSkillId },
        { "cylindrical_face_count",   2 },
        { "annular_planar_face_count", 1 },
        { "bottom_planar_face_present", true },
        { "circular_edge_count",      3 },
        { "upper_dia_mm",             in.upper_dia_mm },
        { "lower_dia_mm",             in.lower_dia_mm },
        { "axis_dir",                 { adir.X(), adir.Y(), adir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type        = "end_mill";
    tooling.tool_dia_mm      = std::min(in.upper_dia_mm, in.lower_dia_mm);
    tooling.tool_length_mm   = (in.upper_depth_mm + in.lower_depth_mm) * 1.3 + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 3;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.04;

    const double upperVol = M_PI * (in.upper_dia_mm * 0.5) * (in.upper_dia_mm * 0.5)
                          * in.upper_depth_mm;
    const double lowerVol = M_PI * (in.lower_dia_mm * 0.5) * (in.lower_dia_mm * 0.5)
                          * in.lower_depth_mm;
    tooling.stock_removed_mm3 = upperVol + lowerVol;
    tooling.est_cycle_time_s  = std::max(3.0,
        (in.upper_depth_mm + in.lower_depth_mm) / 30.0);

    FeatureSignature sig {
        kSkillId,
        params,
        pattern,
        tooling,
    };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::bore_with_shelf applied: u_dia={}/u_dep={} l_dia={}/l_dep={} faces {}→{}",
                  in.upper_dia_mm, in.upper_depth_mm,
                  in.lower_dia_mm, in.lower_depth_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Strategy:
//   1. Collect cylindrical faces.  Group by axis (≈ same gp_Ax1).
//   2. Within a group, find a pair with DIFFERENT radii.
//   3. Verify a planar annular face exists between them at the larger
//      cylinder's "top" edge (i.e. the shelf plane).
//   4. Verify a small planar face below the larger cylinder (= blind bottom).
//   5. Recover upper/lower diameters and depths from the bounding circle Zs.

namespace {

using EdgeFaceMap = NCollection_IndexedDataMap<
    TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>;

EdgeFaceMap buildEdgeFaceMap(const TopoDS_Shape& shape)
{
    EdgeFaceMap m;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, m);
    return m;
}

struct CylInfo {
    int    faceIdx;
    double radius;
    gp_Ax1 axis;
    gp_Pnt topCenter;       // circle center toward entry face
    gp_Pnt bottomCenter;    // circle center toward material interior
};

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const TopoDS_Shape& shape = wp.shape();
    const auto edgeFaces = buildEdgeFaceMap(shape);

    // Collect all cylindrical faces with their bounding circle centers.
    std::vector<CylInfo> cyls;
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double radius = cyl.Radius();
        const gp_Ax1 axis = cyl.Axis();

        std::vector<gp_Pnt> centers;
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            if (std::abs(std::abs(c.Axis().Direction().Dot(axis.Direction())) - 1.0) > 1e-3)
                continue;
            if (std::abs(c.Radius() - radius) > 1e-3) continue;
            centers.push_back(c.Location());
        }
        if (centers.size() < 2) continue;

        const gp_Dir adir = axis.Direction();
        auto projOnAxis = [&](const gp_Pnt& p) {
            return (p.X() - axis.Location().X()) * adir.X() +
                   (p.Y() - axis.Location().Y()) * adir.Y() +
                   (p.Z() - axis.Location().Z()) * adir.Z();
        };
        auto cmp = [&](const gp_Pnt& a, const gp_Pnt& b) {
            return projOnAxis(a) < projOnAxis(b);
        };
        const auto minIt = std::min_element(centers.begin(), centers.end(), cmp);
        const auto maxIt = std::max_element(centers.begin(), centers.end(), cmp);

        CylInfo ci;
        ci.faceIdx      = fIdx;
        ci.radius       = radius;
        ci.axis         = axis;
        ci.topCenter    = *maxIt;      // along +axis_dir = higher Z for top-face drilling
        ci.bottomCenter = *minIt;
        cyls.push_back(ci);
    }

    // For each ordered pair (a = upper/smaller, b = lower/larger), test
    // whether their axes coincide, radii differ, and the smaller cylinder
    // sits "above" the larger one along the shared axis.
    for (size_t i = 0; i < cyls.size(); ++i) {
        for (size_t j = 0; j < cyls.size(); ++j) {
            if (i == j) continue;
            const CylInfo& a = cyls[i];   // candidate upper (smaller)
            const CylInfo& b = cyls[j];   // candidate lower (larger)

            if (a.radius >= b.radius - 1e-4) continue;  // need b strictly wider

            // Axis-direction parallel (allow either sign).
            const double dirDot = a.axis.Direction().Dot(b.axis.Direction());
            if (std::abs(std::abs(dirDot) - 1.0) > 1e-3) continue;

            // Axis-line closeness: vector between origins must be along the axis.
            const gp_Vec ao(a.axis.Location(), b.axis.Location());
            const gp_Vec axVec(a.axis.Direction());
            const gp_Vec perp = ao - axVec * ao.Dot(axVec);
            if (perp.Magnitude() > 5e-3) continue;

            // Geometry of the step: along a.axis.Direction(), b's TOP must be
            // BELOW a's BOTTOM (touching, modulo overlap).  We use the upper
            // cylinder's axis as the canonical direction.
            const gp_Dir adir = a.axis.Direction();
            auto proj = [&](const gp_Pnt& p) {
                return (p.X() - a.axis.Location().X()) * adir.X() +
                       (p.Y() - a.axis.Location().Y()) * adir.Y() +
                       (p.Z() - a.axis.Location().Z()) * adir.Z();
            };
            const double aTop    = proj(a.topCenter);
            const double aBot    = proj(a.bottomCenter);
            const double bTop    = proj(b.topCenter);
            const double bBot    = proj(b.bottomCenter);

            // upper extends from aTop (entry face side) DOWN to aBot.
            // Drilling direction = +axis_dir from caller's POV; in projection
            // language, "down into material" = decreasing projection here
            // because axis.Direction() points "up" out of OCCT's cylinder.
            // We do not assume a sign: simply check that one of (aBot, bTop)
            // is the meeting plane.
            const double meetingTol = 0.5;   // mm: shelf-plane coincidence
            const bool meetsAtAbot = std::abs(aBot - bTop) < meetingTol ||
                                     std::abs(aBot - bBot) < meetingTol;
            const bool meetsAtAtop = std::abs(aTop - bTop) < meetingTol ||
                                     std::abs(aTop - bBot) < meetingTol;
            if (!meetsAtAbot && !meetsAtAtop) continue;

            // Determine which of (aTop, aBot) is the SHELF (touches b) and
            // which is the entry-face top (touches workpiece exterior).
            // Whichever endpoint of a is NOT shared with b is the entry.
            const gp_Pnt entryPt = meetsAtAbot ? a.topCenter : a.bottomCenter;
            const gp_Pnt shelfPt = meetsAtAbot ? a.bottomCenter : a.topCenter;
            // For the lower cylinder, the shelf is the endpoint closest to shelfPt.
            const double dShelfTop = shelfPt.Distance(b.topCenter);
            const double dShelfBot = shelfPt.Distance(b.bottomCenter);
            const gp_Pnt lowerBot  = (dShelfTop < dShelfBot) ? b.bottomCenter : b.topCenter;

            const double upperDepth = entryPt.Distance(shelfPt);
            const double lowerDepth = shelfPt.Distance(lowerBot);
            const double upperDia   = 2.0 * a.radius;
            const double lowerDia   = 2.0 * b.radius;

            // Sanity: must look like a non-trivial step bore.
            if (upperDepth < 1e-3 || lowerDepth < 1e-3) continue;

            // Verify an annular planar face exists between a and b: find a
            // planar face whose boundary includes one edge of a's lower
            // circle and one edge of b's upper circle.  Simpler heuristic:
            // count planar faces adjacent to BOTH cylindrical faces a and b
            // via shared circular edges.
            int annularShelfFaceId = -1;
            {
                const TopoDS_Face& faceA = wp.face(a.faceIdx);
                for (TopExp_Explorer expA(faceA, TopAbs_EDGE); expA.More(); expA.Next()) {
                    const TopoDS_Edge& e = TopoDS::Edge(expA.Current());
                    BRepAdaptor_Curve crv(e);
                    if (crv.GetType() != GeomAbs_Circle) continue;
                    if (!edgeFaces.Contains(e)) continue;
                    const auto& adj = edgeFaces.FindFromKey(e);
                    for (NCollection_List<TopoDS_Shape>::Iterator it(adj); it.More(); it.Next()) {
                        const TopoDS_Face& af = TopoDS::Face(it.Value());
                        if (af.IsSame(faceA)) continue;
                        if (BRepAdaptor_Surface(af).GetType() != GeomAbs_Plane) continue;
                        // Find this same face in WP enumeration
                        for (int k = 0; k < wp.faceCount(); ++k) {
                            if (wp.face(k).IsSame(af)) {
                                annularShelfFaceId = k;
                                break;
                            }
                        }
                        if (annularShelfFaceId >= 0) break;
                    }
                    if (annularShelfFaceId >= 0) break;
                }
            }
            // If no annular shelf candidate, skip — could be two disjoint bores.
            if (annularShelfFaceId < 0) continue;

            // Drill direction (entry → bottom) for downstream consumers.
            gp_Vec drillVec(entryPt, lowerBot);
            if (drillVec.Magnitude() < 1e-6) continue;
            drillVec.Normalize();

            json recovered = {
                { "position_x_mm",  entryPt.X() },
                { "position_y_mm",  entryPt.Y() },
                { "axis_dir",       { drillVec.X(), drillVec.Y(), drillVec.Z() } },
                { "upper_dia_mm",   upperDia },
                { "upper_depth_mm", upperDepth },
                { "lower_dia_mm",   lowerDia },
                { "lower_depth_mm", lowerDepth },
            };
            json matched = {
                { "upper_cyl_face_id", a.faceIdx },
                { "lower_cyl_face_id", b.faceIdx },
                { "shelf_face_id",     annularShelfFaceId },
                { "entry_center",  { entryPt.X(), entryPt.Y(), entryPt.Z() } },
                { "shelf_center",  { shelfPt.X(), shelfPt.Y(), shelfPt.Z() } },
                { "bottom_center", { lowerBot.X(), lowerBot.Y(), lowerBot.Z() } },
            };
            out.push_back(RecognizedFeature{
                kSkillId, recovered, 0.9, matched
            });
        }
    }

    return out;
}

}  // namespace koocadcam::skill::bore_with_shelf
