// @lat: [[engine/skills#turn_internal]]

#include "turn_internal.hpp"

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

namespace koocadcam::skill::turn_internal {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.final_dia_mm <= 0.0 || in.existing_bore_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "turn_internal final_dia and existing_bore_dia must both be > 0");
    }
    if (in.final_dia_mm > 0.0 && in.final_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "turn_internal final_dia " + std::to_string(in.final_dia_mm) +
              " mm < min 0.8 mm");
    }
    if (in.existing_bore_dia_mm > 0.0 && in.existing_bore_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "turn_internal existing_bore_dia " +
              std::to_string(in.existing_bore_dia_mm) + " mm < min 0.8 mm");
    }
    if (in.final_dia_mm > 0.0 && in.existing_bore_dia_mm > 0.0 &&
        in.final_dia_mm <= in.existing_bore_dia_mm) {
        r.add("DFM-INPUT", "error",
              "turn_internal final_dia (" + std::to_string(in.final_dia_mm) +
              ") must be > existing_bore_dia (" +
              std::to_string(in.existing_bore_dia_mm) + ")");
    }
    if (in.end_z_mm <= in.start_z_mm) {
        r.add("DFM-INPUT", "error",
              "turn_internal end_z_mm (" + std::to_string(in.end_z_mm) +
              ") must be > start_z_mm (" + std::to_string(in.start_z_mm) + ")");
    }
    // Z range overlap with bbox (informational): not strictly required.
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "turn_internal DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double finalRadius = in.final_dia_mm / 2.0;

    // Build a cylinder cutter at radius = finalRadius over Z [start_z, end_z]
    // with a small axial overhang to guarantee clean Booleans.
    constexpr double kOverhangZ = 0.05;
    const double startZ = in.start_z_mm - kOverhangZ;
    const double height = (in.end_z_mm + kOverhangZ) - startZ;

    const gp_Ax2 ax(gp_Pnt(0.0, 0.0, startZ), gp::DZ());
    const TopoDS_Shape cutter = pr::cylinder(ax, finalRadius, height);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json params = {
        { "start_z_mm",           in.start_z_mm },
        { "end_z_mm",             in.end_z_mm },
        { "final_dia_mm",         in.final_dia_mm },
        { "existing_bore_dia_mm", in.existing_bore_dia_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     1 },     // the new enlarged bore
        { "coaxial_cyl_radii",          { in.existing_bore_dia_mm / 2.0,
                                          in.final_dia_mm / 2.0 } },
        { "annular_planar_face_count",  1 },     // step shoulder
        { "axis",                       { 0.0, 0.0, 1.0 } },
        { "final_dia_mm",               in.final_dia_mm },
        { "existing_bore_dia_mm",       in.existing_bore_dia_mm },
        { "start_z_mm",                 in.start_z_mm },
        { "end_z_mm",                   in.end_z_mm },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "lathe_boring_bar";
    tooling.tool_dia_mm       = std::min(in.existing_bore_dia_mm * 0.9, 12.0);
    tooling.tool_length_mm    = (in.end_z_mm - in.start_z_mm) + 10.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 1;
    tooling.cutting_speed_sfm = 500.0;
    tooling.feed_per_tooth_mm = 0.08;
    {
        const double dr = (in.final_dia_mm - in.existing_bore_dia_mm) / 2.0;
        const double rMid = (in.final_dia_mm + in.existing_bore_dia_mm) / 4.0;
        tooling.stock_removed_mm3 =
            2.0 * M_PI * rMid * dr * (in.end_z_mm - in.start_z_mm);
    }
    tooling.est_cycle_time_s = std::max(1.0,
        (in.end_z_mm - in.start_z_mm) / 40.0);

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::turn_internal applied: existing_dia={} final_dia={} Z[{},{}] faces {}→{}",
                  in.existing_bore_dia_mm, in.final_dia_mm,
                  in.start_z_mm, in.end_z_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Pattern: an enlarged-bore section is identified by a PAIR of coaxial
// cylindrical faces with the SAME Z axis but DIFFERENT radii, where the
// SMALLER-radius face extends along Z partially "above" the wider section
// (i.e. the wider section is bounded above by a shoulder plane shared with
// the narrower section's cylinder).
//
// This is similar in spirit to bore_with_shelf's recognizer but for INTERNAL
// (concave) cylinders only.

namespace {

using EdgeFaceMap = NCollection_IndexedDataMap<
    TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>;

EdgeFaceMap buildEdgeFaceMap(const TopoDS_Shape& shape)
{
    EdgeFaceMap m;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, m);
    return m;
}

struct CylZ {
    int    faceIdx;
    double radius;
    gp_Pnt centerLow;
    gp_Pnt centerHigh;
};

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const TopoDS_Shape& shape = wp.shape();
    if (shape.IsNull()) return out;

    const pr::Bbox3d bb = pr::optimalBbox(shape);
    const double outerR = bb.outerRadiusXY();
    const auto edgeFaces = buildEdgeFaceMap(shape);

    // Collect cylindrical faces aligned with Z whose radii are smaller than
    // the bbox outer (i.e. internal — not the stock OD).
    std::vector<CylZ> cyls;
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double radius = cyl.Radius();
        const gp_Dir adir = cyl.Axis().Direction();
        if (std::abs(std::abs(adir.Z()) - 1.0) > 1e-3) continue;
        // Skip the stock outer cylinder (≈ bbox outer).
        if (std::abs(radius - outerR) < 0.05) continue;
        if (radius >= outerR - 1e-3) continue;
        // Axis must pass through (≈0,0,*) — concentric with stock axis.
        const gp_Pnt loc = cyl.Axis().Location();
        if (std::sqrt(loc.X() * loc.X() + loc.Y() * loc.Y()) > 0.5) continue;

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
        CylZ c;
        c.faceIdx     = fIdx;
        c.radius      = radius;
        c.centerLow   = *minIt;
        c.centerHigh  = *maxIt;
        cyls.push_back(c);
    }

    // For each pair, the smaller-radius cylinder is "existing", the larger
    // is the enlarged section.  They must meet at a shared Z plane (the
    // shoulder).
    constexpr double kMeetTol = 0.5;   // mm

    for (size_t i = 0; i < cyls.size(); ++i) {
        for (size_t j = 0; j < cyls.size(); ++j) {
            if (i == j) continue;
            const CylZ& small = cyls[i];     // existing
            const CylZ& big   = cyls[j];     // enlarged
            if (big.radius <= small.radius + 1e-4) continue;

            // Meeting plane: small's lowZ ≈ big's highZ  OR  small's highZ ≈ big's lowZ.
            const double sLow  = small.centerLow.Z();
            const double sHigh = small.centerHigh.Z();
            const double bLow  = big.centerLow.Z();
            const double bHigh = big.centerHigh.Z();

            const bool meetsAtSmallLow  = std::abs(sLow  - bHigh) < kMeetTol;
            const bool meetsAtSmallHigh = std::abs(sHigh - bLow ) < kMeetTol;
            if (!meetsAtSmallLow && !meetsAtSmallHigh) continue;

            // Verify a planar shoulder (normal ±Z) shared with the larger cyl.
            bool foundShoulder = false;
            {
                const TopoDS_Face& bigFace = wp.face(big.faceIdx);
                for (TopExp_Explorer exp(bigFace, TopAbs_EDGE); exp.More(); exp.Next()) {
                    const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
                    if (!edgeFaces.Contains(e)) continue;
                    BRepAdaptor_Curve crv(e);
                    if (crv.GetType() != GeomAbs_Circle) continue;
                    const auto& adj = edgeFaces.FindFromKey(e);
                    for (NCollection_List<TopoDS_Shape>::Iterator it(adj); it.More(); it.Next()) {
                        const TopoDS_Face& af = TopoDS::Face(it.Value());
                        if (af.IsSame(bigFace)) continue;
                        BRepAdaptor_Surface as(af);
                        if (as.GetType() != GeomAbs_Plane) continue;
                        if (std::abs(std::abs(as.Plane().Axis().Direction().Z()) - 1.0) > 1e-3)
                            continue;
                        foundShoulder = true;
                        break;
                    }
                    if (foundShoulder) break;
                }
            }
            if (!foundShoulder) continue;

            const double startZ = bLow;
            const double endZ   = bHigh;

            json recovered = {
                { "start_z_mm",           startZ },
                { "end_z_mm",             endZ },
                { "final_dia_mm",         2.0 * big.radius },
                { "existing_bore_dia_mm", 2.0 * small.radius },
            };
            json matched = {
                { "small_cyl_face_id", small.faceIdx },
                { "big_cyl_face_id",   big.faceIdx },
                { "meets_at_small_low", meetsAtSmallLow },
            };
            out.push_back(RecognizedFeature{ kSkillId, recovered, 0.88, matched });
        }
    }
    return out;
}

}  // namespace koocadcam::skill::turn_internal
