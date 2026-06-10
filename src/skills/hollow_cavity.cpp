// @lat: [[engine/skills#hollow_cavity]]

#include "hollow_cavity.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pln.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::hollow_cavity {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

// Detect whether the workpiece is best described as a cylinder (round stock).
// Heuristic: it has at least one large cylindrical face whose radius matches
// half the XY bbox extent (within tolerance) AND that cylindrical face is
// the dominant "side wall".
bool isCylindricalStock(const Workpiece& wp,
                        double& outRadius,
                        gp_Ax1& outAxis)
{
    const auto bb = pr::optimalBbox(wp.shape());
    const double xyHalf = std::max(bb.dx(), bb.dy()) / 2.0;

    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        const gp_Cylinder cyl = surf.Cylinder();
        // Side wall of a round stock has axis along Z.
        if (std::abs(std::abs(cyl.Axis().Direction().Z()) - 1.0) > 1e-2) continue;
        if (std::abs(cyl.Radius() - xyHalf) > 0.05) continue;
        outRadius = cyl.Radius();
        outAxis   = cyl.Axis();
        return true;
    }
    return false;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.wall_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "hollow_cavity wall_thickness must be > 0");
    } else if (in.wall_thickness_mm < 0.4) {
        // DFM-001 baseline for both case + phone.  Watch can go to 0.35 but
        // that's a per-product override; warn (not error) below 0.4.
        r.add("DFM-001", "warning",
              "wall_thickness " + std::to_string(in.wall_thickness_mm) +
              " mm < 0.4 mm baseline (acceptable down to 0.35 mm for watch material)");
    }

    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "hollow_cavity depth must be > 0");
    }

    // Sanity: depth must not exceed workpiece Z span minus a floor of
    // wall_thickness.  We don't enforce this strictly (workpiece may not be
    // top-loaded along Z); warn only.
    if (in.depth_mm > 0.0 && in.wall_thickness_mm > 0.0) {
        double xMin, yMin, zMin, xMax, yMax, zMax;
        wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
        const double zSpan = zMax - zMin;
        if (in.depth_mm + in.wall_thickness_mm > zSpan + 1e-6) {
            r.add("DFM-DEPTH", "warning",
                  "depth + wall_thickness " +
                  std::to_string(in.depth_mm + in.wall_thickness_mm) +
                  " > workpiece Z span " + std::to_string(zSpan));
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "hollow_cavity DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("hollow_cavity: entry_face datum unresolved");

    // Entry face plane gives us the cavity start height & inward normal.
    BRepAdaptor_Surface entrySurf(wp.face(*entryId));
    if (entrySurf.GetType() != GeomAbs_Plane) {
        throw SkillError("hollow_cavity: entry_face must be planar");
    }
    const gp_Pln entryPlane = entrySurf.Plane();
    gp_Dir outward = entryPlane.Axis().Direction();
    if (wp.face(*entryId).Orientation() == TopAbs_REVERSED) outward.Reverse();
    // Inward = into the material = -outward.
    const gp_Dir inward(-outward.X(), -outward.Y(), -outward.Z());

    const auto bb = pr::optimalBbox(wp.shape());

    // Decide cuboid vs cylindrical stock.  We only specialise for the case
    // where the entry-face normal is ±Z (top-loaded hollowing).  For other
    // axes the slice-1 box approximation is used.
    double cylR = 0.0;
    gp_Ax1 cylAx;
    const bool roundStock = isCylindricalStock(wp, cylR, cylAx);
    const bool zAxisLoad  = (std::abs(std::abs(inward.Z()) - 1.0) < 1e-3);

    TopoDS_Shape cutter;
    std::string cavityKind;

    if (roundStock && zAxisLoad) {
        // Cylindrical cavity centered on the stock's central axis.
        const double cavityR = cylR - in.wall_thickness_mm;
        if (cavityR <= 0.0) {
            throw SkillError("hollow_cavity: wall_thickness >= stock radius");
        }
        // Build a cylindrical cutter starting at the entry face and extending
        // by depth into the material (along inward).
        const gp_Pnt entryCenter = (inward.Z() < 0)
            ? gp_Pnt((bb.xMin + bb.xMax) / 2.0, (bb.yMin + bb.yMax) / 2.0, bb.zMax)
            : gp_Pnt((bb.xMin + bb.xMax) / 2.0, (bb.yMin + bb.yMax) / 2.0, bb.zMin);
        const gp_Ax2 ax(entryCenter, inward);
        // Add a small overhang to ensure clean entry cut.
        const double kEntryOverhang = 0.05;
        const gp_Pnt cutterStart(
            entryCenter.X() - inward.X() * kEntryOverhang,
            entryCenter.Y() - inward.Y() * kEntryOverhang,
            entryCenter.Z() - inward.Z() * kEntryOverhang);
        const gp_Ax2 ax2(cutterStart, inward);
        cutter = pr::cylinder(ax2, cavityR, in.depth_mm + kEntryOverhang);
        cavityKind = "cylindrical";
    } else {
        // Rectangular cuboid cavity: shrink the XY bbox by 2 × wall_thickness.
        const double sx = bb.dx() - 2.0 * in.wall_thickness_mm;
        const double sy = bb.dy() - 2.0 * in.wall_thickness_mm;
        if (sx <= 0.0 || sy <= 0.0) {
            throw SkillError("hollow_cavity: wall_thickness >= half of workpiece XY extent");
        }

        // Place cutter so its top sits on/just outside the entry face.  We
        // assume an axis-aligned cuboid stock for slice 1.  For non-Z entry
        // we still extrude along -outward but project the XY footprint onto
        // the entry plane center.
        const double kEntryOverhang = 0.05;
        const gp_Pnt entryCenter = (inward.Z() <= 0)
            ? gp_Pnt((bb.xMin + bb.xMax) / 2.0, (bb.yMin + bb.yMax) / 2.0, bb.zMax)
            : gp_Pnt((bb.xMin + bb.xMax) / 2.0, (bb.yMin + bb.yMax) / 2.0, bb.zMin);

        // Box bottom origin (where prim::box's MakeBox places its first corner).
        // We want the cutter to span from entryCenter (offset by overhang outward)
        // INTO the material along `inward` for `depth_mm + overhang`.
        const double totalZ = in.depth_mm + kEntryOverhang;
        const gp_Pnt boxOrigin = (inward.Z() <= 0)
            ? gp_Pnt(entryCenter.X() - sx / 2.0,
                     entryCenter.Y() - sy / 2.0,
                     entryCenter.Z() + kEntryOverhang - totalZ)   // top at zMax + overhang
            : gp_Pnt(entryCenter.X() - sx / 2.0,
                     entryCenter.Y() - sy / 2.0,
                     entryCenter.Z() - kEntryOverhang);            // bottom at zMin - overhang

        cutter = pr::box(gp_Ax2(boxOrigin, gp::DZ()), sx, sy, totalZ);
        cavityKind = "rectangular";
    }

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json params = {
        { "entry_face_kind",   "resolved_id" },
        { "entry_face_id",     *entryId },
        { "wall_thickness_mm", in.wall_thickness_mm },
        { "depth_mm",          in.depth_mm },
        { "cavity_shape",      cavityKind },   // recorded so metadata replay
                                               // recovers the same params the
                                               // geometric recognizer reports
    };
    json pattern = {
        { "kind",                kSkillId },
        { "cavity_shape",        cavityKind },
        { "wall_thickness_mm",   in.wall_thickness_mm },
        { "depth_mm",            in.depth_mm },
        { "inward_dir",          { inward.X(), inward.Y(), inward.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type        = "end_mill";
    tooling.tool_dia_mm      = 6.0;     // typical roughing end mill
    tooling.tool_length_mm   = in.depth_mm * 1.3 + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 3;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.06;
    // Approximate stock removed = cavity footprint area × depth.
    double cavityArea;
    if (cavityKind == "cylindrical") {
        const double cavityR = cylR - in.wall_thickness_mm;
        cavityArea = M_PI * cavityR * cavityR;
    } else {
        cavityArea = (bb.dx() - 2.0 * in.wall_thickness_mm)
                   * (bb.dy() - 2.0 * in.wall_thickness_mm);
    }
    tooling.stock_removed_mm3 = cavityArea * in.depth_mm;
    tooling.est_cycle_time_s  = std::max(10.0, tooling.stock_removed_mm3 / 500.0);

    FeatureSignature sig {
        kSkillId,
        params,
        pattern,
        tooling,
    };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::hollow_cavity applied: kind={} wall={} depth={} faces {}→{}",
                  cavityKind, in.wall_thickness_mm, in.depth_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Heuristic: find the largest INTERIOR planar face (= candidate cavity
// bottom).  Interior face is detected by:
//   - normal points "up" out of the cavity, i.e. opposite the outer top face.
//   - it is bounded by edges shared with vertical / cylindrical walls only.
// Then recover XY extents from that face's bbox and compare to the
// workpiece's outer XY bbox to derive wall_thickness.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Metadata replay: on a workpiece WE built, the hollow_cavity signature is
    // in the chain history — replay it at full confidence.  This carries the
    // "metadata_replay" source tag, which exempts it from the foreign-CAD
    // fallback cap in re::analyze (so a synthesised cavity stays above the RE
    // inference threshold), while a FOREIGN STEP — which has no signature —
    // falls through to the geometric scan below and is correctly capped.
    // Mirrors every other domain-compound skill; hollow_cavity was missing it.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    const auto outerBb = pr::optimalBbox(wp.shape());

    // Collect all Z+ planar faces with their (z, area).  The outer top sits
    // at the workpiece's highest Z plane; the cavity bottom is at a strictly
    // lower Z.  Don't rank by area — after a cavity cut, the perimeter ring
    // (outer top) can be smaller than the cavity bottom.
    struct ZupFace { int id; double z; double area; };
    std::vector<ZupFace> zup;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        try {
            const gp_Dir n = wp.faceNormal(i);
            if (n.Z() < 0.9) continue;
        } catch (...) { continue; }
        zup.push_back({ i, wp.faceCenter(i).Z(), wp.faceArea(i) });
    }
    if (zup.size() < 2) return out;

    // Outer top = highest Z.  Cavity bottom = the largest face strictly
    // below the outer top (skip near-duplicates within 0.01 mm).
    int outerTopFace = zup.front().id;
    double outerTopZ  = zup.front().z;
    for (const auto& f : zup) if (f.z > outerTopZ) { outerTopZ = f.z; outerTopFace = f.id; }

    int   cavityBottom = -1;
    double cavityBottomArea = 0.0;
    for (const auto& f : zup) {
        if (f.id == outerTopFace) continue;
        if (outerTopZ - f.z < 1e-2) continue;   // same plane (numerical jitter)
        if (f.area < 1.0) continue;
        if (f.area > cavityBottomArea) {
            cavityBottomArea = f.area;
            cavityBottom = f.id;
        }
    }
    if (cavityBottom < 0) return out;

    // Compute cavity-bottom XY bbox.
    const auto cavityBb = pr::optimalBbox(wp.face(cavityBottom));

    // wall_thickness = average of the four XY gaps between outer & cavity.
    const double gXl = cavityBb.xMin - outerBb.xMin;
    const double gXh = outerBb.xMax  - cavityBb.xMax;
    const double gYl = cavityBb.yMin - outerBb.yMin;
    const double gYh = outerBb.yMax  - cavityBb.yMax;
    // All four gaps should be ≈ wall_thickness.  If they differ wildly the
    // face we found is probably not a hollow_cavity bottom — confidence drops.
    const std::vector<double> gaps{ gXl, gXh, gYl, gYh };
    const double gMean = (gXl + gXh + gYl + gYh) / 4.0;
    double gSpread = 0.0;
    for (double g : gaps) gSpread = std::max(gSpread, std::abs(g - gMean));
    if (gMean < 0.1) return out;          // cavity not inset → not a cavity

    // depth = entry-face Z − cavity-bottom Z (assuming +Z entry).
    double depth = 0.0;
    if (outerTopFace >= 0) {
        const gp_Pnt topC = wp.faceCenter(outerTopFace);
        const gp_Pnt botC = wp.faceCenter(cavityBottom);
        depth = topC.Z() - botC.Z();
    } else {
        depth = outerBb.zMax - cavityBb.zMin;
    }

    // Detect cavity shape: if any cylindrical face exists adjacent to the
    // cavity bottom edge, classify as cylindrical.
    bool cylShape = false;
    for (TopExp_Explorer exp(wp.face(cavityBottom), TopAbs_EDGE); exp.More(); exp.Next()) {
        const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
        BRepAdaptor_Curve crv(e);
        if (crv.GetType() == GeomAbs_Circle) { cylShape = true; break; }
    }

    // Confidence: 0.9 when gaps are uniform, scaling down with spread.
    const double confidence = std::clamp(0.95 - gSpread, 0.4, 0.95);

    json recovered = {
        { "wall_thickness_mm", gMean },
        { "depth_mm",          depth },
        { "cavity_shape",      cylShape ? std::string("cylindrical")
                                        : std::string("rectangular") },
    };
    json matched = {
        { "cavity_bottom_face_id", cavityBottom },
        { "outer_top_face_id",     outerTopFace },
        { "gap_xLow",              gXl },
        { "gap_xHigh",             gXh },
        { "gap_yLow",              gYl },
        { "gap_yHigh",             gYh },
        { "cavity_bbox", {
            { "xMin", cavityBb.xMin }, { "xMax", cavityBb.xMax },
            { "yMin", cavityBb.yMin }, { "yMax", cavityBb.yMax },
            { "zMin", cavityBb.zMin }, { "zMax", cavityBb.zMax },
        } },
    };
    out.push_back(RecognizedFeature{
        kSkillId, recovered, confidence, matched
    });
    return out;
}

}  // namespace koocadcam::skill::hollow_cavity
