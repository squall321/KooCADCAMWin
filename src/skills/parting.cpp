// @lat: [[engine/skills#parting]]

#include "parting.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
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

namespace koocadcam::skill::parting {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.cut_width_mm < 1.0) {
        r.add("DFM-INPUT", "error",
              "parting cut_width_mm (" + std::to_string(in.cut_width_mm) +
              ") < min 1.0 mm (parting blade minimum)");
    }
    if (!wp.shape().IsNull()) {
        const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
        const double zLow  = in.cut_z_mm - in.cut_width_mm / 2.0;
        const double zHigh = in.cut_z_mm + in.cut_width_mm / 2.0;
        if (zLow <= bb.zMin || zHigh >= bb.zMax) {
            r.add("DFM-INPUT", "error",
                  "parting kerf [" + std::to_string(zLow) + "," +
                  std::to_string(zHigh) + "] outside (or touches) stock Z bounds [" +
                  std::to_string(bb.zMin) + "," + std::to_string(bb.zMax) +
                  "] — would not separate two solids");
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// We cut a full-OD cylindrical slab from the stock at z = [cut_z − w/2,
// cut_z + w/2].  The Boolean Cut on a single connected stock produces a
// result that, depending on geometry, may be either:
//   - a single TopoDS_Solid (if the slab didn't fully sever — only if the
//     cylinder cutter is exactly the OD; OCCT may still split it into a
//     compound).
//   - a TopoDS_Compound with TWO sub-solids (the typical parting outcome).
//
// To guarantee the second outcome we make the cutter slightly OVERSIZED in
// radius (overhangR > 0).

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "parting DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double outerR = bb.outerRadiusXY();

    constexpr double kOverhangR = 1.0;  // ensure cutter exceeds OD
    const double startZ = in.cut_z_mm - in.cut_width_mm / 2.0;
    const double height = in.cut_width_mm;

    const gp_Ax2 ax(gp_Pnt(0.0, 0.0, startZ), gp::DZ());
    const TopoDS_Shape cutter = pr::cylinder(ax, outerR + kOverhangR, height);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json params = {
        { "cut_z_mm",     in.cut_z_mm },
        { "cut_width_mm", in.cut_width_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "two_coaxial_planar_faces",   true },
        { "kerf_z_lower",               in.cut_z_mm - in.cut_width_mm / 2.0 },
        { "kerf_z_upper",               in.cut_z_mm + in.cut_width_mm / 2.0 },
        { "axis",                       { 0.0, 0.0, 1.0 } },
        { "compound_with_2_solids",     true },
        { "cut_z_mm",                   in.cut_z_mm },
        { "cut_width_mm",               in.cut_width_mm },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "lathe_parting_blade";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = outerR + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 1;
    tooling.cutting_speed_sfm = 400.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 = M_PI * outerR * outerR * in.cut_width_mm;
    tooling.est_cycle_time_s  = std::max(2.0, outerR / 15.0);

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::parting applied: cut_z={} kerf={} faces {}→{}",
                  in.cut_z_mm, in.cut_width_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Pattern: the workpiece is a TopoDS_Compound containing ≥ 2 solid sub-shapes
// that are coaxial along Z (their bbox XY footprints overlap and are stacked
// in Z).  At the meeting interface there are two coaxial planar faces with
// normals -Z (on the upper solid's bottom) and +Z (on the lower solid's top).

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const TopoDS_Shape& shape = wp.shape();
    if (shape.IsNull()) return out;

    // Count solid sub-shapes.
    int solidCount = 0;
    std::vector<TopoDS_Shape> solids;
    for (TopExp_Explorer e(shape, TopAbs_SOLID); e.More(); e.Next()) {
        ++solidCount;
        solids.push_back(e.Current());
    }
    if (solidCount < 2) return out;

    // Workpiece is parted → must be a compound.  Confirm.
    if (shape.ShapeType() != TopAbs_COMPOUND) return out;

    // Collect ALL planar faces with normal ±Z, indexed by Z plane.
    // From wp.face() enumeration.
    struct PFace {
        int    faceId;
        double z;
        double normZsign;   // +1 or -1
        double radius;      // perimeter circle radius if available
    };
    std::vector<PFace> pfaces;
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFacePlanar(fIdx)) continue;
        const TopoDS_Face& f = wp.face(fIdx);
        BRepAdaptor_Surface s(f);
        const gp_Pln pln = s.Plane();
        // Use the orientation-respecting face normal: the surface normal
        // (gp_Pln::Axis::Direction) ignores face flip flags, so reversed
        // faces (e.g. the upper-solid's BOTTOM cut-face whose outward
        // normal is -Z) would otherwise be misclassified as +Z.
        gp_Dir n;
        try { n = wp.faceNormal(fIdx); } catch (...) { continue; }
        if (std::abs(std::abs(n.Z()) - 1.0) > 1e-3) continue;
        // Find a circular boundary edge to recover radius.
        double r = 0.0;
        for (TopExp_Explorer exp(f, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            if (crv.Circle().Radius() > r) r = crv.Circle().Radius();
        }
        PFace pf;
        pf.faceId    = fIdx;
        pf.z         = pln.Location().Z();
        pf.normZsign = (n.Z() > 0) ? 1.0 : -1.0;
        pf.radius    = r;
        pfaces.push_back(pf);
    }

    // Find two coaxial planar faces with OPPOSITE normals whose Z planes are
    // close together (the kerf walls).
    for (size_t i = 0; i < pfaces.size(); ++i) {
        for (size_t j = i + 1; j < pfaces.size(); ++j) {
            const PFace& a = pfaces[i];
            const PFace& b = pfaces[j];
            if (a.normZsign * b.normZsign >= 0) continue;   // need opposite normals
            const double zLow  = std::min(a.z, b.z);
            const double zHigh = std::max(a.z, b.z);
            const double width = zHigh - zLow;
            // Kerf width must be > 0 and reasonable for parting (≤ 10 mm).
            if (width < 1e-3 || width > 10.0) continue;
            // Radii must match (both cut surfaces of the same OD).
            if (a.radius > 0.0 && b.radius > 0.0 &&
                std::abs(a.radius - b.radius) > 0.3) continue;
            // The upper face must have -Z normal (it is the BOTTOM of the
            // upper solid); the lower face must have +Z normal.
            const double aIsUpper = (a.z > b.z) ? a.normZsign : b.normZsign;
            const double aIsLower = (a.z < b.z) ? a.normZsign : b.normZsign;
            if (aIsUpper >= 0.0) continue;   // upper face's normal must be -Z
            if (aIsLower <= 0.0) continue;   // lower face's normal must be +Z

            const double cut_z = (zLow + zHigh) / 2.0;
            json recovered = {
                { "cut_z_mm",     cut_z },
                { "cut_width_mm", width },
            };
            json matched = {
                { "upper_face_id",    (a.z > b.z) ? a.faceId : b.faceId },
                { "lower_face_id",    (a.z < b.z) ? a.faceId : b.faceId },
                { "solid_count",      solidCount },
                { "kerf_z_lower",     zLow },
                { "kerf_z_upper",     zHigh },
            };
            out.push_back(RecognizedFeature{ kSkillId, recovered, 0.90, matched });
        }
    }
    return out;
}

}  // namespace koocadcam::skill::parting
