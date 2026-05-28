// @lat: [[engine/skills#drill_lathe]]

#include "drill_lathe.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::drill_lathe {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.diameter_mm < 0.4) {
        r.add("DFM-INPUT", "error",
              "drill_lathe diameter_mm (" + std::to_string(in.diameter_mm) +
              ") must be ≥ 0.4 mm");
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "drill_lathe depth_mm must be > 0 (got " +
              std::to_string(in.depth_mm) + ")");
    }

    if (!wp.shape().IsNull()) {
        const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
        const double length = bb.dz();
        const double outerD = 2.0 * bb.outerRadiusXY();

        if (in.depth_mm > length + 1e-3) {
            r.add("DFM-LATHE", "error",
                  "drill_lathe depth_mm (" + std::to_string(in.depth_mm) +
                  ") > stock length (" + std::to_string(length) +
                  ") — would over-drill the part");
        }
        if (in.diameter_mm >= outerD - 0.1) {
            r.add("DFM-LATHE", "error",
                  "drill_lathe diameter_mm (" + std::to_string(in.diameter_mm) +
                  ") ≥ stock outer dia (" + std::to_string(outerD) +
                  ") — drill would consume the whole part");
        }
    }

    // Drill-depth aspect-ratio sanity (information only).
    if (in.diameter_mm > 0.0 && in.depth_mm / in.diameter_mm > 10.0) {
        r.add("DFM-DEEPHOLE", "warning",
              "drill_lathe depth/dia " +
              std::to_string(in.depth_mm / in.diameter_mm) +
              " > 10 — consider gun_drill or peck strategy");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "drill_lathe DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double radius = in.diameter_mm / 2.0;

    constexpr double kOverhangZ = 0.05;
    const double startZ = bb.zMax - in.depth_mm;        // bottom of drilled hole
    const double height = in.depth_mm + kOverhangZ;     // overhang OUT through top

    const gp_Ax2 ax(gp_Pnt(0.0, 0.0, startZ), gp::DZ());
    const TopoDS_Shape cutter = pr::cylinder(ax, radius, height);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    const bool isThrough = (in.depth_mm >= bb.dz() - 1e-3);

    json params = {
        { "diameter_mm", in.diameter_mm },
        { "depth_mm",    in.depth_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "axis",                       { 0.0, 0.0, 1.0 } },
        { "diameter_mm",                in.diameter_mm },
        { "depth_mm",                   in.depth_mm },
        { "entry_z_mm",                 bb.zMax },
        { "bottom_z_mm",                bb.zMax - in.depth_mm },
        { "through_hole",               isThrough },
        { "cylindrical_face_count",     1 },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "twist_drill";
    tooling.tool_dia_mm       = in.diameter_mm;
    tooling.tool_length_mm    = in.depth_mm + 10.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 350.0;
    tooling.feed_per_tooth_mm = 0.06;
    tooling.stock_removed_mm3 = M_PI * radius * radius * in.depth_mm;
    tooling.est_cycle_time_s  = std::max(1.0, in.depth_mm / 25.0);
    tooling.extra = json{ { "through_hole", isThrough } };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::drill_lathe applied: dia={} depth={} from zMax={} faces {}→{}",
                  in.diameter_mm, in.depth_mm, bb.zMax,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Pattern: a cylindrical face concentric with the Z axis, radius < stock
// outer, whose UPPER bound coincides with the bbox zMax (drill entry from
// the right face).

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    if (wp.shape().IsNull()) return out;

    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double outerR = bb.outerRadiusXY();
    if (outerR <= 1e-6) return out;

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        const TopoDS_Face& f = wp.face(fIdx);
        BRepAdaptor_Surface surf(f);
        const gp_Cylinder cyl = surf.Cylinder();
        if (std::abs(std::abs(cyl.Axis().Direction().Z()) - 1.0) > 1e-3)
            continue;
        // Axis must pass through (≈0, 0, *) — central drill.
        const gp_Pnt loc = cyl.Axis().Location();
        if (std::sqrt(loc.X() * loc.X() + loc.Y() * loc.Y()) > 0.3) continue;
        const double radius = cyl.Radius();
        if (radius >= outerR - 1e-3) continue;

        // Collect circular bounding edges.
        std::vector<double> centerZs;
        for (TopExp_Explorer exp(f, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            if (std::abs(std::abs(c.Axis().Direction().Z()) - 1.0) > 1e-3)
                continue;
            if (std::abs(c.Radius() - radius) > 1e-3) continue;
            centerZs.push_back(c.Location().Z());
        }
        if (centerZs.empty()) continue;

        const double zHi = *std::max_element(centerZs.begin(), centerZs.end());
        const double zLo = *std::min_element(centerZs.begin(), centerZs.end());

        // Entry must coincide with bbox zMax.
        if (std::abs(zHi - bb.zMax) > 0.2) continue;

        const double depth = zHi - zLo;
        if (depth < 1e-3) continue;

        json recovered = {
            { "diameter_mm", 2.0 * radius },
            { "depth_mm",    depth },
        };
        json matched = {
            { "face_id",      fIdx },
            { "entry_z_mm",   zHi },
            { "bottom_z_mm",  zLo },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.90, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::drill_lathe
