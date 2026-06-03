// @lat: [[engine/skills#antenna_keepout_zone]]

#include "antenna_keepout_zone.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Shape.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace koocadcam::skill::antenna_keepout_zone {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    if (s.IsNull()) return 0.0;
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "antenna_keepout_zone: face_id out of range");
    }
    // Polygon-pocket mode (Slice-13) overrides polyline-trough mode.
    const bool polygonMode = in.polygon_xy_points.size() >= 3;
    if (!polygonMode) {
        if (in.polyline.size() < 2) {
            r.add("DFM-INPUT", "error",
                  "antenna_keepout_zone: polyline must have ≥ 2 points (got " +
                  std::to_string(in.polyline.size()) + ")");
        }
        if (!(in.keep_out_width_mm > 0.0)) {
            r.add("DFM-INPUT", "error",
                  "antenna_keepout_zone: keep_out_width_mm must be > 0");
        }
        if (!(in.keep_out_depth_mm > 0.0)) {
            r.add("DFM-INPUT", "error",
                  "antenna_keepout_zone: keep_out_depth_mm must be > 0");
        }
        if (in.keep_out_width_mm > 0.0 &&
            (in.keep_out_width_mm < 0.5 || in.keep_out_width_mm > 5.0)) {
            r.add("DFM-ANT-RANGE", "warning",
                  "antenna keep-out width " +
                  std::to_string(in.keep_out_width_mm) +
                  " outside typical [0.5, 5] mm");
        }
    } else {
        if (!(in.depth_mm > 0.0)) {
            r.add("DFM-INPUT", "error",
                  "antenna_keepout_zone: depth_mm must be > 0 (polygon mode)");
        }
        if (in.edge_taper_mm < 0.0) {
            r.add("DFM-INPUT", "error",
                  "antenna_keepout_zone: edge_taper_mm must be ≥ 0");
        }
        // 3° outward draft over depth = depth * tan(3°) ≈ depth * 0.0524.
        // The taper budget edge_taper_mm should also imply ≤15° draft.
        if (in.depth_mm > 0.0 && in.edge_taper_mm > 0.0) {
            const double tanDeg = in.edge_taper_mm / in.depth_mm;
            const double degApprox = std::atan(tanDeg) * 180.0 / 3.14159265358979323846;
            if (degApprox > 15.0) {
                r.add("DFM-TAPER", "error",
                      "antenna_keepout_zone: edge_taper implies draft " +
                      std::to_string(degApprox) + "° > 15° maximum");
            }
        }
    }
    return r;
}

// ── Slice-13: polygon-pocket branch ──────────────────────────────────────
namespace {

SkillOutput applyPolygon(const Workpiece& wp, const Input& in)
{
    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const double baseZ = faceC.Z();
    const double depth = in.depth_mm;
    const double taper = std::max(0.0, in.edge_taper_mm);
    constexpr double kEmbed = 0.05;

    // Build a polygon FACE in XY at z = baseZ − (depth + kEmbed), then prism
    // it upward by (depth + 2 × kEmbed) so the cut clears the host face top.
    BRepBuilderAPI_MakePolygon poly;
    for (const auto& pt : in.polygon_xy_points) {
        poly.Add(gp_Pnt(pt.first, pt.second, baseZ - depth - kEmbed));
    }
    poly.Close();
    if (!poly.IsDone()) {
        throw SkillError("antenna_keepout_zone: polygon build failed");
    }

    BRepBuilderAPI_MakeFace face(poly.Wire(), true);
    if (!face.IsDone()) {
        throw SkillError("antenna_keepout_zone: polygon face build failed");
    }

    const gp_Vec ext(0.0, 0.0, depth + 2.0 * kEmbed);
    BRepPrimAPI_MakePrism prism(face.Shape(), ext);
    prism.Build();
    if (!prism.IsDone()) {
        throw SkillError("antenna_keepout_zone: polygon prism build failed");
    }
    TopoDS_Shape primaryCut = prism.Shape();

    const double volBefore = volumeOf(wp.shape());
    TopoDS_Shape newShape = pr::cut(wp.shape(), primaryCut);
    if (newShape.IsNull()) {
        throw SkillError("antenna_keepout_zone: polygon cut returned null");
    }
    const double volAfter   = volumeOf(newShape);
    const double volRemoved = volBefore - volAfter;

    json params = {
        { "face_id",          in.face_id },
        { "polygon_point_count", static_cast<int>(in.polygon_xy_points.size()) },
        { "depth_mm",         in.depth_mm },
        { "edge_taper_mm",    in.edge_taper_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "is_phone_feature",           true },
        { "phone_feature_type",         "antenna_keepout" },
        { "mode",                       "polygon" },
        { "segment_count",              static_cast<int>(in.polygon_xy_points.size()) },
        { "polygon_point_count",        static_cast<int>(in.polygon_xy_points.size()) },
        { "derived_volume_removed_mm3", volRemoved },
        { "depth_mm",                   in.depth_mm },
        { "edge_taper_mm",              in.edge_taper_mm },
        { "draft_outward_deg",          taper > 0.0 ? 3.0 : 0.0 },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = 1.5;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = volRemoved / 80.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::antenna_keepout_zone: polygon pts={} vol-removed={}",
                  in.polygon_xy_points.size(), volRemoved);
    return SkillOutput{ wpNew, sig };
}

}  // namespace

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "antenna_keepout_zone DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Polygon-pocket mode (Slice-13) takes priority over polyline-trough mode.
    if (in.polygon_xy_points.size() >= 3) {
        return applyPolygon(wp, in);
    }

    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const double baseZ = faceC.Z();
    const double W     = in.keep_out_width_mm;
    const double D     = in.keep_out_depth_mm;
    constexpr double kEmbed = 0.05;

    std::vector<TopoDS_Shape> cutters;
    cutters.reserve(in.polyline.size() - 1);

    double totalEstVol = 0.0;

    for (size_t i = 0; i + 1 < in.polyline.size(); ++i) {
        const double x0 = in.polyline[i].first;
        const double y0 = in.polyline[i].second;
        const double x1 = in.polyline[i + 1].first;
        const double y1 = in.polyline[i + 1].second;

        const double dx = x1 - x0;
        const double dy = y1 - y0;
        const double seg = std::sqrt(dx * dx + dy * dy);
        if (seg < 1e-6) continue;

        const double midX = (x0 + x1) * 0.5;
        const double midY = (y0 + y1) * 0.5;

        const gp_Dir xLoc(dx / seg, dy / seg, 0.0);
        const gp_Dir zLoc(0.0, 0.0, 1.0);
        const gp_Dir yLoc(-xLoc.Y(), xLoc.X(), 0.0);

        const double oX = midX - (seg / 2.0) * xLoc.X() - (W / 2.0) * yLoc.X();
        const double oY = midY - (seg / 2.0) * xLoc.Y() - (W / 2.0) * yLoc.Y();
        const double oZ = baseZ - D;

        const gp_Pnt origin(oX, oY, oZ);
        // mainDir = +Z so DZ extrudes upward (D + kEmbed cuts through the face)
        const gp_Ax2 ax(origin, zLoc, xLoc);
        try {
            TopoDS_Shape b = pr::box(ax, seg, W, D + kEmbed);
            if (!b.IsNull()) {
                cutters.push_back(b);
                totalEstVol += seg * W * D;
            }
        } catch (const Standard_Failure& ex) {
            spdlog::debug("antenna_keepout_zone: segment box failed — {}",
                          ex.what());
        }
    }

    if (cutters.empty()) {
        throw SkillError(
            "antenna_keepout_zone: no valid segments produced");
    }

    const double volBefore = volumeOf(wp.shape());
    // Polyline segments overlap at vertices — apply cuts sequentially so OCCT
    // doesn't get a compound with overlapping members (which it can't reliably
    // cut against in OCCT 8.0).
    TopoDS_Shape newShape = wp.shape();
    for (const auto& t : cutters) {
        newShape = pr::cut(newShape, t);
    }
    if (newShape.IsNull()) {
        throw SkillError("antenna_keepout_zone: cut returned null");
    }
    const double volAfter   = volumeOf(newShape);
    const double volRemoved = volBefore - volAfter;

    json params = {
        { "face_id",           in.face_id },
        { "point_count",       static_cast<int>(in.polyline.size()) },
        { "keep_out_width_mm", in.keep_out_width_mm },
        { "keep_out_depth_mm", in.keep_out_depth_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "is_phone_feature",           true },
        { "segment_count",              static_cast<int>(cutters.size()) },
        { "derived_volume_removed_mm3", volRemoved },
        { "estimated_removed_mm3",      totalEstVol },
        { "keep_out_width_mm",          in.keep_out_width_mm },
        { "keep_out_depth_mm",          in.keep_out_depth_mm },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = std::max(0.5, W * 0.5);
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = volRemoved / 80.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::antenna_keepout_zone: segs={} vol-removed={}",
                  cutters.size(), volRemoved);

    return SkillOutput{ wpNew, sig };
}

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 0.92;
        rf.matched_geometry = f.pattern;
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::antenna_keepout_zone
