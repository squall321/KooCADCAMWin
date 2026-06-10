// @lat: [[engine/skills#rib_along_path]]

#include "rib_along_path.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Shape.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>

namespace koocadcam::skill::rib_along_path {

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

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!(in.rib_thickness_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "rib_along_path: rib_thickness_mm must be > 0 (got " +
              std::to_string(in.rib_thickness_mm) + ")");
    }
    if (!(in.rib_height_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "rib_along_path: rib_height_mm must be > 0 (got " +
              std::to_string(in.rib_height_mm) + ")");
    }
    if (in.path_polyline.size() < 2) {
        r.add("DFM-INPUT", "error",
              "rib_along_path: path_polyline must have ≥ 2 points (got " +
              std::to_string(in.path_polyline.size()) + ")");
    }
    if (in.host_face_id < 0 || in.host_face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "rib_along_path: host_face_id out of range");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "rib_along_path DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Use the host face center's Z as the base plane for the ribs.  We treat
    // the host face as the XY plane at that Z and the polyline (u,v) values as
    // world (x,y).  Sufficient for the flat caseback / phone bottom geometry
    // these ribs target.
    const gp_Pnt hostC = wp.faceCenter(in.host_face_id);
    const double baseZ = hostC.Z();

    const double thk  = in.rib_thickness_mm;
    const double hgt  = in.rib_height_mm;
    constexpr double kEmbed = 0.02;       // small overlap for clean fuse

    // For each segment build a thin box and fuse it.
    std::vector<TopoDS_Shape> ribs;
    ribs.reserve(in.path_polyline.size() - 1);
    double totalAddedVol = 0.0;

    for (size_t i = 0; i + 1 < in.path_polyline.size(); ++i) {
        const double x0 = in.path_polyline[i].first;
        const double y0 = in.path_polyline[i].second;
        const double x1 = in.path_polyline[i + 1].first;
        const double y1 = in.path_polyline[i + 1].second;

        const double dx = x1 - x0;
        const double dy = y1 - y0;
        const double seg = std::sqrt(dx * dx + dy * dy);
        if (seg < 1e-6) continue;   // skip degenerate segment

        const double midX = (x0 + x1) * 0.5;
        const double midY = (y0 + y1) * 0.5;

        // Build a box centred at (midX, midY, baseZ - kEmbed):
        //   local Z = +Z (extrude up by hgt + kEmbed)
        //   local X = segment direction (length = seg)
        //   local Y = perpendicular in XY (width = thk)
        const gp_Dir xLoc(dx / seg, dy / seg, 0.0);
        // Origin = mid - (seg/2) xLoc - (thk/2) yLoc, with Y = Z × X
        const gp_Dir zLoc(0.0, 0.0, 1.0);
        gp_Dir yLoc;
        try {
            // yLoc derived: Z × X (right-hand)
            // = (0,0,1) × (xLoc.X, xLoc.Y, 0) = (-xLoc.Y, xLoc.X, 0)
            yLoc = gp_Dir(-xLoc.Y(), xLoc.X(), 0.0);
        } catch (const Standard_Failure&) {
            continue;
        }
        const double oX = midX - (seg / 2.0) * xLoc.X() - (thk / 2.0) * yLoc.X();
        const double oY = midY - (seg / 2.0) * xLoc.Y() - (thk / 2.0) * yLoc.Y();
        const double oZ = baseZ - kEmbed;
        const gp_Pnt origin(oX, oY, oZ);
        const gp_Ax2 ax(origin, zLoc, xLoc);   // mainDir=+Z (DZ), xDir=xLoc
        try {
            // BRepPrimAPI_MakeBox(ax, dx, dy, dz):
            //   DX = seg  (along xLoc = ax.XDirection)
            //   DY = thk  (along ax.YDirection = zLoc × xLoc = yLoc)
            //   DZ = hgt + kEmbed (along zLoc = mainDir)
            TopoDS_Shape b = pr::box(ax, seg, thk, hgt + kEmbed);
            if (!b.IsNull()) {
                ribs.push_back(b);
                totalAddedVol += seg * thk * (hgt + kEmbed);
            }
        } catch (const Standard_Failure& ex) {
            spdlog::debug("rib_along_path: segment box failed — {}",
                          ex.what());
        }
    }

    if (ribs.empty()) {
        throw SkillError(
            "rib_along_path: no valid rib segments produced (all degenerate?)");
    }

    const double volBefore = volumeOf(wp.shape());

    // Fuse all rib boxes onto the workpiece, one at a time.
    TopoDS_Shape newShape = pr::fuseMany(wp.shape(), ribs);
    if (newShape.IsNull()) {
        throw SkillError("rib_along_path: fuseMany returned null shape");
    }

    const double volAfter = volumeOf(newShape);

    json params = {
        { "host_face_id",     in.host_face_id },
        { "rib_thickness_mm", in.rib_thickness_mm },
        { "rib_height_mm",    in.rib_height_mm },
        { "path_point_count", static_cast<int>(in.path_polyline.size()) },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "parameters",          params },
        { "derived_volume",      volAfter - volBefore },
        { "segment_count",       static_cast<int>(ribs.size()) },
        { "estimated_added_mm3", totalAddedVol },
        { "volume_before_mm3",   volBefore },
        { "volume_after_mm3",    volAfter },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "mold_feature";
    tooling.stock_removed_mm3 = -(volAfter - volBefore);   // added
    tooling.est_cycle_time_s  = 0.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::rib_along_path: segments={} vol {}→{} (+{})",
                  ribs.size(), volBefore, volAfter, volAfter - volBefore);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

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

}  // namespace koocadcam::skill::rib_along_path
