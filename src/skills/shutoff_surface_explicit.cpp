// @lat: [[engine/skills#shutoff_surface_explicit]]

#include "shutoff_surface_explicit.hpp"

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
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::shutoff_surface_explicit {

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

// Tiny but non-zero thickness so the ribbon is a proper solid.
constexpr double kRibbonThicknessMm = 0.20;

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.undercut_edge_polyline.size() < 2) {
        r.add("DFM-INPUT", "error",
              "shutoff_surface_explicit: undercut_edge_polyline must have ≥ 2 points (got " +
              std::to_string(in.undercut_edge_polyline.size()) + ")");
    }
    if (!(in.shutoff_face_size_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "shutoff_surface_explicit: shutoff_face_size_mm must be > 0 (got " +
              std::to_string(in.shutoff_face_size_mm) + ")");
    } else if (in.shutoff_face_size_mm < 0.5) {
        r.add("DFM-SHUTOFF-CLEARANCE", "error",
              "shutoff_surface_explicit: shutoff_face_size_mm " +
              std::to_string(in.shutoff_face_size_mm) +
              " < 0.5 mm minimum clearance");
    }
    const double pdLen = std::sqrt(in.pull_dir_x * in.pull_dir_x +
                                   in.pull_dir_y * in.pull_dir_y +
                                   in.pull_dir_z * in.pull_dir_z);
    if (pdLen < 1e-9) {
        r.add("DFM-INPUT", "error",
              "shutoff_surface_explicit: pull_dir_xyz is zero vector");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "shutoff_surface_explicit DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Normalised pull direction.
    const double pdLen = std::sqrt(in.pull_dir_x * in.pull_dir_x +
                                   in.pull_dir_y * in.pull_dir_y +
                                   in.pull_dir_z * in.pull_dir_z);
    const gp_Dir pullDir(in.pull_dir_x / pdLen,
                         in.pull_dir_y / pdLen,
                         in.pull_dir_z / pdLen);

    const double thk  = kRibbonThicknessMm;
    const double size = in.shutoff_face_size_mm;

    std::vector<TopoDS_Shape> ribbons;
    ribbons.reserve(in.undercut_edge_polyline.size() - 1);

    for (size_t i = 0; i + 1 < in.undercut_edge_polyline.size(); ++i) {
        const gp_Pnt& p0 = in.undercut_edge_polyline[i];
        const gp_Pnt& p1 = in.undercut_edge_polyline[i + 1];
        gp_Vec axVec(p0, p1);
        const double segLen = axVec.Magnitude();
        if (segLen < 1e-6) continue;
        axVec.Normalize();
        const gp_Dir xLoc(axVec.X(), axVec.Y(), axVec.Z());

        // Build a box with local Z = pullDir (height = size), DX along
        // segment (length = segLen), DY = xLoc × pullDir (width = thk).
        gp_Vec yVec = gp_Vec(pullDir).Crossed(gp_Vec(xLoc));
        if (yVec.Magnitude() < 1e-9) {
            // pullDir parallel to segment — pick world Y or Z as fallback.
            yVec = (std::abs(xLoc.Z()) > 0.9) ? gp_Vec(0, 1, 0) : gp_Vec(0, 0, 1);
            // Reproject so it's perpendicular to xLoc.
            gp_Vec xVec(xLoc);
            yVec = yVec - xVec * yVec.Dot(xVec);
            if (yVec.Magnitude() < 1e-9) yVec = gp_Vec(0, 1, 0);
        }
        yVec.Normalize();

        // Origin: at p0 minus half-thickness in yLoc, minus half-thickness
        // axially in xLoc isn't needed (box starts at p0 and grows DX
        // toward p1).  Shift along -yLoc by thk/2 to centre the ribbon on
        // the edge line.
        const gp_Pnt origin(
            p0.X() - 0.5 * thk * yVec.X(),
            p0.Y() - 0.5 * thk * yVec.Y(),
            p0.Z() - 0.5 * thk * yVec.Z());

        // gp_Ax2(origin, mainDir, xDir):
        //   mainDir = pullDir (local +Z so DZ extrudes along pullDir)
        //   xDir    = xLoc    (local +X so DX runs from p0→p1)
        //   yDir    = mainDir × xDir = pullDir × xLoc (the y vector we use)
        const gp_Ax2 ax(origin, pullDir, xLoc);
        try {
            ribbons.push_back(pr::box(ax, segLen, thk, size));
        } catch (const Standard_Failure& ex) {
            spdlog::debug("shutoff_surface_explicit: segment {} box failed — {}",
                          i, ex.what());
        }
    }

    if (ribbons.empty()) {
        throw SkillError(
            "shutoff_surface_explicit: no valid ribbon segments produced");
    }

    const double volBefore = volumeOf(wp.shape());
    TopoDS_Shape newShape  = pr::fuseMany(wp.shape(), ribbons);
    if (newShape.IsNull()) {
        throw SkillError("shutoff_surface_explicit: fuseMany returned null shape");
    }
    const double volAfter = volumeOf(newShape);
    const double added    = volAfter - volBefore;

    json polyArr = json::array();
    for (const auto& p : in.undercut_edge_polyline) {
        polyArr.push_back({ p.X(), p.Y(), p.Z() });
    }
    json params = {
        { "undercut_edge_polyline", polyArr },
        { "pull_dir",               { pullDir.X(), pullDir.Y(), pullDir.Z() } },
        { "shutoff_face_size_mm",   in.shutoff_face_size_mm },
    };
    json pattern = {
        { "kind",                    kSkillId },
        { "is_compound",             true },
        { "mold_feature_type",       "shutoff_surface" },
        { "derived_volume_removed",  -added },     // negative = added
        { "segment_count",           static_cast<int>(ribbons.size()) },
        { "volume_before_mm3",       volBefore },
        { "volume_after_mm3",        volAfter },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "mold_feature";
    tooling.stock_removed_mm3 = -added;
    tooling.est_cycle_time_s  = 0.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::shutoff_surface_explicit applied: segments={} added_vol={}",
                  ribbons.size(), added);

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

}  // namespace koocadcam::skill::shutoff_surface_explicit
