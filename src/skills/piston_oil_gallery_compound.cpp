// @lat: [[engine/skills#piston_oil_gallery_compound]]

#include "piston_oil_gallery_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::piston_oil_gallery_compound {

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

// Return any unit vector perpendicular to `n`.
gp_Dir perpendicularTo(const gp_Dir& n)
{
    // Pick world axis least aligned with n.
    const double ax = std::abs(n.X());
    const double ay = std::abs(n.Y());
    const double az = std::abs(n.Z());
    gp_Vec ref;
    if (az <= ax && az <= ay) ref = gp_Vec(0, 0, 1);
    else if (ay <= ax)        ref = gp_Vec(0, 1, 0);
    else                      ref = gp_Vec(1, 0, 0);
    const gp_Vec nv(n.X(), n.Y(), n.Z());
    gp_Vec perp = nv.Crossed(ref);
    if (perp.Magnitude() < 1e-9) perp = gp_Vec(1, 0, 0);
    perp.Normalize();
    return gp_Dir(perp);
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "piston_oil_gallery_compound: face_id out of range");
    }
    if (!(in.gallery_OD_mm > 0.0) || !(in.gallery_ID_mm > 0.0) ||
        !(in.gallery_height_mm > 0.0) ||
        !(in.inlet_hole_dia_mm > 0.0) ||
        !(in.drain_hole_dia_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "piston_oil_gallery_compound: all dimensions must be > 0");
        return r;
    }
    if (in.gallery_OD_mm <= in.gallery_ID_mm) {
        r.add("DFM-GALLERY-RING", "error",
              "piston_oil_gallery_compound: gallery_OD (" +
              std::to_string(in.gallery_OD_mm) +
              ") must be > gallery_ID (" +
              std::to_string(in.gallery_ID_mm) + ")");
    }
    const double wallThk = (in.gallery_OD_mm - in.gallery_ID_mm) / 2.0;
    if (in.inlet_hole_dia_mm >= wallThk) {
        r.add("DFM-INLET-WALL", "warning",
              "piston_oil_gallery_compound: inlet_hole_dia (" +
              std::to_string(in.inlet_hole_dia_mm) +
              ") >= gallery wall thickness (" +
              std::to_string(wallThk) + ") — undersized wall");
    }
    if (in.drain_hole_dia_mm >= wallThk) {
        r.add("DFM-DRAIN-WALL", "warning",
              "piston_oil_gallery_compound: drain_hole_dia >= gallery wall thickness");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "piston_oil_gallery_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Piston axis frame.
    const gp_Ax2 pistonAx(in.piston_axis_origin, in.piston_axis_dir);

    const double rO = in.gallery_OD_mm / 2.0;
    const double rI = in.gallery_ID_mm / 2.0;

    constexpr double kEmbed = 0.05;

    // ── Sub-feature 1: annular gallery cavity ────────────────────────────
    const TopoDS_Shape galleryAnnulus =
        pr::annularRing(pistonAx, rO, rI, in.gallery_height_mm + kEmbed);

    // ── Sub-features 2/3: inlet + drain holes ───────────────────────────
    // Holes run RADIALLY through the gallery wall (perpendicular to axis).
    // Inlet on one side, drain on opposite side.  We pick a perp axis and
    // its negation.
    const gp_Dir radialDir = perpendicularTo(in.piston_axis_dir);

    // Compute mid-Z of gallery along the piston axis.
    const gp_Vec axVec(in.piston_axis_dir.X(),
                       in.piston_axis_dir.Y(),
                       in.piston_axis_dir.Z());
    const gp_Pnt galleryMid(
        in.piston_axis_origin.X() + axVec.X() * (in.gallery_height_mm / 2.0),
        in.piston_axis_origin.Y() + axVec.Y() * (in.gallery_height_mm / 2.0),
        in.piston_axis_origin.Z() + axVec.Z() * (in.gallery_height_mm / 2.0));

    // Inlet starts outside OD on the +radial side and goes radially inward.
    // For a robust cut, make the hole long enough to span 2 * OD.
    const double inletLen = in.gallery_OD_mm * 1.2;
    const gp_Pnt inletStart(
        galleryMid.X() - radialDir.X() * (in.gallery_OD_mm * 0.6),
        galleryMid.Y() - radialDir.Y() * (in.gallery_OD_mm * 0.6),
        galleryMid.Z() - radialDir.Z() * (in.gallery_OD_mm * 0.6));
    const gp_Ax2 inletAx(inletStart, radialDir);
    const TopoDS_Shape inletHole =
        pr::cylinder(inletAx, in.inlet_hole_dia_mm / 2.0, inletLen);

    // Drain — opposite side (negative radial), separate axial offset (lower
    // edge of gallery) so it geometrically is distinct from the inlet.
    const gp_Dir drainRadial(-radialDir.X(), -radialDir.Y(), -radialDir.Z());
    const gp_Pnt drainCenter(
        in.piston_axis_origin.X() + axVec.X() * (in.gallery_height_mm * 0.25),
        in.piston_axis_origin.Y() + axVec.Y() * (in.gallery_height_mm * 0.25),
        in.piston_axis_origin.Z() + axVec.Z() * (in.gallery_height_mm * 0.25));
    const double drainLen = in.gallery_OD_mm * 1.2;
    const gp_Pnt drainStart(
        drainCenter.X() - drainRadial.X() * (in.gallery_OD_mm * 0.6),
        drainCenter.Y() - drainRadial.Y() * (in.gallery_OD_mm * 0.6),
        drainCenter.Z() - drainRadial.Z() * (in.gallery_OD_mm * 0.6));
    const gp_Ax2 drainAx(drainStart, drainRadial);
    const TopoDS_Shape drainHole =
        pr::cylinder(drainAx, in.drain_hole_dia_mm / 2.0, drainLen);

    // Cut all three at once.
    const double volBefore = volumeOf(wp.shape());
    const TopoDS_Shape allCutters =
        pr::fuse(pr::fuse(galleryAnnulus, inletHole), drainHole);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), allCutters);
    const double volAfter  = volumeOf(newShape);
    const double removedVol = volBefore - volAfter;

    json params = {
        { "face_id",                 in.face_id },
        { "piston_axis_dir",         { in.piston_axis_dir.X(),
                                       in.piston_axis_dir.Y(),
                                       in.piston_axis_dir.Z() } },
        { "piston_axis_origin",      { in.piston_axis_origin.X(),
                                       in.piston_axis_origin.Y(),
                                       in.piston_axis_origin.Z() } },
        { "gallery_OD_mm",           in.gallery_OD_mm },
        { "gallery_ID_mm",           in.gallery_ID_mm },
        { "gallery_height_mm",       in.gallery_height_mm },
        { "inlet_hole_dia_mm",       in.inlet_hole_dia_mm },
        { "drain_hole_dia_mm",       in.drain_hole_dia_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "engine_feature_type",        "piston_oil_gallery" },
        { "subfeature_count",           3 },
        { "gallery_OD_mm",              in.gallery_OD_mm },
        { "gallery_ID_mm",              in.gallery_ID_mm },
        { "gallery_height_mm",          in.gallery_height_mm },
        { "inlet_hole_dia_mm",          in.inlet_hole_dia_mm },
        { "drain_hole_dia_mm",          in.drain_hole_dia_mm },
        { "wall_thickness_mm",          (in.gallery_OD_mm - in.gallery_ID_mm) / 2.0 },
        { "derived_volume_removed_mm3", removedVol },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "core_pin;drill;drill";
    tooling.tool_dia_mm       = in.gallery_OD_mm;
    tooling.tool_length_mm    = in.gallery_height_mm + 5.0;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  =
        std::max(5.0, in.gallery_height_mm * 0.5 + 4.0);
    tooling.extra = {
        { "feature_standard", "Cast/forged piston oil cooling gallery" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::piston_oil_gallery_compound applied: "
                  "OD={}mm ID={}mm h={}mm",
                  in.gallery_OD_mm, in.gallery_ID_mm, in.gallery_height_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition — metadata replay ────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        out.push_back(RecognizedFeature{
            kSkillId, f.params, 1.0,
            { { "source",               "metadata_replay" },
              { "is_compound",          true },
              { "engine_feature_type",  "piston_oil_gallery" },
              { "subfeature_count",     3 } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::piston_oil_gallery_compound
