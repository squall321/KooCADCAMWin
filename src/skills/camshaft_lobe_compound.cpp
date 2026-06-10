// @lat: [[engine/skills#camshaft_lobe_compound]]

#include "camshaft_lobe_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::camshaft_lobe_compound {

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

// Cam profile r(theta_deg).  theta = 0 deg = nose direction (max lift).
// Profile rises in duration window centered at 0 deg, peaks at center, falls
// off symmetrically.  Ramp portion has linear rise; flat top inside (duration
// - 2*ramp) range at max lift; base circle outside duration.
double camProfileRadius(double theta_deg, double baseR, double maxLift,
                        double durationDeg, double rampDeg)
{
    // Normalize theta to [-180, 180].
    double t = std::fmod(theta_deg + 180.0, 360.0);
    if (t < 0) t += 360.0;
    t -= 180.0;
    const double a = std::abs(t);
    const double halfDur  = durationDeg / 2.0;
    if (a >= halfDur) return baseR;
    const double flatHalf = std::max(0.0, halfDur - rampDeg);
    if (a <= flatHalf) return baseR + maxLift;
    // Linear ramp from base at a=halfDur to peak at a=flatHalf.
    const double frac = (halfDur - a) / std::max(rampDeg, 1.0);
    return baseR + maxLift * std::min(1.0, std::max(0.0, frac));
}

// Build a rectangular wedge cutter: a thin angular slice from r=rIn to r=rOut
// at azimuth theta_deg, with axial extent [zLo, zHi].
TopoDS_Shape wedgeCutter(double theta_deg, double rIn, double rOut,
                         double zLo, double zHi, double sliceWidthDeg)
{
    const double theta = theta_deg * M_PI / 180.0;
    const double dtheta = (sliceWidthDeg * M_PI / 180.0) / 2.0;
    const double cx0 = std::cos(theta - dtheta);
    const double sx0 = std::sin(theta - dtheta);
    const double cx1 = std::cos(theta + dtheta);
    const double sx1 = std::sin(theta + dtheta);

    const gp_Pnt p1(rIn  * cx0, rIn  * sx0, zLo);
    const gp_Pnt p2(rOut * cx0, rOut * sx0, zLo);
    const gp_Pnt p3(rOut * cx1, rOut * sx1, zLo);
    const gp_Pnt p4(rIn  * cx1, rIn  * sx1, zLo);

    BRepBuilderAPI_MakeWire wm;
    wm.Add(BRepBuilderAPI_MakeEdge(p1, p2).Edge());
    wm.Add(BRepBuilderAPI_MakeEdge(p2, p3).Edge());
    wm.Add(BRepBuilderAPI_MakeEdge(p3, p4).Edge());
    wm.Add(BRepBuilderAPI_MakeEdge(p4, p1).Edge());
    if (!wm.IsDone()) throw Standard_Failure("wedgeCutter: wire failed");
    const TopoDS_Wire w = wm.Wire();

    BRepBuilderAPI_MakeFace fm(gp_Pln(gp_Pnt(0, 0, zLo), gp::DZ()), w);
    if (!fm.IsDone()) throw Standard_Failure("wedgeCutter: face failed");
    const TopoDS_Face f = fm.Face();

    BRepPrimAPI_MakePrism prism(f, gp_Vec(0, 0, zHi - zLo));
    prism.Build();
    if (!prism.IsDone()) throw Standard_Failure("wedgeCutter: prism failed");
    return prism.Shape();
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "camshaft_lobe_compound: face_id out of range");
    }
    if (!(in.base_circle_radius_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "camshaft_lobe_compound: base_circle_radius_mm must be > 0");
    }
    if (!(in.max_lift_mm > 0.5)) {
        r.add("DFM-CAM-LIFT", "error",
              "camshaft_lobe_compound: max_lift_mm (" +
              std::to_string(in.max_lift_mm) +
              ") must be > 0.5 mm (SAE J realistic cam lift)");
    }
    if (in.ramp_angle_deg <= 0.0 || in.ramp_angle_deg >= 90.0) {
        r.add("DFM-CAM-RAMP", "error",
              "camshaft_lobe_compound: ramp_angle_deg must be in (0, 90)");
    }
    if (in.duration_deg <= 10.0 || in.duration_deg >= 350.0) {
        r.add("DFM-CAM-DURATION", "error",
              "camshaft_lobe_compound: duration_deg must be in (10, 350)");
    }
    if (in.lobe_width_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "camshaft_lobe_compound: lobe_width_mm must be > 0");
    }
    if (in.profile_sample_count < 8 || in.profile_sample_count > 360) {
        r.add("DFM-CAM-SAMPLES", "error",
              "camshaft_lobe_compound: profile_sample_count must be in [8, 360]");
    }
    if (!wp.shape().IsNull()) {
        const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
        const double shaftR = bb.outerRadiusXY();
        if (in.base_circle_radius_mm + in.max_lift_mm >= shaftR + 0.05) {
            r.add("DFM-CAM-OD", "error",
                  "camshaft_lobe_compound: base + max_lift (" +
                  std::to_string(in.base_circle_radius_mm + in.max_lift_mm) +
                  ") must be < shaft OD radius (" +
                  std::to_string(shaftR) + ")");
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "camshaft_lobe_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double shaftR = bb.outerRadiusXY();
    const double zLo    = in.lobe_position_z_mm - in.lobe_width_mm / 2.0;
    const double zHi    = in.lobe_position_z_mm + in.lobe_width_mm / 2.0;

    constexpr double kOverhangR = 0.5;
    constexpr double kOverhangZ = 0.01;
    const double zLoCut = zLo - kOverhangZ;
    const double zHiCut = zHi + kOverhangZ;
    const double rOut   = shaftR + kOverhangR;

    // Build N wedge cutters: each cutter eats material from r=r_profile(theta)
    // outward to rOut over the lobe Z-band.  Wedge angular width is 360/N.
    const int    N         = in.profile_sample_count;
    const double sliceDeg  = 360.0 / static_cast<double>(N);

    std::vector<TopoDS_Shape> cutters;
    cutters.reserve(static_cast<std::size_t>(N));
    int activeCutters = 0;
    for (int i = 0; i < N; ++i) {
        const double theta = (i + 0.5) * sliceDeg - 180.0;
        const double rProfile = camProfileRadius(
            theta, in.base_circle_radius_mm,
            in.max_lift_mm, in.duration_deg, in.ramp_angle_deg);
        // Only cut if profile is strictly less than shaft OD.
        if (rProfile + 1e-3 >= shaftR) continue;
        try {
            const TopoDS_Shape w = wedgeCutter(
                theta, rProfile, rOut, zLoCut, zHiCut, sliceDeg + 0.5);
            cutters.push_back(w);
            ++activeCutters;
        } catch (const Standard_Failure& ex) {
            spdlog::debug("camshaft_lobe_compound: wedge {} failed — {}",
                          i, ex.what());
        }
    }

    if (cutters.empty()) {
        throw SkillError("camshaft_lobe_compound: no valid wedge cutters built");
    }

    const double volBefore = volumeOf(wp.shape());
    TopoDS_Shape newShape = wp.shape();
    int appliedCuts = 0;
    for (const auto& c : cutters) {
        try {
            const TopoDS_Shape cut1 = pr::cut(newShape, c);
            if (!cut1.IsNull()) {
                newShape = cut1;
                ++appliedCuts;
            }
        } catch (const Standard_Failure& ex) {
            spdlog::debug("camshaft_lobe_compound: wedge cut failed — {}",
                          ex.what());
        }
    }
    if (appliedCuts == 0) {
        throw SkillError("camshaft_lobe_compound: no wedge cut succeeded");
    }
    const double volAfter   = volumeOf(newShape);
    const double removedVol = volBefore - volAfter;

    json params = {
        { "face_id",                in.face_id },
        { "lobe_position_z_mm",     in.lobe_position_z_mm },
        { "base_circle_radius_mm",  in.base_circle_radius_mm },
        { "max_lift_mm",            in.max_lift_mm },
        { "ramp_angle_deg",         in.ramp_angle_deg },
        { "duration_deg",           in.duration_deg },
        { "lobe_width_mm",          in.lobe_width_mm },
        { "profile_sample_count",   in.profile_sample_count },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "engine_feature_type",        "camshaft_lobe" },
        { "subfeature_count",           activeCutters },
        { "profile_sample_count",       in.profile_sample_count },
        { "base_circle_radius_mm",      in.base_circle_radius_mm },
        { "max_lift_mm",                in.max_lift_mm },
        { "duration_deg",               in.duration_deg },
        { "lobe_width_mm",              in.lobe_width_mm },
        { "derived_volume_removed_mm3", removedVol },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "cam_grinder";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = in.lobe_width_mm + 5.0;
    tooling.tool_material     = "CBN";
    tooling.flute_count       = 1;
    tooling.cutting_speed_sfm = 6000.0;     // cam grinder spec
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  =
        std::max(10.0, in.lobe_width_mm * 1.0 + 5.0);
    tooling.extra = {
        { "feature_standard", "SAE J camshaft lobe profile" },
        { "ramp_angle_deg",   in.ramp_angle_deg },
        { "duration_deg",     in.duration_deg },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::camshaft_lobe_compound applied: "
                  "baseR={}mm lift={}mm duration={} samples={}",
                  in.base_circle_radius_mm, in.max_lift_mm,
                  in.duration_deg, in.profile_sample_count);

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
              { "engine_feature_type",  "camshaft_lobe" } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::camshaft_lobe_compound
