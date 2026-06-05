// @lat: [[engine/skills#gripper_finger_dovetail_mount]]

#include "gripper_finger_dovetail_mount.hpp"

#include "Workpiece.hpp"
#include "_iso_thread_table.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <Standard_Failure.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::gripper_finger_dovetail_mount {

namespace pr = koocadcam::engine::prim;
namespace tt = koocadcam::skill::thread_table;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.dovetail_width_mm <= 0.0 || in.dovetail_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "gripper_finger_dovetail_mount: all dimensions must be > 0");
    }

    if (in.dovetail_angle_deg <= 0.0 || in.dovetail_angle_deg > 30.0) {
        r.add("DFM-ROBOTICS-ANGLE", "error",
              "gripper_finger_dovetail_mount: dovetail_angle_deg (" +
              std::to_string(in.dovetail_angle_deg) +
              ") must be in (0, 30] degrees from vertical");
    }

    if (!tt::findMetric(in.clamp_thread_key)) {
        r.add("DFM-THREAD", "error",
              "gripper_finger_dovetail_mount: clamp_thread_key '" +
              in.clamp_thread_key + "' not in central metric thread table");
    }

    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "gripper_finger_dovetail_mount DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const tt::MetricThreadSpec* thread = tt::findMetric(in.clamp_thread_key);
    if (!thread) throw SkillError("gripper_finger_dovetail_mount: thread lookup failed");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double cx = in.center_xy.X();

    // ── 1) Dovetail slot — trapezoid prism (wider at the bottom) ───────
    // Cross-section in the XZ plane, extruded along +Y through the blank.
    const double topHalf = in.dovetail_width_mm / 2.0;
    const double botHalf = topHalf +
        in.dovetail_depth_mm * std::tan(in.dovetail_angle_deg * M_PI / 180.0);
    const double zTop = topZ + kOver;                  // slightly proud of top
    const double zBot = topZ - in.dovetail_depth_mm;   // slot floor
    const double yStart = yMin - kOver;
    const double slotLenY = (yMax - yMin) + 2.0 * kOver;

    // Trapezoid vertices (XZ plane at y = yStart): mouth narrow, floor wide.
    const gp_Pnt p0(cx - topHalf, yStart, zTop);
    const gp_Pnt p1(cx + topHalf, yStart, zTop);
    const gp_Pnt p2(cx + botHalf, yStart, zBot);
    const gp_Pnt p3(cx - botHalf, yStart, zBot);

    TopoDS_Shape dovetailTool;
    try {
        BRepBuilderAPI_MakePolygon poly(p0, p1, p2, p3, /*Close=*/true);
        if (!poly.IsDone()) throw Standard_Failure("dovetail polygon");
        BRepBuilderAPI_MakeFace fm(poly.Wire(), true);
        if (!fm.IsDone()) throw Standard_Failure("dovetail face");
        BRepPrimAPI_MakePrism prism(fm.Face(), gp_Vec(0.0, slotLenY, 0.0));
        prism.Build();
        if (!prism.IsDone()) throw Standard_Failure("dovetail prism");
        dovetailTool = prism.Shape();
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("gripper_finger_dovetail_mount: ") + ex.what());
    }

    TopoDS_Shape current = pr::cut(wp.shape(), dovetailTool);  // sequential

    // ── 2) Transverse clamp-screw bore (across X, through the slot) ────
    // Drilled along +X at mid-depth of the dovetail, on the finger's Y centre.
    const double clampR = thread->tap_pilot_dia_mm / 2.0;
    const double clampZ = topZ - in.dovetail_depth_mm / 2.0;
    const double clampY = (yMin + yMax) / 2.0;
    const double clampLen = (xMax - xMin) + 2.0 * kOver;
    const gp_Pnt clampStart(xMin - kOver, clampY, clampZ);
    const gp_Ax2 clampAx(clampStart, gp::DX());
    current = pr::cut(current, pr::cylinder(clampAx, clampR, clampLen));  // sequential

    const TopoDS_Shape newShape = current;

    // ── Derived volume (analytic) ──────────────────────────────────────
    const double slotLen = (yMax - yMin);
    const double trapArea = (topHalf + botHalf) * in.dovetail_depth_mm;  // (a+b)/2*h *2 halves
    const double vDovetail = trapArea * slotLen;
    const double vClamp = M_PI * clampR * clampR * (xMax - xMin);
    const double volRemoved = vDovetail + vClamp;

    const int subfeatureCount = 2;

    json params = {
        { "center_xy",          { in.center_xy.X(),
                                  in.center_xy.Y(),
                                  in.center_xy.Z() } },
        { "dovetail_width_mm",  in.dovetail_width_mm },
        { "dovetail_angle_deg", in.dovetail_angle_deg },
        { "dovetail_depth_mm",  in.dovetail_depth_mm },
        { "clamp_thread_key",   in.clamp_thread_key },
    };

    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "robotics_feature_type",      "gripper_finger_dovetail" },
        { "subfeature_count",           subfeatureCount },
        { "dovetail_width_mm",          in.dovetail_width_mm },
        { "dovetail_angle_deg",         in.dovetail_angle_deg },
        { "dovetail_depth_mm",          in.dovetail_depth_mm },
        { "clamp_thread_key",           in.clamp_thread_key },
        { "derived_dovetail_bottom_mm", 2.0 * botHalf },
        { "derived_clamp_pilot_dia_mm", thread->tap_pilot_dia_mm },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "gripper finger dovetail interface practice" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "dovetail_cutter;drill";
    tooling.tool_dia_mm       = std::max(in.dovetail_width_mm, thread->nominal_dia_mm);
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = 45.0;
    tooling.extra = {
        { "robotics_application", "gripper_finger_dovetail" },
        { "removed_volume_mm3",   volRemoved },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::gripper_finger_dovetail_mount: w={} angle={} depth={} faces {}→{}",
                  in.dovetail_width_mm, in.dovetail_angle_deg,
                  in.dovetail_depth_mm, wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

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

    // Geometric fallback: a transverse clamp-bore cylinder along X plus the
    // extra planar flanks of the dovetail slot.
    int clampCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Cylinder c = s.Cylinder();
            if (std::abs(c.Axis().Direction().X()) < 0.9) continue;  // along X
            const double radius = c.Radius();
            if (radius >= 1.0 && radius < 6.0) ++clampCyls;
        } catch (...) {}
    }
    if (clampCyls >= 1) {
        json recovered = { { "dovetail_width_mm", 16.0 },
                           { "clamp_thread_key",  "M4" } };
        json matched   = { { "source",     "geometric_dovetail_clamp" },
                           { "clamp_cyls", clampCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::gripper_finger_dovetail_mount
