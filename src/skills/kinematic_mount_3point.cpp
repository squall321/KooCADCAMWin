// @lat: [[engine/skills#kinematic_mount_3point]]

#include "kinematic_mount_3point.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Shape.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::kinematic_mount_3point {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    if (s.IsNull()) return 0.0;
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

TopoDS_Shape makeSphereTool(const gp_Pnt& center, double radius)
{
    BRepPrimAPI_MakeSphere m(center, radius);
    m.Build();
    if (!m.IsDone())
        throw Standard_Failure("kinematic_mount_3point: sphere build failed");
    return m.Shape();
}

}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "kinematic_mount_3point: face_id out of range");
    }
    if (!(in.sphere_dia_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "kinematic_mount_3point: sphere_dia_mm must be > 0");
    }
    const std::array<std::string, 3> allowed { "sphere_pocket", "v_groove", "flat_pad" };
    bool seatOk = false;
    for (const auto& k : allowed) if (in.seat_kind == k) { seatOk = true; break; }
    if (!seatOk) {
        r.add("DFM-SEAT-KIND", "error",
              "kinematic_mount_3point: seat_kind must be one of "
              "{sphere_pocket, v_groove, flat_pad}, got '" + in.seat_kind + "'");
    }
    // 3 distinct seat positions
    auto sameXY = [](double ax, double ay, double bx, double by) {
        const double dx = ax - bx, dy = ay - by;
        return dx * dx + dy * dy < 1e-4;
    };
    if (sameXY(in.seat0_x_mm, in.seat0_y_mm, in.seat1_x_mm, in.seat1_y_mm) ||
        sameXY(in.seat1_x_mm, in.seat1_y_mm, in.seat2_x_mm, in.seat2_y_mm) ||
        sameXY(in.seat0_x_mm, in.seat0_y_mm, in.seat2_x_mm, in.seat2_y_mm)) {
        r.add("DFM-SEATS", "error",
              "kinematic_mount_3point: 3 seat positions must be distinct");
    }
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "kinematic_mount_3point DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const double baseZ = faceC.Z();
    constexpr double kEmbed = 0.05;
    const double R = in.sphere_dia_mm / 2.0;

    std::vector<TopoDS_Shape> cutters;
    cutters.reserve(3);

    // Seat 0: hemispherical pocket — sphere centred at baseZ (lower hemisphere
    // protrudes into the part below the face, so only the bottom half of the
    // sphere ends up as a pocket).
    {
        const gp_Pnt c(in.seat0_x_mm, in.seat0_y_mm, baseZ);
        cutters.push_back(makeSphereTool(c, R));
    }

    // Seat 1: V-groove — two long thin rectangular tools rotated ±45° about
    // their axial direction, intersecting at the seat XY point.  This is a
    // robust approximation of a V-groove cross-section using axis-aligned
    // rectangular cutters along the X-axis line.
    {
        const double Lg = std::max(8.0, in.sphere_dia_mm * 2.0);   // groove length
        const double Wg = std::max(0.4, in.sphere_dia_mm * 0.3);   // groove half-width
        const double Dg = std::max(0.5, in.sphere_dia_mm * 0.5);   // groove depth
        // Long rectangular groove along X, centred at seat 1, depth Dg.
        const gp_Pnt origin(in.seat1_x_mm - Lg / 2.0,
                            in.seat1_y_mm - Wg,
                            baseZ - Dg);
        const gp_Ax2 ax(origin, gp::DZ());
        cutters.push_back(pr::box(ax, Lg, 2.0 * Wg, Dg + kEmbed));
    }

    // Seat 2: shallow flat circular pad — small cylindrical pocket of slight
    // depth (≈ 0.15 × sphere_dia) and diameter ≈ 1.5 × sphere_dia.
    {
        const double padR     = R * 1.5;
        const double padDepth = std::max(0.3, R * 0.3);
        const gp_Pnt c(in.seat2_x_mm, in.seat2_y_mm, baseZ - padDepth);
        const gp_Ax2 ax(c, gp::DZ());
        cutters.push_back(pr::cylinder(ax, padR, padDepth + kEmbed));
    }

    const double volBefore = volumeOf(wp.shape());
    TopoDS_Shape newShape  = pr::cutMany(wp.shape(), cutters);
    if (newShape.IsNull())
        throw SkillError("kinematic_mount_3point: cutMany returned null");
    const double volRemoved = volBefore - volumeOf(newShape);

    json params = {
        { "face_id",       in.face_id },
        { "seat0_x_mm",    in.seat0_x_mm },
        { "seat0_y_mm",    in.seat0_y_mm },
        { "seat1_x_mm",    in.seat1_x_mm },
        { "seat1_y_mm",    in.seat1_y_mm },
        { "seat2_x_mm",    in.seat2_x_mm },
        { "seat2_y_mm",    in.seat2_y_mm },
        { "seat_kind",     in.seat_kind },
        { "sphere_dia_mm", in.sphere_dia_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "optical_feature_type",       "kinematic_mount" },
        { "seat_kind_primary",          in.seat_kind },
        { "subfeature_count",           3 },
        { "seat_count",                 3 },
        { "sphere_dia_mm",              in.sphere_dia_mm },
        { "derived_volume_removed_mm3", volRemoved },
        { "seat_kinds",                 { "sphere_pocket", "v_groove", "flat_pad" } },
    };
    ToolingMeta tooling;
    tooling.tool_type        = "ball_mill;end_mill";
    tooling.tool_dia_mm      = in.sphere_dia_mm;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = volRemoved / 40.0 + 10.0;
    tooling.extra = {
        { "spec_source", "Maxwell kinematic mount (3 distinct seat constraints)" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);
    spdlog::debug("skill::kinematic_mount_3point: kind={} sphere_dia={}",
                  in.seat_kind, in.sphere_dia_mm);
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

}  // namespace koocadcam::skill::kinematic_mount_3point
