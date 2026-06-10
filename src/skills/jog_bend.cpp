// @lat: [[engine/skills#jog_bend]]

#include "jog_bend.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <Standard_Failure.hxx>
#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::jog_bend {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

double sheetThicknessOf(const Workpiece& wp)
{
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    return std::min({ xMax - xMin, yMax - yMin, zMax - zMin });
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    const double t = sheetThicknessOf(wp);
    if (t < 0.5 || t > 6.0) {
        r.add("DFM-J-SHEET", "error",
              "jog_bend: stock thickness " + std::to_string(t) +
              " mm outside 0.5–6.0 mm sheet-metal range");
    }
    if (in.jog_angle_deg < 30.0 || in.jog_angle_deg > 60.0) {
        r.add("DFM-J-ANGLE", "error",
              "jog_bend: jog_angle_deg " + std::to_string(in.jog_angle_deg) +
              " outside 30–60° typical range");
    }
    if (t > 0.0 && in.jog_offset_mm <= t) {
        r.add("DFM-J-OFFSET", "error",
              "jog_bend: jog_offset_mm " + std::to_string(in.jog_offset_mm) +
              " mm ≤ sheet thickness (" + std::to_string(t) +
              " mm) — cannot form jog");
    }
    if (t > 0.0 && in.inner_radius_mm < 0.5 * t) {
        r.add("DFM-J-MIN-R", "error",
              "jog_bend: inner_radius_mm " + std::to_string(in.inner_radius_mm) +
              " mm < 0.5 × thickness — cracking risk");
    }
    const double dx = in.bend_axis_end_xy[0] - in.bend_axis_start_xy[0];
    const double dy = in.bend_axis_end_xy[1] - in.bend_axis_start_xy[1];
    if (std::hypot(dx, dy) < 1e-6) {
        r.add("DFM-INPUT", "error",
              "jog_bend: bend axis endpoints must be distinct");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "jog_bend DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double t = sheetThicknessOf(wp);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    // Bend axis line in XY @ sheet bottom.
    const gp_Pnt pA(in.bend_axis_start_xy[0], in.bend_axis_start_xy[1], zMin);
    const gp_Pnt pB(in.bend_axis_end_xy[0],   in.bend_axis_end_xy[1],   zMin);
    gp_Vec dirV(pA, pB);
    const double lineLen = dirV.Magnitude();
    dirV.Normalize();
    const gp_Vec perpRight(-dirV.Y(), dirV.X(), 0.0);

    // Split sheet at axis: keep LEFT half fixed, lift RIGHT half by jog_offset.
    auto buildHalfCutter = [&](const gp_Vec& removeDir) -> TopoDS_Shape
    {
        const double L  = bboxDiag * 2.0;
        const gp_Pnt mid((pA.X() + pB.X()) * 0.5,
                         (pA.Y() + pB.Y()) * 0.5,
                         (zMin + zMax) * 0.5);
        // Cross product DZ × dirV → localY.
        const gp_Vec localY(-dirV.Y(), dirV.X(), 0.0);
        const double sign = (localY.Dot(removeDir) > 0.0) ? 1.0 : -1.0;
        const double yShift = (sign > 0.0) ? 0.0 : -L;
        const gp_Pnt corner(
            mid.X() - dirV.X() * L + localY.X() * yShift,
            mid.Y() - dirV.Y() * L + localY.Y() * yShift,
            zMin - 1.0);
        const gp_Ax2 ax(corner, gp::DZ(), gp_Dir(dirV));
        return pr::box(ax, 2.0 * L, L, bboxDiag * 2.0 + 2.0);
    };

    const TopoDS_Shape rightCutter = buildHalfCutter( perpRight);
    const TopoDS_Shape leftCutter  = buildHalfCutter(-perpRight);
    TopoDS_Shape H_fixed, H_far_raw;
    try {
        H_fixed   = pr::cut(wp.shape(), rightCutter);
        H_far_raw = pr::cut(wp.shape(), leftCutter);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("jog_bend: split failed: ") + ex.what());
    }

    // Translate far half UP by jog_offset_mm.
    gp_Trsf trs;
    trs.SetTranslation(gp_Vec(0.0, 0.0, in.jog_offset_mm));
    BRepBuilderAPI_Transform xfTrs(H_far_raw, trs, /*Copy=*/false);
    if (!xfTrs.IsDone())
        throw SkillError("jog_bend: jog translation failed");
    const TopoDS_Shape H_far = xfTrs.Shape();

    // Build two bend wedges as cylindrical annuli at the transitions:
    //   - inner annulus at the LEFT (fixed) side at axis (cylinder axis = bend axis)
    //   - inner annulus at the RIGHT (far) side at axis raised by jog_offset
    auto buildAnnulus = [&](const gp_Pnt& origin) -> TopoDS_Shape
    {
        // gp_Ax2 main = dirV (bend axis); xDir = perpendicular.
        gp_Dir bdir(dirV);
        gp_Dir xDir;
        if (std::abs(bdir.Dot(gp::DX())) < 0.9) xDir = gp::DX();
        else                                     xDir = gp::DY();
        // Orthogonalize.
        gp_Vec v(xDir);
        gp_Vec b(bdir);
        v = v - b.Multiplied(v.Dot(b));
        v.Normalize();
        xDir = gp_Dir(v);
        const gp_Ax2 ax(origin, bdir, xDir);
        const double H = lineLen + 0.2;
        BRepPrimAPI_MakeCylinder outerC(ax, in.inner_radius_mm + t, H);
        outerC.Build();
        if (!outerC.IsDone()) throw Standard_Failure("jog_bend: outer cyl failed");
        BRepPrimAPI_MakeCylinder innerC(ax, in.inner_radius_mm, H);
        innerC.Build();
        if (!innerC.IsDone()) throw Standard_Failure("jog_bend: inner cyl failed");
        return pr::cut(outerC.Shape(), innerC.Shape());
    };

    TopoDS_Shape result;
    try {
        result = pr::fuse(H_fixed, H_far);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("jog_bend: fuse halves failed: ") + ex.what());
    }

    try {
        const TopoDS_Shape wedgeLow = buildAnnulus(pA);
        const TopoDS_Shape wedgeHigh = buildAnnulus(
            gp_Pnt(pA.X(), pA.Y(), pA.Z() + in.jog_offset_mm));
        result = pr::fuse(result, wedgeLow);
        result = pr::fuse(result, wedgeHigh);
    } catch (const Standard_Failure& ex) {
        spdlog::warn("jog_bend: wedge fuse failed: {}; keeping plain offset", ex.what());
    }

    // Signature.
    json params = {
        { "bend_axis_start_xy", { in.bend_axis_start_xy[0], in.bend_axis_start_xy[1] } },
        { "bend_axis_end_xy",   { in.bend_axis_end_xy[0],   in.bend_axis_end_xy[1]   } },
        { "jog_offset_mm",      in.jog_offset_mm },
        { "jog_angle_deg",      in.jog_angle_deg },
        { "inner_radius_mm",    in.inner_radius_mm },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "sheet_feature_type",    "jog_z_bend" },
        { "jog_offset_mm",         in.jog_offset_mm },
        { "jog_angle_deg",         in.jog_angle_deg },
        { "inner_radius_mm",       in.inner_radius_mm },
        { "bend_axis_dir",         { dirV.X(), dirV.Y(), dirV.Z() } },
        { "sheet_thickness_mm",    t },
        { "bend_count",            2 },             // two bends in a jog
    };
    ToolingMeta tooling;
    tooling.tool_type         = "press_brake";
    tooling.tool_dia_mm       = in.inner_radius_mm * 2.0;
    tooling.tool_length_mm    = lineLen;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 4.0 + lineLen * 0.05;   // 2 brake hits
    tooling.extra["machining_constraint"] =
        "Z-bend (jog) — two opposing press-brake bends in sequence; "
        "needs special jog tooling or two die stations; "
        "min jog_offset > sheet thickness";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(result, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::jog_bend applied: offset={} angle={} R={} faces {}→{}",
                  in.jog_offset_mm, in.jog_angle_deg, in.inner_radius_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// A jog leaves:
//   - 2 large +Z planar flats at DIFFERENT z levels (offset = jog_offset)
//   - 2 horizontal-axis cylindrical bend faces between them.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const double t = sheetThicknessOf(wp);
    if (t <= 0.0 || t > 6.0) return out;

    // Collect large +Z planar faces.
    struct Flat { int id; double z; double area; };
    std::vector<Flat> flats;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (n.Z() < 0.85) continue;
        flats.push_back({ i, wp.faceCenter(i).Z(), wp.faceArea(i) });
    }
    if (flats.size() < 2) return out;

    // Sort by area DESC.
    std::sort(flats.begin(), flats.end(),
              [](const Flat& a, const Flat& b) { return a.area > b.area; });

    // Find two large flats at different z levels.
    int idA = -1, idB = -1;
    double zA = 0, zB = 0;
    for (size_t i = 0; i < flats.size() && idA < 0; ++i) {
        idA = flats[i].id; zA = flats[i].z;
        for (size_t j = i + 1; j < flats.size(); ++j) {
            if (std::abs(flats[j].z - zA) > 0.5 * t &&
                std::abs(flats[j].z - zA) > 0.5) {
                idB = flats[j].id; zB = flats[j].z;
                break;
            }
        }
        if (idB < 0) idA = -1;
    }
    if (idA < 0 || idB < 0) return out;

    const double jogOffset = std::abs(zA - zB);

    // Count horizontal-axis cylindrical bend faces.
    int bendCount = 0;
    double bendR = 0.0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        const gp_Cylinder cyl = surf.Cylinder();
        if (std::abs(cyl.Axis().Direction().Z()) > 0.3) continue;
        ++bendCount;
        if (bendR == 0.0 || cyl.Radius() < bendR) bendR = cyl.Radius();
    }
    if (bendCount < 2) return out;

    json recovered = {
        { "bend_axis_start_xy", { 0.0, 0.0 } },
        { "bend_axis_end_xy",   { 1.0, 0.0 } },
        { "jog_offset_mm",      jogOffset },
        { "jog_angle_deg",      45.0 },          // canonical
        { "inner_radius_mm",    bendR > 0.0 ? bendR : 1.0 },
    };
    json matched = {
        { "near_flat_id",   idA },
        { "far_flat_id",    idB },
        { "bend_count",     bendCount },
        { "sheet_thickness_mm", t },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.70, matched });
    return out;
}

}  // namespace koocadcam::skill::jog_bend
