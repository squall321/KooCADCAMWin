// @lat: [[engine/skills#dowel_pin_h6_bore]]
//
// ISO 286-1 H6 dowel-pin locating bore — drill, precision ream, micro-chamfer.

#include "dowel_pin_h6_bore.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Cylinder.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>

namespace koocadcam::skill::dowel_pin_h6_bore {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

struct H6Row { double dia_min; double dia_max; double upper_um; };

constexpr std::array<H6Row, 10> kH6Table {{
    { 0.0,   3.0,    6.0 },
    { 3.0,   6.0,    8.0 },
    { 6.0,  10.0,    9.0 },
    { 10.0, 18.0,   11.0 },
    { 18.0, 30.0,   13.0 },
    { 30.0, 50.0,   16.0 },
    { 50.0, 80.0,   19.0 },
    { 80.0, 120.0,  22.0 },
    { 120.0,180.0,  25.0 },
    { 180.0,250.0,  29.0 },
}};

// Standard cylindrical dowel-pin sizes (DIN 7 / ISO 2338).
constexpr std::array<double, 11> kStandardDowelDia {{
    1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0, 16.0
}};

TopoDS_Shape buildLeadInChamfer(const gp_Pnt& entryCenter, const gp_Dir& axis,
                                double finalDia, double chamfer_mm,
                                double entryOverhang)
{
    const double smallR = finalDia / 2.0;
    const double bigR   = smallR + chamfer_mm;
    const gp_Pnt apex(
        entryCenter.X() - axis.X() * entryOverhang,
        entryCenter.Y() - axis.Y() * entryOverhang,
        entryCenter.Z() - axis.Z() * entryOverhang);
    const gp_Ax2 ax(apex, axis);
    return pr::coneFrustum(ax, bigR, smallR, entryOverhang + chamfer_mm);
}

}  // namespace

double upperDeviationMicronsH6(double nominal_dia_mm)
{
    for (const auto& r : kH6Table) {
        if (nominal_dia_mm > r.dia_min && nominal_dia_mm <= r.dia_max) {
            return r.upper_um;
        }
    }
    return -1.0;
}

bool isStandardDowelSize(double nominal_dia_mm)
{
    for (double s : kStandardDowelDia) {
        if (std::abs(s - nominal_dia_mm) < 1e-3) return true;
    }
    return false;
}

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.spec_key != "H6") {
        r.add("DFM-SPEC", "error",
              "dowel_pin_h6_bore: unknown spec_key '" + in.spec_key +
              "' (only 'H6' is supported — see ISO 286-1 / DIN 7)");
    }
    if (in.nominal_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "dowel_pin_h6_bore: nominal_dia_mm must be > 0");
        return r;
    }
    if (upperDeviationMicronsH6(in.nominal_dia_mm) < 0.0) {
        r.add("DFM-SPEC-RANGE", "error",
              "dowel_pin_h6_bore: nominal_dia_mm " +
              std::to_string(in.nominal_dia_mm) +
              " out of ISO 286-1 H6 table range (1..250 mm)");
    }
    if (!isStandardDowelSize(in.nominal_dia_mm)) {
        r.add("DFM-DOWEL-SIZE", "warning",
              "dowel_pin_h6_bore: nominal_dia_mm " +
              std::to_string(in.nominal_dia_mm) +
              " not in standard DIN 7 dowel sizes "
              "(1.5/2/2.5/3/4/5/6/8/10/12/16) — custom pin will be needed");
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "dowel_pin_h6_bore: depth_mm must be > 0");
    }
    if (in.depth_mm < in.nominal_dia_mm) {
        r.add("DFM-DOWEL-ENGAGE", "warning",
              "dowel_pin_h6_bore: depth (" + std::to_string(in.depth_mm) +
              ") < dia (" + std::to_string(in.nominal_dia_mm) +
              ") — pin engagement < 1×D may shear under lateral load");
    }
    if (in.chamfer_mm > 0.5) {
        r.add("DFM-DOWEL-CHAMFER", "warning",
              "dowel_pin_h6_bore: chamfer_mm " + std::to_string(in.chamfer_mm) +
              " > 0.5 mm — large chamfer reduces effective locating contact");
    }
    const double ratio = (in.nominal_dia_mm > 0.0)
        ? (in.depth_mm / in.nominal_dia_mm) : 0.0;
    if (ratio > 6.0) {
        r.add("DFM-DOWEL-RATIO", "warning",
              "dowel_pin_h6_bore: depth/dia ratio " + std::to_string(ratio) +
              " > 6 — deep dowel hole hard to ream to H6 with handheld reamer");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "dowel_pin_h6_bore DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("dowel_pin_h6_bore: entry_face datum unresolved");

    const double upperDevMm = upperDeviationMicronsH6(in.nominal_dia_mm) * 1e-3;
    const double finalDia   = in.nominal_dia_mm + upperDevMm / 2.0;
    const double drillDia   = in.nominal_dia_mm;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kEntryOverhang = 0.05 + in.chamfer_mm + 0.5;
    const gp_Dir adir = in.axis_dir;

    gp_Pnt entryCenter(in.position_x_mm, in.position_y_mm, (zMin + zMax) / 2.0);
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        if (adir.Z() < 0) entryCenter.SetZ(zMax);
        else              entryCenter.SetZ(zMin);
    }
    const gp_Pnt toolStart(
        entryCenter.X() - adir.X() * kEntryOverhang,
        entryCenter.Y() - adir.Y() * kEntryOverhang,
        entryCenter.Z() - adir.Z() * kEntryOverhang);
    const gp_Ax2 toolAx(toolStart, adir);

    const TopoDS_Shape drillTool =
        pr::cylinder(toolAx, drillDia / 2.0, in.depth_mm + kEntryOverhang);
    const TopoDS_Shape reamTool =
        pr::cylinder(toolAx, finalDia / 2.0, in.depth_mm + kEntryOverhang);
    TopoDS_Shape fused = pr::fuse(drillTool, reamTool);

    if (in.chamfer_mm > 0.0) {
        const TopoDS_Shape chamferTool =
            buildLeadInChamfer(entryCenter, adir, finalDia, in.chamfer_mm,
                               kEntryOverhang);
        fused = pr::fuse(fused, chamferTool);
    }
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    json params = {
        { "entry_face_kind",  "resolved_id" },
        { "entry_face_id",    *entryId },
        { "position_x_mm",    in.position_x_mm },
        { "position_y_mm",    in.position_y_mm },
        { "axis_dir",         { adir.X(), adir.Y(), adir.Z() } },
        { "nominal_dia_mm",   in.nominal_dia_mm },
        { "depth_mm",         in.depth_mm },
        { "chamfer_mm",       in.chamfer_mm },
        { "spec_key",         in.spec_key },
        { "final_dia_mm",     finalDia },
        { "upper_dev_um",     upperDevMm * 1e3 },
        { "is_standard_dowel", isStandardDowelSize(in.nominal_dia_mm) },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "subfeature_count",           3 },
        { "spec_key",                   in.spec_key },
        { "nominal_dia_mm",             in.nominal_dia_mm },
        { "final_dia_mm",               finalDia },
        { "depth_mm",                   in.depth_mm },
        { "chamfer_mm",                 in.chamfer_mm },
        { "cylindrical_face_count",     1 },
        { "conical_face_count",         in.chamfer_mm > 0.0 ? 1 : 0 },
        { "bottom_planar_face_present", true },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type        = "drill;reamer;chamfer";
    tooling.tool_dia_mm      = finalDia;
    tooling.tool_length_mm   = in.depth_mm * 1.5 + 8.0;
    tooling.tool_material    = "hss-cobalt";    // H6 reamer typically HSS-Co
    tooling.flute_count      = 6;
    tooling.cutting_speed_sfm = 40.0;           // slow for H6
    tooling.feed_per_tooth_mm = 0.03;
    const double rF = finalDia / 2.0;
    tooling.stock_removed_mm3 = M_PI * rF * rF * in.depth_mm;
    tooling.est_cycle_time_s  = std::max(2.0, in.depth_mm / 20.0) + 2.5;
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },   { "tool_dia_mm", drillDia }, { "depth_mm", in.depth_mm } },
            { { "tool_type", "reamer" }, { "tool_dia_mm", finalDia },
              { "depth_mm", in.depth_mm }, { "spec_key", in.spec_key } },
            { { "tool_type", "chamfer" }, { "chamfer_mm", in.chamfer_mm } },
        } },
        { "spec_source", "ISO 286-1:2010 Table 2 (H6)" },
        { "mating_part", "DIN 7 / ISO 2338 cylindrical dowel pin (m6)" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);
    spdlog::debug("skill::dowel_pin_h6_bore applied: nominal {} → final {} mm",
                  in.nominal_dia_mm, finalDia);
    return SkillOutput{ wpNew, sig };
}

namespace {

struct CylInfo {
    int    faceIdx = -1;
    double radius  = 0.0;
    gp_Ax1 axis;
    gp_Pnt topCenter;
    gp_Pnt botCenter;
    double length = 0.0;
};

std::vector<CylInfo> collectCylinders(const Workpiece& wp)
{
    std::vector<CylInfo> out;
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        const TopoDS_Face& cf = wp.face(fIdx);
        BRepAdaptor_Surface surf(cf);
        const gp_Cylinder cyl = surf.Cylinder();
        CylInfo info; info.faceIdx = fIdx; info.radius = cyl.Radius(); info.axis = cyl.Axis();
        std::vector<gp_Pnt> centers;
        for (TopExp_Explorer exp(cf, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            if (std::abs(std::abs(c.Axis().Direction().Dot(info.axis.Direction())) - 1.0)
                > 1e-3) continue;
            if (std::abs(c.Radius() - info.radius) > 1e-2) continue;
            centers.push_back(c.Location());
        }
        if (centers.size() < 2) continue;
        const gp_Dir adir = info.axis.Direction();
        auto proj = [&](const gp_Pnt& p) {
            return (p.X() - info.axis.Location().X()) * adir.X() +
                   (p.Y() - info.axis.Location().Y()) * adir.Y() +
                   (p.Z() - info.axis.Location().Z()) * adir.Z();
        };
        auto minIt = std::min_element(centers.begin(), centers.end(),
            [&](const gp_Pnt& a, const gp_Pnt& b) { return proj(a) < proj(b); });
        auto maxIt = std::max_element(centers.begin(), centers.end(),
            [&](const gp_Pnt& a, const gp_Pnt& b) { return proj(a) < proj(b); });
        info.topCenter = *minIt;
        info.botCenter = *maxIt;
        info.length    = info.topCenter.Distance(info.botCenter);
        out.push_back(info);
    }
    return out;
}

// Snap to a standard dowel size, then verify H6 mid-tolerance.
double nearestStandardDowel(double measured_dia_mm, double tol_mm)
{
    double best = -1.0;
    double bestErr = tol_mm;
    for (double s : kStandardDowelDia) {
        const double dev = upperDeviationMicronsH6(s);
        if (dev < 0.0) continue;
        const double finalDia = s + (dev * 1e-3) / 2.0;
        const double err = std::abs(finalDia - measured_dia_mm);
        if (err < bestErr) { bestErr = err; best = s; }
    }
    return best;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const auto cyls = collectCylinders(wp);
    for (const auto& cyl : cyls) {
        const double measuredDia = 2.0 * cyl.radius;
        if (measuredDia < 1.4 || measuredDia > 20.0) continue;
        const double nominal = nearestStandardDowel(measuredDia, 0.05);
        if (nominal < 0.0) continue;
        const double depth = cyl.length;
        if (depth < 1e-3) continue;

        const gp_Dir adir = cyl.axis.Direction();
        const gp_Pnt& entry = cyl.topCenter;
        json rp = {
            { "position_x_mm",   entry.X() },
            { "position_y_mm",   entry.Y() },
            { "axis_dir",        { adir.X(), adir.Y(), adir.Z() } },
            { "nominal_dia_mm",  nominal },
            { "depth_mm",        depth },
            { "chamfer_mm",      0.2 },
            { "spec_key",        "H6" },
            { "final_dia_mm",    measuredDia },
        };
        double conf = 0.75;
        for (const auto& f : wp.features()) {
            const auto& p = f.params;
            auto sk = p.find("spec_key");
            if (sk == p.end() || sk->get<std::string>() != "H6") continue;
            auto px = p.find("position_x_mm"); auto py = p.find("position_y_mm");
            if (px == p.end() || py == p.end()) continue;
            const double dx = px->get<double>() - entry.X();
            const double dy = py->get<double>() - entry.Y();
            if (dx * dx + dy * dy < 1e-2) { conf = 0.96; break; }
        }
        json matched = {
            { "cyl_face_id", cyl.faceIdx },
            { "entry_center", { entry.X(), entry.Y(), entry.Z() } },
        };
        out.push_back(RecognizedFeature{ kSkillId, rp, conf, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::dowel_pin_h6_bore
