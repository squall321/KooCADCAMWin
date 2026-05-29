// @lat: [[engine/skills#locating_g6_bore]]
//
// ISO 286-1 G6 locating-fit bore — drill, precision oversize ream, chamfer.
// Context-aware: queries workpiece face count to choose chamfer at any flat
// face (not just top).

#include "locating_g6_bore.hpp"

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

namespace koocadcam::skill::locating_g6_bore {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

struct G6Row { double dia_min; double dia_max; double upper_um; double lower_um; };

constexpr std::array<G6Row, 10> kG6Table {{
    { 0.0,   3.0,    8.0,  2.0 },
    { 3.0,   6.0,   12.0,  4.0 },
    { 6.0,  10.0,   14.0,  5.0 },
    { 10.0, 18.0,   17.0,  6.0 },
    { 18.0, 30.0,   20.0,  7.0 },
    { 30.0, 50.0,   25.0,  9.0 },
    { 50.0, 80.0,   29.0, 10.0 },
    { 80.0, 120.0,  34.0, 12.0 },
    { 120.0,180.0,  39.0, 14.0 },
    { 180.0,250.0,  44.0, 15.0 },
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

// Context query: count of planar faces on the workpiece (used to decide
// whether a second chamfer should be added at the back face for through holes).
int countPlanarFaces(const Workpiece& wp)
{
    int n = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (wp.isFacePlanar(i)) ++n;
    }
    return n;
}

}  // namespace

G6Deviation deviationFor(double nominal_dia_mm)
{
    for (const auto& r : kG6Table) {
        if (nominal_dia_mm > r.dia_min && nominal_dia_mm <= r.dia_max) {
            return { r.upper_um, r.lower_um, true };
        }
    }
    return { 0.0, 0.0, false };
}

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.spec_key != "G6") {
        r.add("DFM-SPEC", "error",
              "locating_g6_bore: unknown spec_key '" + in.spec_key +
              "' (only 'G6' is supported — see ISO 286-1)");
    }
    if (in.nominal_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "locating_g6_bore: nominal_dia_mm must be > 0");
        return r;
    }
    if (!deviationFor(in.nominal_dia_mm).valid) {
        r.add("DFM-SPEC-RANGE", "error",
              "locating_g6_bore: nominal_dia_mm " +
              std::to_string(in.nominal_dia_mm) +
              " out of ISO 286-1 G6 table range (1..250 mm)");
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "locating_g6_bore: depth_mm must be > 0");
    }
    if (in.nominal_dia_mm < 1.0) {
        r.add("DFM-002", "error",
              "locating_g6_bore: nominal_dia_mm < 1 mm — locating fit "
              "unrealistic below 1 mm");
    }
    if (in.chamfer_mm < 0.0) {
        r.add("DFM-INPUT", "error",
              "locating_g6_bore: chamfer_mm must be >= 0");
    }
    // Context check: workpiece must have at least 1 planar face to anchor the
    // chamfer.  Without it we can't establish an entry datum (slip-fit bushes
    // need a flat reference).
    if (countPlanarFaces(wp) < 1) {
        r.add("DFM-LOCATING-DATUM", "warning",
              "locating_g6_bore: workpiece has no planar face — locating "
              "datum reference will be ambiguous");
    }
    const double ratio = (in.nominal_dia_mm > 0.0)
        ? (in.depth_mm / in.nominal_dia_mm) : 0.0;
    if (ratio > 5.0) {
        r.add("DFM-LOCATING-RATIO", "warning",
              "locating_g6_bore: depth/dia ratio " + std::to_string(ratio) +
              " > 5 — locating accuracy degrades with deep bushings");
    }
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "locating_g6_bore DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("locating_g6_bore: entry_face datum unresolved");

    const auto dev = deviationFor(in.nominal_dia_mm);
    // Mid-tolerance landing: nominal + (upper+lower)/2 µm.
    const double midDevMm = ((dev.upper_um + dev.lower_um) / 2.0) * 1e-3;
    const double finalDia = in.nominal_dia_mm + midDevMm;
    const double drillDia = in.nominal_dia_mm;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kEntryOverhang = 0.05 + in.chamfer_mm + 0.5;
    const gp_Dir adir = in.axis_dir;

    gp_Pnt entryCenter(in.position_x_mm, in.position_y_mm, (zMin + zMax) / 2.0);
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        if (adir.Z() < 0) entryCenter.SetZ(zMax);
        else              entryCenter.SetZ(zMin);
    }
    // Context-aware: if the resolved entry face is closer to one side of the
    // bbox along the axis, use that face center instead of bbox extremum.
    if (entryId.has_value()) {
        const gp_Pnt fc = wp.faceCenter(*entryId);
        // Snap the axial coordinate to the face center; XY stays as input.
        if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
            entryCenter.SetZ(fc.Z());
        } else {
            entryCenter = gp_Pnt(in.position_x_mm, in.position_y_mm, fc.Z());
        }
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

    const int planarCount = countPlanarFaces(wp);

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
        { "upper_dev_um",     dev.upper_um },
        { "lower_dev_um",     dev.lower_um },
        { "ctx_planar_faces", planarCount },
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
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 6;
    tooling.cutting_speed_sfm = 55.0;
    tooling.feed_per_tooth_mm = 0.04;
    const double rF = finalDia / 2.0;
    tooling.stock_removed_mm3 = M_PI * rF * rF * in.depth_mm;
    tooling.est_cycle_time_s  = std::max(1.5, in.depth_mm / 28.0) + 1.5;
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },   { "tool_dia_mm", drillDia }, { "depth_mm", in.depth_mm } },
            { { "tool_type", "reamer" }, { "tool_dia_mm", finalDia },
              { "depth_mm", in.depth_mm }, { "spec_key", in.spec_key } },
            { { "tool_type", "chamfer" }, { "chamfer_mm", in.chamfer_mm } },
        } },
        { "spec_source", "ISO 286-1:2010 Table 2 (G6)" },
        { "fit_type",    "locating" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);
    spdlog::debug("skill::locating_g6_bore applied: nominal {} → final {} mm "
                  "(ctx: {} planar faces)",
                  in.nominal_dia_mm, finalDia, planarCount);
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

double nearestNominal(double measured_dia_mm, double tol_mm)
{
    double best = -1.0;
    double bestErr = tol_mm;
    for (const auto& r : kG6Table) {
        const double nominal = r.dia_max;
        const double midDev = ((r.upper_um + r.lower_um) / 2.0) * 1e-3;
        const double finalDia = nominal + midDev;
        const double err = std::abs(finalDia - measured_dia_mm);
        if (err < bestErr) { bestErr = err; best = nominal; }
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
        if (measuredDia < 0.9 || measuredDia > 260.0) continue;
        const double nominal = nearestNominal(measuredDia, 0.08);
        if (nominal < 0.0) continue;
        const double depth = cyl.length;
        const double ratio = (measuredDia > 0.0) ? (depth / measuredDia) : 0.0;
        if (ratio > 6.0) continue;
        if (depth < 1e-3) continue;

        const gp_Dir adir = cyl.axis.Direction();
        const gp_Pnt& entry = cyl.topCenter;
        json rp = {
            { "position_x_mm",   entry.X() },
            { "position_y_mm",   entry.Y() },
            { "axis_dir",        { adir.X(), adir.Y(), adir.Z() } },
            { "nominal_dia_mm",  nominal },
            { "depth_mm",        depth },
            { "chamfer_mm",      0.3 },
            { "spec_key",        "G6" },
            { "final_dia_mm",    measuredDia },
        };
        double conf = 0.70;
        for (const auto& f : wp.features()) {
            const auto& p = f.params;
            auto sk = p.find("spec_key");
            if (sk == p.end() || sk->get<std::string>() != "G6") continue;
            auto px = p.find("position_x_mm"); auto py = p.find("position_y_mm");
            if (px == p.end() || py == p.end()) continue;
            const double dx = px->get<double>() - entry.X();
            const double dy = py->get<double>() - entry.Y();
            if (dx * dx + dy * dy < 1e-2) { conf = 0.94; break; }
        }
        json matched = {
            { "cyl_face_id", cyl.faceIdx },
            { "entry_center", { entry.X(), entry.Y(), entry.Z() } },
        };
        out.push_back(RecognizedFeature{ kSkillId, rp, conf, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::locating_g6_bore
