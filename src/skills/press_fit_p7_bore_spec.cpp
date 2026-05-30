// @lat: [[engine/skills#press_fit_p7_bore_spec]]
//
// ISO 286-1 P7 press-fit bore — drill, ream undersize, chamfer for press start.

#include "press_fit_p7_bore_spec.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <gp_Cone.hxx>
#include <gp_Cylinder.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <string>

namespace koocadcam::skill::press_fit_p7_bore_spec {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── ISO 286-1 P7 deviation table ─────────────────────────────────────────
//
// Both deviations are NEGATIVE — bore is undersize, producing interference
// against an h7 shaft.  Source: ISO 286-1:2010 Table 2 (P-grade).
namespace {

struct P7Row { double dia_min; double dia_max; double upper_um; double lower_um; };

constexpr std::array<P7Row, 10> kP7Table {{
    { 0.0,   3.0,   -6.0, -16.0 },
    { 3.0,   6.0,   -8.0, -20.0 },
    { 6.0,  10.0,   -9.0, -24.0 },
    { 10.0, 18.0,  -11.0, -29.0 },
    { 18.0, 30.0,  -14.0, -35.0 },
    { 30.0, 50.0,  -17.0, -42.0 },
    { 50.0, 80.0,  -21.0, -51.0 },
    { 80.0, 120.0, -24.0, -59.0 },
    { 120.0,180.0, -28.0, -68.0 },
    { 180.0,250.0, -33.0, -79.0 },
}};

TopoDS_Shape buildLeadInChamfer(const gp_Pnt& entryCenter, const gp_Dir& axis,
                                double finalDia, double chamfer_mm,
                                double entryOverhang)
{
    const double smallR = finalDia / 2.0;
    const double bigR   = smallR + chamfer_mm;
    const gp_Pnt apexAbove(
        entryCenter.X() - axis.X() * entryOverhang,
        entryCenter.Y() - axis.Y() * entryOverhang,
        entryCenter.Z() - axis.Z() * entryOverhang);
    const gp_Ax2 ax(apexAbove, axis);
    return pr::coneFrustum(ax, bigR, smallR, entryOverhang + chamfer_mm);
}

}  // namespace

P7Deviation deviationFor(double nominal_dia_mm)
{
    for (const auto& r : kP7Table) {
        if (nominal_dia_mm > r.dia_min && nominal_dia_mm <= r.dia_max) {
            return { r.upper_um, r.lower_um, true };
        }
    }
    return { 0.0, 0.0, false };
}

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.spec_key != "P7") {
        r.add("DFM-SPEC", "error",
              "press_fit_p7_bore_spec: unknown spec_key '" + in.spec_key +
              "' (only 'P7' is supported — see ISO 286-1)");
    }
    if (in.nominal_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "press_fit_p7_bore_spec: nominal_dia_mm must be > 0");
        return r;
    }
    const auto dev = deviationFor(in.nominal_dia_mm);
    if (!dev.valid) {
        r.add("DFM-SPEC-RANGE", "error",
              "press_fit_p7_bore_spec: nominal_dia_mm " +
              std::to_string(in.nominal_dia_mm) +
              " out of ISO 286-1 P7 table range (1..250 mm)");
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "press_fit_p7_bore_spec: depth_mm must be > 0");
    }
    if (in.nominal_dia_mm < 1.0) {
        r.add("DFM-002", "error",
              "press_fit_p7_bore_spec: nominal_dia_mm < 1 mm — press-fit "
              "interference uncontrollable below 1 mm");
    }
    if (in.chamfer_mm < 0.2) {
        r.add("DFM-PRESS-CHAMFER", "warning",
              "press_fit_p7_bore_spec: chamfer < 0.2 mm — press-start will "
              "shave shaft material (recommend 0.3..1.0 mm lead-in)");
    }
    const double ratio = (in.nominal_dia_mm > 0.0)
        ? (in.depth_mm / in.nominal_dia_mm) : 0.0;
    if (ratio > 4.0) {
        r.add("DFM-PRESS-RATIO", "warning",
              "press_fit_p7_bore_spec: depth/dia ratio " + std::to_string(ratio) +
              " > 4 — long press fits prone to galling, consider shorter "
              "engagement");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "press_fit_p7_bore_spec DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("press_fit_p7_bore_spec: entry_face datum unresolved");

    const auto dev = deviationFor(in.nominal_dia_mm);
    // Final dia: nominal + (upper + lower)/2 µm  (mid-tolerance, undersize)
    const double midDevMm = ((dev.upper_um + dev.lower_um) / 2.0) * 1e-3;
    const double finalDia = in.nominal_dia_mm + midDevMm;
    const double drillDia = in.nominal_dia_mm;  // rough at nominal

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
        { "upper_dev_um",     dev.upper_um },
        { "lower_dev_um",     dev.lower_um },
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
    tooling.tool_length_mm   = in.depth_mm * 1.5 + 10.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 6;
    tooling.cutting_speed_sfm = 50.0;  // slower for tight P7 interference
    tooling.feed_per_tooth_mm = 0.04;
    const double rF = finalDia / 2.0;
    tooling.stock_removed_mm3 = M_PI * rF * rF * in.depth_mm;
    tooling.est_cycle_time_s  = std::max(1.5, in.depth_mm / 25.0) + 2.0;
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },   { "tool_dia_mm", drillDia }, { "depth_mm", in.depth_mm } },
            { { "tool_type", "reamer" }, { "tool_dia_mm", finalDia },
              { "depth_mm", in.depth_mm }, { "spec_key", in.spec_key } },
            { { "tool_type", "chamfer" }, { "chamfer_mm", in.chamfer_mm } },
        } },
        { "spec_source", "ISO 286-1:2010 Table 2 (P7)" },
        { "fit_type",    "interference" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::press_fit_p7_bore_spec applied: nominal {} → final {} mm (interference fit)",
                  in.nominal_dia_mm, finalDia);

    return SkillOutput{ wpNew, sig };
}

namespace {

struct CylInfo
{
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
        CylInfo info;
        info.faceIdx = fIdx;
        info.radius  = cyl.Radius();
        info.axis    = cyl.Axis();

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

// Preferred press-fit shaft sizes — DIN 6325 / ISO 286 hardened dowel pin
// nominals + common bearing shaft sizes.  recognize() snaps the measured
// undersize bore to one of these as the back-mapped key.
constexpr std::array<double, 20> kPreferredP7Nominal {{
    1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0, 14.0,
    16.0, 18.0, 20.0, 25.0, 30.0, 40.0, 50.0, 60.0, 80.0,
}};

// Closest P7 nominal whose mid-tolerance landing matches measured_dia.
double nearestNominal(double measured_dia_mm, double tol_mm, double* out_err = nullptr)
{
    double best = -1.0;
    double bestErr = tol_mm;
    for (double nominal : kPreferredP7Nominal) {
        const auto dev = deviationFor(nominal);
        if (!dev.valid) continue;
        const double midDev = ((dev.upper_um + dev.lower_um) / 2.0) * 1e-3;
        const double finalDia = nominal + midDev;
        const double err = std::abs(finalDia - measured_dia_mm);
        if (err < bestErr) { bestErr = err; best = nominal; }
    }
    if (out_err) *out_err = bestErr;
    return best;
}

std::string p7Key(double nominal_dia_mm)
{
    char buf[32];
    if (std::abs(nominal_dia_mm - std::round(nominal_dia_mm)) < 1e-3) {
        std::snprintf(buf, sizeof(buf), "P7-%d",
                      static_cast<int>(std::round(nominal_dia_mm)));
    } else {
        std::snprintf(buf, sizeof(buf), "P7-%.1f", nominal_dia_mm);
    }
    return std::string(buf);
}

using EdgeFaceMap = NCollection_IndexedDataMap<
    TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>;

EdgeFaceMap buildEdgeFaceMap(const TopoDS_Shape& shape)
{
    EdgeFaceMap m;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, m);
    return m;
}

bool hasAdjacentChamferCone(const EdgeFaceMap& edgeFaces,
                            const TopoDS_Face& cylFace,
                            const gp_Ax1& cylAxis)
{
    for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
        const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
        if (!edgeFaces.Contains(e)) continue;
        const auto& adj = edgeFaces.FindFromKey(e);
        for (NCollection_List<TopoDS_Shape>::Iterator it(adj); it.More(); it.Next()) {
            const TopoDS_Face& af = TopoDS::Face(it.Value());
            if (af.IsSame(cylFace)) continue;
            BRepAdaptor_Surface as(af);
            if (as.GetType() != GeomAbs_Cone) continue;
            const gp_Cone cn = as.Cone();
            if (std::abs(std::abs(cn.Axis().Direction().Dot(cylAxis.Direction())) - 1.0)
                < 1e-2) return true;
        }
    }
    return false;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const auto cyls = collectCylinders(wp);
    const auto edgeFaces = buildEdgeFaceMap(wp.shape());

    for (const auto& cyl : cyls) {
        const double measuredDia = 2.0 * cyl.radius;
        if (measuredDia < 0.9 || measuredDia > 260.0) continue;

        // Back-map measured diameter → preferred ISO 286 P7 nominal.
        double err = 0.0;
        const double nominal = nearestNominal(measuredDia, 0.08, &err);
        if (nominal < 0.0) continue;

        const double depth = cyl.length;
        const double ratio = (measuredDia > 0.0) ? (depth / measuredDia) : 0.0;
        if (ratio > 5.0) continue;
        if (depth < 1e-3) continue;

        const gp_Dir adir = cyl.axis.Direction();
        const gp_Pnt& entry = cyl.topCenter;
        const bool hasChamfer = hasAdjacentChamferCone(
            edgeFaces, wp.face(cyl.faceIdx), cyl.axis);
        const std::string specKeyTable = p7Key(nominal);

        json rp = {
            { "position_x_mm",     entry.X() },
            { "position_y_mm",     entry.Y() },
            { "axis_dir",          { adir.X(), adir.Y(), adir.Z() } },
            { "nominal_dia_mm",    nominal },
            { "depth_mm",          depth },
            { "chamfer_mm",        hasChamfer ? 0.5 : 0.0 },
            { "spec_key",          "P7" },
            { "spec_key_table",    specKeyTable },
            { "final_dia_mm",      measuredDia },
            { "fit_match_err_mm",  err },
        };
        double conf = 0.60;
        if (err < 0.03) conf += 0.10;
        if (hasChamfer) conf += 0.10;
        for (const auto& f : wp.features()) {
            const auto& p = f.params;
            auto sk = p.find("spec_key");
            if (sk == p.end() || sk->get<std::string>() != "P7") continue;
            auto px = p.find("position_x_mm"); auto py = p.find("position_y_mm");
            if (px == p.end() || py == p.end()) continue;
            const double dx = px->get<double>() - entry.X();
            const double dy = py->get<double>() - entry.Y();
            if (dx * dx + dy * dy < 1e-2) { conf = std::max(conf, 0.93); break; }
        }
        if (conf > 0.99) conf = 0.99;

        json matched = {
            { "cyl_face_id",      cyl.faceIdx },
            { "entry_center",     { entry.X(), entry.Y(), entry.Z() } },
            { "spec_key_table",   specKeyTable },
            { "has_chamfer_cone", hasChamfer },
            { "fit_err_mm",       err },
        };
        out.push_back(RecognizedFeature{ kSkillId, rp, conf, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::press_fit_p7_bore_spec
