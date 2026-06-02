// @lat: [[engine/skills#slip_fit_h11_bore_spec]]
//
// ISO 286-1 H11 slip-fit bore — drill, oversize ream/bore, generous chamfer.

#include "slip_fit_h11_bore_spec.hpp"

#include "_iso286_fits.hpp"
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

namespace koocadcam::skill::slip_fit_h11_bore_spec {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

struct H11Row { double dia_min; double dia_max; double upper_um; };

// Extension table: only > 100 mm rows the central iso286::kFits omits.
constexpr std::array<H11Row, 3> kH11TableExt {{
    { 100.0, 120.0, 220.0 },
    { 120.0, 180.0, 250.0 },
    { 180.0, 250.0, 290.0 },
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

double upperDeviationMicronsH11(double nominal_dia_mm)
{
    // ≤ 100 mm: central table.
    if (const iso286::FitBand* b = iso286::findBand(nominal_dia_mm)) {
        return static_cast<double>(b->h11_dev_um);
    }
    // > 100 mm: extension rows.
    for (const auto& r : kH11TableExt) {
        if (nominal_dia_mm > r.dia_min && nominal_dia_mm <= r.dia_max) {
            return r.upper_um;
        }
    }
    return -1.0;
}

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.spec_key != "H11") {
        r.add("DFM-SPEC", "error",
              "slip_fit_h11_bore_spec: unknown spec_key '" + in.spec_key +
              "' (only 'H11' is supported — see ISO 286-1)");
    }
    if (in.nominal_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "slip_fit_h11_bore_spec: nominal_dia_mm must be > 0");
        return r;
    }
    if (upperDeviationMicronsH11(in.nominal_dia_mm) < 0.0) {
        r.add("DFM-SPEC-RANGE", "error",
              "slip_fit_h11_bore_spec: nominal_dia_mm " +
              std::to_string(in.nominal_dia_mm) +
              " out of ISO 286-1 H11 table range (1..250 mm)");
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "slip_fit_h11_bore_spec: depth_mm must be > 0");
    }
    if (in.nominal_dia_mm < 1.0) {
        r.add("DFM-002", "error",
              "slip_fit_h11_bore_spec: nominal_dia_mm < 1 mm — H11 tolerance "
              "becomes meaningless at sub-mm scale");
    }
    if (in.chamfer_mm < 0.0) {
        r.add("DFM-INPUT", "error",
              "slip_fit_h11_bore_spec: chamfer_mm must be >= 0");
    }
    const double ratio = (in.nominal_dia_mm > 0.0)
        ? (in.depth_mm / in.nominal_dia_mm) : 0.0;
    if (ratio > 10.0) {
        r.add("DFM-DEEP-HOLE", "warning",
              "slip_fit_h11_bore_spec: depth/dia ratio " + std::to_string(ratio) +
              " > 10 — gun-drilling recommended even for H11 tolerance");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "slip_fit_h11_bore_spec DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("slip_fit_h11_bore_spec: entry_face datum unresolved");

    const double upperDevMm = upperDeviationMicronsH11(in.nominal_dia_mm) * 1e-3;
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
    tooling.flute_count      = 4;
    tooling.cutting_speed_sfm = 80.0;   // H11 = loose, run fast
    tooling.feed_per_tooth_mm = 0.08;
    const double rF = finalDia / 2.0;
    tooling.stock_removed_mm3 = M_PI * rF * rF * in.depth_mm;
    tooling.est_cycle_time_s  = std::max(0.8, in.depth_mm / 40.0) + 0.5;
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },   { "tool_dia_mm", drillDia }, { "depth_mm", in.depth_mm } },
            { { "tool_type", "reamer" }, { "tool_dia_mm", finalDia },
              { "depth_mm", in.depth_mm }, { "spec_key", in.spec_key } },
            { { "tool_type", "chamfer" }, { "chamfer_mm", in.chamfer_mm } },
        } },
        { "spec_source", "ISO 286-1:2010 Table 2 (H11)" },
        { "fit_type",    "clearance" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);
    spdlog::debug("skill::slip_fit_h11_bore_spec applied: nominal {} → final {} mm (slip fit)",
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

double nearestNominal(double measured_dia_mm, double tol_mm)
{
    double best = -1.0;
    double bestErr = tol_mm;
    // ≤ 100 mm: walk the central kFits.
    for (const auto& b : iso286::kFits) {
        const double nominal  = b.size_max_mm;
        const double finalDia = nominal + (b.h11_dev_um * 1e-3) / 2.0;
        const double err      = std::abs(finalDia - measured_dia_mm);
        if (err < bestErr) { bestErr = err; best = nominal; }
    }
    // > 100 mm: extension rows.
    for (const auto& r : kH11TableExt) {
        const double nominal  = r.dia_max;
        const double finalDia = nominal + (r.upper_um * 1e-3) / 2.0;
        const double err      = std::abs(finalDia - measured_dia_mm);
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
        const double nominal = nearestNominal(measuredDia, 0.30);  // H11 = loose
        if (nominal < 0.0) continue;
        const double depth = cyl.length;
        const double ratio = (measuredDia > 0.0) ? (depth / measuredDia) : 0.0;
        if (ratio > 12.0) continue;
        if (depth < 1e-3) continue;

        const gp_Dir adir = cyl.axis.Direction();
        const gp_Pnt& entry = cyl.topCenter;
        json rp = {
            { "position_x_mm",   entry.X() },
            { "position_y_mm",   entry.Y() },
            { "axis_dir",        { adir.X(), adir.Y(), adir.Z() } },
            { "nominal_dia_mm",  nominal },
            { "depth_mm",        depth },
            { "chamfer_mm",      0.5 },
            { "spec_key",        "H11" },
            { "final_dia_mm",    measuredDia },
        };
        double conf = 0.65;   // H11 overlaps heavily with plain drill_hole
        for (const auto& f : wp.features()) {
            const auto& p = f.params;
            auto sk = p.find("spec_key");
            if (sk == p.end() || sk->get<std::string>() != "H11") continue;
            auto px = p.find("position_x_mm"); auto py = p.find("position_y_mm");
            if (px == p.end() || py == p.end()) continue;
            const double dx = px->get<double>() - entry.X();
            const double dy = py->get<double>() - entry.Y();
            if (dx * dx + dy * dy < 1e-2) { conf = 0.92; break; }
        }
        json matched = {
            { "cyl_face_id", cyl.faceIdx },
            { "entry_center", { entry.X(), entry.Y(), entry.Z() } },
        };
        out.push_back(RecognizedFeature{ kSkillId, rp, conf, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::slip_fit_h11_bore_spec
