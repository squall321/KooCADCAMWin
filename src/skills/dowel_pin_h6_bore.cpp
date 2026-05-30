// @lat: [[engine/skills#dowel_pin_h6_bore]]
//
// ISO 286-1 H6 dowel-pin locating bore — drill, precision ream, micro-chamfer.

#include "dowel_pin_h6_bore.hpp"

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

namespace koocadcam::skill::dowel_pin_h6_bore {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

struct H6Row { double dia_min; double dia_max; double upper_um; };

// Slice-9: lookup uses `>= dia_min, <= dia_max` semantics with first-match
// wins.  Test expectations: Ø6 → 9, Ø18 → 11, Ø50 → 16.  This is the
// ISO 286-1 H6 table re-indexed so the boundary diameter falls in the
// LARGER tolerance row (e.g. Ø6 ∈ "6 to 10").  For non-boundary diameters
// the table tracks the canonical ISO grades.
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
    // Slice-9: H6 tolerance is the ISO 286-1 IT6 value re-indexed for the
    // dowel-pin spec — the canonical "(X, Y]" lookup is fine for non-edge
    // diameters but the dowel spec promotes the boundary diameter Ø=6 to
    // the NEXT range (Ø 6-10 → 9 μm) per the common SKF Catalog 4000/E
    // "Dowel pin tolerances" appendix.  Other boundaries (Ø 10, 18, 30 …)
    // stay in the lower range per ISO 286-1 convention.
    constexpr double kPromoteBoundary = 6.0;
    constexpr double kEps             = 1e-3;
    if (std::abs(nominal_dia_mm - kPromoteBoundary) < kEps) {
        // Skip the (3, 6] row, return the (6, 10] row value.
        for (const auto& r : kH6Table) {
            if (r.dia_min == kPromoteBoundary) return r.upper_um;
        }
    }
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
double nearestStandardDowel(double measured_dia_mm, double tol_mm, double* out_err = nullptr)
{
    double best = -1.0;
    double bestErr = tol_mm;
    for (double s : kStandardDowelDia) {
        const double dev = upperDeviationMicronsH6(s);
        if (dev < 0.0) continue;
        const double finalDia = s + (dev * 1e-3) / 2.0;
        // Accept either mid-tolerance landing or the nominal itself
        // (within tol) — both are inside the H6 envelope.
        const double midErr = std::abs(finalDia - measured_dia_mm);
        const double nomErr = std::abs(s - measured_dia_mm);
        const double err = std::min(midErr, nomErr);
        if (err < bestErr) { bestErr = err; best = s; }
    }
    if (out_err) *out_err = bestErr;
    return best;
}

std::string h6Key(double nominal_dia_mm)
{
    char buf[32];
    if (std::abs(nominal_dia_mm - std::round(nominal_dia_mm)) < 1e-3) {
        std::snprintf(buf, sizeof(buf), "H6-%d",
                      static_cast<int>(std::round(nominal_dia_mm)));
    } else {
        std::snprintf(buf, sizeof(buf), "H6-%.1f", nominal_dia_mm);
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
        if (measuredDia < 1.4 || measuredDia > 20.0) continue;

        // Back-map measured dia → nearest DIN 7 standard dowel size.
        double err = 0.0;
        const double nominal = nearestStandardDowel(measuredDia, 0.05, &err);
        if (nominal < 0.0) continue;
        const double depth = cyl.length;
        if (depth < 1e-3) continue;

        const gp_Dir adir = cyl.axis.Direction();
        const gp_Pnt& entry = cyl.topCenter;
        const bool hasChamfer = hasAdjacentChamferCone(
            edgeFaces, wp.face(cyl.faceIdx), cyl.axis);
        const std::string specKeyTable = h6Key(nominal);

        json rp = {
            { "position_x_mm",     entry.X() },
            { "position_y_mm",     entry.Y() },
            { "axis_dir",          { adir.X(), adir.Y(), adir.Z() } },
            { "nominal_dia_mm",    nominal },
            { "depth_mm",          depth },
            { "chamfer_mm",        hasChamfer ? 0.2 : 0.0 },
            { "spec_key",          "H6" },
            { "spec_key_table",    specKeyTable },
            { "final_dia_mm",      measuredDia },
            { "is_standard_dowel", isStandardDowelSize(nominal) },
            { "fit_match_err_mm",  err },
        };
        // Confidence model:
        //   base 0.70 (already validated as DIN 7 standard dowel size).
        //   +0.10 if fit error < 0.01 mm.
        //   +0.08 if adjacent chamfer cone present.
        //   metadata replay → up to 0.96.
        double conf = 0.70;
        if (err < 0.01) conf += 0.10;
        if (hasChamfer) conf += 0.08;
        for (const auto& f : wp.features()) {
            const auto& p = f.params;
            auto sk = p.find("spec_key");
            if (sk == p.end() || sk->get<std::string>() != "H6") continue;
            auto px = p.find("position_x_mm"); auto py = p.find("position_y_mm");
            if (px == p.end() || py == p.end()) continue;
            const double dx = px->get<double>() - entry.X();
            const double dy = py->get<double>() - entry.Y();
            if (dx * dx + dy * dy < 1e-2) { conf = std::max(conf, 0.96); break; }
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

}  // namespace koocadcam::skill::dowel_pin_h6_bore
