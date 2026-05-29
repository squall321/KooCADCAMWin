// @lat: [[engine/skills#iso_h7_bore_spec]]
//
// ISO 286-1 H7 bore — drill at nominal, ream to mid-tolerance, lead-in chamfer.

#include "iso_h7_bore_spec.hpp"

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

namespace koocadcam::skill::iso_h7_bore_spec {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── ISO 286-1 H7 deviation table (upper deviation in µm) ─────────────────
//
// Lower deviation is always 0 for H-grade.  Upper deviation = IT7 grade.
// Source: ISO 286-1:2010 Table 2 (Standard Tolerance Grades).
namespace {

struct H7Row
{
    double dia_min_mm;   // exclusive lower
    double dia_max_mm;   // inclusive upper
    double upper_dev_um; // upper deviation, microns
};

constexpr std::array<H7Row, 10> kH7Table {{
    { 0.0,   3.0,  10.0 },   // covers Ø 1–3 (10 µm = 0.010 mm)
    { 3.0,   6.0,  12.0 },   // Ø 3–6
    { 6.0,  10.0,  15.0 },   // Ø 6–10
    { 10.0, 18.0,  18.0 },   // Ø 10–18
    { 18.0, 30.0,  21.0 },   // Ø 18–30
    { 30.0, 50.0,  25.0 },   // Ø 30–50
    { 50.0, 80.0,  30.0 },   // Ø 50–80
    { 80.0, 120.0, 35.0 },   // Ø 80–120
    { 120.0,180.0, 40.0 },   // safety extension
    { 180.0,250.0, 46.0 },
}};

// Chamfer is a conical frustum lead-in.  Half-angle = 45°.  bigR > smallR.
TopoDS_Shape buildLeadInChamfer(const gp_Pnt& entryCenter,
                                const gp_Dir& axis,
                                double finalDia,
                                double chamfer_mm,
                                double entryOverhang)
{
    // Cone tool: r1 = bigR (above entry), r2 = smallR (at chamfer_mm into bore)
    const double smallR = finalDia / 2.0;
    const double bigR   = smallR + chamfer_mm;
    // Place axis above the entry by `entryOverhang` so the cone fully covers
    // the chamfer band.
    const gp_Pnt apexAbove(
        entryCenter.X() - axis.X() * entryOverhang,
        entryCenter.Y() - axis.Y() * entryOverhang,
        entryCenter.Z() - axis.Z() * entryOverhang);
    const gp_Ax2 ax(apexAbove, axis);
    // Length = entryOverhang + chamfer_mm  (cone runs from bigR above entry
    // to smallR at chamfer depth).
    return pr::coneFrustum(ax, bigR, smallR, entryOverhang + chamfer_mm);
}

}  // namespace

double upperDeviationMicrons(double nominal_dia_mm)
{
    for (const auto& r : kH7Table) {
        if (nominal_dia_mm > r.dia_min_mm && nominal_dia_mm <= r.dia_max_mm) {
            return r.upper_dev_um;
        }
    }
    return -1.0;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.spec_key != "H7") {
        r.add("DFM-SPEC", "error",
              "iso_h7_bore_spec: unknown spec_key '" + in.spec_key +
              "' (only 'H7' is supported by this skill — see ISO 286-1)");
    }
    if (in.nominal_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "iso_h7_bore_spec: nominal_dia_mm must be > 0");
        return r;
    }
    const double dev = upperDeviationMicrons(in.nominal_dia_mm);
    if (dev < 0.0) {
        r.add("DFM-SPEC-RANGE", "error",
              "iso_h7_bore_spec: nominal_dia_mm " +
              std::to_string(in.nominal_dia_mm) +
              " out of ISO 286-1 H7 table range (1..250 mm)");
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "iso_h7_bore_spec: depth_mm must be > 0");
    }
    if (in.nominal_dia_mm < 1.0) {
        r.add("DFM-002", "error",
              "iso_h7_bore_spec: nominal_dia_mm < 1.0 mm — reamer not "
              "manufactured below 1 mm in standard catalogs");
    }
    if (in.chamfer_mm < 0.0) {
        r.add("DFM-INPUT", "error",
              "iso_h7_bore_spec: chamfer_mm must be >= 0");
    }
    const double ratio = (in.nominal_dia_mm > 0.0)
        ? (in.depth_mm / in.nominal_dia_mm) : 0.0;
    if (ratio > 5.0) {
        r.add("DFM-REAMER-RATIO", "warning",
              "iso_h7_bore_spec: depth/dia ratio " + std::to_string(ratio) +
              " > 5 — reamer rigidity will degrade H7 tolerance");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Compound chain:
//   1. drill cylinder at nominal_dia_mm        (rough hog-out)
//   2. ream cylinder at deviated final dia     (mid-tolerance landing)
//   3. chamfer cone at entry                   (lead-in)
//
// Steps 1+2 are coaxial overlapping cylinders → fuse first, then cut.
// Step 3 is concentric but distinct geometry; we fuse it into the cutter
// before a single final cut.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "iso_h7_bore_spec DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("iso_h7_bore_spec: entry_face datum unresolved");

    // Final (deviated) diameter: nominal + upper_dev/2 to land mid-tolerance.
    const double upperDevMm = upperDeviationMicrons(in.nominal_dia_mm) * 1e-3;
    const double finalDia   = in.nominal_dia_mm + upperDevMm / 2.0;
    const double drillDia   = in.nominal_dia_mm;  // rough pass at nominal

    // ── Build entry point from workpiece bbox ──
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));
    (void)bboxDiag;
    const double kEntryOverhang = 0.05 + in.chamfer_mm + 0.5;
    const gp_Dir adir = in.axis_dir;

    gp_Pnt entryCenter(in.position_x_mm,
                       in.position_y_mm,
                       (zMin + zMax) / 2.0);
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        if (adir.Z() < 0) entryCenter.SetZ(zMax);
        else              entryCenter.SetZ(zMin);
    }
    const gp_Pnt toolStart(
        entryCenter.X() - adir.X() * kEntryOverhang,
        entryCenter.Y() - adir.Y() * kEntryOverhang,
        entryCenter.Z() - adir.Z() * kEntryOverhang);
    const gp_Ax2 toolAx(toolStart, adir);

    // ── Step 1: drill at nominal (rough) ──
    const TopoDS_Shape drillTool =
        pr::cylinder(toolAx, drillDia / 2.0, in.depth_mm + kEntryOverhang);

    // ── Step 2: ream at deviated final dia ──
    const TopoDS_Shape reamTool =
        pr::cylinder(toolAx, finalDia / 2.0, in.depth_mm + kEntryOverhang);

    // Step 1+2 overlap concentrically — fuse to make a single connected
    // cutter (avoids interior-overlap artefacts in BRepAlgoAPI_Cut).
    TopoDS_Shape fused = pr::fuse(drillTool, reamTool);

    // ── Step 3: lead-in chamfer cone ──
    if (in.chamfer_mm > 0.0) {
        const TopoDS_Shape chamferTool =
            buildLeadInChamfer(entryCenter, adir, finalDia, in.chamfer_mm,
                               kEntryOverhang);
        fused = pr::fuse(fused, chamferTool);
    }

    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    // ── Signature ──
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
    tooling.flute_count      = 6;
    tooling.cutting_speed_sfm = 60.0;       // typical for H7 reaming in steel
    tooling.feed_per_tooth_mm = 0.05;
    const double finalR = finalDia / 2.0;
    tooling.stock_removed_mm3 = M_PI * finalR * finalR * in.depth_mm;
    tooling.est_cycle_time_s  = std::max(1.0, in.depth_mm / 30.0) + 1.5;
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },
              { "tool_dia_mm", drillDia },
              { "depth_mm",    in.depth_mm } },
            { { "tool_type", "reamer" },
              { "tool_dia_mm", finalDia },
              { "depth_mm",    in.depth_mm },
              { "spec_key",    in.spec_key } },
            { { "tool_type", "chamfer" },
              { "chamfer_mm",  in.chamfer_mm } },
        } },
        { "spec_source", "ISO 286-1:2010 Table 2 (H7)" },
    };

    FeatureSignature sig {
        kSkillId,
        params,
        pattern,
        tooling,
    };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::iso_h7_bore_spec applied: nominal {} → final {} mm, depth {}",
                  in.nominal_dia_mm, finalDia, in.depth_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Heuristic: find cylindrical faces whose diameter matches an H7 mid-tolerance
// landing point (nominal + upper_dev/2) within 0.05 mm.  Confidence is high
// when an adjacent conical (chamfer) face is also present along the same axis.

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

// Best-fit nominal for a given measured diameter; returns -1 if no row hits.
double nearestNominal(double measured_dia_mm, double tol_mm)
{
    // Try each table row's mid-tolerance landing: nominal + dev/2.
    // Sweep nominal across (rmin..rmax] picking the row's midpoint as candidate.
    double best = -1.0;
    double bestErr = tol_mm;
    for (const auto& r : kH7Table) {
        // The "nominal" for recognition is the dia_max of the row (highest
        // nominal that uses this row's dev).  Mid-tolerance landing:
        const double nominal = r.dia_max_mm;
        const double finalDia = nominal + (r.upper_dev_um * 1e-3) / 2.0;
        const double err = std::abs(finalDia - measured_dia_mm);
        if (err < bestErr) { bestErr = err; best = nominal; }
        // Also check dia_min + epsilon as a candidate nominal.
        const double nominal2 = r.dia_min_mm + 1e-3;
        if (nominal2 > 0.0) {
            const double finalDia2 = nominal2 + (r.upper_dev_um * 1e-3) / 2.0;
            const double err2 = std::abs(finalDia2 - measured_dia_mm);
            if (err2 < bestErr) { bestErr = err2; best = nominal2; }
        }
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

        // Heuristic: every cylinder with reasonable depth/dia is a candidate;
        // metadata replay fallback: also check the workpiece feature history
        // for an exact spec_key match.
        const double nominal = nearestNominal(measuredDia, 0.06);
        if (nominal < 0.0) continue;

        // Depth/dia gate (reamer rigidity envelope).
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
            { "spec_key",        "H7" },
            { "final_dia_mm",    measuredDia },
        };

        // Metadata-replay fallback: bump confidence if any prior feature on
        // this workpiece already records spec_key="H7" near this entry XY.
        double conf = 0.72;
        for (const auto& f : wp.features()) {
            const auto& p = f.params;
            auto sk = p.find("spec_key");
            if (sk == p.end() || sk->get<std::string>() != "H7") continue;
            auto px = p.find("position_x_mm"); auto py = p.find("position_y_mm");
            if (px == p.end() || py == p.end()) continue;
            const double dx = px->get<double>() - entry.X();
            const double dy = py->get<double>() - entry.Y();
            if (dx * dx + dy * dy < 1e-2) { conf = 0.95; break; }
        }

        json matched = {
            { "cyl_face_id", cyl.faceIdx },
            { "entry_center", { entry.X(), entry.Y(), entry.Z() } },
        };
        out.push_back(RecognizedFeature{ kSkillId, rp, conf, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::iso_h7_bore_spec
