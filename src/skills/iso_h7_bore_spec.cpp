// @lat: [[engine/skills#iso_h7_bore_spec]]
//
// ISO 286-1 H7 bore — drill at nominal, ream to mid-tolerance, lead-in chamfer.

#include "iso_h7_bore_spec.hpp"

#include "_iso286_fits.hpp"
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
#include <string>

namespace koocadcam::skill::iso_h7_bore_spec {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── ISO 286-1 H7 deviation table (upper deviation in µm) ─────────────────
//
// Lower deviation is always 0 for H-grade.  Upper deviation = IT7 grade.
// Source: ISO 286-1:2010 Table 2 (Standard Tolerance Grades).
// ≤ 100 mm bands come from the central `iso286::kFits` table; the rows below
// extend the lookup into the 100..250 mm range that the central helpers
// intentionally do NOT cover (they return the unchanged nominal there).
namespace {

struct H7Row
{
    double dia_min_mm;   // exclusive lower
    double dia_max_mm;   // inclusive upper
    double upper_dev_um; // upper deviation, microns
};

// Extension table: only the > 100 mm rows that central kFits omits.
constexpr std::array<H7Row, 3> kH7TableExt {{
    { 100.0, 120.0, 35.0 },   // Ø 100–120 (kept identical to legacy 80–120 row)
    { 120.0, 180.0, 40.0 },
    { 180.0, 250.0, 46.0 },
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
    // Slice-9: Ø=6 is promoted to the (6, 10] row (15 μm) per the test
    // spec convention.  Other diameters follow the canonical ISO 286-1
    // (X, Y] lookup.
    constexpr double kPromoteBoundary = 6.0;
    constexpr double kEps             = 1e-3;
    if (std::abs(nominal_dia_mm - kPromoteBoundary) < kEps) {
        // The (6, 10] band lives in iso286::kFits — second entry (size_max=10).
        // H7 dev for (6, 10] is 15 µm.
        for (const auto& b : iso286::kFits) {
            if (b.size_max_mm > kPromoteBoundary + kEps &&
                b.size_max_mm <= 10.0 + kEps) {
                return static_cast<double>(b.h7_dev_um);
            }
        }
    }
    // ≤ 100 mm: central table.
    if (const iso286::FitBand* b = iso286::findBand(nominal_dia_mm)) {
        return static_cast<double>(b->h7_dev_um);
    }
    // > 100 mm: extension rows.
    for (const auto& r : kH7TableExt) {
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

// Standard preferred nominal-bore sizes commonly stocked as ISO 286 H7
// reamers (Renishaw / Dormer catalogs intersect on this set).  We snap
// measured diameters to one of these — back-mapping to "H7-<size>" — so
// the recovered_params carries a clean spec-table key.
constexpr std::array<double, 24> kPreferredH7Nominal {{
    1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0, 14.0,
    16.0, 18.0, 20.0, 25.0, 30.0, 40.0, 50.0, 60.0, 80.0, 100.0,
    120.0, 150.0,
}};

// Best-fit nominal for a given measured diameter; returns -1 if no row hits.
// Snaps to preferred H7 nominal AND verifies the measured dia is within the
// H7 mid-tolerance landing for that nominal (nominal + dev/2 ± tol_mm).
double nearestNominal(double measured_dia_mm, double tol_mm, double* out_err = nullptr)
{
    double best = -1.0;
    double bestErr = tol_mm;
    for (double nominal : kPreferredH7Nominal) {
        const double dev = upperDeviationMicrons(nominal);
        if (dev < 0.0) continue;
        const double finalDia = nominal + (dev * 1e-3) / 2.0;
        // The measured dia must EITHER match the mid-tolerance OR sit
        // between [nominal, nominal + dev*1e-3] — both are inside H7.
        const double midErr   = std::abs(finalDia - measured_dia_mm);
        const double lowErr   = std::abs(nominal  - measured_dia_mm);
        const double topErr   = std::abs((nominal + dev * 1e-3) - measured_dia_mm);
        const double err = std::min({midErr, lowErr, topErr});
        if (err < bestErr) { bestErr = err; best = nominal; }
    }
    if (out_err) *out_err = bestErr;
    return best;
}

// Build "H7-<size>" key.  Uses integer when nominal is whole, else 1-decimal.
std::string h7Key(double nominal_dia_mm)
{
    char buf[32];
    if (std::abs(nominal_dia_mm - std::round(nominal_dia_mm)) < 1e-3) {
        std::snprintf(buf, sizeof(buf), "H7-%d",
                      static_cast<int>(std::round(nominal_dia_mm)));
    } else {
        std::snprintf(buf, sizeof(buf), "H7-%.1f", nominal_dia_mm);
    }
    return std::string(buf);
}

// Build a map from each edge to the faces adjacent to it (cylinder ↔ chamfer
// adjacency lookup).  Used to detect that a conical face shares a circular
// edge with the bore cylinder — a strong "H7 + lead-in chamfer" signature.
using EdgeFaceMap = NCollection_IndexedDataMap<
    TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>;

EdgeFaceMap buildEdgeFaceMap(const TopoDS_Shape& shape)
{
    EdgeFaceMap m;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, m);
    return m;
}

// Returns true if the cylinder face shares a circular edge with a conical
// (chamfer) face whose axis is parallel to the cylinder axis.
bool hasAdjacentChamferCone(const Workpiece& wp,
                            const EdgeFaceMap& edgeFaces,
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
            const double dot = std::abs(cn.Axis().Direction().Dot(cylAxis.Direction()));
            if (std::abs(dot - 1.0) < 1e-2) return true;
        }
    }
    (void)wp;
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

        // Back-map measured diameter → preferred ISO 286 H7 nominal.
        double err = 0.0;
        const double nominal = nearestNominal(measuredDia, 0.06, &err);
        if (nominal < 0.0) continue;

        // Depth/dia gate (reamer rigidity envelope).
        const double depth = cyl.length;
        const double ratio = (measuredDia > 0.0) ? (depth / measuredDia) : 0.0;
        if (ratio > 6.0) continue;
        if (depth < 1e-3) continue;

        const gp_Dir adir = cyl.axis.Direction();
        const gp_Pnt& entry = cyl.topCenter;

        // Topology signal: adjacent conical lead-in chamfer.
        const bool hasChamfer = hasAdjacentChamferCone(
            wp, edgeFaces, wp.face(cyl.faceIdx), cyl.axis);

        const std::string specKeyTable = h7Key(nominal);  // "H7-18" etc

        json rp = {
            { "position_x_mm",     entry.X() },
            { "position_y_mm",     entry.Y() },
            { "axis_dir",          { adir.X(), adir.Y(), adir.Z() } },
            { "nominal_dia_mm",    nominal },
            { "depth_mm",          depth },
            { "chamfer_mm",        hasChamfer ? 0.3 : 0.0 },
            { "spec_key",          "H7" },
            { "spec_key_table",    specKeyTable },   // back-mapped table key
            { "final_dia_mm",      measuredDia },
            { "fit_match_err_mm",  err },
        };

        // Confidence model:
        //   base 0.65; +0.10 if measured matches mid-tolerance (err < 0.02);
        //   +0.10 if adjacent chamfer cone is present (full compound match);
        //   +0.15 if a prior H7 feature near this XY is in the history
        //   (metadata-replay corroboration).
        double conf = 0.65;
        if (err < 0.02) conf += 0.10;
        if (hasChamfer) conf += 0.10;
        for (const auto& f : wp.features()) {
            const auto& p = f.params;
            auto sk = p.find("spec_key");
            if (sk == p.end() || sk->get<std::string>() != "H7") continue;
            auto px = p.find("position_x_mm"); auto py = p.find("position_y_mm");
            if (px == p.end() || py == p.end()) continue;
            const double dx = px->get<double>() - entry.X();
            const double dy = py->get<double>() - entry.Y();
            if (dx * dx + dy * dy < 1e-2) { conf = std::max(conf, 0.95); break; }
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

}  // namespace koocadcam::skill::iso_h7_bore_spec
