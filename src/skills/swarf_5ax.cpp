// @lat: [[engine/skills#swarf_5ax]]

#include "swarf_5ax.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRep_Builder.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Compound.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace koocadcam::skill::swarf_5ax {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.tool_dia_mm < 1.5) {
        r.add("DFM-002", "error",
              "swarf_5ax tool_dia_mm " + std::to_string(in.tool_dia_mm) +
              " < 1.5 mm (5-axis end-mill min)");
    }
    if (in.guide_polyline_top.size() < 2) {
        r.add("DFM-INPUT", "error",
              "swarf_5ax guide_polyline_top requires >= 2 points (got " +
              std::to_string(in.guide_polyline_top.size()) + ")");
    }
    if (in.guide_polyline_top.size() != in.guide_polyline_bottom.size()) {
        r.add("DFM-INPUT", "error",
              "swarf_5ax: guide_polyline_top and guide_polyline_bottom must "
              "have matching point counts (" +
              std::to_string(in.guide_polyline_top.size()) + " vs " +
              std::to_string(in.guide_polyline_bottom.size()) + ")");
    }
    r.add("DFM-005", "info",
          "swarf_5ax requires 5-axis machine kinematics — tool axis "
          "continuously interpolated between top + bottom guide rails");
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

namespace {

// Build a box "ruler" for one quad segment between (Ti, Ti+1, Bi+1, Bi).
// We pick:
//   xLocal = unit(Ti → Ti+1)                   — sweep direction
//   yLocal = the "ruling" direction projected perpendicular to xLocal
//   zLocal = xLocal × yLocal                   — surface normal
// Then build a box of size (sweepLen, rulerLen, 2*toolRadius) centred on
// the quad midpoint, with its z-axis along zLocal so the box straddles
// the surface +/- toolRadius along zLocal.
TopoDS_Shape buildQuadBox(const std::array<double, 3>& Ti,
                          const std::array<double, 3>& Tn,
                          const std::array<double, 3>& Bi,
                          const std::array<double, 3>& Bn,
                          double toolRadius)
{
    const gp_Pnt pTi(Ti[0], Ti[1], Ti[2]);
    const gp_Pnt pTn(Tn[0], Tn[1], Tn[2]);
    const gp_Pnt pBi(Bi[0], Bi[1], Bi[2]);
    const gp_Pnt pBn(Bn[0], Bn[1], Bn[2]);

    const gp_Vec xVec(pTi, pTn);
    const double sweepLen = xVec.Magnitude();
    if (sweepLen < 1e-9) throw Standard_Failure("swarf_5ax: degenerate sweep");

    // Average ruling vector = midpoint(B) - midpoint(T) projected
    // perpendicular to xVec.
    const gp_Vec rulingRaw(
        0.5 * (pBi.X() + pBn.X()) - 0.5 * (pTi.X() + pTn.X()),
        0.5 * (pBi.Y() + pBn.Y()) - 0.5 * (pTi.Y() + pTn.Y()),
        0.5 * (pBi.Z() + pBn.Z()) - 0.5 * (pTi.Z() + pTn.Z()));
    const double rulerLen = rulingRaw.Magnitude();
    if (rulerLen < 1e-9) throw Standard_Failure("swarf_5ax: degenerate ruler");

    const gp_Dir xDir(xVec);
    gp_Vec yVec = rulingRaw;
    yVec -= gp_Vec(xDir) * yVec.Dot(gp_Vec(xDir));
    if (yVec.Magnitude() < 1e-9) {
        // Ruler is parallel to sweep — pick any perpendicular.
        yVec = gp_Vec(xDir).Crossed(gp_Vec(0.0, 0.0, 1.0));
        if (yVec.Magnitude() < 1e-9) {
            yVec = gp_Vec(xDir).Crossed(gp_Vec(1.0, 0.0, 0.0));
        }
    }
    yVec.Normalize();
    const gp_Dir yDir(yVec);

    // zLocal = xDir × yDir; the box's "depth" axis straddles the surface.
    gp_Vec zRaw = gp_Vec(xDir).Crossed(gp_Vec(yDir));
    if (zRaw.Magnitude() < 1e-9) {
        zRaw = gp_Vec(0.0, 0.0, 1.0);
    }
    zRaw.Normalize();
    const gp_Dir zDir(zRaw);

    // Quad mid-point (centre of the box's xy face).
    const gp_Pnt mid(
        0.25 * (pTi.X() + pTn.X() + pBi.X() + pBn.X()),
        0.25 * (pTi.Y() + pTn.Y() + pBi.Y() + pBn.Y()),
        0.25 * (pTi.Z() + pTn.Z() + pBi.Z() + pBn.Z()));

    // Box origin is the corner from which DX/DY/DZ extend along
    // (mainDir, xDir, mainDir × xDir).  In gp_Ax2(P, V, Vx): V is local Z
    // (the DZ direction), Vx is local X (the DX direction).  We want:
    //   DX along sweep (xDir) of length sweepLen
    //   DY along ruling (yDir) of length rulerLen
    //   DZ along surface normal (zDir) of length 2*toolRadius
    // So: V = zDir, Vx = xDir, then DY = zDir × xDir = yDir (good).
    //
    // Position the corner so the box is centred on `mid`.
    const gp_Pnt origin(
        mid.X() - 0.5 * sweepLen * xDir.X()
                 - 0.5 * rulerLen * yDir.X()
                 - toolRadius * zDir.X(),
        mid.Y() - 0.5 * sweepLen * xDir.Y()
                 - 0.5 * rulerLen * yDir.Y()
                 - toolRadius * zDir.Y(),
        mid.Z() - 0.5 * sweepLen * xDir.Z()
                 - 0.5 * rulerLen * yDir.Z()
                 - toolRadius * zDir.Z());
    const gp_Ax2 ax(origin, zDir, xDir);
    return pr::box(ax, sweepLen, rulerLen, 2.0 * toolRadius);
}

}  // namespace

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "swarf_5ax DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face_datum);
    if (!entryId) throw SkillError("swarf_5ax: entry_face datum unresolved");

    const double radius = in.tool_dia_mm / 2.0;
    const size_t N      = in.guide_polyline_top.size();

    std::vector<TopoDS_Shape> boxes;
    boxes.reserve(N);
    for (size_t i = 0; i + 1 < N; ++i) {
        try {
            boxes.push_back(buildQuadBox(in.guide_polyline_top[i],
                                         in.guide_polyline_top[i + 1],
                                         in.guide_polyline_bottom[i],
                                         in.guide_polyline_bottom[i + 1],
                                         radius));
        } catch (const Standard_Failure&) {
            continue;  // skip degenerate quad silently
        }
    }
    if (boxes.empty())
        throw SkillError("swarf_5ax: no valid swept boxes built from guides");

    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);
    for (const auto& b : boxes) builder.Add(compound, b);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), compound);

    // Estimate path length along the top guide and ruled surface area.
    double pathLen = 0.0;
    double surfaceArea = 0.0;
    for (size_t i = 1; i < N; ++i) {
        const auto& a = in.guide_polyline_top[i - 1];
        const auto& b = in.guide_polyline_top[i];
        const double d = std::sqrt(
            (b[0] - a[0]) * (b[0] - a[0]) +
            (b[1] - a[1]) * (b[1] - a[1]) +
            (b[2] - a[2]) * (b[2] - a[2]));
        pathLen += d;
        // Ruler at i-1: average of top→bottom at i-1 and i.
        const auto& tA = in.guide_polyline_top[i - 1];
        const auto& tB = in.guide_polyline_top[i];
        const auto& bA = in.guide_polyline_bottom[i - 1];
        const auto& bB = in.guide_polyline_bottom[i];
        const double rA = std::sqrt(
            (bA[0] - tA[0]) * (bA[0] - tA[0]) +
            (bA[1] - tA[1]) * (bA[1] - tA[1]) +
            (bA[2] - tA[2]) * (bA[2] - tA[2]));
        const double rB = std::sqrt(
            (bB[0] - tB[0]) * (bB[0] - tB[0]) +
            (bB[1] - tB[1]) * (bB[1] - tB[1]) +
            (bB[2] - tB[2]) * (bB[2] - tB[2]));
        surfaceArea += d * 0.5 * (rA + rB);
    }

    json topJ = json::array();
    for (const auto& p : in.guide_polyline_top)    topJ.push_back({ p[0], p[1], p[2] });
    json botJ = json::array();
    for (const auto& p : in.guide_polyline_bottom) botJ.push_back({ p[0], p[1], p[2] });

    json params = {
        { "entry_face_id",         *entryId },
        { "guide_polyline_top",    topJ },
        { "guide_polyline_bottom", botJ },
        { "tool_dia_mm",           in.tool_dia_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_5axis_cut",               true },
        { "ruled_surface_present",      true },
        { "guide_top_length",           pathLen },
        { "ruled_surface_area_mm2",     surfaceArea },
        { "waypoint_count",             N },
        { "geometry",                   "ruled_surface_box_chain_approximation" },
        { "tool_dia_mm",                in.tool_dia_mm },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = in.tool_dia_mm;
    tooling.tool_length_mm    = std::max(20.0, 4.0 * in.tool_dia_mm);
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.025;
    // Stock removed ~ ruled surface area × kerf (2 × tool_radius).
    tooling.stock_removed_mm3 = surfaceArea * in.tool_dia_mm;
    tooling.est_cycle_time_s  = std::max(5.0, pathLen / 20.0);
    tooling.extra["dfm_findings"]         = findings;
    tooling.extra["machining_constraint"] =
        "5-axis swarf milling on ruled surface — top/bottom guides drive "
        "continuous tool-axis interpolation";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::swarf_5ax applied: N={} dia={} sweep_len={:.2f} faces {}->{}",
                  N, in.tool_dia_mm, pathLen,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Topology-only detection of a ruled surface is hard.  Heuristic: collect
// planar faces whose normals are NOT parallel to any global axis (tilted),
// whose centres form a roughly linear/connected chain in 3D space — these
// look like the facets that the box-chain cut leaves on the workpiece.

namespace {

bool isAxisAlignedDir(const gp_Dir& d)
{
    const double ax = std::abs(d.X());
    const double ay = std::abs(d.Y());
    const double az = std::abs(d.Z());
    return ax > 0.99 || ay > 0.99 || az > 0.99;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    struct Facet {
        int     faceIdx;
        gp_Pnt  centre;
        double  area;
    };
    std::vector<Facet> tilted;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n(0, 0, 1);
        try { n = wp.faceNormal(i); }
        catch (...) { continue; }
        if (isAxisAlignedDir(n)) continue;
        const double a = wp.faceArea(i);
        if (a < 1.0) continue;
        tilted.push_back({ i, wp.faceCenter(i), a });
    }
    if (tilted.size() < 2) return out;

    // Sort by linear extent along the longest axis to form a chain.
    int bestI = 0, bestJ = 1;
    double bestD = 0.0;
    for (size_t a = 0; a < tilted.size(); ++a)
        for (size_t b = a + 1; b < tilted.size(); ++b) {
            const double d = tilted[a].centre.Distance(tilted[b].centre);
            if (d > bestD) { bestD = d; bestI = static_cast<int>(a); bestJ = static_cast<int>(b); }
        }
    const gp_Pnt& A = tilted[bestI].centre;
    const gp_Pnt& B = tilted[bestJ].centre;
    const gp_Vec lineDir(A, B);
    if (lineDir.Magnitude() < 1e-6) return out;
    const gp_Vec lineUnit = lineDir.Normalized();

    struct Sortable {
        double  t;
        int     faceIdx;
        gp_Pnt  centre;
        double  area;
    };
    std::vector<Sortable> sortable;
    for (const auto& f : tilted) {
        const gp_Vec v(A, f.centre);
        sortable.push_back({ v.Dot(lineUnit), f.faceIdx, f.centre, f.area });
    }
    std::sort(sortable.begin(), sortable.end(),
              [](const Sortable& a, const Sortable& b) { return a.t < b.t; });

    json poly    = json::array();
    json faceIds = json::array();
    double areaSum = 0.0;
    for (const auto& s : sortable) {
        poly.push_back({ s.centre.X(), s.centre.Y(), s.centre.Z() });
        faceIds.push_back(s.faceIdx);
        areaSum += s.area;
    }

    json recovered = {
        // We don't have a separate top/bottom guide from recognition; pass
        // the chain as a single approximation.
        { "guide_polyline_top",    poly },
        { "guide_polyline_bottom", poly },   // unrecoverable
        { "tool_dia_mm",           0.0 },    // unrecoverable
    };
    json matched = {
        { "tilted_face_ids", faceIds },
        { "linear_extent",   bestD },
        { "total_area",      areaSum },
    };
    out.push_back(RecognizedFeature{
        kSkillId, recovered, /*confidence*/ 0.40, matched
    });
    return out;
}

}  // namespace koocadcam::skill::swarf_5ax
