// @lat: [[engine/skills#hem_open_closed]]

#include "hem_open_closed.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <Standard_Failure.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::hem_open_closed {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

double sheetThicknessOf(const Workpiece& wp)
{
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    return std::min({ xMax - xMin, yMax - yMin, zMax - zMin });
}

double styleGap(HemStyle style, double r)
{
    switch (style) {
        case HemStyle::Closed:   return 0.0;
        case HemStyle::Open:     return r * 0.5;
        case HemStyle::Teardrop: return r;
    }
    return 0.0;
}

double polylineLength(const std::vector<gp_Pnt>& poly)
{
    double L = 0.0;
    for (size_t i = 1; i < poly.size(); ++i)
        L += poly[i - 1].Distance(poly[i]);
    return L;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    const double t = sheetThicknessOf(wp);
    if (t < 0.5 || t > 6.0) {
        r.add("DFM-HOC-SHEET", "error",
              "hem_open_closed: stock thickness " + std::to_string(t) +
              " mm outside 0.5–6.0 mm");
    }
    if (in.edge_polyline.size() < 2u) {
        r.add("DFM-HOC-POLY", "error",
              "hem_open_closed: edge_polyline needs ≥ 2 points");
    }
    if (t > 0.0 && in.hem_radius_mm < t) {
        r.add("DFM-HOC-RADIUS", "error",
              "hem_open_closed: hem_radius_mm " +
              std::to_string(in.hem_radius_mm) +
              " mm < sheet thickness — crack risk");
    }
    if (in.hem_radius_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "hem_open_closed: hem_radius_mm must be > 0");
    }
    const double minLen = 2.0 * t + M_PI * in.hem_radius_mm;
    if (t > 0.0 && in.hem_length_mm < minLen) {
        r.add("DFM-HOC-LENGTH", "error",
              "hem_open_closed: hem_length_mm " +
              std::to_string(in.hem_length_mm) +
              " mm < 2 × thickness + π × radius (" +
              std::to_string(minLen) + " mm)");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// For each polyline segment we add:
//   - a back-slab of size (back_len × seg_len × t) offset above the sheet
//     by (t + gap)
//   - a half-cylinder wrap (axis along the segment, radius hem_radius_mm)
//     at the end of the segment to model the 180° fold

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "hem_open_closed DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double t   = sheetThicknessOf(wp);
    const double gap = styleGap(in.hem_style, in.hem_radius_mm);
    // Back-slab length = hem_length minus the half-circle wrap (π·R).
    const double backLen = std::max(in.hem_length_mm - M_PI * in.hem_radius_mm, t);

    TopoDS_Shape result = wp.shape();

    for (size_t i = 1; i < in.edge_polyline.size(); ++i) {
        const gp_Pnt& pA = in.edge_polyline[i - 1];
        const gp_Pnt& pB = in.edge_polyline[i];
        gp_Vec dirV(pA, pB);
        const double segLen = dirV.Magnitude();
        if (segLen < 1e-6) continue;
        dirV.Normalize();
        // Inward normal (XY perpendicular, away from outside edge).  For
        // robustness we pick the side that points TOWARD bbox center.
        gp_Vec perp(-dirV.Y(), dirV.X(), 0.0);
        // Cross-check with bbox-center direction.
        gp_Pnt mid((pA.X() + pB.X()) * 0.5,
                   (pA.Y() + pB.Y()) * 0.5,
                   (pA.Z() + pB.Z()) * 0.5);
        const gp_Vec toCenter(
            (xMin + xMax) * 0.5 - mid.X(),
            (yMin + yMax) * 0.5 - mid.Y(),
            0.0);
        if (perp.Dot(toCenter) < 0.0) perp = -perp;

        // 1. Build back-slab box at (sheet_top + gap), extending backLen
        //    inward along perp from the edge.
        const double topZ = zMax;
        const gp_Pnt slabOrigin(
            pA.X(), pA.Y(), topZ + gap);
        // gp_Ax2: mainZ = +Z, xDir = perp (so dx extrudes inward = back_len).
        const gp_Ax2 ax(slabOrigin, gp::DZ(), gp_Dir(perp));
        TopoDS_Shape slab;
        try {
            slab = pr::box(ax, backLen, segLen, t);
        } catch (const Standard_Failure& ex) {
            spdlog::warn("hem_open_closed: slab box failed: {}", ex.what());
            continue;
        }

        // 2. Build a half-cylinder wrap at the far end of the slab.
        //    Cylinder axis = +dirV (segment direction), at edge of slab.
        //    For simplicity build a FULL annulus (outer R = R+t, inner R = R)
        //    centred on the wrap axis; Boolean fuse trims.
        // Place wrap axis at sheet edge, midway through hem_radius vertically.
        const gp_Pnt wrapAxisOrigin(pA.X(), pA.Y(), topZ + gap * 0.5);
        // gp_Ax2: main = dirV; xDir = -perp (so the wrap covers the inward side).
        gp_Vec xv = -perp;
        // Orthogonalize against dirV.
        const gp_Vec b(dirV);
        xv = xv - b.Multiplied(xv.Dot(b));
        if (xv.Magnitude() < 1e-6) xv = gp_Vec(0.0, 0.0, 1.0);
        xv.Normalize();
        try {
            const gp_Ax2 axCyl(wrapAxisOrigin, gp_Dir(dirV), gp_Dir(xv));
            BRepPrimAPI_MakeCylinder outerC(axCyl, in.hem_radius_mm + t, segLen);
            outerC.Build();
            if (!outerC.IsDone()) throw Standard_Failure("hem_open_closed: outer cyl failed");
            BRepPrimAPI_MakeCylinder innerC(axCyl, in.hem_radius_mm, segLen);
            innerC.Build();
            if (!innerC.IsDone()) throw Standard_Failure("hem_open_closed: inner cyl failed");
            const TopoDS_Shape wrap = pr::cut(outerC.Shape(), innerC.Shape());
            result = pr::fuse(result, wrap);
        } catch (const Standard_Failure& ex) {
            spdlog::warn("hem_open_closed: wrap fuse failed: {}; slab only", ex.what());
        }

        try {
            result = pr::fuse(result, slab);
        } catch (const Standard_Failure& ex) {
            spdlog::warn("hem_open_closed: slab fuse failed: {}", ex.what());
        }
    }

    // Signature.
    json polyJson = json::array();
    for (const auto& p : in.edge_polyline)
        polyJson.push_back({ p.X(), p.Y(), p.Z() });

    json params = {
        { "edge_polyline",  polyJson },
        { "hem_style",      hemStyleToString(in.hem_style) },
        { "hem_radius_mm",  in.hem_radius_mm },
        { "hem_length_mm",  in.hem_length_mm },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "sheet_feature_type",    "hem" },
        { "hem_style",             hemStyleToString(in.hem_style) },
        { "hem_radius_mm",         in.hem_radius_mm },
        { "hem_gap_mm",            gap },
        { "developed_length_mm",   in.hem_length_mm },
        { "polyline_length_mm",    polylineLength(in.edge_polyline) },
        { "sheet_thickness_mm",    t },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "press_brake";
    tooling.tool_dia_mm       = in.hem_radius_mm * 2.0;
    tooling.tool_length_mm    = polylineLength(in.edge_polyline);
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 5.0 + polylineLength(in.edge_polyline) * 0.05;
    tooling.extra["machining_constraint"] =
        "Open/closed/teardrop hem — two-hit forming on press brake; "
        "closed = flatten die hit, teardrop = leave R-die pickup, "
        "open = controlled half-fold";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(result, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::hem_open_closed applied: style={} R={} L={} faces {}→{}",
                  hemStyleToString(in.hem_style),
                  in.hem_radius_mm, in.hem_length_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Find a horizontal-axis cylindrical wrap face + parallel +Z planar pair
// stacked above the sheet top.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const double t = sheetThicknessOf(wp);
    if (t <= 0.0 || t > 6.0) return out;

    // Find smallest-radius horizontal-axis cylinder (the wrap).
    int    wrapId  = -1;
    double wrapR   = 0.0;
    gp_Ax1 wrapAxis;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        const gp_Cylinder cyl = surf.Cylinder();
        if (std::abs(cyl.Axis().Direction().Z()) > 0.3) continue;
        if (wrapId < 0 || cyl.Radius() < wrapR) {
            wrapId   = i;
            wrapR    = cyl.Radius();
            wrapAxis = cyl.Axis();
        }
    }
    if (wrapId < 0) return out;

    // Find two +Z planar faces above the sheet plane.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    (void)xMin; (void)yMin; (void)zMin; (void)xMax; (void)yMax;
    const double sheetTopZApprox = zMax - t;          // rough sheet top
    int    backFaceCount = 0;
    double backZHighest  = -1e9;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (n.Z() < 0.85) continue;
        const double z = wp.faceCenter(i).Z();
        if (z > sheetTopZApprox + 0.01) {
            ++backFaceCount;
            if (z > backZHighest) backZHighest = z;
        }
    }
    if (backFaceCount < 1) return out;

    // Determine style from cylinder radius vs. measured gap.
    const double gap = std::max(backZHighest - (sheetTopZApprox + t), 0.0);
    HemStyle style = HemStyle::Closed;
    if (gap > wrapR * 0.75) style = HemStyle::Teardrop;
    else if (gap > wrapR * 0.25) style = HemStyle::Open;

    json polyJson = json::array();
    polyJson.push_back({ wrapAxis.Location().X(),
                         wrapAxis.Location().Y(),
                         wrapAxis.Location().Z() });

    json recovered = {
        { "edge_polyline",  polyJson },
        { "hem_style",      hemStyleToString(style) },
        { "hem_radius_mm",  wrapR > 0.0 ? wrapR : 1.0 },
        { "hem_length_mm",  3.0 * t + M_PI * wrapR },
    };
    json matched = {
        { "wrap_face_id",    wrapId },
        { "back_face_count", backFaceCount },
        { "measured_gap_mm", gap },
        { "sheet_thickness_mm", t },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.70, matched });
    return out;
}

}  // namespace koocadcam::skill::hem_open_closed
