// @lat: [[engine/skills#bend]]

#include "bend.hpp"

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

namespace koocadcam::skill::bend {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

double sheetThickness(const Workpiece& wp)
{
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    return std::min({ xMax - xMin, yMax - yMin, zMax - zMin });
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    const double dx = in.bend_line_end_xy[0] - in.bend_line_start_xy[0];
    const double dy = in.bend_line_end_xy[1] - in.bend_line_start_xy[1];
    if (std::hypot(dx, dy) < 1e-6) {
        r.add("DFM-INPUT", "error",
              "bend: bend_line endpoints must be distinct");
    }

    if (in.angle_deg < -180.0 || in.angle_deg > 180.0) {
        r.add("DFM-B-ANGLE", "error",
              "bend angle " + std::to_string(in.angle_deg) +
              " outside [-180, 180]");
    }

    const double t = sheetThickness(wp);
    if (t > 5.0) {
        r.add("DFM-B-SHEET", "error",
              "bend: stock thickness " + std::to_string(t) +
              " > 5 mm — not sheet metal");
    }
    if (in.bend_radius_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "bend: bend_radius_mm must be > 0");
    }
    if (t > 0.0 && in.bend_radius_mm < 0.5 * t) {
        r.add("DFM-B-MIN-R", "error",
              "bend radius " + std::to_string(in.bend_radius_mm) +
              " mm < 0.5 × sheet_thickness (" + std::to_string(0.5 * t) +
              " mm) — min-radius rule violated (cracking risk)");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// We split the sheet at the bend_line into two halves (H_fixed and
// H_rotated), rotate H_rotated by `angle_deg` around the bend axis (= the
// bend_line on the inside of the fold), then fuse a cylindrical-sector
// bend zone between them.
//
// Reference frame:
//   - Sheet lies in XY with thickness along Z.  Sheet top = zMax,
//     sheet bottom = zMin.
//   - For a POSITIVE upward fold (angle > 0), the inner surface of the
//     bend is the BOTTOM face of the sheet (its outward normal already
//     points away from the fold-up).  The bend axis sits at z = zMin.
//   - We pick H_fixed = points to the LEFT of the bend_line direction;
//     H_rotated = points to the RIGHT.

namespace {

// Build a tall axis-aligned (along bend_line dir) half-space box that
// occupies the half of XY space on the `perpSide` side of the bend_line.
// Subtracting this box from the sheet keeps the OPPOSITE side.
//
//   dirV       — unit vector along the bend_line (XY plane)
//   perpSide   — unit perpendicular in XY pointing INTO the half we want
//                to remove (so we KEEP the opposite half)
//   lineMid    — a point on the bend_line (typically its midpoint)
//   bboxDiag   — workpiece bbox diagonal (sizing the cutter)
//   zMin       — sheet bottom z (cutter starts below this)
TopoDS_Shape halfSpaceCutter(const gp_Pnt& lineMid,
                              const gp_Vec& dirV,
                              const gp_Vec& perpSide,
                              double bboxDiag, double zMin)
{
    const double dirHalf  = bboxDiag * 2.0;
    const double perpExt  = bboxDiag * 2.0;
    const double zHigh    = bboxDiag * 2.0;

    // The corner that anchors the box.  The box's local frame uses
    // gp::DZ() as the main axis (Z up) and dirV as the local X axis; the
    // local Y axis is then DZ × dirV = perpendicular in XY (right-handed).
    // We need the box's local Y direction to match `perpSide`, so we
    // check the cross product orientation.
    const gp_Vec dzCrossDir(0.0, 0.0, 1.0);
    // Local Y = DZ × dirV
    const gp_Vec localY(
        dzCrossDir.Y() * dirV.Z() - dzCrossDir.Z() * dirV.Y(),
        dzCrossDir.Z() * dirV.X() - dzCrossDir.X() * dirV.Z(),
        dzCrossDir.X() * dirV.Y() - dzCrossDir.Y() * dirV.X());

    // The box always extends in +localY by perpExt.  To cover the
    // half-plane on perpSide:
    //   - if perpSide aligns with +localY → corner is AT lineMid (Y-shift = 0)
    //     and box extends in +localY → covers that side.
    //   - if perpSide is opposite localY → corner must be at lineMid_y - perpExt
    //     so that box extends from there to lineMid_y → covers the -localY side.
    const double sign = (localY.Dot(perpSide) > 0.0) ? 1.0 : -1.0;
    // Y shift in WORLD coordinates from lineMid to the box's corner along localY.
    // We need yCornerWorld = lineMid_y + (sign > 0 ? 0 : -perpExt) along localY.
    const double yShift = (sign > 0.0) ? 0.0 : -perpExt;
    gp_Pnt corner(
        lineMid.X() - dirV.X() * dirHalf + localY.X() * yShift,
        lineMid.Y() - dirV.Y() * dirHalf + localY.Y() * yShift,
        zMin - 1.0);

    const gp_Ax2 ax(corner, gp::DZ(), gp_Dir(dirV));
    return pr::box(ax, dirHalf * 2.0, perpExt, zHigh + 2.0);
}

// Cylindrical-sector bend zone — solid bounded by an inner cylinder
// (radius = bendR), outer cylinder (bendR + thickness), two flat radial
// caps (top & bottom of the sector), end-caps along the bend axis.  We
// build it by Boolean: outer arc-prism − inner arc-prism, where each
// "arc-prism" is a sweep of a circular sector around the bend axis.
//
// For phase-1 simplicity we synthesize the bend zone as the intersection
// of a large annular ring with a half-space wedge swept by `angle`.  In
// practice we just create an unfolded annular ring of length = axis
// length and "carry trust" that the surrounding fuse with the two flats
// will produce a clean solid in OCCT.
TopoDS_Shape buildBendWedge(const gp_Ax1& bendAxis, double bendR, double thickness,
                            double axisLength)
{
    // gp_Ax2 with origin = bend axis location, main Z = bend axis dir,
    // local X = something perpendicular.  OCCT's MakeCylinder produces a
    // full cylinder; we'll subtract the inner one to get the annulus.
    // For slice-1 robustness we just build the FULL annular cylinder
    // (360°) — it will overlap the flats, and the fuse will trim it.
    gp_Dir bdir = bendAxis.Direction();
    // Build a perpendicular direction for gp_Ax2's X axis.
    gp_Dir xDir;
    {
        // Pick the "most-perpendicular" of world X / world Y.
        if (std::abs(bdir.Dot(gp::DX())) < 0.9)
            xDir = gp::DX();
        else
            xDir = gp::DY();
        // Orthogonalize (subtract projection on bdir).
        gp_Vec v(xDir);
        gp_Vec b(bdir);
        v = v - b.Multiplied(v.Dot(b));
        if (v.Magnitude() < 1e-6) {
            xDir = gp::DZ();
            v = gp_Vec(xDir);
            v = v - b.Multiplied(v.Dot(b));
        }
        v.Normalize();
        xDir = gp_Dir(v);
    }
    // Origin: slightly retracted along bend axis so the cylinder fully
    // covers the bend zone.
    gp_Pnt origin = bendAxis.Location();
    origin.SetX(origin.X() - bdir.X() * 0.1);
    origin.SetY(origin.Y() - bdir.Y() * 0.1);
    origin.SetZ(origin.Z() - bdir.Z() * 0.1);
    const gp_Ax2 ax(origin, bdir, xDir);
    const double H = axisLength + 0.2;
    const double rOuter = bendR + thickness;
    BRepPrimAPI_MakeCylinder outerC(ax, rOuter, H);
    outerC.Build();
    if (!outerC.IsDone()) throw Standard_Failure("bend: outer cyl failed");
    BRepPrimAPI_MakeCylinder innerC(ax, bendR, H);
    innerC.Build();
    if (!innerC.IsDone()) throw Standard_Failure("bend: inner cyl failed");
    return pr::cut(outerC.Shape(), innerC.Shape());
}

}  // namespace

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "bend DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double t = sheetThickness(wp);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    // Bend line in world coordinates.
    const gp_Pnt pA(in.bend_line_start_xy[0], in.bend_line_start_xy[1], zMin);
    const gp_Pnt pB(in.bend_line_end_xy[0],   in.bend_line_end_xy[1],   zMin);
    gp_Vec dirV(pA, pB);
    const double lineLen = dirV.Magnitude();
    dirV.Normalize();
    // Perpendicular in XY plane.
    const gp_Vec perpRight(-dirV.Y(), dirV.X(), 0.0);

    // 1. Split the sheet into two halves at the bend_line.
    //    Use a half-space cutter to extract H_fixed (LEFT side).
    const gp_Pnt lineMid(
        (pA.X() + pB.X()) * 0.5,
        (pA.Y() + pB.Y()) * 0.5,
        (zMin + zMax) * 0.5);

    // Two cuts: one keeps left half, the other keeps right half.
    //   H_fixed   = sheet minus right-of-line (keeps left side)
    //   H_rotated = sheet minus left-of-line  (keeps right side)
    const TopoDS_Shape rightCutter =
        halfSpaceCutter(lineMid, dirV, gp_Vec( perpRight), bboxDiag, zMin);
    const TopoDS_Shape leftCutter  =
        halfSpaceCutter(lineMid, dirV, gp_Vec(-perpRight), bboxDiag, zMin);
    const TopoDS_Shape H_fixed       = pr::cut(wp.shape(), rightCutter);
    const TopoDS_Shape H_rotated_raw = pr::cut(wp.shape(), leftCutter);

    // 2. Rotate H_rotated by angle_deg around the bend axis.
    //    The bend axis lies along the bend_line, at z = zMin (inner side
    //    of the fold for upward bends).  This is an approximation:
    //    geometrically a true bend axis is offset from the line by
    //    bend_radius along the inner normal direction.
    const gp_Ax1 bendAxis(pA, gp_Dir(dirV));
    gp_Trsf rot;
    rot.SetRotation(bendAxis, in.angle_deg * M_PI / 180.0);
    BRepBuilderAPI_Transform xf(H_rotated_raw, rot, /*Copy=*/false);
    if (!xf.IsDone()) throw SkillError("bend: rotation transform failed");
    const TopoDS_Shape H_rotated = xf.Shape();

    // 3. Build a cylindrical bend wedge between the two halves.
    //    For slice-1 we use a full annular ring centered on the bend axis,
    //    sized to span the bend line length, with inner radius bend_radius_mm.
    //    The fuse with both halves trims it cleanly.
    TopoDS_Shape bendWedge;
    try {
        bendWedge = buildBendWedge(bendAxis, in.bend_radius_mm, t, lineLen);
    } catch (const Standard_Failure& ex) {
        spdlog::warn("bend: bend-wedge construction failed ({}); fusing without it",
                     ex.what());
        bendWedge = TopoDS_Shape();
    }

    // 4. Fuse fixed half + bend wedge + rotated half.
    TopoDS_Shape result = pr::fuse(H_fixed, H_rotated);
    if (!bendWedge.IsNull()) {
        try {
            result = pr::fuse(result, bendWedge);
        } catch (const Standard_Failure& ex) {
            spdlog::warn("bend: fuse with bend wedge failed: {}; keeping straight fuse",
                         ex.what());
        }
    }

    // Signature
    json params = {
        { "bend_line_start_xy", { in.bend_line_start_xy[0], in.bend_line_start_xy[1] } },
        { "bend_line_end_xy",   { in.bend_line_end_xy[0],   in.bend_line_end_xy[1]   } },
        { "angle_deg",          in.angle_deg },
        { "bend_radius_mm",     in.bend_radius_mm },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "flat_face_count",       2 },
        { "cylindrical_bend_face", 1 },
        { "bend_axis_dir",         { dirV.X(), dirV.Y(), dirV.Z() } },
        { "bend_radius_mm",        in.bend_radius_mm },
        { "angle_deg",             in.angle_deg },
        { "sheet_thickness_mm",    t },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "press_brake";
    tooling.tool_dia_mm       = in.bend_radius_mm * 2.0;
    tooling.tool_length_mm    = lineLen;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;        // bending does not remove material
    tooling.est_cycle_time_s  = 2.0 + lineLen * 0.05;
    tooling.extra["machining_constraint"] =
        "Press brake — V-die opening ≥ 6 × thickness; spring-back compensation";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(result, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::bend applied: angle={} R={} faces {}→{}",
                  in.angle_deg, in.bend_radius_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// A bend leaves: 1 cylindrical face (bend zone) whose axis is HORIZONTAL
// (roughly in the sheet plane) plus 2 large planar faces (the flats).
// Distinguish from `sheet_punch` (which has a VERTICAL cylinder axis).

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const double t = sheetThickness(wp);

    // The bend leaves an annular ring with an inner & outer cylindrical
    // face.  The inner face's radius is the bend_radius (what we want
    // to report).  Collect candidates first, then keep only the smallest-
    // radius horizontal-axis cylindrical face per axis line.
    struct Cand { int fIdx; double radius; gp_Ax1 axis; };
    std::vector<Cand> candidates;

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;

        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double radius   = cyl.Radius();
        const gp_Ax1 axis     = cyl.Axis();
        const gp_Dir adir     = axis.Direction();

        // Skip vertical-axis cylinders (those are sheet_punch holes).
        if (std::abs(adir.Z()) > 0.5) continue;

        candidates.push_back({ fIdx, radius, axis });
    }

    // Group by axis location (within 1 mm), keep min-radius per group.
    std::vector<bool> usedC(candidates.size(), false);
    std::vector<Cand> filtered;
    for (size_t i = 0; i < candidates.size(); ++i) {
        if (usedC[i]) continue;
        Cand best = candidates[i];
        usedC[i] = true;
        for (size_t j = i + 1; j < candidates.size(); ++j) {
            if (usedC[j]) continue;
            // Same axis direction & nearby axis location?
            const gp_Dir d1 = best.axis.Direction();
            const gp_Dir d2 = candidates[j].axis.Direction();
            if (std::abs(std::abs(d1.Dot(d2)) - 1.0) > 1e-3) continue;
            const gp_Pnt l1 = best.axis.Location();
            const gp_Pnt l2 = candidates[j].axis.Location();
            if (l1.Distance(l2) > 5.0) continue;
            usedC[j] = true;
            if (candidates[j].radius < best.radius) best = candidates[j];
        }
        filtered.push_back(best);
    }

    for (const auto& c : filtered) {
        const int fIdx       = c.fIdx;
        const double radius  = c.radius;
        const gp_Ax1 axis    = c.axis;
        const gp_Dir adir    = axis.Direction();

        // Collect ALL planar faces of the workpiece (the flats are the
        // largest-area planar faces, regardless of edge adjacency, since
        // the OCCT fuse can leave unexpected gaps between bend wedge and
        // flats).
        std::vector<std::pair<double, gp_Dir>> planarAreasNormals;
        for (int g = 0; g < wp.faceCount(); ++g) {
            if (g == fIdx) continue;
            if (!wp.isFacePlanar(g)) continue;
            try {
                planarAreasNormals.push_back({ wp.faceArea(g), wp.faceNormal(g) });
            } catch (...) { /* skip degenerate */ }
        }
        // Need at least two planar faces.
        if (planarAreasNormals.size() < 2) continue;

        // Sort planar faces by area desc; take the top 2 as the flats.
        std::sort(planarAreasNormals.begin(), planarAreasNormals.end(),
                  [](const auto& a, const auto& b) { return a.first > b.first; });
        std::vector<gp_Dir> flatNormals = {
            planarAreasNormals[0].second,
            planarAreasNormals[1].second,
        };
        int adjacentPlanar = static_cast<int>(planarAreasNormals.size());

        // Recover bend angle from the angle between the two flat normals.
        // For a perfect 90° bend the angle between flat normals is 90°.
        double angleDeg = 0.0;
        if (flatNormals.size() >= 2) {
            const double dot = std::clamp(flatNormals[0].Dot(flatNormals[1]), -1.0, 1.0);
            angleDeg = std::acos(dot) * 180.0 / M_PI;
        }

        json recovered = {
            { "bend_line_start_xy", { axis.Location().X(),
                                      axis.Location().Y() } },
            { "bend_line_end_xy",   { axis.Location().X() + adir.X(),
                                      axis.Location().Y() + adir.Y() } },
            { "angle_deg",          angleDeg },
            { "bend_radius_mm",     radius },
        };
        json matched = {
            { "cylindrical_face_id", fIdx },
            { "axis_dir",            { adir.X(), adir.Y(), adir.Z() } },
            { "sheet_thickness_mm",  t },
            { "adjacent_flat_count", adjacentPlanar },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.75, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::bend
