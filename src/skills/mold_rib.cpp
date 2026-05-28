// @lat: [[engine/skills#mold_rib]]

#include "mold_rib.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <TopoDS_Wire.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::mold_rib {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// Default wall_thickness reference used when DFM cannot infer the surrounding
// wall.  Production: a real value should come from process_plan context.
static constexpr double kAssumedWallThicknessMm = 1.6;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    const double rlen = std::hypot(in.end_x_mm - in.start_x_mm,
                                   in.end_y_mm - in.start_y_mm);

    if (in.thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "mold_rib thickness_mm must be > 0");
    } else if (in.thickness_mm < 0.6) {
        r.add("DFM-RIB-MIN-THICK", "error",
              "mold_rib thickness " + std::to_string(in.thickness_mm) +
              " mm < 0.6 mm injection moldable minimum");
    }

    if (in.height_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "mold_rib height_mm must be > 0");
    }

    if (rlen < 1e-6) {
        r.add("DFM-INPUT", "error",
              "mold_rib start and end XY must differ (length > 0)");
    }

    if (in.thickness_mm > 0.0 && in.height_mm > in.thickness_mm * 3.0 + 1e-9) {
        r.add("DFM-RIB-STAB", "warning",
              "mold_rib height/thickness ratio " +
              std::to_string(in.height_mm / in.thickness_mm) +
              " > 3 — unstable, prone to deflection");
    }

    if (in.draft_angle_deg < 0.5) {
        r.add("DFM-RIB-DRAFT", "warning",
              "mold_rib draft_angle " + std::to_string(in.draft_angle_deg) +
              " deg < 0.5 — mould release will be problematic");
    }

    // Sink-mark check vs assumed surrounding wall.
    if (in.thickness_mm > kAssumedWallThicknessMm * 0.6 + 1e-9) {
        r.add("DFM-RIB-SINK", "info",
              "mold_rib thickness " + std::to_string(in.thickness_mm) +
              " > 0.6 × assumed wall " +
              std::to_string(kAssumedWallThicknessMm) +
              " — risk of sink marks on opposite face");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

namespace {

// Build a rib solid in the entry plane.
//   center XY axis : start → end
//   plan footprint : length × thickness (thickness perpendicular to axis)
//   extrusion      : perpendicular to entry_plane, height `height`,
//                    tapered by draft (top is narrower if draft > 0).
//
// Implementation: ThruSections between bottom rectangle (full thickness)
// and top rectangle (drafted thickness).  ThruSections produces a solid
// with a planar bottom + planar top + 4 side faces.
TopoDS_Shape buildRibSolid(const gp_Pnt& startPnt,
                           const gp_Pnt& endPnt,
                           double thickness,
                           double height,
                           const gp_Dir& extrudeDir,
                           double draftDeg)
{
    // In-plane axis.
    const gp_Vec axVec(startPnt, endPnt);
    const double length = axVec.Magnitude();
    if (length < 1e-9) {
        throw SkillError("mold_rib: zero length axis");
    }
    const gp_Dir xLoc(axVec.X() / length, axVec.Y() / length, axVec.Z() / length);
    // yLoc = extrudeDir × xLoc (in-plane perpendicular)
    gp_Vec yVec = gp_Vec(extrudeDir).Crossed(gp_Vec(xLoc));
    if (yVec.Magnitude() < 1e-9) {
        throw SkillError("mold_rib: axis parallel to extrude direction");
    }
    yVec.Normalize();
    const gp_Dir yLoc(yVec.X(), yVec.Y(), yVec.Z());

    auto cornerAt = [&](const gp_Pnt& along, double yOffset, double zOffset) {
        return gp_Pnt(
            along.X() + yOffset * yLoc.X() + zOffset * extrudeDir.X(),
            along.Y() + yOffset * yLoc.Y() + zOffset * extrudeDir.Y(),
            along.Z() + yOffset * yLoc.Z() + zOffset * extrudeDir.Z());
    };

    const double halfT = thickness / 2.0;
    // Bottom rectangle (in entry plane).
    const gp_Pnt b00 = cornerAt(startPnt, -halfT, 0.0);
    const gp_Pnt b01 = cornerAt(startPnt,  halfT, 0.0);
    const gp_Pnt b10 = cornerAt(endPnt,    halfT, 0.0);
    const gp_Pnt b11 = cornerAt(endPnt,   -halfT, 0.0);

    BRepBuilderAPI_MakePolygon bottomPoly;
    bottomPoly.Add(b00); bottomPoly.Add(b01); bottomPoly.Add(b10); bottomPoly.Add(b11);
    bottomPoly.Close();
    if (!bottomPoly.IsDone()) throw SkillError("mold_rib: bottom polygon failed");
    const TopoDS_Wire bottomWire = bottomPoly.Wire();

    // Top rectangle (offset by `height` along extrudeDir, drafted).
    // Drafted half-thickness = halfT - height * tan(draftDeg).
    const double draftRad = draftDeg * M_PI / 180.0;
    double halfTTop = halfT - height * std::tan(draftRad);
    if (halfTTop < 1e-4) halfTTop = 1e-4;  // clamp to avoid degenerate top
    // Drafted length: top is also slightly shorter along the rib axis
    // (so the rib end caps are also tapered).  Symmetric shrink.
    double lengthOffset = height * std::tan(draftRad);
    if (lengthOffset > length / 2.0 - 1e-4) lengthOffset = length / 2.0 - 1e-4;

    // Recompute top end points (shrunk along xLoc).
    const gp_Pnt topStart(
        startPnt.X() + lengthOffset * xLoc.X() + height * extrudeDir.X(),
        startPnt.Y() + lengthOffset * xLoc.Y() + height * extrudeDir.Y(),
        startPnt.Z() + lengthOffset * xLoc.Z() + height * extrudeDir.Z());
    const gp_Pnt topEnd(
        endPnt.X()   - lengthOffset * xLoc.X() + height * extrudeDir.X(),
        endPnt.Y()   - lengthOffset * xLoc.Y() + height * extrudeDir.Y(),
        endPnt.Z()   - lengthOffset * xLoc.Z() + height * extrudeDir.Z());

    const gp_Pnt t00 = cornerAt(topStart, -halfTTop, 0.0);
    const gp_Pnt t01 = cornerAt(topStart,  halfTTop, 0.0);
    const gp_Pnt t10 = cornerAt(topEnd,    halfTTop, 0.0);
    const gp_Pnt t11 = cornerAt(topEnd,   -halfTTop, 0.0);

    BRepBuilderAPI_MakePolygon topPoly;
    topPoly.Add(t00); topPoly.Add(t01); topPoly.Add(t10); topPoly.Add(t11);
    topPoly.Close();
    if (!topPoly.IsDone()) throw SkillError("mold_rib: top polygon failed");
    const TopoDS_Wire topWire = topPoly.Wire();

    BRepOffsetAPI_ThruSections lofter(true /*isSolid*/, true /*ruled*/);
    lofter.AddWire(bottomWire);
    lofter.AddWire(topWire);
    lofter.Build();
    if (!lofter.IsDone()) throw SkillError("mold_rib: ThruSections failed");
    return lofter.Shape();
}

}  // namespace

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "mold_rib DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("mold_rib: entry_face datum unresolved");

    BRepAdaptor_Surface entrySurf(wp.face(*entryId));
    if (entrySurf.GetType() != GeomAbs_Plane) {
        throw SkillError("mold_rib: entry_face must be planar");
    }
    const gp_Pln entryPlane = entrySurf.Plane();
    gp_Dir outward = entryPlane.Axis().Direction();
    if (wp.face(*entryId).Orientation() == TopAbs_REVERSED) outward.Reverse();

    // Project start/end onto entry plane.  (Caller provides world XY, but
    // entry face may not be Z = const; project the (x,y,0) point onto the
    // plane along the outward normal.)
    auto projectOntoPlane = [&](double x, double y) {
        // Find Z so that the point lies on entry plane.
        const gp_Pnt P0 = entryPlane.Location();
        const gp_Dir n  = entryPlane.Axis().Direction();
        // Plane: n · (P - P0) = 0.  Set P = (x, y, z), solve for z.
        if (std::abs(n.Z()) < 1e-9) {
            // Plane is vertical — fall back to using P0.Z() for Z.
            return gp_Pnt(x, y, P0.Z());
        }
        const double z = P0.Z() - (n.X() * (x - P0.X()) + n.Y() * (y - P0.Y())) / n.Z();
        return gp_Pnt(x, y, z);
    };

    gp_Pnt startPnt = projectOntoPlane(in.start_x_mm, in.start_y_mm);
    gp_Pnt endPnt   = projectOntoPlane(in.end_x_mm,   in.end_y_mm);

    // Embed the rib base slightly INTO the workpiece to guarantee a clean
    // fuse Boolean (avoids coplanar-face issues).
    constexpr double kEmbed = 0.02;
    startPnt = gp_Pnt(startPnt.X() - outward.X() * kEmbed,
                      startPnt.Y() - outward.Y() * kEmbed,
                      startPnt.Z() - outward.Z() * kEmbed);
    endPnt   = gp_Pnt(endPnt.X()   - outward.X() * kEmbed,
                      endPnt.Y()   - outward.Y() * kEmbed,
                      endPnt.Z()   - outward.Z() * kEmbed);

    // Build rib solid and fuse onto workpiece.
    const TopoDS_Shape rib = buildRibSolid(startPnt, endPnt,
                                           in.thickness_mm,
                                           in.height_mm + kEmbed,
                                           outward,
                                           in.draft_angle_deg);
    const TopoDS_Shape newShape = pr::fuse(wp.shape(), rib);

    const double rlen = std::hypot(in.end_x_mm - in.start_x_mm,
                                   in.end_y_mm - in.start_y_mm);

    json params = {
        { "entry_face_kind",  "resolved_id" },
        { "entry_face_id",    *entryId },
        { "start_x_mm",       in.start_x_mm },
        { "start_y_mm",       in.start_y_mm },
        { "end_x_mm",         in.end_x_mm },
        { "end_y_mm",         in.end_y_mm },
        { "height_mm",        in.height_mm },
        { "thickness_mm",     in.thickness_mm },
        { "draft_angle_deg",  in.draft_angle_deg },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "length_mm",        rlen },
        { "thickness_mm",     in.thickness_mm },
        { "height_mm",        in.height_mm },
        { "aspect_ratio",     rlen / std::max(1e-6, in.thickness_mm) },
        { "draft_angle_deg",  in.draft_angle_deg },
        { "extrude_dir",      { outward.X(), outward.Y(), outward.Z() } },
    };

    ToolingMeta tooling;
    // Rib is an additive design feature — no machining tool, but classify
    // for downstream cost models.
    tooling.tool_type        = "mold_feature";
    tooling.tool_dia_mm      = 0.0;
    tooling.tool_length_mm   = 0.0;
    tooling.tool_material    = "n/a";
    tooling.flute_count      = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    // Volume ≈ length × thickness × height (rectangular approx).
    tooling.stock_removed_mm3 = -(rlen * in.thickness_mm * in.height_mm);  // negative = added
    tooling.est_cycle_time_s  = 0.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::mold_rib applied: len={} thick={} height={} faces {}→{}",
                  rlen, in.thickness_mm, in.height_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Heuristic: scan all signatures already on the workpiece.  Because ribs are
// thin elongated extrusions and the OCCT fuse merges them seamlessly with
// the base workpiece, robust topology-only recognition is hard.  Slice-1
// strategy: replay the feature history.  If no history is present, attempt
// to find a thin elongated protrusion via face-area heuristics.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Primary strategy: replay stored signatures.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "start_x_mm",      f.params.value("start_x_mm",      0.0) },
            { "start_y_mm",      f.params.value("start_y_mm",      0.0) },
            { "end_x_mm",        f.params.value("end_x_mm",        0.0) },
            { "end_y_mm",        f.params.value("end_y_mm",        0.0) },
            { "height_mm",       f.params.value("height_mm",       0.0) },
            { "thickness_mm",    f.params.value("thickness_mm",    0.0) },
            { "draft_angle_deg", f.params.value("draft_angle_deg", 0.0) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, /*confidence*/ 0.95,
            json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // Fallback geometric heuristic: look for a planar face whose bounding
    // box has aspect ratio > 3 and is "thin" relative to the workpiece bbox.
    const auto wpBb = pr::optimalBbox(wp.shape());
    const double wpSpan = std::max({ wpBb.dx(), wpBb.dy(), wpBb.dz() });
    if (wpSpan < 1e-6) return out;

    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        try {
            const gp_Dir n = wp.faceNormal(i);
            // Looking for top of rib: a small face whose normal is roughly
            // perpendicular to one of the workpiece's larger faces.
            (void)n;
        } catch (...) { continue; }
        const auto faceBb = pr::optimalBbox(wp.face(i));
        const double a = std::max(faceBb.dx(), faceBb.dy());
        const double b = std::min(faceBb.dx(), faceBb.dy());
        if (b < 1e-3) continue;
        const double aspect = a / b;
        if (aspect < 3.0) continue;
        // Must be small relative to workpiece.
        if (a > wpSpan * 0.9) continue;
        if (b > wpSpan * 0.2) continue;
        // Likely a rib top face.
        json rec = {
            { "start_x_mm",     faceBb.xMin },
            { "start_y_mm",     (faceBb.yMin + faceBb.yMax) / 2.0 },
            { "end_x_mm",       faceBb.xMax },
            { "end_y_mm",       (faceBb.yMin + faceBb.yMax) / 2.0 },
            { "height_mm",      faceBb.dz() },
            { "thickness_mm",   b },
            { "draft_angle_deg", 0.0 },
        };
        json matched = { { "top_face_id", i }, { "aspect_ratio", aspect } };
        out.push_back(RecognizedFeature{ kSkillId, rec, 0.45, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::mold_rib
