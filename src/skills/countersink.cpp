// @lat: [[engine/skills#countersink]]

#include "countersink.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Cone.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pln.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <optional>

namespace koocadcam::skill::countersink {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Derived geometry ─────────────────────────────────────────────────────

double computeConeDepth(const Input& in)
{
    if (in.cone_top_dia_mm <= in.pilot_dia_mm) return 0.0;
    if (in.cone_angle_deg <= 0.0 || in.cone_angle_deg >= 180.0) return 0.0;
    const double halfRad = (in.cone_angle_deg * 0.5) * M_PI / 180.0;
    const double t = std::tan(halfRad);
    if (t < 1e-9) return 0.0;
    return (in.cone_top_dia_mm - in.pilot_dia_mm) * 0.5 / t;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    // DFM-002 — minimum pilot diameter (matches drill_hole floor per ISO 235).
    constexpr double kMinPilotDiaMm = 0.8;  // ISO 235:2016 smallest standard HSS drill
    if (in.pilot_dia_mm < kMinPilotDiaMm) {
        r.add("DFM-002", "error",
              "countersink pilot diameter " + std::to_string(in.pilot_dia_mm) +
              " mm < min 0.8 mm (ISO 235:2016 smallest standard HSS drill)");
    }
    if (in.pilot_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "countersink pilot diameter must be > 0");
    }
    if (in.pilot_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "countersink pilot depth must be > 0");
    }
    if (in.cone_top_dia_mm <= in.pilot_dia_mm) {
        r.add("DFM-COUNTERSINK-GEOM", "error",
              "countersink cone top dia " + std::to_string(in.cone_top_dia_mm) +
              " must be > pilot dia " + std::to_string(in.pilot_dia_mm) +
              " (else feature is not a countersink)");
    }
    // DFM-COUNTERSINK-ANGLE — sane envelope around standardised flat-head
    // included angles.  Standards reference:
    //   - ISO 7721 / DIN 974-1 (metric flat-head): 90°
    //   - ASME B18.6.3 (UNC flat-head): 82°
    //   - SAE/AS aerospace (high-strength flat-head): 100°
    //   - Specialty deep countersinks (e.g., ISO 13715 chamfer): up to 120°
    //   - Pipe-end bevels (ASME B16.25): 37.5° lower bound
    // We allow [45°, 120°] as the union "sane" envelope covering all of the
    // above; the strict {82, 90, 100} check lives in countersunk_bolt_seat.
    constexpr double kMinConeAngleDeg = 45.0;   // ISO 7721 / ASME B18.6.3 / SAE flat-head envelope
    constexpr double kMaxConeAngleDeg = 120.0;  // ISO 13715 specialty chamfer upper bound
    if (in.cone_angle_deg < kMinConeAngleDeg || in.cone_angle_deg > kMaxConeAngleDeg) {
        r.add("DFM-COUNTERSINK-ANGLE", "error",
              "countersink cone angle " + std::to_string(in.cone_angle_deg) +
              " deg outside ISO 7721 / ASME B18.6.3 / ISO 13715 envelope [45, 120]");
    }
    const double ratio = (in.pilot_dia_mm > 0.0) ? (in.pilot_depth_mm / in.pilot_dia_mm) : 0.0;
    if (ratio > 8.0) {
        r.add("DFM-PECK", "warning",
              "pilot depth/dia ratio " + std::to_string(ratio) +
              " > 8 — peck drilling recommended (tool length / chip evacuation)");
    }
    // Also ensure cone fits within pilot depth (the cone-pilot junction has
    // to be ABOVE the pilot bottom — else the feature degenerates).
    const double cd = computeConeDepth(in);
    if (cd > 0.0 && cd >= in.pilot_depth_mm) {
        r.add("DFM-COUNTERSINK-GEOM", "error",
              "countersink cone depth " + std::to_string(cd) +
              " ≥ pilot depth " + std::to_string(in.pilot_depth_mm) +
              " — pilot must extend past the cone");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    // 1) DFM gate
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "countersink DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // 2) Resolve entry face datum
    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("countersink: entry_face datum unresolved");

    const double coneDepth = computeConeDepth(in);
    if (coneDepth <= 1e-6)
        throw SkillError("countersink: degenerate cone depth (angle/dia mismatch)");

    // 3) Compute drill geometry (same pattern as drill_hole)
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const double kEntryOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    // True entry point = where the tool axis pierces the resolved entry-face
    // plane (correct for any axis, including tilted).  A prior bug placed the
    // cone/pilot cutters a whole bboxDiag behind the part for a tilted axis, so a
    // tilted countersink cut NOTHING and stamped a garbage entry Z.
    const gp_Pnt entryPlanePoint =
        entryPointOnFacePlane(wp, *entryId, in.position_x_mm, in.position_y_mm,
                              adir, zMin, zMax);
    // The cutter launch point sits kEntryOverhang OUTSIDE the entry surface,
    // back along -adir from the true entry point.
    gp_Pnt toolStartCyl(entryPlanePoint.X() - adir.X() * kEntryOverhang,
                        entryPlanePoint.Y() - adir.Y() * kEntryOverhang,
                        entryPlanePoint.Z() - adir.Z() * kEntryOverhang);

    // 4) Build the two cutters.
    //    Pilot cylinder runs from above-entry (with overhang) down for the
    //    full pilot depth.  Cone runs from EXACTLY the entry plane down for
    //    cone_depth.  We don't overhang the cone because that would shrink
    //    the entry-plane radius below cone_top_dia / 2.
    const gp_Ax2 cylAx (toolStartCyl,    adir);
    const gp_Ax2 coneAx(entryPlanePoint, adir);

    const TopoDS_Shape pilotTool =
        pr::cylinder(cylAx, in.pilot_dia_mm / 2.0, in.pilot_depth_mm + kEntryOverhang);

    // coneFrustum(axis, r1_at_origin, r2_at_+height, height)
    // r1 = cone_top_dia/2 at entry plane (axis origin = entryPlanePoint)
    // r2 = pilot_dia/2    at +coneDepth along adir
    const TopoDS_Shape coneTool =
        pr::coneFrustum(coneAx,
                        in.cone_top_dia_mm / 2.0,
                        in.pilot_dia_mm    / 2.0,
                        coneDepth);

    // 5) Pilot cylinder and cone tool are CONCENTRIC overlapping solids.
    //    OCCT's BRepAlgoAPI_Cut on a compound containing overlapping solids
    //    can miss interior overlap; fuse them into a single connected
    //    cutter first, then perform a single cut.
    const TopoDS_Shape fusedCutter = pr::fuse(pilotTool, coneTool);
    const TopoDS_Shape newShape    = pr::cut(wp.shape(), fusedCutter);

    // 6) Build signature.  Stamp the entry-plane Z (entryPlanePoint is the exact
    //    surface point where the axis crosses the entry face) so downstream CAM
    //    machines from the real surface, not Z=0.
    json params = {
        { "entry_face_kind",  "resolved_id" },
        { "entry_face_id",    *entryId },
        { "position_x_mm",    in.position_x_mm },
        { "position_y_mm",    in.position_y_mm },
        { "position_z_mm",    entryPlanePoint.Z() },
        { "axis_dir",         { adir.X(), adir.Y(), adir.Z() } },
        { "pilot_dia_mm",     in.pilot_dia_mm },
        { "pilot_depth_mm",   in.pilot_depth_mm },
        { "cone_top_dia_mm",  in.cone_top_dia_mm },
        { "cone_angle_deg",   in.cone_angle_deg },
        { "cone_depth_mm",    coneDepth },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     1 },
        { "conical_face_count",         1 },
        { "bottom_planar_face_present", true },
        { "pilot_dia_mm",               in.pilot_dia_mm },
        { "cone_top_dia_mm",            in.cone_top_dia_mm },
        { "cone_angle_deg",             in.cone_angle_deg },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type        = "countersink";
    tooling.tool_dia_mm      = in.cone_top_dia_mm;
    tooling.tool_length_mm   = in.pilot_depth_mm * 1.5 + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    const double pilotR    = in.pilot_dia_mm    / 2.0;
    const double coneTopR  = in.cone_top_dia_mm / 2.0;
    // Cylinder volume + cone frustum volume (above the cylinder portion
    // already counted is the FRUSTUM minus the pilot core within cone_depth).
    const double cylVol   = M_PI * pilotR * pilotR * in.pilot_depth_mm;
    // Frustum volume = (π h / 3) (R² + Rr + r²)
    const double frustVol = (M_PI * coneDepth / 3.0) *
                            (coneTopR * coneTopR + coneTopR * pilotR + pilotR * pilotR);
    // The "extra" carved by the cone beyond the pilot core within cone_depth:
    const double coneExtra = frustVol - M_PI * pilotR * pilotR * coneDepth;
    tooling.stock_removed_mm3 = cylVol + std::max(0.0, coneExtra);
    tooling.est_cycle_time_s  = std::max(1.0, in.pilot_depth_mm / 50.0);
    tooling.extra = {
        { "two_tool_sequence", {
            { { "tool_type", "drill" },       { "tool_dia_mm", in.pilot_dia_mm } },
            { { "tool_type", "countersink" }, { "tool_dia_mm", in.cone_top_dia_mm },
              { "cone_angle_deg", in.cone_angle_deg } },
        } }
    };

    FeatureSignature sig {
        kSkillId,
        params,
        pattern,
        tooling,
    };

    // 7) Build new workpiece, register feature
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::countersink applied: pilot {}x{}, cone {}°@{}, faces {}→{}",
                  in.pilot_dia_mm, in.pilot_depth_mm,
                  in.cone_angle_deg, in.cone_top_dia_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Fast-path: if the workpiece still carries a countersink FeatureSignature,
// replay its authored params at confidence 1.0 (metadata-replay).
//
// Geometric fallback (FOREIGN STEP, no feature history) — all dims MEASURED:
//   1. Collect every CONCAVE cylindrical face (pilot bore) — convex rods/pins
//      and freeform slivers are rejected by an orientation-aware concavity +
//      U-span gate (drill_hole::isDrillableHoleWall idiom).
//   2. Collect every CONCAVE conical face (countersink seat) — a convex
//      pointed boss/spike is rejected by the same concavity gate.
//   3. For each (cyl, cone) pair sharing the same axis (parallel direction +
//      same infinite-line, size-adaptive position tol), check that the cone's
//      SMALLER bounding radius matches the cylinder radius (cone meets pilot).
//   4. Recover parameters:
//      - pilot_dia    = 2 × cyl.Radius()
//      - cone_top_dia = 2 × (cone's larger bounding-rim radius)
//      - cone_angle   = 2 × |cone.SemiAngle()|     (INCLUDED angle)
//      - axis_dir     = cylinder axis direction (any orientation)
//      - pilot_depth  = entry → cylinder-bottom span along the axis
//      - position     = entry rim centre (full 3-D: position_x/y/z_mm)
//
// Tolerances are size-adaptive (max(abs_floor, rel·radius)) and rims are
// grouped from full OR partial arcs, so STEP-split faces and big/small bores
// all recover correctly.

namespace {

struct CylInfo
{
    int     faceIdx = -1;
    double  radius  = 0.0;
    gp_Ax1  axis;
    gp_Pnt  topCenter;
    gp_Pnt  botCenter;
};

struct ConeInfo
{
    int     faceIdx       = -1;
    double  semiAngleRad  = 0.0;        // absolute value
    gp_Ax1  axis;
    double  rLarge        = 0.0;
    double  rSmall        = 0.0;
    gp_Pnt  largeCenter;
    gp_Pnt  smallCenter;
};

// ── Concavity gates ───────────────────────────────────────────────────────
//
// A countersink seat is a CONCAVE cone and its pilot is a CONCAVE cylinder:
// the solid sits OUTSIDE the surface, so the orientation-aware outward normal
// (pointing into the machined void) has a component pointing TOWARD the axis
// (negative radial dot).  A pointed boss / turned rod / spike is CONVEX — its
// outward normal points AWAY from the axis (positive radial dot) — and must be
// rejected so we never report a phantom countersink on a protruding cone.
// Modelled on drill_hole::isDrillableHoleWall().  Also guards against freeform
// tessellation slivers that barely wrap the axis (U-span guard).
bool surfaceOutwardNormalPointsToAxis(const TopoDS_Face& f,
                                      const gp_Ax1& axis,
                                      double minUSpan)
{
    BRepAdaptor_Surface s(f);

    // (a) Sliver guard: the wall must wrap a meaningful arc of the axis.  A
    //     real seat/bore is a full surface (2π) or a few face-splits; a
    //     freeform tessellation patch barely wraps the axis at all.
    const double uSpan = s.LastUParameter() - s.FirstUParameter();
    if (uSpan < minUSpan) return false;

    // (b) Concavity: sample the orientation-aware normal at the UV midpoint
    //     and compare against the outward-radial direction from the axis.
    const double u = 0.5 * (s.FirstUParameter() + s.LastUParameter());
    const double v = 0.5 * (s.FirstVParameter() + s.LastVParameter());
    gp_Pnt P; gp_Vec du, dv;
    s.D1(u, v, P, du, dv);
    gp_Vec n = du.Crossed(dv);
    if (n.Magnitude() < 1e-9) return false;
    n.Normalize();
    if (f.Orientation() == TopAbs_REVERSED) n.Reverse();

    const gp_Pnt aLoc = axis.Location();
    const gp_Vec aDir(axis.Direction());
    gp_Vec ap(aLoc, P);
    const gp_Vec radial = ap - aDir * ap.Dot(aDir);   // outward from the axis
    if (radial.Magnitude() < 1e-9) return false;
    return n.Dot(radial) < 0.0;                        // inward normal ⇒ concave
}

// A rim of a cone/cylinder, recovered from one OR MORE circular edges that
// share the same radius and axial level.  On foreign STEP a face is often
// split into halves and each rim arrives as several partial arcs; we group
// them so the recovered rim diameter/centre is correct rather than counting a
// partial arc as a distinct (phantom) full circle.
struct Rim
{
    double radius   = 0.0;
    gp_Pnt center;          // circle centre (on the axis)
    double axialProj = 0.0; // signed projection along the feature axis
};

// Gather rims of a face by grouping its circular edges (full or partial) along
// the axis.  `radius` is taken from the surface (NOT a per-arc guess), so even
// a 90° trim arc resolves to the true rim radius via its own Circle().Radius()
// after the axis/radius filter.
std::vector<Rim> gatherRims(const TopoDS_Face& f, const gp_Ax1& axis,
                            double radiusHint, double radTol)
{
    const gp_Dir adir = axis.Direction();
    auto proj = [&](const gp_Pnt& p) {
        return (p.X() - axis.Location().X()) * adir.X() +
               (p.Y() - axis.Location().Y()) * adir.Y() +
               (p.Z() - axis.Location().Z()) * adir.Z();
    };

    std::vector<Rim> rims;
    for (TopExp_Explorer exp(f, TopAbs_EDGE); exp.More(); exp.Next()) {
        const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
        BRepAdaptor_Curve crv(e);
        if (crv.GetType() != GeomAbs_Circle) continue;
        const gp_Circ c = crv.Circle();
        // Must ring the feature: circle normal parallel to the axis.
        if (std::abs(std::abs(c.Axis().Direction().Dot(adir)) - 1.0) > 1e-3)
            continue;
        // For a cylinder all rims share the surface radius; for a cone the rim
        // radius varies, so only filter against the hint when one is given.
        if (radiusHint > 0.0 && std::abs(c.Radius() - radiusHint) > radTol)
            continue;

        const double pr = proj(c.Location());
        // Merge with an existing rim at the same axial level + radius (collapses
        // partial-arc duplicates from STEP face-splitting).  Arcs of the SAME
        // rim are coincident to rebuild precision, so a tight ABSOLUTE level tol
        // is correct (and avoids over-merging on parts placed far from origin);
        // the radius tol is size-adaptive.
        constexpr double kLevelTol = 1e-4;   // mm — same-rim arcs are coincident
        bool merged = false;
        for (Rim& r : rims) {
            const double rTol = std::max(1e-4, 1e-3 * std::max(r.radius, c.Radius()));
            if (std::abs(r.axialProj - pr) < kLevelTol &&
                std::abs(r.radius   - c.Radius()) < rTol) {
                merged = true;
                break;
            }
        }
        if (!merged) rims.push_back(Rim{ c.Radius(), c.Location(), pr });
    }
    return rims;
}

// Collect cylinders along with their bounding-circle endpoints.  Only CONCAVE,
// axis-wrapping cylinders (real pilot bores) survive — rods/bosses rejected.
std::vector<CylInfo> collectCylinders(const Workpiece& wp)
{
    std::vector<CylInfo> out;
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        const TopoDS_Face& f = wp.face(fIdx);
        BRepAdaptor_Surface surf(f);
        const gp_Cylinder cyl = surf.Cylinder();

        CylInfo info;
        info.faceIdx = fIdx;
        info.radius  = cyl.Radius();
        info.axis    = cyl.Axis();
        if (info.radius <= 0.0) continue;

        // Concavity gate: a pilot bore wraps ≥ ~81° and is concave (solid
        // outside).  Rejects turned rods / pins that are coaxial with a cone.
        if (!surfaceOutwardNormalPointsToAxis(f, info.axis, 0.45 * M_PI))
            continue;

        // Size-adaptive radius tolerance (drill_hole idiom): tolerant of STEP
        // rebuild noise on big bores, tight on sub-mm pilots.
        const double radTol = std::max(1e-4, 1e-3 * info.radius);
        const auto rims = gatherRims(f, info.axis, info.radius, radTol);
        if (rims.size() < 2) continue;

        const auto minIt = std::min_element(rims.begin(), rims.end(),
            [](const Rim& a, const Rim& b){ return a.axialProj < b.axialProj; });
        const auto maxIt = std::max_element(rims.begin(), rims.end(),
            [](const Rim& a, const Rim& b){ return a.axialProj < b.axialProj; });
        info.topCenter = minIt->center;   // shallow (lower projection along axis)
        info.botCenter = maxIt->center;   // deeper
        out.push_back(info);
    }
    return out;
}

// Collect conical faces with bounding-circle radii + endpoints.  Only CONCAVE,
// axis-wrapping cones (real countersink seats) survive — pointed bosses
// (convex cones / spikes) and freeform slivers are rejected.
std::vector<ConeInfo> collectCones(const Workpiece& wp)
{
    std::vector<ConeInfo> out;
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        const TopoDS_Face& f = wp.face(fIdx);
        BRepAdaptor_Surface surf(f);
        if (surf.GetType() != GeomAbs_Cone) continue;
        const gp_Cone cone = surf.Cone();

        ConeInfo info;
        info.faceIdx      = fIdx;
        info.semiAngleRad = std::abs(cone.SemiAngle());     // MEASURED half-angle
        info.axis         = cone.Axis();

        // Concavity gate: a countersink seat is a CONCAVE cone (solid outside).
        // Rejects a pointed boss / turned spike (a convex protruding cone),
        // mirroring drill_hole's RejectsConvexBossAsHole guarantee.
        if (!surfaceOutwardNormalPointsToAxis(f, info.axis, 0.45 * M_PI))
            continue;

        // Gather the cone's bounding rims (large entry, small junction).  Cone
        // rim radius VARIES with axial level, so we pass no radius hint and let
        // gatherRims group partial arcs by axial level — STEP-split-safe.
        const auto rims = gatherRims(f, info.axis, /*radiusHint*/ 0.0, /*radTol*/ 0.0);
        if (rims.size() < 2) continue;

        // Largest / smallest by radius.
        const auto minR = std::min_element(rims.begin(), rims.end(),
            [](const Rim& a, const Rim& b){ return a.radius < b.radius; });
        const auto maxR = std::max_element(rims.begin(), rims.end(),
            [](const Rim& a, const Rim& b){ return a.radius < b.radius; });
        // Frustum guard: the two rims must differ in radius.  Tolerance scales
        // with the rim size so it works from sub-mm seats to large bores.
        const double rimSep = std::max(1e-4, 1e-3 * maxR->radius);
        if (std::abs(maxR->radius - minR->radius) < rimSep) continue;  // not a frustum

        info.rSmall      = minR->radius;
        info.smallCenter = minR->center;
        info.rLarge      = maxR->radius;
        info.largeCenter = maxR->center;
        out.push_back(info);
    }
    return out;
}

// Two axes share an infinite-line (parallel + same support line)?  The caller
// passes a size-adaptive posTolMm so coaxiality survives STEP rebuild jitter on
// large features while staying tight on small ones.
bool sameAxis(const gp_Ax1& a, const gp_Ax1& b,
              double angTolDeg = 0.5, double posTolMm = 1e-3)
{
    const gp_Dir da = a.Direction();
    const gp_Dir db = b.Direction();
    const double dot = std::abs(da.X() * db.X() + da.Y() * db.Y() + da.Z() * db.Z());
    if (dot < std::cos(angTolDeg * M_PI / 180.0)) return false;
    const gp_Pnt& pa = a.Location();
    const gp_Pnt& pb = b.Location();
    gp_Vec v(pa, pb);
    gp_Vec axV(da.X(), da.Y(), da.Z());
    gp_Vec perp = v - axV * v.Dot(axV);
    return perp.Magnitude() < posTolMm;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // ── Metadata-replay fast-path ─────────────────────────────────────────
    // If this workpiece still carries the countersink feature in its history
    // (same instance that apply() produced, or a chain mirrored by the
    // Executor), replay the authored params verbatim at confidence 1.0.  The
    // GEOMETRIC fallback below is what hardens recognition on FOREIGN STEP
    // (re-imported shapes with no feature history).
    for (const auto& feat : wp.features()) {
        if (feat.skill_id != kSkillId) continue;
        const json& p = feat.params;
        json recovered = {
            { "position_x_mm",   p.value("position_x_mm",   0.0) },
            { "position_y_mm",   p.value("position_y_mm",   0.0) },
            // Entry-plane Z: apply() stamps it (entryPointOnFacePlane); omitting
            // it here made a HISTORY-BEARING countersunk ring recover with every
            // member at z=0 — the ring compound's entry plane and pilot-mouth
            // hole_centers then sat off the real part, breaking z-aware drill
            // subsumption.  (The counterbore atom has no replay fast-path, so
            // this was a countersink-only gap.)
            { "position_z_mm",   p.value("position_z_mm",   0.0) },
            { "axis_dir",        p.value("axis_dir", json::array({ 0.0, 0.0, -1.0 })) },
            { "pilot_dia_mm",    p.value("pilot_dia_mm",    0.0) },
            { "pilot_depth_mm",  p.value("pilot_depth_mm",  0.0) },
            { "cone_top_dia_mm", p.value("cone_top_dia_mm", 0.0) },
            { "cone_angle_deg",  p.value("cone_angle_deg",  0.0) },
            { "cone_depth_mm",   p.value("cone_depth_mm",   0.0) },
        };
        json matched = { { "source", "metadata_replay" } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 1.0, matched });
    }
    if (!out.empty()) return out;

    // ── Geometric fallback ────────────────────────────────────────────────
    const auto cyls  = collectCylinders(wp);
    const auto cones = collectCones(wp);
    if (cyls.empty() || cones.empty()) return out;

    std::vector<bool> consumedCyl(cyls.size(),   false);
    std::vector<bool> consumedCone(cones.size(), false);

    for (size_t ci = 0; ci < cones.size(); ++ci) {
        if (consumedCone[ci]) continue;
        const ConeInfo& cone = cones[ci];

        for (size_t yi = 0; yi < cyls.size(); ++yi) {
            if (consumedCyl[yi]) continue;
            const CylInfo& cyl = cyls[yi];

            // Size-adaptive coaxiality tolerance: scales with the rim radius so
            // big bores survive STEP rebuild jitter without loosening sub-mm fits.
            const double axisPosTol = std::max(1e-3, 1e-3 * std::max(cone.rLarge, cyl.radius));
            if (!sameAxis(cone.axis, cyl.axis, 0.5, axisPosTol)) continue;
            // The cone's SMALL end must match the pilot cylinder radius.
            // Size-adaptive: tight on sub-mm pilots, tolerant of STEP rebuild
            // noise on large bores.
            const double dtol = std::max(1e-3, 1e-2 * cyl.radius);
            if (std::abs(cone.rSmall - cyl.radius) > dtol) continue;

            // Drilling axis direction = from cone's large circle toward
            // cylinder's bottom (deeper end).
            const gp_Dir adir = cyl.axis.Direction();
            auto projOn = [&](const gp_Ax1& ax, const gp_Pnt& p) {
                const gp_Dir d = ax.Direction();
                return (p.X() - ax.Location().X()) * d.X() +
                       (p.Y() - ax.Location().Y()) * d.Y() +
                       (p.Z() - ax.Location().Z()) * d.Z();
            };

            const double coneLargeProj = projOn(cyl.axis, cone.largeCenter);
            const double coneSmallProj = projOn(cyl.axis, cone.smallCenter);
            const double cylTopProj    = projOn(cyl.axis, cyl.topCenter);
            const double cylBotProj    = projOn(cyl.axis, cyl.botCenter);

            // Entry should be the cone's LARGE circle, on the shallow side.
            const double entryProj = std::min({
                coneLargeProj, coneSmallProj, cylTopProj, cylBotProj
            });

            // Position/junction tolerances scale with the feature size (rim
            // radius), so they hold on both watch-scale and big-bore parts.
            const double sizeRef   = std::max(cone.rLarge, cyl.radius);
            const double entryTol  = std::max(1e-2, 1e-2 * sizeRef);
            const double junctTol  = std::max(1e-1, 2e-2 * sizeRef);

            if (std::abs(coneLargeProj - entryProj) > entryTol) {
                // The cone's LARGE end is NOT on the entry side → not a
                // countersink in our orientation (this also rejects a cone
                // whose wide end points INTO the material).
                continue;
            }

            // The cone's small end should meet the cylinder's top.
            const double junctionGap =
                std::abs(coneSmallProj - cylTopProj);
            if (junctionGap > junctTol) continue;  // cone doesn't meet cylinder

            // Recover parameters — all MEASURED off the OCCT surfaces:
            //   pilot_dia    = 2 × cyl.Radius()
            //   cone_top_dia = 2 × cone's larger bounding-rim radius
            //   cone_angle   = 2 × |cone.SemiAngle()|   (INCLUDED angle)
            //   pilot_depth  = entry→cylinder-bottom span along the axis
            //   position     = entry rim centre (full 3-D, any axis)
            const double coneDepth = std::abs(coneLargeProj - coneSmallProj);
            const double pilotDepth = std::abs(cylBotProj - entryProj);
            const double coneAngleDeg = 2.0 * cone.semiAngleRad * 180.0 / M_PI;

            // Confidence heuristic (size-adaptive junction penalty).
            double conf = 0.92;
            if (junctionGap > std::max(1e-3, 1e-3 * sizeRef)) conf -= 0.1;

            json recovered = {
                { "position_x_mm",    cone.largeCenter.X() },
                { "position_y_mm",    cone.largeCenter.Y() },
                { "position_z_mm",    cone.largeCenter.Z() },   // full 3-D entry (any axis)
                { "axis_dir",         { adir.X(), adir.Y(), adir.Z() } },
                { "pilot_dia_mm",     2.0 * cyl.radius },
                { "pilot_depth_mm",   pilotDepth },
                { "cone_top_dia_mm",  2.0 * cone.rLarge },
                { "cone_angle_deg",   coneAngleDeg },
                { "cone_depth_mm",    coneDepth },
            };
            // Claim the planar ENTRY face (the plane holding the cone's large
            // rim) in matched_geometry — mirroring counterbore.  Downstream the
            // ring compound harvests it as the Executor's entry_face_id datum
            // anchor; without it a recovered countersunk ring always replays on
            // the by-normal fallback face, which can resolve the WRONG deck on
            // a multi-level part.
            int    entryFaceId = -1;
            double entryArea   = 0.0;
            for (int fi = 0; fi < wp.faceCount(); ++fi) {
                if (!wp.isFacePlanar(fi)) continue;
                gp_Dir n;
                try { n = wp.faceNormal(fi); } catch (...) { continue; }
                const double nDot = std::abs(n.X() * adir.X() +
                                             n.Y() * adir.Y() +
                                             n.Z() * adir.Z());
                if (nDot < 0.99) continue;            // not ⊥ to the bore axis
                const gp_Pnt c = wp.faceCenter(fi);
                const gp_Vec rel(cone.largeCenter, c);
                const double along = rel.X() * adir.X() +
                                     rel.Y() * adir.Y() +
                                     rel.Z() * adir.Z();
                if (std::abs(along) < 0.05) {
                    // Entry plane: the largest coplanar face at the rim height.
                    const double area = wp.faceArea(fi);
                    if (area > entryArea) { entryArea = area; entryFaceId = fi; }
                }
            }

            json matched = {
                { "cone_face_id",     cone.faceIdx },
                { "cyl_face_id",      cyl.faceIdx },
                { "entry_center", { cone.largeCenter.X(), cone.largeCenter.Y(),
                                    cone.largeCenter.Z() } },
            };
            if (entryFaceId >= 0) matched["entry_face_id"] = entryFaceId;
            out.push_back(RecognizedFeature{
                kSkillId, recovered, conf, matched
            });
            consumedCone[ci] = true;
            consumedCyl[yi]  = true;
            break;
        }
    }
    return out;
}

}  // namespace koocadcam::skill::countersink
