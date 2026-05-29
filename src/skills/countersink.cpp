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

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

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
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    const double kEntryOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    // Default toolStart (general-axis case): set just outside the workpiece
    // along -adir.
    gp_Pnt toolStartCyl(
        in.position_x_mm - adir.X() * (bboxDiag + kEntryOverhang),
        in.position_y_mm - adir.Y() * (bboxDiag + kEntryOverhang),
        (zMin + zMax) / 2.0 - adir.Z() * (bboxDiag + kEntryOverhang));
    gp_Pnt entryPlanePoint(toolStartCyl);

    // Common-case shortcut: drilling along ±Z, planar entry face.
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        const double entryZ = (adir.Z() < 0) ? zMax : zMin;
        toolStartCyl     = gp_Pnt(in.position_x_mm, in.position_y_mm,
                                  entryZ - adir.Z() * kEntryOverhang);
        entryPlanePoint  = gp_Pnt(in.position_x_mm, in.position_y_mm, entryZ);
    }

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

    // 6) Build signature
    json params = {
        { "entry_face_kind",  "resolved_id" },
        { "entry_face_id",    *entryId },
        { "position_x_mm",    in.position_x_mm },
        { "position_y_mm",    in.position_y_mm },
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
    wpNew->addFeature(sig);

    spdlog::debug("skill::countersink applied: pilot {}x{}, cone {}°@{}, faces {}→{}",
                  in.pilot_dia_mm, in.pilot_depth_mm,
                  in.cone_angle_deg, in.cone_top_dia_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Pattern matching strategy:
//   1. Collect every cylindrical face (with axis, radius, endpoints).
//   2. Collect every conical face (with axis, semi-angle, radii at the two
//      bounding circles).
//   3. For each (cyl, cone) pair sharing the same axis (parallel direction
//      + same infinite-line), check that the cone's SMALLER bounding
//      radius matches the cylinder radius (cone tip meets cylinder top).
//   4. Recover parameters:
//      - pilot_dia    = 2 × cyl.Radius
//      - cone_top_dia = 2 × (cone's larger bounding radius)
//      - cone_angle   = 2 × cone.SemiAngle (absolute value)
//      - axis_dir     = from cone entry → cylinder bottom
//      - pilot_depth  = distance from entry circle to cylinder's bottom
//      - position     = entry circle's center XY

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

// Collect cylinders along with their bounding-circle endpoints.
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

        std::vector<gp_Pnt> centers;
        for (TopExp_Explorer exp(f, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            if (std::abs(std::abs(c.Axis().Direction().Dot(info.axis.Direction())) - 1.0) > 1e-3)
                continue;
            if (std::abs(c.Radius() - info.radius) > 1e-3) continue;
            centers.push_back(c.Location());
        }
        if (centers.size() < 2) continue;

        const gp_Dir adir = info.axis.Direction();
        auto proj = [&](const gp_Pnt& p) {
            return (p.X() - info.axis.Location().X()) * adir.X() +
                   (p.Y() - info.axis.Location().Y()) * adir.Y() +
                   (p.Z() - info.axis.Location().Z()) * adir.Z();
        };
        auto cmp = [&](const gp_Pnt& a, const gp_Pnt& b) {
            return proj(a) < proj(b);
        };
        const auto minIt = std::min_element(centers.begin(), centers.end(), cmp);
        const auto maxIt = std::max_element(centers.begin(), centers.end(), cmp);
        info.topCenter = *minIt;   // shallow (lower projection along axis direction)
        info.botCenter = *maxIt;   // deeper
        out.push_back(info);
    }
    return out;
}

// Collect conical faces with bounding-circle radii + endpoints.
std::vector<ConeInfo> collectCones(const Workpiece& wp)
{
    std::vector<ConeInfo> out;
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        BRepAdaptor_Surface surf(wp.face(fIdx));
        if (surf.GetType() != GeomAbs_Cone) continue;
        const gp_Cone cone = surf.Cone();

        ConeInfo info;
        info.faceIdx      = fIdx;
        info.semiAngleRad = std::abs(cone.SemiAngle());
        info.axis         = cone.Axis();

        // Collect circular edges with axis-aligned normal.
        std::vector<std::pair<double, gp_Pnt>> circles;  // (radius, center)
        for (TopExp_Explorer exp(wp.face(fIdx), TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            if (std::abs(std::abs(c.Axis().Direction().Dot(info.axis.Direction())) - 1.0) > 1e-3)
                continue;
            circles.emplace_back(c.Radius(), c.Location());
        }
        if (circles.size() < 2) continue;

        // Largest / smallest by radius.
        const auto minR = std::min_element(circles.begin(), circles.end(),
            [](const auto& a, const auto& b){ return a.first < b.first; });
        const auto maxR = std::max_element(circles.begin(), circles.end(),
            [](const auto& a, const auto& b){ return a.first < b.first; });
        if (std::abs(maxR->first - minR->first) < 1e-4) continue;  // not a frustum

        info.rSmall      = minR->first;
        info.smallCenter = minR->second;
        info.rLarge      = maxR->first;
        info.largeCenter = maxR->second;
        out.push_back(info);
    }
    return out;
}

// Two axes share an infinite-line (parallel + same support line)?
bool sameAxis(const gp_Ax1& a, const gp_Ax1& b, double angTolDeg = 0.5, double posTolMm = 1e-3)
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

            if (!sameAxis(cone.axis, cyl.axis)) continue;
            // The cone's SMALL end must match the cylinder radius
            if (std::abs(cone.rSmall - cyl.radius) > 1e-2) continue;

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

            if (std::abs(coneLargeProj - entryProj) > 1e-2) {
                // The cone's LARGE end is NOT on the entry side → not a
                // countersink in our orientation.
                continue;
            }

            // The cone's small end should meet the cylinder's top.
            const double junctionGap =
                std::abs(coneSmallProj - cylTopProj);
            if (junctionGap > 1e-1) continue;  // cone doesn't meet cylinder

            // Recover parameters.
            const double coneDepth = std::abs(coneLargeProj - coneSmallProj);
            // pilot depth = total distance from entry to cylinder bottom.
            const double pilotDepth = std::abs(cylBotProj - entryProj);
            const double coneAngleDeg = 2.0 * cone.semiAngleRad * 180.0 / M_PI;

            // Confidence heuristic
            double conf = 0.92;
            if (junctionGap > 1e-3) conf -= 0.1;

            json recovered = {
                { "position_x_mm",    cone.largeCenter.X() },
                { "position_y_mm",    cone.largeCenter.Y() },
                { "axis_dir",         { adir.X(), adir.Y(), adir.Z() } },
                { "pilot_dia_mm",     2.0 * cyl.radius },
                { "pilot_depth_mm",   pilotDepth },
                { "cone_top_dia_mm",  2.0 * cone.rLarge },
                { "cone_angle_deg",   coneAngleDeg },
                { "cone_depth_mm",    coneDepth },
            };
            json matched = {
                { "cone_face_id",     cone.faceIdx },
                { "cyl_face_id",      cyl.faceIdx },
                { "entry_center", { cone.largeCenter.X(), cone.largeCenter.Y(),
                                    cone.largeCenter.Z() } },
            };
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
