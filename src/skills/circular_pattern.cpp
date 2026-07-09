// @lat: [[engine/skills#circular_pattern]]

#include "circular_pattern.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <GeomAbs_CurveType.hxx>
#include <TopAbs_State.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
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
#include <limits>
#include <numeric>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::circular_pattern {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    (void)wp;
    DFMReport r;
    if (in.count < 1) {
        r.add("DFM-INPUT", "error", "circular_pattern: count must be >= 1");
    }
    if (in.hole_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "circular_pattern: hole_dia_mm must be > 0");
    }
    if (in.hole_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "circular_pattern: hole_depth_mm must be > 0");
    }
    if (in.radial_offset_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "circular_pattern: radial_offset_mm must be > 0");
    }
    if (in.total_angle_deg <= 0.0 || in.total_angle_deg > 360.0) {
        r.add("DFM-INPUT", "error",
              "circular_pattern: total_angle_deg " +
              std::to_string(in.total_angle_deg) + " out of (0, 360]");
    }
    if (in.hole_dia_mm > 0.0 && in.hole_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "circular_pattern: hole_dia_mm " + std::to_string(in.hole_dia_mm) +
              " < 0.8 mm (ISO 235:2016)");
    }
    const double an2 = in.axis_dir_xyz[0] * in.axis_dir_xyz[0] +
                       in.axis_dir_xyz[1] * in.axis_dir_xyz[1] +
                       in.axis_dir_xyz[2] * in.axis_dir_xyz[2];
    if (an2 < 1e-18) {
        r.add("DFM-INPUT", "error",
              "circular_pattern: axis_dir_xyz is zero-length");
    }
    // DFM-PATT-2 — chord_pitch = 2*R*sin(step/2) must exceed hole_dia.
    if (in.count > 1 && in.radial_offset_mm > 0.0 && in.total_angle_deg > 0.0) {
        const double stepDeg = in.total_angle_deg / in.count;
        const double stepRad = stepDeg * M_PI / 180.0;
        const double chord   = 2.0 * in.radial_offset_mm * std::sin(stepRad / 2.0);
        if (chord <= in.hole_dia_mm) {
            r.add("DFM-PATT-2", "error",
                  "circular_pattern: chord_pitch " + std::to_string(chord) +
                  " <= hole_dia_mm " + std::to_string(in.hole_dia_mm) +
                  " (instances would overlap)");
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "circular_pattern DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double an = std::sqrt(
        in.axis_dir_xyz[0] * in.axis_dir_xyz[0] +
        in.axis_dir_xyz[1] * in.axis_dir_xyz[1] +
        in.axis_dir_xyz[2] * in.axis_dir_xyz[2]);
    const gp_Dir axisDir(in.axis_dir_xyz[0] / an,
                         in.axis_dir_xyz[1] / an,
                         in.axis_dir_xyz[2] / an);
    const gp_Pnt axisOrg(in.axis_origin_xyz[0],
                         in.axis_origin_xyz[1],
                         in.axis_origin_xyz[2]);
    const gp_Ax1 rotAxis(axisOrg, axisDir);

    // Pick an "X̂_⊥" perpendicular to axisDir.
    gp_Dir refX(1.0, 0.0, 0.0);
    if (std::abs(gp_Vec(axisDir).Dot(gp_Vec(refX))) > 0.95)
        refX = gp_Dir(0.0, 1.0, 0.0);
    gp_Vec vx(refX);
    vx = vx - gp_Vec(axisDir) * vx.Dot(gp_Vec(axisDir));
    vx.Normalize();
    const gp_Dir radialDir(vx);

    // Base seed cylinder at theta=0, on the +radialDir at radial_offset_mm.
    constexpr double kOverhang = 0.1;
    const double toolLen = in.hole_depth_mm + 2.0 * kOverhang;
    const double r       = in.hole_dia_mm * 0.5;

    // Start the cylinder slightly +axisDir-side of the axis origin so the
    // cut starts cleanly.  Cut direction = -axisDir (into the body).
    const gp_Pnt seedStart(
        axisOrg.X() + radialDir.X() * in.radial_offset_mm + axisDir.X() * kOverhang,
        axisOrg.Y() + radialDir.Y() * in.radial_offset_mm + axisDir.Y() * kOverhang,
        axisOrg.Z() + radialDir.Z() * in.radial_offset_mm + axisDir.Z() * kOverhang);

    const gp_Dir cutDir(-axisDir.X(), -axisDir.Y(), -axisDir.Z());
    const gp_Ax2 seedAx(seedStart, cutDir);
    const TopoDS_Shape seedCyl = pr::cylinder(seedAx, r, toolLen);

    std::vector<TopoDS_Shape> cutters;
    cutters.reserve(static_cast<size_t>(in.count));
    json holeCenters = json::array();   // 3-D entry point of every hole (for CAM)
    const double stepDeg = in.total_angle_deg / in.count;
    // The entry-plane radial direction at theta=0 (radialDir), rotated per hole.
    const gp_Vec vy = gp_Vec(axisDir).Crossed(gp_Vec(radialDir));  // second in-plane axis
    for (int i = 0; i < in.count; ++i) {
        const double angleRad = i * stepDeg * M_PI / 180.0;
        gp_Trsf rot;
        rot.SetRotation(rotAxis, angleRad);
        BRepBuilderAPI_Transform xform(seedCyl, rot, true);
        if (!xform.IsDone())
            throw SkillError("circular_pattern: transform failed");
        cutters.push_back(xform.Shape());
        // Entry centre = axisOrg + (radialDir*cos + vy*sin) * radial_offset.
        const double cph = std::cos(angleRad), sph = std::sin(angleRad);
        holeCenters.push_back({
            axisOrg.X() + (radialDir.X() * cph + vy.X() * sph) * in.radial_offset_mm,
            axisOrg.Y() + (radialDir.Y() * cph + vy.Y() * sph) * in.radial_offset_mm,
            axisOrg.Z() + (radialDir.Z() * cph + vy.Z() * sph) * in.radial_offset_mm });
    }

    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), cutters);

    const double perVol = M_PI * r * r * in.hole_depth_mm;
    const double totalVol = perVol * in.count;

    json params = {
        { "axis_origin_xyz",  in.axis_origin_xyz },
        { "axis_dir_xyz",     { axisDir.X(), axisDir.Y(), axisDir.Z() } },
        { "hole_dia_mm",      in.hole_dia_mm },
        { "hole_depth_mm",    in.hole_depth_mm },
        { "radial_offset_mm", in.radial_offset_mm },
        { "count",            in.count },
        { "total_angle_deg",  in.total_angle_deg },
        { "hole_centers",     holeCenters },   // 3-D entry points (CAM)
    };
    json pattern = {
        { "kind",           kSkillId },
        { "is_pattern",     true },
        { "instance_count", in.count },
        { "derived_params", {
            { "hole_dia_mm",       in.hole_dia_mm },
            { "hole_depth_mm",     in.hole_depth_mm },
            { "radial_offset_mm",  in.radial_offset_mm },
            { "total_angle_deg",   in.total_angle_deg },
            { "step_deg",          stepDeg },
            { "per_inst_vol_mm3",  perVol },
            { "total_vol_mm3",     totalVol },
        } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill";
    tooling.tool_dia_mm       = in.hole_dia_mm;
    tooling.tool_length_mm    = in.hole_depth_mm * 1.5 + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = totalVol;
    tooling.est_cycle_time_s  = std::max(1.0, in.hole_depth_mm / 50.0) * in.count;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::circular_pattern applied: count={} R={} dia={} vol={}",
                  in.count, in.radial_offset_mm, in.hole_dia_mm, totalVol);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Heuristic: ≥3 cylindrical faces sharing radius (±1%) whose centers lie on
// a common circle (variance of distance-from-mean-center < 5%) → candidate.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 0.95;
        r.matched_geometry = f.pattern;
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Each candidate hole cylinder carries its 3-D centre, radius, AXIS direction,
    // and the axial extent (depth) + entry point ALONG that axis.  The pattern's
    // plane is perpendicular to the shared hole axis — which is NOT necessarily
    // world +Z (a side bolt-circle / radial pusher ring on a watch case bores along
    // +/-X or +/-Y).  So we keep the full axis per hole and do all ring geometry in
    // the pattern's OWN frame, generalising the former Z-only recogniser.
    // Each hole cylinder's ENTRY (mouth) is the end that lies on an OUTER surface —
    // the point just beyond it is OUTSIDE the solid.  OCCT reports a cylinder's axis
    // direction with an ARBITRARY sign, so we cannot trust it to point outward; we
    // classify a probe point just past each end against the solid to find the mouth
    // and orient the axis outward (bottom → entry), matching apply()'s convention
    // (it bores from axis_origin along -axis_dir into the body).  A bbox-centroid
    // distance heuristic was WRONG for through holes on non-parallel faces, recessed
    // inner-floor mouths, and non-convex parts — solid classification is robust.
    BRepClass3d_SolidClassifier classifier(wp.shape());
    const double kProbe = 0.2;   // probe distance beyond an end face (mm)
    auto endIsOutside = [&](const gp_Pnt& end, const gp_Dir& outwardGuess) {
        const gp_Pnt probe(end.X() + outwardGuess.X() * kProbe,
                           end.Y() + outwardGuess.Y() * kProbe,
                           end.Z() + outwardGuess.Z() * kProbe);
        classifier.Perform(probe, 1e-6);
        return classifier.State() == TopAbs_OUT;
    };

    struct C {
        gp_Pnt loc; gp_Dir axis; double r, depth; gp_Pnt entry; int faceId;
    };
    std::vector<C> all;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Cylinder gc = s.Cylinder();
            gp_Dir adir = gc.Axis().Direction();
            // CANONICALISE the axis sign before measuring extents: OCCT assigns a
            // cylinder's parametrisation direction ARBITRARILY per face, and on
            // foreign STEP two holes of one ring can carry opposite signs.  Flip so
            // the largest-|component| is positive — then aMin/aMax (and the
            // through-hole fallback below) pick the SAME geometric side for every
            // hole of a ring, whatever each face's native parametrisation was.
            {
                const double axc[3] = { adir.X(), adir.Y(), adir.Z() };
                int big = 0;
                if (std::abs(axc[1]) > std::abs(axc[big])) big = 1;
                if (std::abs(axc[2]) > std::abs(axc[big])) big = 2;
                if (axc[big] < 0.0) adir = gp_Dir(-adir.X(), -adir.Y(), -adir.Z());
            }
            const gp_Pnt aorg = gc.Axis().Location();
            // Recover the hole DEPTH and both END points from the cylinder face's
            // extent ALONG ITS OWN AXIS (project the bounding-circle edge centres
            // onto the axis).  The old code read the world-Z bbox span, which is only
            // the depth when the axis is +/-Z; for a side hole the Z-span is the
            // DIAMETER, not the depth.  Axis projection is axis-agnostic.
            double aMin = std::numeric_limits<double>::max();
            double aMax = std::numeric_limits<double>::lowest();
            bool any = false;
            for (TopExp_Explorer exp(wp.face(i), TopAbs_EDGE); exp.More(); exp.Next()) {
                BRepAdaptor_Curve crv(TopoDS::Edge(exp.Current()));
                if (crv.GetType() != GeomAbs_Circle) continue;
                const gp_Pnt p = crv.Circle().Location();
                const double proj = (p.X() - aorg.X()) * adir.X()
                                  + (p.Y() - aorg.Y()) * adir.Y()
                                  + (p.Z() - aorg.Z()) * adir.Z();
                aMin = std::min(aMin, proj);
                aMax = std::max(aMax, proj);
                any = true;
            }
            if (!any) continue;
            const double depth = aMax - aMin;
            const gp_Pnt endHi(aorg.X() + adir.X() * aMax,
                               aorg.Y() + adir.Y() * aMax,
                               aorg.Z() + adir.Z() * aMax);
            const gp_Pnt endLo(aorg.X() + adir.X() * aMin,
                               aorg.Y() + adir.Y() * aMin,
                               aorg.Z() + adir.Z() * aMin);
            // The mouth = the end whose OUTWARD probe lands outside the solid.  Probe
            // +axis past endHi and -axis past endLo; the one that is OUT is the mouth.
            const gp_Dir plusAx = adir;
            const gp_Dir minusAx(-adir.X(), -adir.Y(), -adir.Z());
            bool hiOut = endIsOutside(endHi, plusAx);
            bool loOut = endIsOutside(endLo, minusAx);
            if (hiOut && loOut) {
                // AMBIGUOUS: both near probes are outside.  This is either a genuine
                // through hole — or a BLIND hole whose bottom ends in a drill-point
                // cone / bottom fillet: the cylinder face stops at the cone junction
                // and the short probe lands inside the hole's own conical void.  A
                // drill cone is shallow (height ~ 0.3·dia = 0.6·r), so re-probe both
                // ends FARTHER (1.2·r, past any cone tip): the cone end now hits
                // material (IN) while a true mouth stays open (OUT).
                const double far = std::max(1.2 * gc.Radius(), 1.0);
                auto endIsOutsideAt = [&](const gp_Pnt& end, const gp_Dir& dir) {
                    const gp_Pnt probe(end.X() + dir.X() * far,
                                       end.Y() + dir.Y() * far,
                                       end.Z() + dir.Z() * far);
                    classifier.Perform(probe, 1e-6);
                    return classifier.State() == TopAbs_OUT;
                };
                hiOut = endIsOutsideAt(endHi, plusAx);
                loOut = endIsOutsideAt(endLo, minusAx);
            }
            bool hiIsEntry;
            if (hiOut != loOut) {
                hiIsEntry = hiOut;                 // exactly one end is a mouth (blind hole)
            } else {
                // Both ends open (a genuine through hole) or neither classified:
                // fall back to the +axis end.  The axis was CANONICALISED above, so
                // this pick is deterministic and lands on the SAME geometric face
                // for every hole of a ring (both ends of a through hole are valid
                // CAM entries anyway).
                hiIsEntry = true;
            }
            const gp_Pnt entry = hiIsEntry ? endHi : endLo;
            if (!hiIsEntry) adir = minusAx;
            all.push_back(C{ gc.Location(), adir, gc.Radius(), depth, entry, i });
        } catch (...) {}
    }
    if (all.size() < 3) return out;

    // Only cylinders sharing BOTH radius AND axis direction can be one bolt circle
    // (a ring of holes all bored parallel along the same axis).  Pick the LARGEST
    // such group of >= 3 — this discards the stock's own outer wall and any holes
    // on a different face/axis that would pollute the centre/radius statistics.
    auto axisParallel = [](const gp_Dir& a, const gp_Dir& b) {
        return std::abs(a.X() * b.X() + a.Y() * b.Y() + a.Z() * b.Z()) > 0.999;
    };
    std::vector<C> cyls;
    {
        std::vector<bool> used(all.size(), false);
        for (size_t g = 0; g < all.size(); ++g) {
            if (used[g]) continue;
            std::vector<C> grp;
            for (size_t k = 0; k < all.size(); ++k)
                if (!used[k] && std::abs(all[k].r - all[g].r) < 1e-2 &&
                    axisParallel(all[k].axis, all[g].axis)) {
                    grp.push_back(all[k]); used[k] = true;
                }
            // Prefer the larger group; on a TIE (e.g. a counterbored bolt circle
            // with N pilot + N seat cylinders) prefer the SMALLER radius — that is
            // the pilot bore = the true drill diameter, so subsumption drops the
            // matching-diameter drills correctly instead of the oversized seats.
            if (grp.size() > cyls.size() ||
                (grp.size() == cyls.size() && !cyls.empty() &&
                 grp.front().r < cyls.front().r))
                cyls = grp;
        }
    }
    if (cyls.size() < 3) return out;

    // The shared pattern axis (mean of the group's hole axes, sign-aligned to the
    // first) and an orthonormal in-plane basis (u, v) perpendicular to it.  All
    // ring geometry — centre, radial offset, angular evenness — is measured in this
    // (u, v) plane so it works for any axis, not just +/-Z.
    const gp_Dir axisDir = cyls.front().axis;
    const gp_Vec axV(axisDir);
    gp_Dir refX(1.0, 0.0, 0.0);
    if (std::abs(axV.Dot(gp_Vec(refX))) > 0.95) refX = gp_Dir(0.0, 1.0, 0.0);
    gp_Vec uVec = gp_Vec(refX) - axV * gp_Vec(refX).Dot(axV);
    if (uVec.Magnitude() < 1e-9) return out;
    uVec.Normalize();
    const gp_Vec vVec = axV.Crossed(uVec);

    // Provisional ring centre = mean of ALL grouped hole centres.  Two CONCENTRIC
    // rings of the same hole diameter + axis (a double-row flange) share this centre
    // but sit at DIFFERENT bolt-circle radii — the group so far keys only on hole
    // radius + axis, so they are still mixed here.
    gp_Pnt meanC(0, 0, 0);
    for (const auto& c : cyls) meanC.SetXYZ(meanC.XYZ() + c.loc.XYZ());
    meanC.SetXYZ(meanC.XYZ() / static_cast<double>(cyls.size()));

    // Radial distance of each hole from the provisional centre, in the (u, v) plane.
    auto radialOf = [&](const gp_Pnt& loc) {
        const gp_Vec rel(meanC, loc);
        return std::hypot(rel.Dot(uVec), rel.Dot(vVec));
    };
    // SUB-CLUSTER by bolt-circle radius: keep only the largest set of holes whose
    // radial distances fall inside one tight band (max/min within 5%).  Without this
    // two staggered concentric rings within ~5% would merge into a false 2N ring at
    // the mean radius that slips the radial + angular gates below.  Sort the holes by
    // radius and slide a window that holds a <=5%-spread band; keep the biggest.
    {
        std::vector<C> sorted = cyls;
        std::sort(sorted.begin(), sorted.end(),
                  [&](const C& a, const C& b) { return radialOf(a.loc) < radialOf(b.loc); });
        std::vector<C> best;
        for (size_t lo = 0; lo < sorted.size(); ++lo) {
            const double rLo = radialOf(sorted[lo].loc);
            if (rLo < 1e-6) continue;
            size_t hi = lo;
            while (hi + 1 < sorted.size() &&
                   (radialOf(sorted[hi + 1].loc) - rLo) / rLo <= 0.05)
                ++hi;
            if (hi - lo + 1 > best.size())
                best.assign(sorted.begin() + lo, sorted.begin() + hi + 1);
        }
        if (best.size() < cyls.size()) {   // a tighter sub-ring was isolated
            cyls = best;
            if (cyls.size() < 3) return out;
            // Recompute the centre on the isolated ring only.
            meanC = gp_Pnt(0, 0, 0);
            for (const auto& c : cyls) meanC.SetXYZ(meanC.XYZ() + c.loc.XYZ());
            meanC.SetXYZ(meanC.XYZ() / static_cast<double>(cyls.size()));
        }
    }

    // In-plane (u, v) coordinate of each hole centre relative to the ring centre.
    std::vector<double> ds, angles;
    for (const auto& c : cyls) {
        const gp_Vec rel(meanC, c.loc);
        const double u = rel.Dot(uVec), v = rel.Dot(vVec);
        ds.push_back(std::hypot(u, v));
        angles.push_back(std::atan2(v, u));
    }
    const double meanD = std::accumulate(ds.begin(), ds.end(), 0.0) / ds.size();
    if (meanD < 1e-3) return out;
    double maxDev = 0.0;
    for (double d : ds) maxDev = std::max(maxDev, std::abs(d - meanD));
    if (maxDev / meanD > 0.05) return out;

    // Require a roughly EVEN angular distribution too — N holes equally spaced on
    // the circle — so an irregular cluster that merely sits on a common radius
    // (e.g. 3 unrelated holes) is not mis-read as a bolt circle.
    std::sort(angles.begin(), angles.end());
    std::vector<double> aGaps;
    for (size_t k = 1; k < angles.size(); ++k)
        aGaps.push_back(angles[k] - angles[k - 1]);
    aGaps.push_back(2.0 * M_PI - (angles.back() - angles.front()));  // wrap gap
    const double meanA = (2.0 * M_PI) / cyls.size();
    double maxADev = 0.0;
    for (double a : aGaps) maxADev = std::max(maxADev, std::abs(a - meanA));
    // Allow up to 25 % of the nominal step (a partial arc still reads as even).
    if (meanA > 1e-6 && maxADev / meanA > 0.25) return out;

    // A recovered hole must carry a positive depth (apply/validate reject
    // depth<=0).  A cylinder face always has axial extent, but guard anyway.
    if (cyls.front().depth <= 0.0) return out;

    // The ring ORIGIN is the ring centre lifted to the shared entry plane: take the
    // mean entry point of the holes (their shallow ends) so apply() bores from the
    // real surface along -axis_dir, for any axis.
    gp_Pnt entryOrigin(0, 0, 0);
    for (const auto& c : cyls) entryOrigin.SetXYZ(entryOrigin.XYZ() + c.entry.XYZ());
    entryOrigin.SetXYZ(entryOrigin.XYZ() / static_cast<double>(cyls.size()));

    json centers = json::array();
    for (const auto& c : cyls) centers.push_back({ c.loc.X(), c.loc.Y(), c.loc.Z() });
    json recovered = {
        { "axis_origin_xyz",  { entryOrigin.X(), entryOrigin.Y(), entryOrigin.Z() } },
        { "axis_dir_xyz",     { axisDir.X(), axisDir.Y(), axisDir.Z() } },
        { "hole_dia_mm",      2.0 * cyls.front().r },
        { "hole_depth_mm",    cyls.front().depth },   // recovered from cyl axial length
        { "radial_offset_mm", meanD },
        { "count",            static_cast<int>(cyls.size()) },
        { "total_angle_deg",  360.0 },
        { "hole_centers",     centers },     // 3-D centres for pattern subsumption
    };
    // Emit the constituent cylinder FACE IDs so dedupe (collectFaceIds) can
    // arbitrate this candidate against the grammar's bolt_circle_pattern for the
    // SAME holes — without a face-id key both dedupe paths are no-ops and the two
    // patterns survive as duplicate steps (which diverge under an edited replay).
    json cylFaceIds = json::array();
    for (const auto& c : cyls) cylFaceIds.push_back(c.faceId);
    json matched = {
        { "cyl_count",  static_cast<int>(cyls.size()) },
        { "mean_radius", meanD },
        { "max_radial_deviation", maxDev },
        { "axis_dir", { axisDir.X(), axisDir.Y(), axisDir.Z() } },
        { "cyl_face_ids", cylFaceIds },   // for dedupe face-id subsumption
    };
    // 0.7 (reaches the Executor): with depth now recovered the pattern is
    // regeneratable, and the co-radial + even-angular gates make a false bolt
    // circle unlikely.  (Was 0.65 — sub-threshold — which, combined with the
    // zero-depth abort, made the whole path dead.)
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.7, matched });
    return out;
}

}  // namespace koocadcam::skill::circular_pattern
