// @lat: [[engine/skills#crown_knurl]]

#include "crown_knurl.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <TopoDS.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cone.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::crown_knurl {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    (void)wp;
    DFMReport r;
    if (in.count < 3)
        r.add("DFM-INPUT", "error", "crown_knurl: count must be >= 3");
    if (in.knob_radius_mm <= 0.0)
        r.add("DFM-INPUT", "error", "crown_knurl: knob_radius_mm must be > 0");
    if (in.notch_radius_mm <= 0.0)
        r.add("DFM-INPUT", "error", "crown_knurl: notch_radius_mm must be > 0");
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "crown_knurl DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Ring frame: centre + axis, with two in-plane axes.
    const gp_Pnt centre(in.world_cx_mm, in.world_cy_mm, in.world_cz_mm);
    gp_Dir axis(in.world_ax, in.world_ay, in.world_az);
    // In-plane X/Y perpendicular to the axis.
    gp_Vec seed = std::abs(axis.Z()) < 0.9 ? gp_Vec(0, 0, 1) : gp_Vec(1, 0, 0);
    gp_Vec kx = gp_Vec(axis).Crossed(seed);
    if (kx.Magnitude() < 1e-9) kx = gp_Vec(1, 0, 0);
    kx.Normalize();
    const gp_Vec ky = gp_Vec(axis).Crossed(kx);

    // Cut N notches around the ring, each a small cylinder pointing INWARD (its
    // axis perpendicular to the knob axis, toward the knob axis).
    TopoDS_Shape current = wp.shape();
    for (int i = 0; i < in.count; ++i) {
        const double phi = 2.0 * M_PI * i / in.count;
        const double ux = std::cos(phi), uy = std::sin(phi);
        const gp_Pnt c(centre.X() + (kx.X() * ux + ky.X() * uy) * in.knob_radius_mm,
                       centre.Y() + (kx.Y() * ux + ky.Y() * uy) * in.knob_radius_mm,
                       centre.Z() + (kx.Z() * ux + ky.Z() * uy) * in.knob_radius_mm);
        const gp_Dir inDir(-(kx.X() * ux + ky.X() * uy),
                           -(kx.Y() * ux + ky.Y() * uy),
                           -(kx.Z() * ux + ky.Z() * uy));
        const TopoDS_Shape notch =
            pr::cylinder(gp_Ax2(c, inDir), in.notch_radius_mm, in.notch_radius_mm * 2.0);
        current = pr::cut(current, notch);
    }

    const double perVol = M_PI * in.notch_radius_mm * in.notch_radius_mm *
                          (in.notch_radius_mm * 2.0);
    const double vol = perVol * in.count;   // upper bound (notches overlap the knob)

    json params = {
        { "world_center",   { centre.X(), centre.Y(), centre.Z() } },
        { "world_axis",     { axis.X(), axis.Y(), axis.Z() } },
        { "knob_radius_mm", in.knob_radius_mm },
        { "notch_radius_mm", in.notch_radius_mm },
        { "count",          in.count },
    };
    json pattern = {
        { "kind",         kSkillId },
        { "is_pattern",   true },
        { "instance_count", in.count },
        { "derived_volume_mm3", vol },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill;knurl";
    tooling.tool_dia_mm       = in.notch_radius_mm * 2.0;
    tooling.tool_length_mm    = in.notch_radius_mm * 2.0 + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.stock_removed_mm3 = vol;
    tooling.est_cycle_time_s  = std::max(2.0, static_cast<double>(in.count) * 0.5);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::crown_knurl applied: N={} knobR={} notchR={}",
                  in.count, in.knob_radius_mm, in.notch_radius_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // metadata replay.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric path B: a knurl is a ring of N small equal-radius cylinders whose
    // axes are PERPENDICULAR to a knob's axis, evenly spaced around it.  Anchor on
    // the knob CONE frustum, then find the notch cylinders about its axis.
    struct KnobInfo { gp_Ax1 axis; gp_Pnt apex; double refR; double axLo, axHi; };
    std::vector<KnobInfo> knobs;
    for (int i = 0; i < wp.faceCount(); ++i) {
        BRepAdaptor_Surface s(wp.face(i));
        if (s.GetType() != GeomAbs_Cone) continue;
        const gp_Cone cone = s.Cone();
        // The cone FACE's axial extent (project the face bbox corners onto the
        // cone axis, relative to the axis location) — bounds where the knob
        // actually IS, so notches can be required to sit ON it (not far along the
        // infinite axis).
        Bnd_Box fb; BRepBndLib::Add(wp.face(i), fb);
        double axLo = 0.0, axHi = 0.0;
        if (!fb.IsVoid()) {
            double bx0, by0, bz0, bx1, by1, bz1; fb.Get(bx0, by0, bz0, bx1, by1, bz1);
            const double cxr[2] = { bx0, bx1 }, cyr[2] = { by0, by1 }, czr[2] = { bz0, bz1 };
            const gp_Pnt L = cone.Axis().Location();
            const gp_Dir D = cone.Axis().Direction();
            axLo = 1e30; axHi = -1e30;
            for (int ix = 0; ix < 2; ++ix)
                for (int iy = 0; iy < 2; ++iy)
                    for (int iz = 0; iz < 2; ++iz) {
                        const double a = gp_Vec(L, gp_Pnt(cxr[ix], cyr[iy], czr[iz]))
                                             .Dot(gp_Vec(D));
                        axLo = std::min(axLo, a); axHi = std::max(axHi, a);
                    }
        }
        knobs.push_back({ cone.Axis(), cone.Apex(), cone.RefRadius(), axLo, axHi });
    }
    if (knobs.empty()) return out;

    // Collect all cylinder faces (the notch candidates + the shaft + others).
    struct Cyl { gp_Ax1 axis; gp_Pnt loc; double radius; int faceId; };
    std::vector<Cyl> cyls;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface s(wp.face(i));
        const gp_Cylinder gc = s.Cylinder();
        cyls.push_back({ gc.Axis(), gc.Axis().Location(), gc.Radius(), i });
    }

    for (const auto& knob : knobs) {
        const gp_Dir kAxis = knob.axis.Direction();
        const gp_Pnt kLoc  = knob.axis.Location();

        // Notch candidates: small cylinders whose axis is PERPENDICULAR to the
        // knob axis (radial) — this immediately excludes the crown SHAFT (a bore
        // whose axis is PARALLEL to the knob).
        struct Notch { double angle; double radius; double ringR; gp_Pnt centre; int faceId; };
        std::vector<Notch> notches;
        double notchR = -1.0;
        for (const auto& c : cyls) {
            const double axDot = std::abs(gp_Vec(c.axis.Direction()).Dot(gp_Vec(kAxis)));
            if (axDot > 0.2) continue;                 // not perpendicular → not a radial notch
            // The notch centre must sit near the knob axis at a consistent radius.
            const gp_Vec rel(kLoc, c.loc);
            const double axProj = rel.Dot(gp_Vec(kAxis));   // notch axial coord on the knob axis
            const gp_Vec axial = gp_Vec(kAxis) * axProj;
            const gp_Vec radial = rel - axial;
            const double ringR = radial.Magnitude();
            if (ringR < 1e-3) continue;
            // AXIAL-CONTAINMENT gate: the notch must sit ON the knob's axial span
            // (within a margin), NOT anywhere along the infinite cone axis.  A cone
            // axis is unbounded, so without this a matching-radius radial ring far
            // up/down the axis (a grub-screw collar, an unrelated ring) would
            // falsely qualify at precise-tier confidence.
            const double axMargin = std::max(1.0, 0.5 * (knob.axHi - knob.axLo));
            if (axProj < knob.axLo - axMargin || axProj > knob.axHi + axMargin) continue;
            // NEAR-KNOB (radial) gate: a knurl notch sits ON the knob rim, so its
            // ring radius must be close to the knob's reference radius.  Allow a
            // generous band (the cone tapers).
            if (std::abs(ringR - knob.refR) > std::max(0.5, 0.5 * knob.refR)) continue;
            // First notch sets the reference notch radius; the rest must match.
            if (notchR < 0.0) notchR = c.radius;
            else if (std::abs(c.radius - notchR) > 0.1 * std::max(0.1, notchR)) continue;
            // Angle about the knob axis (in a fixed in-plane frame).
            gp_Vec kx = gp_Vec(kAxis).Crossed(gp_Vec(0, 0, 1));
            if (kx.Magnitude() < 1e-6) kx = gp_Vec(kAxis).Crossed(gp_Vec(1, 0, 0));
            kx.Normalize();
            const gp_Vec ky = gp_Vec(kAxis).Crossed(kx);
            const double ang = std::atan2(radial.Dot(ky), radial.Dot(kx));
            notches.push_back({ ang, c.radius, ringR, c.loc, c.faceId });
        }
        if (static_cast<int>(notches.size()) < 3) continue;   // need >= 3 notches

        // RING-RADIUS consistency: all notches must lie on ONE ring (not a radial
        // blob at mixed radii).  Reject if the ring radii spread too much.
        {
            double rMin = 1e30, rMax = -1e30;
            for (const auto& nn : notches) { rMin = std::min(rMin, nn.ringR); rMax = std::max(rMax, nn.ringR); }
            if (rMax - rMin > 0.25 * std::max(0.1, rMax)) continue;
        }

        // Even angular spacing: sort by angle, require constant delta ~ 2π/N.
        std::sort(notches.begin(), notches.end(),
                  [](const Notch& a, const Notch& b){ return a.angle < b.angle; });
        const int N = static_cast<int>(notches.size());
        const double step = 2.0 * M_PI / N;
        double maxDev = 0.0;
        for (int i = 0; i < N; ++i) {
            const double a0 = notches[i].angle;
            const double a1 = (i + 1 < N) ? notches[i + 1].angle
                                          : notches[0].angle + 2.0 * M_PI;
            maxDev = std::max(maxDev, std::abs((a1 - a0) - step));
        }
        if (maxDev > 0.25 * step) continue;            // not evenly spaced

        // Ring centre + radius from the notch centres, projected onto the knob axis.
        double sumRing = 0.0;
        gp_Vec sumAxial(0, 0, 0);
        for (const auto& nn : notches) {
            const gp_Vec rel(kLoc, nn.centre);
            const gp_Vec axial = gp_Vec(kAxis) * rel.Dot(gp_Vec(kAxis));
            sumRing += (rel - axial).Magnitude();
            sumAxial += axial;
        }
        const double ringRadius = sumRing / N;
        const gp_Vec meanAxial = sumAxial * (1.0 / N);
        const gp_Pnt ringCentre(kLoc.X() + meanAxial.X(),
                                kLoc.Y() + meanAxial.Y(),
                                kLoc.Z() + meanAxial.Z());

        json recovered = {
            { "world_center",   { ringCentre.X(), ringCentre.Y(), ringCentre.Z() } },
            { "world_axis",     { kAxis.X(), kAxis.Y(), kAxis.Z() } },
            { "knob_radius_mm", ringRadius },
            { "notch_radius_mm", notchR },
            { "count",          N },
        };
        // Emit the notch cylinder FACE IDs so dedupe (collectFaceIds) can
        // arbitrate this candidate — without a face-id key it can neither subsume
        // its constituent notch faces nor be arbitrated against a competing claim.
        json cylFaceIds = json::array();
        for (const auto& nn : notches) cylFaceIds.push_back(nn.faceId);
        json matched = {
            { "source",     "geometry" },
            { "notch_count", N },
            { "knob_ref_radius_mm", knob.refR },
            { "spacing_dev", maxDev },
            { "cyl_face_ids", cylFaceIds },   // notch faces, for dedupe
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.85, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::crown_knurl
