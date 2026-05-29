// @lat: [[engine/skills#tap_thread]]
//
// tap_thread — internal thread cut into an existing pilot bore by a tap.
//
// Slice 4: geometry is now REAL — `prim::helicalSweep` builds the V-thread
// helical solid and we subtract it from the workpiece.  Previous behaviour
// (metadata-only no-op) is gone.  Recognition still works by replaying the
// signature in `wp.features()`; geometric round-trip recognition (after
// STEP I/O) requires inspecting helical face topology, which is left for
// a future slice — see TODO at the bottom of recognize().

#include "tap_thread.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/HelicalSweep.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp_Ax1.hxx>
#include <gp_Cylinder.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::tap_thread {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Tap-drill chart (ISO metric coarse, from DFM-015 references) ─────────

double tapDrillDiameter(const std::string& thread_size)
{
    // Standard ISO metric coarse tap-drill chart (mm).
    if (thread_size == "M1.6") return 1.25;
    if (thread_size == "M2")   return 1.60;
    if (thread_size == "M2.5") return 2.05;
    if (thread_size == "M3")   return 2.50;
    if (thread_size == "M4")   return 3.30;
    if (thread_size == "M5")   return 4.20;
    if (thread_size == "M6")   return 5.00;
    if (thread_size == "M8")   return 6.80;
    if (thread_size == "M10")  return 8.50;
    return 0.0;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.pitch_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "tap_thread pitch_mm must be > 0");
    }
    if (in.thread_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "tap_thread thread_depth_mm must be > 0");
    }
    // Minimum thread engagement.
    // Standards reference: Machinery's Handbook 31st ed. §1908 "Tapped Hole
    // Length of Engagement" — a minimum of 1.5 P (1.5 × pitch ≈ 1½ full
    // threads) is required to develop the geometric thread profile so the
    // tap can cut a recognizable form.  Full strength engagement (~1×
    // nominal diameter) is a process-planner-level requirement and is not
    // enforced here; the gate below is the absolute geometric floor.
    constexpr double kMinEngagePitchMultiple = 1.5;  // Machinery's Handbook 31st ed. §1908
    if (in.pitch_mm > 0.0 &&
        in.thread_depth_mm < in.pitch_mm * kMinEngagePitchMultiple) {
        r.add("DFM-TAP-ENGAGE", "error",
              "thread_depth_mm " + std::to_string(in.thread_depth_mm) +
              " < 1.5 × pitch (" + std::to_string(in.pitch_mm * 1.5) +
              ") — Machinery's Handbook 31st ed. §1908: minimum tapped engagement is 1.5 P");
    }

    const double expected = tapDrillDiameter(in.thread_size);
    if (expected <= 0.0) {
        r.add("DFM-TAP-SIZE", "warning",
              "unknown thread size '" + in.thread_size + "' — tap-drill chart miss");
    } else {
        auto pilotId = wp.resolve(in.existing_hole_datum);
        if (pilotId) {
            BRepAdaptor_Surface surf(wp.face(*pilotId));
            if (surf.GetType() == GeomAbs_Cylinder) {
                const double pilotDia = 2.0 * surf.Cylinder().Radius();
                // DFM-015: pilot must be within ±0.1 mm of the chart value.
                if (std::abs(pilotDia - expected) > 0.1) {
                    r.add("DFM-015", "warning",
                          "pilot diameter " + std::to_string(pilotDia) +
                          " mm does not match tap-drill chart for " +
                          in.thread_size + " (expected " +
                          std::to_string(expected) + " mm)");
                }
            } else {
                r.add("DFM-TAP-DATUM", "warning",
                      "existing_hole_datum did not resolve to a cylindrical face");
            }
        } else {
            r.add("DFM-TAP-DATUM", "warning",
                  "existing_hole_datum unresolved — cannot verify pilot diameter");
        }
    }

    return r;
}

// ── Synthesis (real helical sweep — slice 4) ─────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    // Only block on actual errors; warnings (size mismatch) pass through.
    if (!dfm.passed) {
        std::string msg = "tap_thread DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto pilotId = wp.resolve(in.existing_hole_datum);
    const int pilotIdOut = pilotId.value_or(-1);

    // Default axis: -Z drilling direction; refined below from the resolved face.
    gp_Ax1 boreAxis(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
    double pilotDia   = 0.0;
    double pilotRad   = 0.0;
    bool   gotAxis    = false;
    if (pilotId) {
        BRepAdaptor_Surface surf(wp.face(*pilotId));
        if (surf.GetType() == GeomAbs_Cylinder) {
            const gp_Cylinder cyl = surf.Cylinder();
            pilotRad = cyl.Radius();
            pilotDia = 2.0 * pilotRad;
            boreAxis = cyl.Axis();
            gotAxis  = true;
        }
    }

    // Resolve workpiece bbox so we can position the helix.  The thread sits
    // BETWEEN the bore-entry plane (at the top of the workpiece) and
    // entry-plane - thread_depth_mm (downward along boreAxis).
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const double pitch         = in.pitch_mm;
    // V-thread valley depth: standard rule-of-thumb is pitch × 0.65.  The
    // V protrudes OUTWARD from the bore into the surrounding material, so
    // we don't clamp by pilotRad.
    const double threadDepthV  = pitch * 0.65;
    const double helixLength   = in.thread_depth_mm;

    // ─── Build the helical cutter ────────────────────────────────────────
    // The helix sweeps a V-profile of inward radial depth `threadDepthV`.
    // Major radius = pilotRad → the V tips just touch the bore wall, so
    // the cut creates the thread valley (recessing into the surrounding
    // material).  For an internal tap, we SUBTRACT this from the workpiece.
    //
    // We must align the helix so it starts at the bore entry plane and
    // extends `helixLength` along -boreAxis.Direction() (into the bore).
    // helicalSweep's axis.Direction() is the helix progression direction,
    // so we want axis = (entry_pnt, -boreAxis.Direction()) — i.e., axis
    // points downward into the bore.
    //
    // entry_pnt: project the bore's axis location onto the top plane of
    // the workpiece (zMax for the common +Z bore).  For arbitrary axes we
    // simply use boreAxis.Location() shifted to the workpiece's top.
    gp_Dir helixDir = boreAxis.Direction();
    // If the cylinder's "natural" axis points up (+Z component), flip it to
    // point INTO the bore (downward) so the helix descends into material.
    if (helixDir.Z() > 0.0) helixDir = gp_Dir(-helixDir.X(), -helixDir.Y(), -helixDir.Z());

    // Entry point: bore axis projected onto z = zMax (top face).
    const gp_Pnt& O = boreAxis.Location();
    // We want a point on the bore axis that is at the workpiece top
    // (z = zMax) along the axis line.  For a vertical axis this is trivial.
    // For a tilted axis, we walk along boreAxis.Direction() until z = zMax.
    gp_Pnt entryPnt = O;
    const double bdZ = boreAxis.Direction().Z();
    if (std::abs(bdZ) > 1e-9) {
        const double t = (zMax - O.Z()) / bdZ;
        entryPnt = gp_Pnt(O.X() + t * boreAxis.Direction().X(),
                          O.Y() + t * boreAxis.Direction().Y(),
                          O.Z() + t * boreAxis.Direction().Z());
    } else {
        // Horizontal bore: leave entryPnt at O (the bore's axis location).
        entryPnt = O;
    }

    const gp_Ax1 helixAxis(entryPnt, helixDir);

    TopoDS_Shape threadCutter;
    bool         geomSwept = true;
    koocadcam::engine::HelicalSweepProfile profileUsed =
        koocadcam::engine::HelicalSweepProfile::AnnularFallback;
    try {
        if (gotAxis && pilotRad > 0.0 && threadDepthV > 0.0) {
            threadCutter = pr::helicalSweep(helixAxis, pitch, helixLength,
                                            pilotRad, threadDepthV,
                                            &profileUsed);
        }
    }
    catch (const std::exception& e) {
        spdlog::warn("tap_thread: helicalSweep threw ({}); will leave geometry unchanged",
                     e.what());
        threadCutter.Nullify();
        geomSwept = false;
    }

    TopoDS_Shape newShape = wp.shape();
    if (!threadCutter.IsNull()) {
        try {
            newShape = pr::cut(wp.shape(), threadCutter);
        }
        catch (const std::exception& e) {
            spdlog::warn("tap_thread: pr::cut failed ({}); leaving workpiece unchanged",
                         e.what());
            newShape = wp.shape();
            geomSwept = false;
        }
    } else {
        geomSwept = false;
    }

    // ─── Build signature ─────────────────────────────────────────────────
    json params = {
        { "pilot_face_id",   pilotIdOut },
        { "thread_size",     in.thread_size },
        { "pitch_mm",        in.pitch_mm },
        { "thread_depth_mm", in.thread_depth_mm },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "thread_size",       in.thread_size },
        { "pitch_mm",          in.pitch_mm },
        { "thread_depth_mm",   in.thread_depth_mm },
        { "pilot_diameter_mm", pilotDia },
        { "geometry",          geomSwept ? "helical_swept" : "metadata_only" },
        { "profile",           geomSwept
                                  ? koocadcam::engine::helicalSweepProfileTag(profileUsed)
                                  : "metadata_only" },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "tap";
    tooling.tool_dia_mm       = pilotDia;
    tooling.tool_length_mm    = in.thread_depth_mm * 2.0 + 10.0;
    tooling.tool_material     = "hss";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 30.0;   // taps run slow
    tooling.feed_per_tooth_mm = in.pitch_mm / 4.0;
    // V-thread valley cross-section (triangle) × helix length × turns.
    {
        const double triArea = 0.5 * threadDepthV * (in.pitch_mm * 0.5);
        const double helLen  = 2.0 * M_PI * pilotRad * (in.thread_depth_mm / in.pitch_mm);
        tooling.stock_removed_mm3 = triArea * helLen;
    }
    tooling.est_cycle_time_s  = std::max(2.0, in.thread_depth_mm / 5.0);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    // ─── Build new workpiece ────────────────────────────────────────────
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    // Preserve existing feature history so prior skills remain queryable.
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::tap_thread applied: {} pitch={} depth={} geom={}",
                  in.thread_size, in.pitch_mm, in.thread_depth_mm,
                  geomSwept ? "swept" : "metadata");

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// TODO (slice 5+): geometric recognition of helical thread faces.  Today
// the swept helical solid would show up as a B-spline face — neither the
// drill_hole cylindrical-face pattern nor any obvious helical-edge pattern
// is easy to detect without dedicated tooling.  For now, recognize relies
// on the metadata trail in `wp.features()`.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;  // exact match — we synthesized it
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::tap_thread
