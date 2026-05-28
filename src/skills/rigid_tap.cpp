// @lat: [[engine/skills#rigid_tap]]
//
// rigid_tap — internal thread cut by a rigid (synchronized) tap.  Same
// V-thread helical geometry as tap_thread, distinguished only by tooling
// metadata (rigid_tap_mode = true) and the additional DFM-RIGID-TAP-PITCH
// check against the standard metric coarse chart.

#include "rigid_tap.hpp"

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

namespace koocadcam::skill::rigid_tap {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Chart lookups ────────────────────────────────────────────────────────

double tapDrillDiameter(const std::string& thread_size)
{
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

double standardCoarsePitch(const std::string& thread_size)
{
    // ISO 261 standard metric coarse pitch series.
    if (thread_size == "M1.6") return 0.35;
    if (thread_size == "M2")   return 0.40;
    if (thread_size == "M2.5") return 0.45;
    if (thread_size == "M3")   return 0.50;
    if (thread_size == "M4")   return 0.70;
    if (thread_size == "M5")   return 0.80;
    if (thread_size == "M6")   return 1.00;
    if (thread_size == "M8")   return 1.25;
    if (thread_size == "M10")  return 1.50;
    if (thread_size == "M12")  return 1.75;
    return 0.0;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.pitch_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "rigid_tap pitch_mm must be > 0");
    }
    if (in.thread_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "rigid_tap thread_depth_mm must be > 0");
    }
    if (in.pitch_mm > 0.0 && in.thread_depth_mm < in.pitch_mm * 1.5) {
        r.add("DFM-TAP-ENGAGE", "error",
              "thread_depth_mm " + std::to_string(in.thread_depth_mm) +
              " < 1.5 × pitch (" + std::to_string(in.pitch_mm * 1.5) +
              ") — insufficient thread engagement");
    }

    // DFM-RIGID-TAP-PITCH — pitch must match the standard coarse chart for
    // the nominal thread size (within 0.01 mm).  A mismatch is a warning,
    // not an error, because fine-pitch rigid taps exist; the warning lets
    // downstream CAPP flag a tool-room request.
    const double stdPitch = standardCoarsePitch(in.thread_size);
    if (stdPitch > 0.0 && std::abs(in.pitch_mm - stdPitch) > 0.01) {
        r.add("DFM-RIGID-TAP-PITCH", "warning",
              "pitch_mm " + std::to_string(in.pitch_mm) +
              " is not the standard coarse pitch (" +
              std::to_string(stdPitch) + ") for " + in.thread_size);
    }

    // DFM-015 — pilot diameter check.
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

// ── Synthesis (real helical sweep) ───────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    // Only block on actual errors; warnings (DFM-RIGID-TAP-PITCH, DFM-015)
    // pass through.
    if (!dfm.passed) {
        std::string msg = "rigid_tap DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto pilotId = wp.resolve(in.existing_hole_datum);
    const int pilotIdOut = pilotId.value_or(-1);

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

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const double pitch        = in.pitch_mm;
    const double threadDepthV = pitch * 0.65;
    const double helixLength  = in.thread_depth_mm;

    gp_Dir helixDir = boreAxis.Direction();
    if (helixDir.Z() > 0.0) helixDir = gp_Dir(-helixDir.X(), -helixDir.Y(), -helixDir.Z());

    const gp_Pnt& O = boreAxis.Location();
    gp_Pnt entryPnt = O;
    const double bdZ = boreAxis.Direction().Z();
    if (std::abs(bdZ) > 1e-9) {
        const double t = (zMax - O.Z()) / bdZ;
        entryPnt = gp_Pnt(O.X() + t * boreAxis.Direction().X(),
                          O.Y() + t * boreAxis.Direction().Y(),
                          O.Z() + t * boreAxis.Direction().Z());
    } else {
        entryPnt = O;
    }

    const gp_Ax1 helixAxis(entryPnt, helixDir);

    TopoDS_Shape threadCutter;
    bool         geomSwept = true;
    try {
        if (gotAxis && pilotRad > 0.0 && threadDepthV > 0.0) {
            threadCutter = pr::helicalSweep(helixAxis, pitch, helixLength,
                                            pilotRad, threadDepthV);
        }
    }
    catch (const std::exception& e) {
        spdlog::warn("rigid_tap: helicalSweep threw ({}); leaving geometry unchanged",
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
            spdlog::warn("rigid_tap: pr::cut failed ({}); leaving workpiece unchanged",
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
        { "kind",                kSkillId },
        { "thread_size",         in.thread_size },
        { "pitch_mm",            in.pitch_mm },
        { "thread_depth_mm",     in.thread_depth_mm },
        { "pilot_diameter_mm",   pilotDia },
        { "geometry",            geomSwept ? "helical_swept" : "metadata_only" },
        { "rigid_tap_mode",      true },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "rigid_tap";
    tooling.tool_dia_mm       = pilotDia;
    tooling.tool_length_mm    = in.thread_depth_mm * 2.0 + 10.0;
    tooling.tool_material     = "hss";
    tooling.flute_count       = 4;
    // Rigid tapping permits higher SFM than tension-compression because
    // spindle synchronization eliminates accumulated lag.
    tooling.cutting_speed_sfm = 40.0;
    tooling.feed_per_tooth_mm = in.pitch_mm / 4.0;
    tooling.extra["rigid_tap_mode"] = true;
    tooling.extra["nc_cycle"]       = "G84.2";
    {
        const double triArea = 0.5 * threadDepthV * (in.pitch_mm * 0.5);
        const double helLen  = (pilotRad > 0.0 && in.pitch_mm > 0.0)
            ? 2.0 * M_PI * pilotRad * (in.thread_depth_mm / in.pitch_mm)
            : 0.0;
        tooling.stock_removed_mm3 = triArea * helLen;
    }
    tooling.est_cycle_time_s  = std::max(2.0, in.thread_depth_mm / 6.0);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::rigid_tap applied: {} pitch={} depth={} geom={}",
                  in.thread_size, in.pitch_mm, in.thread_depth_mm,
                  geomSwept ? "swept" : "metadata");

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Topologically a rigid_tap leaves the same helical V-groove face as
// tap_thread.  Without metadata we cannot tell rigid-tap apart from
// tension-compression-tap, so confidence is fixed at 0.5 and the
// recovery is metadata-driven (we walk wp.features()).

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 0.5;   // metadata-only — geometry alone cannot disambiguate
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::rigid_tap
