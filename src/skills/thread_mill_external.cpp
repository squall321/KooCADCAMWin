// @lat: [[engine/skills#thread_mill_external]]
//
// thread_mill_external — cut an external thread by milling a helical
// V-groove valley into a cylindrical outer surface.  Geometrically the
// inverse of the internal thread case: same V-thread helical sweep, but
// the datum is the EXTERNAL cylinder and we SUBTRACT the tool (which
// removes the valleys, leaving the remaining material as the thread).

#include "thread_mill_external.hpp"

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

namespace koocadcam::skill::thread_mill_external {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.pitch_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "thread_mill_external pitch_mm must be > 0");
    }
    if (in.thread_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "thread_mill_external thread_length_mm must be > 0");
    }
    if (in.tool_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "thread_mill_external tool_dia_mm " +
              std::to_string(in.tool_dia_mm) +
              " < 0.8 mm (thread-mill minimum)");
    }

    // DFM-EXT-TURNS — need enough turns to form a real thread.
    if (in.pitch_mm > 0.0 && in.thread_length_mm > 0.0) {
        const double turns = in.thread_length_mm / in.pitch_mm;
        if (turns < 3.0) {
            r.add("DFM-EXT-TURNS", "error",
                  "thread_length / pitch = " + std::to_string(turns) +
                  " < 3 turns — insufficient thread length");
        }
    }

    // DFM-EXT-MIN — external cylinder must be at least M3 minor diameter.
    auto cylId = wp.resolve(in.cylinder_datum);
    if (cylId) {
        BRepAdaptor_Surface surf(wp.face(*cylId));
        if (surf.GetType() == GeomAbs_Cylinder) {
            const double dia = 2.0 * surf.Cylinder().Radius();
            if (dia < 2.5) {
                r.add("DFM-EXT-MIN", "error",
                      "outer cylinder diameter " + std::to_string(dia) +
                      " mm < 2.5 mm (M3 minor) — too small to thread");
            }
        } else {
            r.add("DFM-EXT-DATUM", "warning",
                  "cylinder_datum did not resolve to a cylindrical face");
        }
    } else {
        r.add("DFM-EXT-DATUM", "warning",
              "cylinder_datum unresolved — cannot verify outer diameter");
    }

    return r;
}

// ── Synthesis (real helical sweep, subtractive) ──────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "thread_mill_external DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto cylId = wp.resolve(in.cylinder_datum);
    const int cylIdOut = cylId.value_or(-1);

    gp_Ax1 cylAxis(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
    double cylDia = 0.0;
    double cylRad = 0.0;
    bool   gotAxis = false;
    if (cylId) {
        BRepAdaptor_Surface surf(wp.face(*cylId));
        if (surf.GetType() == GeomAbs_Cylinder) {
            const gp_Cylinder cyl = surf.Cylinder();
            cylRad = cyl.Radius();
            cylDia = 2.0 * cylRad;
            cylAxis = cyl.Axis();
            gotAxis = true;
        }
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const double pitch        = in.pitch_mm;
    const double threadDepthV = pitch * 0.65;       // ISO V-thread depth
    const double helixLength  = in.thread_length_mm;

    // Orient helix axis: prefer to descend from the top of the threaded
    // region (start_z_mm if positive and within bbox) downward.  For a
    // stable default we follow the cylinder's natural direction.
    gp_Dir helixDir = cylAxis.Direction();
    if (helixDir.Z() > 0.0) helixDir = gp_Dir(-helixDir.X(), -helixDir.Y(), -helixDir.Z());

    // Compute helix start point.  If `start_z_mm` is explicitly set (and
    // differs from end_z_mm), use it; otherwise project the cylinder's
    // axis location onto the bbox top.
    const gp_Pnt& O = cylAxis.Location();
    gp_Pnt startPnt = O;
    bool   gotStart = false;
    if (std::abs(in.start_z_mm - in.end_z_mm) > 1e-9) {
        // Caller specified start/end; pick the higher Z as the helix start.
        const double zStart = std::max(in.start_z_mm, in.end_z_mm);
        const double bdZ = cylAxis.Direction().Z();
        if (std::abs(bdZ) > 1e-9) {
            const double t = (zStart - O.Z()) / bdZ;
            startPnt = gp_Pnt(O.X() + t * cylAxis.Direction().X(),
                              O.Y() + t * cylAxis.Direction().Y(),
                              O.Z() + t * cylAxis.Direction().Z());
            gotStart = true;
        }
    }
    if (!gotStart) {
        const double bdZ = cylAxis.Direction().Z();
        if (std::abs(bdZ) > 1e-9) {
            const double t = (zMax - O.Z()) / bdZ;
            startPnt = gp_Pnt(O.X() + t * cylAxis.Direction().X(),
                              O.Y() + t * cylAxis.Direction().Y(),
                              O.Z() + t * cylAxis.Direction().Z());
        } else {
            startPnt = O;
        }
    }

    const gp_Ax1 helixAxis(startPnt, helixDir);

    TopoDS_Shape threadCutter;
    bool         geomSwept = true;
    try {
        if (gotAxis && cylRad > 0.0 && threadDepthV > 0.0) {
            // Position the helix INSIDE the cylinder by sweeping at
            // major_radius = cylRad.  The V-profile pokes outward by
            // threadDepthV, so the cutter's V tips sit at radius
            // (cylRad + threadDepthV) — i.e., outside the cylinder.
            // When we SUBTRACT this from the workpiece, the part of the
            // cutter that lies inside the cylinder removes a helical
            // valley from its outer surface.
            //
            // The V-profile apex (inner vertex) sits at cylRad, the outer
            // pair at (cylRad + threadDepthV).  This means subtraction
            // removes from radius cylRad outward by threadDepthV — but
            // that's OUTSIDE the cylinder.  Instead we shift the helix
            // inward: use (cylRad - threadDepthV) as major_radius so that
            // the V's outer pair lands AT cylRad (the surface) and the
            // apex sits at cylRad - threadDepthV (inside the material).
            const double sweepRadius = cylRad - threadDepthV;
            if (sweepRadius <= 0.0)
                throw std::runtime_error("cylinder radius - thread depth ≤ 0");
            threadCutter = pr::helicalSweep(helixAxis, pitch, helixLength,
                                            sweepRadius, threadDepthV);
        }
    }
    catch (const std::exception& e) {
        spdlog::warn("thread_mill_external: helicalSweep threw ({}); "
                     "leaving geometry unchanged", e.what());
        threadCutter.Nullify();
        geomSwept = false;
    }

    TopoDS_Shape newShape = wp.shape();
    if (!threadCutter.IsNull()) {
        try {
            // External thread = subtract the V-helix from the cylinder.
            // The bits of the cutter that protrude outside the cylinder
            // are no-ops; the bits that intersect the cylinder carve
            // the thread valleys.
            newShape = pr::cut(wp.shape(), threadCutter);
        }
        catch (const std::exception& e) {
            spdlog::warn("thread_mill_external: pr::cut failed ({}); "
                         "leaving workpiece unchanged", e.what());
            newShape = wp.shape();
            geomSwept = false;
        }
    } else {
        geomSwept = false;
    }

    const double turns = (in.pitch_mm > 0.0) ? (in.thread_length_mm / in.pitch_mm) : 0.0;

    json params = {
        { "cylinder_face_id", cylIdOut },
        { "thread_size",      in.thread_size },
        { "pitch_mm",         in.pitch_mm },
        { "thread_length_mm", in.thread_length_mm },
        { "start_z_mm",       in.start_z_mm },
        { "end_z_mm",         in.end_z_mm },
        { "tool_dia_mm",      in.tool_dia_mm },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "thread_size",           in.thread_size },
        { "pitch_mm",              in.pitch_mm },
        { "thread_length_mm",      in.thread_length_mm },
        { "outer_diameter_mm",     cylDia },
        { "is_external",           true },
        { "turns",                 turns },
        { "geometry",              geomSwept ? "helical_swept" : "metadata_only" },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "thread_mill_external";
    tooling.tool_dia_mm       = in.tool_dia_mm;
    tooling.tool_length_mm    = in.thread_length_mm + 10.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 400.0;
    tooling.feed_per_tooth_mm = 0.02;
    tooling.extra["is_external"] = true;
    tooling.extra["turns"]       = turns;
    {
        // Stock removed ≈ V-cross-section × helix-arc-length.
        const double triArea = 0.5 * threadDepthV * (in.pitch_mm * 0.5);
        const double helLen  = (cylRad > 0.0)
            ? 2.0 * M_PI * cylRad * turns
            : 0.0;
        tooling.stock_removed_mm3 = triArea * helLen;
    }
    tooling.est_cycle_time_s  = std::max(2.0, in.thread_length_mm / 4.0);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::thread_mill_external applied: {} pitch={} length={} "
                  "outerDia={} geom={}",
                  in.thread_size, in.pitch_mm, in.thread_length_mm, cylDia,
                  geomSwept ? "swept" : "metadata");

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// External thread surfaces are difficult to recognize geometrically: the
// swept helical face is a B-spline, often degraded through STEP I/O.  For
// slice 1 we rely on metadata only.  A future slice can add geometric
// recognition by:
//   1. Detect external cylindrical face;
//   2. Sample its surface for helical "ridge" pattern (axial Z vs
//      azimuthal θ correlation).

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 0.5;   // metadata-only — see TODO above
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::thread_mill_external
