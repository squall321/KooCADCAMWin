// @lat: [[engine/skills#thread_mill]]
//
// thread_mill — external or internal thread produced by a thread-mill cutter
// traversing a 3-axis helical path.  Geometrically identical to tap_thread
// (a V-thread helix) but the tool is a smaller-diameter end-mill following
// the helix in NC code rather than a self-feeding tap.
//
// Slice 4: geometry is now REAL — `prim::helicalSweep` builds the V-thread
// helical solid.  For internal threads (`is_external=false`) we SUBTRACT
// the cutter from the workpiece; for external threads (`is_external=true`)
// we FUSE the helical thread material onto the workpiece.

#include "thread_mill.hpp"

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

namespace koocadcam::skill::thread_mill {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.tool_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "thread_mill tool_dia_mm " + std::to_string(in.tool_dia_mm) +
              " < 0.8 mm (end-mill minimum)");
    }
    if (in.pitch_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "thread_mill pitch_mm must be > 0");
    }
    if (in.thread_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "thread_mill thread_depth_mm must be > 0");
    }
    if (in.pitch_mm > 0.0 && in.thread_depth_mm < in.pitch_mm * 1.5) {
        r.add("DFM-THREAD-ENGAGE", "error",
              "thread_depth_mm " + std::to_string(in.thread_depth_mm) +
              " < 1.5 × pitch (" + std::to_string(in.pitch_mm * 1.5) +
              ") — insufficient thread engagement");
    }
    (void)wp;
    return r;
}

// ── Synthesis (real helical sweep — slice 4) ─────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "thread_mill DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto datumId = wp.resolve(in.existing_hole_datum);
    const int datumIdOut = datumId.value_or(-1);

    gp_Ax1 cylAxis(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
    double datumDia = 0.0;
    double datumRad = 0.0;
    bool   gotAxis  = false;
    if (datumId) {
        BRepAdaptor_Surface surf(wp.face(*datumId));
        if (surf.GetType() == GeomAbs_Cylinder) {
            const gp_Cylinder cyl = surf.Cylinder();
            datumRad = cyl.Radius();
            datumDia = 2.0 * datumRad;
            cylAxis  = cyl.Axis();
            gotAxis  = true;
        }
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const double pitch         = in.pitch_mm;
    // V-thread valley depth: standard rule-of-thumb is pitch × 0.65.
    const double threadDepthV  = pitch * 0.65;
    const double helixLength   = in.thread_depth_mm;

    // Orient helix axis to descend into the bore (for internal) or up along
    // the cylinder (for external — direction doesn't matter geometrically,
    // we just need a stable progression).
    gp_Dir helixDir = cylAxis.Direction();
    if (helixDir.Z() > 0.0) helixDir = gp_Dir(-helixDir.X(), -helixDir.Y(), -helixDir.Z());

    // Entry point: same projection rule as tap_thread.
    const gp_Pnt& O = cylAxis.Location();
    gp_Pnt entryPnt = O;
    const double bdZ = cylAxis.Direction().Z();
    if (std::abs(bdZ) > 1e-9) {
        const double t = (zMax - O.Z()) / bdZ;
        entryPnt = gp_Pnt(O.X() + t * cylAxis.Direction().X(),
                          O.Y() + t * cylAxis.Direction().Y(),
                          O.Z() + t * cylAxis.Direction().Z());
    }

    const gp_Ax1 helixAxis(entryPnt, helixDir);

    TopoDS_Shape thread;
    bool         geomSwept = true;
    try {
        if (gotAxis && datumRad > 0.0 && threadDepthV > 0.0) {
            thread = pr::helicalSweep(helixAxis, pitch, helixLength,
                                      datumRad, threadDepthV);
        }
    }
    catch (const std::exception& e) {
        spdlog::warn("thread_mill: helicalSweep threw ({}); leaving geometry unchanged",
                     e.what());
        thread.Nullify();
        geomSwept = false;
    }

    TopoDS_Shape newShape = wp.shape();
    if (!thread.IsNull()) {
        try {
            // Internal (cut) vs external (fuse).
            newShape = in.is_external ? pr::fuse(wp.shape(), thread)
                                      : pr::cut (wp.shape(), thread);
        }
        catch (const std::exception& e) {
            spdlog::warn("thread_mill: boolean failed ({}); leaving workpiece unchanged",
                         e.what());
            newShape = wp.shape();
            geomSwept = false;
        }
    } else {
        geomSwept = false;
    }

    json params = {
        { "datum_face_id",   datumIdOut },
        { "thread_size",     in.thread_size },
        { "pitch_mm",        in.pitch_mm },
        { "thread_depth_mm", in.thread_depth_mm },
        { "is_external",     in.is_external },
        { "tool_dia_mm",     in.tool_dia_mm },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "thread_size",       in.thread_size },
        { "pitch_mm",          in.pitch_mm },
        { "thread_depth_mm",   in.thread_depth_mm },
        { "is_external",       in.is_external },
        { "datum_diameter_mm", datumDia },
        { "geometry",          geomSwept ? "helical_swept" : "metadata_only" },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "thread_mill";
    tooling.tool_dia_mm       = in.tool_dia_mm;
    tooling.tool_length_mm    = in.thread_depth_mm * 2.0 + 10.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 400.0;
    tooling.feed_per_tooth_mm = 0.02;
    {
        const double triArea = 0.5 * threadDepthV * (in.pitch_mm * 0.5);
        const double helLen  = (datumRad > 0.0)
            ? 2.0 * M_PI * datumRad * (in.thread_depth_mm / in.pitch_mm)
            : 0.0;
        tooling.stock_removed_mm3 = triArea * helLen;
    }
    tooling.est_cycle_time_s  = std::max(2.0, in.thread_depth_mm / 3.0);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::thread_mill applied: {} pitch={} depth={} ext={} geom={}",
                  in.thread_size, in.pitch_mm, in.thread_depth_mm,
                  in.is_external, geomSwept ? "swept" : "metadata");

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// As with tap_thread, geometric recognition of helical thread faces is
// deferred to a future slice; today's recognize() replays metadata.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::thread_mill
