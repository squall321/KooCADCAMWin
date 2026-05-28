// @lat: [[engine/skills#thread_grind]]
//
// thread_grind — precision finishing of a pre-existing thread via a grind
// wheel.  Geometry handling has two branches:
//
//   1. If removal_mm < kSubToleranceRemovalMm (≈ 5 µm) the change is below
//      reliable OCCT modeling tolerance: we register metadata only.
//   2. Otherwise we run a reduced-depth helical sweep and subtract.

#include "thread_grind.hpp"

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
#include <string>

namespace koocadcam::skill::thread_grind {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

// Look for any prior thread feature in the workpiece history.  Returns the
// index of the parent thread signature (or -1 if none).
int findParentThreadIndex(const Workpiece& wp)
{
    const auto& feats = wp.features();
    for (int i = static_cast<int>(feats.size()) - 1; i >= 0; --i) {
        const std::string& sid = feats[i].skill_id;
        if (sid == "tap_thread"           ||
            sid == "rigid_tap"            ||
            sid == "form_tap"             ||
            sid == "thread_mill"          ||
            sid == "thread_mill_external" ||
            sid == "thread_chase")
        {
            return i;
        }
    }
    return -1;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.pitch_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "thread_grind pitch_mm must be > 0");
    }
    if (in.thread_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "thread_grind thread_length_mm must be > 0");
    }

    // DFM-GRIND-AMOUNT — removal must be in the precision-grind range.
    if (in.removal_mm < 0.001 || in.removal_mm > 0.05) {
        r.add("DFM-GRIND-AMOUNT", "error",
              "removal_mm " + std::to_string(in.removal_mm) +
              " outside the valid range [0.001, 0.05] mm for thread_grind");
    }

    // DFM-GRIND-MIN — thread diameter must be ≥ 5 mm.
    auto cylId = wp.resolve(in.cylinder_datum);
    if (cylId) {
        BRepAdaptor_Surface surf(wp.face(*cylId));
        if (surf.GetType() == GeomAbs_Cylinder) {
            const double dia = 2.0 * surf.Cylinder().Radius();
            if (dia < 5.0) {
                r.add("DFM-GRIND-MIN", "error",
                      "thread diameter " + std::to_string(dia) +
                      " mm < 5 mm — too small to accept a grind wheel");
            }
        } else {
            r.add("DFM-GRIND-DATUM", "warning",
                  "cylinder_datum did not resolve to a cylindrical face");
        }
    } else {
        r.add("DFM-GRIND-DATUM", "warning",
              "cylinder_datum unresolved — cannot verify thread diameter");
    }

    // DFM-GRIND-PARENT — must follow a previously-applied thread feature.
    if (findParentThreadIndex(wp) < 0) {
        r.add("DFM-GRIND-PARENT", "error",
              "thread_grind requires a pre-existing thread feature "
              "(tap_thread / rigid_tap / form_tap / thread_mill / "
              "thread_mill_external / thread_chase) in workpiece.features()");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "thread_grind DFM failed:";
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

    const int parentIdx = findParentThreadIndex(wp);

    // ─── Sub-tolerance branch ────────────────────────────────────────────
    // If `removal_mm` is below the OCCT modeling tolerance (≈ 5 µm), the
    // geometric difference is not reliably representable: leave the shape
    // unchanged, record metadata only.
    bool         geomSwept = false;
    bool         subTol    = false;
    TopoDS_Shape newShape  = wp.shape();

    if (in.removal_mm < kSubToleranceRemovalMm) {
        subTol = true;
        spdlog::debug("thread_grind: removal_mm {} < sub-tolerance threshold {}; "
                      "registering metadata only",
                      in.removal_mm, kSubToleranceRemovalMm);
    } else if (gotAxis && cylRad > 0.0) {
        // Real reduced-depth helical sweep.
        gp_Dir helixDir = cylAxis.Direction();
        if (helixDir.Z() > 0.0)
            helixDir = gp_Dir(-helixDir.X(), -helixDir.Y(), -helixDir.Z());

        const gp_Pnt& O = cylAxis.Location();
        gp_Pnt startPnt = O;
        const double zStart = (std::abs(in.start_z_mm - in.end_z_mm) > 1e-9)
            ? std::max(in.start_z_mm, in.end_z_mm)
            : zMax;
        const double bdZ = cylAxis.Direction().Z();
        if (std::abs(bdZ) > 1e-9) {
            const double t = (zStart - O.Z()) / bdZ;
            startPnt = gp_Pnt(O.X() + t * cylAxis.Direction().X(),
                              O.Y() + t * cylAxis.Direction().Y(),
                              O.Z() + t * cylAxis.Direction().Z());
        }

        const gp_Ax1 helixAxis(startPnt, helixDir);

        // Grind wheel removes a small radial amount — V-depth = removal_mm.
        const double threadDepthV = in.removal_mm;
        const double helixLength  = in.thread_length_mm;
        // External: sweep at (cylRad - removal); internal: sweep at cylRad
        // (the V tips poke into the wall by `removal_mm`).
        const double sweepRadius = in.is_external
            ? std::max(0.001, cylRad - threadDepthV)
            : cylRad;

        try {
            TopoDS_Shape grindCutter = pr::helicalSweep(
                helixAxis, in.pitch_mm, helixLength, sweepRadius, threadDepthV);
            newShape  = pr::cut(wp.shape(), grindCutter);
            geomSwept = true;
        }
        catch (const std::exception& e) {
            spdlog::warn("thread_grind: sweep/cut failed ({}); leaving geometry unchanged",
                         e.what());
            newShape  = wp.shape();
            geomSwept = false;
        }
    }

    const double turns = (in.pitch_mm > 0.0) ? (in.thread_length_mm / in.pitch_mm) : 0.0;
    const std::string geomTag = geomSwept ? "helical_swept"
                              : subTol    ? "sub_tolerance_metadata_only"
                              :             "metadata_only";

    json params = {
        { "cylinder_face_id", cylIdOut },
        { "thread_size",      in.thread_size },
        { "pitch_mm",         in.pitch_mm },
        { "thread_length_mm", in.thread_length_mm },
        { "start_z_mm",       in.start_z_mm },
        { "end_z_mm",         in.end_z_mm },
        { "removal_mm",       in.removal_mm },
        { "is_external",      in.is_external },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "thread_size",         in.thread_size },
        { "pitch_mm",            in.pitch_mm },
        { "thread_length_mm",    in.thread_length_mm },
        { "outer_diameter_mm",   cylDia },
        { "is_external",         in.is_external },
        { "turns",               turns },
        { "removal_mm",          in.removal_mm },
        { "parent_thread_id",    parentIdx },
        { "geometry",            geomTag },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "thread_grind_wheel";
    tooling.tool_dia_mm       = std::max(cylDia * 0.6, 6.0);
    tooling.tool_length_mm    = in.thread_length_mm + 10.0;
    tooling.tool_material     = "cbn";   // cubic boron nitride or vitrified Al2O3
    tooling.flute_count       = 0;       // grinding wheel
    tooling.cutting_speed_sfm = 6000.0;  // grind wheels run very fast
    tooling.feed_per_tooth_mm = 0.0;     // continuous feed, not per-tooth
    tooling.extra["removal_mm"]   = in.removal_mm;
    tooling.extra["is_external"]  = in.is_external;
    tooling.extra["finish_class"] = "precision";
    tooling.extra["parent_thread_index"] = parentIdx;
    {
        // Stock removed ≈ V-cross-section × helix-arc-length.  For grinding,
        // V-area is small — the integration is identical to thread_mill.
        const double triArea = 0.5 * in.removal_mm * (in.pitch_mm * 0.5);
        const double helLen  = (cylRad > 0.0)
            ? 2.0 * M_PI * cylRad * turns
            : 0.0;
        tooling.stock_removed_mm3 = triArea * helLen;
    }
    tooling.est_cycle_time_s  = std::max(5.0, in.thread_length_mm / 2.0);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::thread_grind applied: {} pitch={} removal={} mode={} geom={}",
                  in.thread_size, in.pitch_mm, in.removal_mm,
                  in.is_external ? "ext" : "int", geomTag);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Thread grinding leaves almost no detectable geometric signature distinct
// from a thread_mill / tap_thread (the grind removes a few microns from
// already-helical surfaces).  Recognition is metadata-driven.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 0.5;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::thread_grind
