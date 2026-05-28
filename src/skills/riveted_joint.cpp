// @lat: [[engine/skills#riveted_joint]]

#include "riveted_joint.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp_Ax2.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::riveted_joint {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.rivet_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "riveted_joint rivet_dia_mm must be > 0");
    } else if (in.rivet_dia_mm < 1.6) {
        r.add("DFM-RJ-DIA", "error",
              "riveted_joint rivet_dia_mm " + std::to_string(in.rivet_dia_mm) +
              " < 1.6 mm (smallest standard rivet body)");
    }
    if (in.head_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "riveted_joint head_dia_mm must be > 0");
    } else if (in.rivet_dia_mm > 0.0 &&
               in.head_dia_mm < in.rivet_dia_mm * 1.5) {
        r.add("DFM-RJ-HEAD-RATIO", "error",
              "riveted_joint head_dia " + std::to_string(in.head_dia_mm) +
              " < 1.5 × rivet_dia " + std::to_string(in.rivet_dia_mm) +
              " — head too small to clamp");
    }
    if (in.head_height_mm < 0.3) {
        r.add("DFM-RJ-HEAD-H", "error",
              "riveted_joint head_height_mm " + std::to_string(in.head_height_mm) +
              " < 0.3 mm minimum");
    } else if (in.head_dia_mm > 0.0 &&
               in.head_height_mm > in.head_dia_mm * 0.5) {
        r.add("DFM-RJ-HEAD-H", "error",
              "riveted_joint head_height_mm " + std::to_string(in.head_height_mm) +
              " > 0.5 × head_dia_mm — upset is over-formed");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "riveted_joint DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("riveted_joint: entry_face datum unresolved");

    // 1) Drill the through-hole.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    const double kEntryOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    gp_Pnt toolStart;
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        if (adir.Z() < 0) {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMax + kEntryOverhang);
        } else {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMin - kEntryOverhang);
        }
    } else {
        const double margin = bboxDiag + 1.0;
        toolStart = gp_Pnt(
            in.position_x_mm - adir.X() * (margin + kEntryOverhang),
            in.position_y_mm - adir.Y() * (margin + kEntryOverhang),
            (zMin + zMax) / 2.0 - adir.Z() * (margin + kEntryOverhang));
    }

    const double cutterHeight = bboxDiag + 2.0 * kEntryOverhang;
    const gp_Ax2 cutterAx(toolStart, adir);
    const TopoDS_Shape cutter =
        pr::cylinder(cutterAx, in.rivet_dia_mm / 2.0, cutterHeight);

    // 2) Build head bumps.  We need ENTRY and EXIT faces, but the only data
    //    we have are the bbox extents along axis_dir.  The entry face is at
    //    the cutter start; the exit face is at toolStart + bboxDiag * adir.
    //    For axis-aligned ±Z drilling we know the surface Z exactly; for
    //    other directions we project against the bbox.
    //
    //    Each head is a low cylinder sunk slightly into the parent surface
    //    by kSink to give the fuse a real shared volume.
    const double kSink = std::max(0.01, in.head_height_mm * 0.05);
    const double headH = in.head_height_mm + kSink;

    // Entry head — protrudes opposite to adir (outward from the entry face).
    // The entry face's outward normal is -adir.
    gp_Pnt entryFacePt;
    gp_Pnt exitFacePt;
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        // Axis-aligned Z drilling — pick surface Z exactly.
        if (adir.Z() < 0) {
            entryFacePt = gp_Pnt(in.position_x_mm, in.position_y_mm, zMax);
            exitFacePt  = gp_Pnt(in.position_x_mm, in.position_y_mm, zMin);
        } else {
            entryFacePt = gp_Pnt(in.position_x_mm, in.position_y_mm, zMin);
            exitFacePt  = gp_Pnt(in.position_x_mm, in.position_y_mm, zMax);
        }
    } else {
        // Arbitrary axis — head positions approximate to bbox boundaries.
        const double margin = bboxDiag;
        entryFacePt = gp_Pnt(
            in.position_x_mm,
            in.position_y_mm,
            (zMin + zMax) / 2.0);
        exitFacePt = gp_Pnt(
            in.position_x_mm + adir.X() * margin,
            in.position_y_mm + adir.Y() * margin,
            (zMin + zMax) / 2.0 + adir.Z() * margin);
    }

    // Entry head cylinder — built along (-adir) starting at entry face -kSink
    // inside the workpiece.
    const gp_Dir outwardEntry(-adir.X(), -adir.Y(), -adir.Z());
    const gp_Pnt entryHeadOrigin(
        entryFacePt.X() - outwardEntry.X() * kSink,
        entryFacePt.Y() - outwardEntry.Y() * kSink,
        entryFacePt.Z() - outwardEntry.Z() * kSink);
    const gp_Ax2 entryHeadAx(entryHeadOrigin, outwardEntry);
    const TopoDS_Shape entryHead =
        pr::cylinder(entryHeadAx, in.head_dia_mm / 2.0, headH);

    // Exit head — outward direction is +adir.
    const gp_Dir outwardExit = adir;
    const gp_Pnt exitHeadOrigin(
        exitFacePt.X() - outwardExit.X() * kSink,
        exitFacePt.Y() - outwardExit.Y() * kSink,
        exitFacePt.Z() - outwardExit.Z() * kSink);
    const gp_Ax2 exitHeadAx(exitHeadOrigin, outwardExit);
    const TopoDS_Shape exitHead =
        pr::cylinder(exitHeadAx, in.head_dia_mm / 2.0, headH);

    // 3) Apply Boolean ops.  Order matters: cut the hole first, then fuse
    //    each head.  Heads are larger than the bore so they overlap the
    //    workpiece annulus around the hole — fuse is safe.
    TopoDS_Shape shape = pr::cut(wp.shape(), cutter);
    shape = pr::fuseMany(shape, { entryHead, exitHead });

    // 4) Signature
    json params = {
        { "entry_face_id",  *entryId },
        { "position_x_mm",  in.position_x_mm },
        { "position_y_mm",  in.position_y_mm },
        { "axis_dir",       { adir.X(), adir.Y(), adir.Z() } },
        { "rivet_dia_mm",   in.rivet_dia_mm },
        { "head_dia_mm",    in.head_dia_mm },
        { "head_height_mm", in.head_height_mm },
        { "material",       in.material },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "cylindrical_face_count", 3 },     // bore + 2 heads
        { "circular_edge_count",    6 },     // bore (2) + head sides (2 × 2)
        { "head_disc_count",        2 },
        { "rivet_dia_mm",           in.rivet_dia_mm },
        { "head_dia_mm",            in.head_dia_mm },
        { "head_height_mm",         in.head_height_mm },
        { "axis_dir",               { adir.X(), adir.Y(), adir.Z() } },
        { "additive",               false },  // net volume change is small either way
    };
    ToolingMeta tooling;
    tooling.tool_type         = "rivet_set";
    tooling.tool_dia_mm       = in.rivet_dia_mm;
    tooling.tool_length_mm    = bboxDiag + 2.0 * in.head_height_mm;
    tooling.tool_material     = "hardened_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    // Net stock removed = hole - 2 heads (net often slightly negative for big heads).
    const double holeVol = M_PI * (in.rivet_dia_mm / 2.0) * (in.rivet_dia_mm / 2.0) * bboxDiag;
    const double headVol = M_PI * (in.head_dia_mm / 2.0) * (in.head_dia_mm / 2.0) * in.head_height_mm;
    tooling.stock_removed_mm3 = holeVol - 2.0 * headVol;
    tooling.est_cycle_time_s  = 3.0;
    tooling.extra = {
        { "fastener_type",      "rivet" },
        { "rivet_dia_mm",       in.rivet_dia_mm },
        { "head_dia_mm",        in.head_dia_mm },
        { "head_height_mm",     in.head_height_mm },
        { "assembly_step",      "drill_clearance_hole_then_set_rivet" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(shape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::riveted_joint applied: rivet={} head_d={} head_h={} faces {}→{}",
                  in.rivet_dia_mm, in.head_dia_mm, in.head_height_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// History replay only — the deformed rivet topology can be confused with a
// spot_weld bump after Boolean fuse simplification.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "position_x_mm",  f.params.value("position_x_mm",  0.0) },
            { "position_y_mm",  f.params.value("position_y_mm",  0.0) },
            { "axis_dir",       f.params.value("axis_dir",       json::array({0.0,0.0,-1.0})) },
            { "rivet_dia_mm",   f.params.value("rivet_dia_mm",   0.0) },
            { "head_dia_mm",    f.params.value("head_dia_mm",    0.0) },
            { "head_height_mm", f.params.value("head_height_mm", 0.0) },
            { "material",       f.params.value("material",       std::string("aluminum")) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 1.0, json{ { "source", "feature_history" } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::riveted_joint
