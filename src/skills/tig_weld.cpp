// @lat: [[engine/skills#tig_weld]]

#include "tig_weld.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::tig_weld {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.bead_width_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "tig_weld bead_width_mm must be > 0");
    }
    if (in.bead_height_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "tig_weld bead_height_mm must be > 0");
    }
    if (in.bead_width_mm > 0.0 && in.bead_width_mm < 1.0) {
        r.add("DFM-TIG-WIDTH", "error",
              "tig_weld bead_width_mm " + std::to_string(in.bead_width_mm) +
              " < 1.0 mm (TIG arc instability at sub-mm bead widths)");
    }
    if (in.bead_width_mm > 8.0) {
        r.add("DFM-TIG-WIDTH", "error",
              "tig_weld bead_width_mm " + std::to_string(in.bead_width_mm) +
              " > 8.0 mm (single-pass TIG limit; use MIG for wider beads)");
    }
    if (in.bead_width_mm > 0.0 &&
        in.bead_height_mm > in.bead_width_mm * 0.5) {
        r.add("DFM-TIG-RATIO", "error",
              "tig_weld bead_height " + std::to_string(in.bead_height_mm) +
              " > 0.5 × bead_width " + std::to_string(in.bead_width_mm) +
              " — multi-pass build-up, not a single TIG pass");
    }
    const double seamLen = std::hypot(in.end_x_mm - in.start_x_mm,
                                      in.end_y_mm - in.start_y_mm);
    if (seamLen <= 1e-6) {
        r.add("DFM-INPUT", "error", "tig_weld start and end must differ");
    }
    if (in.bead_width_mm > 0.0 && seamLen > 0.0 && seamLen <= in.bead_width_mm) {
        r.add("DFM-TIG-LEN", "warning",
              "tig_weld length " + std::to_string(seamLen) +
              " ≤ bead_width " + std::to_string(in.bead_width_mm) +
              " — consider spot tack instead");
    }
    // Gas/material compatibility — INFORMATIONAL.
    if (in.gas != "argon" && in.gas != "argon_helium") {
        r.add("DFM-TIG-GAS", "warning",
              "tig_weld unknown gas '" + in.gas +
              "' (supported: 'argon', 'argon_helium')");
    }
    if (in.material == "aluminum" && in.gas == "argon" &&
        in.bead_width_mm > 5.0) {
        r.add("DFM-TIG-GAS", "info",
              "tig_weld aluminum bead > 5 mm — argon_helium recommended for "
              "better penetration");
    }
    if (in.material != "steel" && in.material != "stainless" &&
        in.material != "aluminum") {
        r.add("DFM-TIG-MAT", "info",
              "tig_weld material '" + in.material +
              "' not in {steel, stainless, aluminum} — verify weldability");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "tig_weld DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("tig_weld: entry_face datum unresolved");
    if (!wp.isFacePlanar(*entryId))
        throw SkillError("tig_weld: entry_face must be planar");

    const gp_Dir outNorm = wp.faceNormal(*entryId);
    const gp_Pnt faceCtr = wp.faceCenter(*entryId);

    const double radius = in.bead_width_mm / 2.0;
    const double kSink  = in.bead_height_mm * 0.01;
    const double beadH  = in.bead_height_mm + kSink;
    const double baseZ  = faceCtr.Z();

    // Stadium bead = cylinder_start + box + cylinder_end (additive).
    const gp_Pnt sBase(in.start_x_mm - outNorm.X() * kSink,
                       in.start_y_mm - outNorm.Y() * kSink,
                       baseZ          - outNorm.Z() * kSink);
    const gp_Pnt eBase(in.end_x_mm   - outNorm.X() * kSink,
                       in.end_y_mm   - outNorm.Y() * kSink,
                       baseZ          - outNorm.Z() * kSink);

    const gp_Ax2 axStart(sBase, outNorm);
    const gp_Ax2 axEnd  (eBase, outNorm);
    const TopoDS_Shape cylStart = pr::cylinder(axStart, radius, beadH);
    const TopoDS_Shape cylEnd   = pr::cylinder(axEnd,   radius, beadH);

    const gp_Vec dir2D(in.end_x_mm - in.start_x_mm,
                       in.end_y_mm - in.start_y_mm,
                       0.0);
    const double seamLen = dir2D.Magnitude();
    const gp_Dir xLoc(dir2D.X() / seamLen, dir2D.Y() / seamLen, 0.0);
    const gp_Dir yLoc(-xLoc.Y(), xLoc.X(), 0.0);

    const gp_Pnt boxOrigin(
        in.start_x_mm - radius * yLoc.X() - outNorm.X() * kSink,
        in.start_y_mm - radius * yLoc.Y() - outNorm.Y() * kSink,
        baseZ          - outNorm.Z() * kSink);
    const gp_Ax2 boxAx(boxOrigin, outNorm, xLoc);
    const TopoDS_Shape boxConn = pr::box(boxAx,
                                         seamLen,
                                         in.bead_width_mm,
                                         beadH);

    const TopoDS_Shape bead = pr::fuseMany(cylStart, { cylEnd, boxConn });
    const TopoDS_Shape newShape = pr::fuse(wp.shape(), bead);

    json params = {
        { "entry_face_id",   *entryId },
        { "start_x_mm",      in.start_x_mm },
        { "start_y_mm",      in.start_y_mm },
        { "end_x_mm",        in.end_x_mm },
        { "end_y_mm",        in.end_y_mm },
        { "bead_width_mm",   in.bead_width_mm },
        { "bead_height_mm",  in.bead_height_mm },
        { "material",        in.material },
        { "gas",             in.gas },
        { "entry_face_normal", { outNorm.X(), outNorm.Y(), outNorm.Z() } },
    };
    json pattern = {
        { "kind",                    kSkillId },
        { "end_cylinder_count",      2 },
        { "long_wall_planar_count",  2 },
        { "top_planar_face",         true },
        { "bead_width_mm",           in.bead_width_mm },
        { "bead_height_mm",          in.bead_height_mm },
        { "seam_length_mm",          seamLen },
        { "material",                in.material },
        { "gas",                     in.gas },
        { "additive",                true },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "tig_torch";
    tooling.tool_dia_mm       = in.bead_width_mm;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "tungsten";   // non-consumable electrode
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    const double beadVol =
        (M_PI * radius * radius + seamLen * in.bead_width_mm) * in.bead_height_mm;
    tooling.stock_removed_mm3 = -beadVol;
    // TIG deposition rate ~ 10 mm/s — slowest of arc-welding family.
    tooling.est_cycle_time_s  = std::max(1.0, seamLen / 10.0);
    tooling.extra = {
        { "process",   "tig" },
        { "material",  in.material },
        { "gas",       in.gas },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::tig_weld applied: len={} w={} h={} gas={} faces {}→{}",
                  seamLen, in.bead_width_mm, in.bead_height_mm, in.gas,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// TIG and seam_weld leave identical geometric footprints (stadium bead).
// We rely on the feature-history metadata to disambiguate.  An anonymous
// workpiece (no features) cannot be conclusively classified as TIG from
// shape alone — returning no candidates is the correct (conservative) answer.

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

}  // namespace koocadcam::skill::tig_weld
