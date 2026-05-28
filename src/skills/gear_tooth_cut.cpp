// @lat: [[engine/skills#gear_tooth_cut]]

#include "gear_tooth_cut.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::gear_tooth_cut {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.module_mm < 0.2 || in.module_mm > 50.0) {
        r.add("DFM-GEAR-MOD", "error",
              "gear_tooth_cut module_mm " + std::to_string(in.module_mm) +
              " out of [0.2, 50.0]");
    }
    if (in.pressure_angle_deg < 14.5 || in.pressure_angle_deg > 25.0) {
        r.add("DFM-GEAR-PA", "error",
              "pressure_angle " + std::to_string(in.pressure_angle_deg) +
              " out of [14.5, 25.0] deg");
    }
    if (in.root_radius_mm <= 0.0 || in.tip_radius_mm <= in.root_radius_mm) {
        r.add("DFM-GEAR-RAD", "error",
              "root_radius must be > 0 and < tip_radius");
    }
    if (in.face_width_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "face_width_mm must be > 0");
    }
    if (in.num_teeth < 6) {
        r.add("DFM-GEAR-N", "warning",
              "num_teeth < 6 — very small gears need profile shift");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "gear_tooth_cut DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("gear_tooth_cut: entry_face datum unresolved");

    // Geometry: approximate the gap as a wedge BOX in cylindrical coords.
    //   - Tangent width = circular_pitch / 2 = π · module / 2 at the pitch radius.
    //   - Radial extent = tip_radius − root_radius (cuts from tip toward root).
    //   - Axial extent  = face_width_mm + small overhang.
    const double tangentHalf = M_PI * in.module_mm / 4.0;   // half-pitch chord
    const double radialDepth = in.tip_radius_mm - in.root_radius_mm;
    const double axialExt    = in.face_width_mm + 0.1;

    // Build the wedge box centered at angle = in.angle_rad on the gear.
    // Local frame:
    //   x_loc = radial outward    (from axis through angle_rad)
    //   y_loc = tangent CCW       (= z × x)
    //   z_loc = gear axis_dir
    const gp_Dir z_loc = in.axis_dir;
    const double cA = std::cos(in.angle_rad);
    const double sA = std::sin(in.angle_rad);
    // For axis_dir = +Z, world x_loc = (cos A, sin A, 0), y_loc = (-sin A, cos A, 0).
    // For other axes we fall back to the same construction — slice-1 assumes +Z.
    const gp_Dir x_loc(cA, sA, 0.0);
    const gp_Dir y_loc(-sA, cA, 0.0);

    // Origin: at root_radius along x_loc, offset by -tangentHalf along y_loc,
    // axial start = bbox zMin - small overhang.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double axialStartZ = (zMin + zMax) / 2.0 - axialExt / 2.0;

    const gp_Pnt origin(
        /*X*/ (in.root_radius_mm) * x_loc.X() - tangentHalf * y_loc.X(),
        /*Y*/ (in.root_radius_mm) * x_loc.Y() - tangentHalf * y_loc.Y(),
        /*Z*/ axialStartZ);

    // gp_Ax2(P, mainDir = z_loc, xDir = x_loc) → DX along x_loc (radial),
    // DY along y_loc (tangent), DZ along z_loc (axial).
    const gp_Ax2 boxAx(origin, z_loc, x_loc);
    const TopoDS_Shape wedge = pr::box(boxAx,
        radialDepth,            // DX = radial
        2.0 * tangentHalf,      // DY = tangent
        axialExt);              // DZ = axial

    const TopoDS_Shape newShape = pr::cut(wp.shape(), wedge);

    json params = {
        { "entry_face_id",       *entryId },
        { "angle_rad",           in.angle_rad },
        { "module_mm",           in.module_mm },
        { "pressure_angle_deg",  in.pressure_angle_deg },
        { "helix_angle_deg",     in.helix_angle_deg },
        { "num_teeth",           in.num_teeth },
        { "root_radius_mm",      in.root_radius_mm },
        { "tip_radius_mm",       in.tip_radius_mm },
        { "face_width_mm",       in.face_width_mm },
        { "axis_dir",            { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "flank_planar_count",   2 },
        { "root_planar_face",     true },
        { "module_mm",            in.module_mm },
        { "pressure_angle_deg",   in.pressure_angle_deg },
        { "helix_angle_deg",      in.helix_angle_deg },
        { "num_teeth",            in.num_teeth },
        { "root_radius_mm",       in.root_radius_mm },
        { "tip_radius_mm",        in.tip_radius_mm },
        { "approximation",        "wedge_box" },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "gear_hob";
    tooling.tool_dia_mm       = in.module_mm * 5.0;        // typical hob OD
    tooling.tool_length_mm    = in.face_width_mm * 2.0 + 10.0;
    tooling.tool_material     = "hss";
    tooling.flute_count       = 12;
    tooling.cutting_speed_sfm = 100.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = (2.0 * tangentHalf) * radialDepth * in.face_width_mm;
    tooling.est_cycle_time_s  = std::max(2.0, in.face_width_mm * 1.5);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::gear_tooth_cut applied: module={} N={} faces {}→{}",
                  in.module_mm, in.num_teeth, wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Pure geometric recognition of an involute tooth gap is intractable for a
// slice-1 wedge approximation: the wedge looks topologically like a plain
// rectangular slot.  We therefore rely on the FeatureSignature metadata
// already recorded on the workpiece (params + pattern) — i.e. if a prior
// apply() left a `kSkillId` signature on the workpiece's feature history,
// we replay it as a recognized feature.  This is the same "metadata replay"
// strategy used for analytically-coarse skills throughout slice-1.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json recovered = f.params;
        json matched   = {
            { "via",    "metadata_replay" },
            { "kind",   f.pattern.value("kind", std::string()) },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.85, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::gear_tooth_cut
