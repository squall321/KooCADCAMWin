// @lat: [[engine/skills#mounting_clamp_rail_slot]]

#include "mounting_clamp_rail_slot.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>
#include <string>

namespace koocadcam::skill::mounting_clamp_rail_slot {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "mounting_clamp_rail_slot: face_id out of range");
    }
    if (!(in.rail_length_mm > 0.0) || !(in.slot_top_width_mm > 0.0) ||
        !(in.slot_throat_width_mm > 0.0) || !(in.slot_depth_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "mounting_clamp_rail_slot: all dimensions must be > 0");
        return r;
    }

    if (in.slot_throat_width_mm >= in.slot_top_width_mm) {
        r.add("DFM-TSLOT-THROAT", "error",
              "mounting_clamp_rail_slot: slot_throat_width (" +
              std::to_string(in.slot_throat_width_mm) +
              ") must be < slot_top_width (" +
              std::to_string(in.slot_top_width_mm) +
              ") for a valid hammer-head T-slot");
    }

    if (in.slot_depth_mm < 4.0 || in.slot_depth_mm > 30.0) {
        r.add("DFM-TSLOT-DEPTH", "warning",
              "mounting_clamp_rail_slot: slot_depth " +
              std::to_string(in.slot_depth_mm) +
              " mm outside typical PV-rail range [4, 30] mm");
    }

    if (in.slot_top_width_mm > 25.0) {
        r.add("DFM-TSLOT-WIDE", "warning",
              "mounting_clamp_rail_slot: slot_top_width " +
              std::to_string(in.slot_top_width_mm) +
              " mm wider than typical PV-clamp rails");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "mounting_clamp_rail_slot DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const gp_Dir n     = wp.faceNormal(in.face_id);
    const double baseZ = faceC.Z();
    const gp_Dir drillDir(-n.X(), -n.Y(), -n.Z());

    constexpr double kEmbed = 0.05;

    // Rail axis runs along +X.  Cross-section in (X, Y) on the face plane,
    // depth into housing (along drillDir = -Z when face normal = +Z).
    //
    // Layout:
    //   - Upper throat: width = slot_throat_width, height = throat_h
    //     at the face mouth (top of slot).  Throat occupies the top
    //     ~40 % of slot_depth.
    //   - Lower bay: width = slot_top_width, height = bay_h, below throat.

    const double throatH  = in.slot_depth_mm * 0.40;
    const double bayH     = in.slot_depth_mm - throatH;

    // ── 1. CUT lower wider bay ───────────────────────────────────────────
    // Bay sits at Z in [baseZ - slot_depth, baseZ - throatH].
    // box(ax, dx, dy, dz): ax origin = bay's (-X, -Y, -Z) corner.
    // We use gp::DZ() so dz extends along +Z; that is OK because we place
    // origin at the BOTTOM of the bay, dz = bayH lifts up to throat.
    const gp_Pnt bayOrigin(
        in.rail_axis_origin_x_mm,
        in.rail_axis_origin_y_mm - in.slot_top_width_mm / 2.0,
        baseZ - in.slot_depth_mm - kEmbed);
    const gp_Ax2 bayAx(bayOrigin, gp::DZ());
    const TopoDS_Shape bayTool = pr::box(
        bayAx,
        in.rail_length_mm,
        in.slot_top_width_mm,
        bayH + kEmbed);
    const TopoDS_Shape s1 = pr::cut(wp.shape(), bayTool);

    // ── 2. CUT upper narrow throat ───────────────────────────────────────
    // Throat sits at Z in [baseZ - throatH, baseZ + kEmbed].
    const gp_Pnt throatOrigin(
        in.rail_axis_origin_x_mm,
        in.rail_axis_origin_y_mm - in.slot_throat_width_mm / 2.0,
        baseZ - throatH);
    const gp_Ax2 throatAx(throatOrigin, gp::DZ());
    const TopoDS_Shape throatTool = pr::box(
        throatAx,
        in.rail_length_mm,
        in.slot_throat_width_mm,
        throatH + kEmbed);
    const TopoDS_Shape newShape = pr::cut(s1, throatTool);

    (void)drillDir;  // future: support non-Z face normals; for now slot axis = Z

    // ── Volumes ──────────────────────────────────────────────────────────
    const double bayVol    = in.rail_length_mm * in.slot_top_width_mm * bayH;
    const double throatVol = in.rail_length_mm * in.slot_throat_width_mm * throatH;
    const double removedVol = bayVol + throatVol;

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "face_id",              in.face_id },
        { "rail_axis_origin_x_mm", in.rail_axis_origin_x_mm },
        { "rail_axis_origin_y_mm", in.rail_axis_origin_y_mm },
        { "rail_length_mm",        in.rail_length_mm },
        { "slot_top_width_mm",     in.slot_top_width_mm },
        { "slot_throat_width_mm",  in.slot_throat_width_mm },
        { "slot_depth_mm",         in.slot_depth_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "is_solar_feature",    true },
        { "solar_feature_type",  "t_slot_rail" },
        { "subfeature_count",    2 },
        { "slot_top_width_mm",   in.slot_top_width_mm },
        { "slot_throat_width_mm", in.slot_throat_width_mm },
        { "slot_depth_mm",       in.slot_depth_mm },
        { "rail_length_mm",      in.rail_length_mm },
        { "derived_volume_removed_mm3", removedVol },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "T_slot_cutter;end_mill";
    tooling.tool_dia_mm       = in.slot_top_width_mm;
    tooling.tool_length_mm    = in.slot_depth_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 400.0;
    tooling.feed_per_tooth_mm = 0.06;
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  = std::max(6.0, removedVol / 300.0);
    tooling.extra = {
        { "feature_standard", "Aluminium PV-clamp rail (T-slot extrusion)" },
        { "removed_volume_mm3", removedVol },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::mounting_clamp_rail_slot applied: top={} throat={} depth={} len={}",
        in.slot_top_width_mm, in.slot_throat_width_mm,
        in.slot_depth_mm, in.rail_length_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition: metadata replay ─────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = {
            { "source",             "metadata_replay" },
            { "is_compound",        true },
            { "solar_feature_type", "t_slot_rail" },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::mounting_clamp_rail_slot
