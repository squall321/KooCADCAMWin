// @lat: [[engine/skills#diver_case_full_compound]]

#include "diver_case_full_compound.hpp"

#include "Workpiece.hpp"
#include "case_back_screw_down.hpp"
#include "crown_guard_shoulder_compound.hpp"
#include "crown_stem_cavity_compound.hpp"
#include "diving_bezel_unidirectional.hpp"
#include "lug_integrated_curved.hpp"
#include "sapphire_glass_seat.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::diver_case_full_compound {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    (void)wp;
    DFMReport r;

    if (in.case_dia_mm <= 0.0 || in.case_height_mm <= 0.0 ||
        in.bezel_outer_dia_mm <= 0.0 || in.lug_width_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "diver_case_full_compound: all dimensions must be > 0");
        return r;
    }
    if (in.case_dia_mm < 30.0 || in.case_dia_mm > 60.0) {
        r.add("DFM-CASE-DIA", "error",
              "diver_case_full_compound: case_dia " +
              std::to_string(in.case_dia_mm) +
              " mm outside typical [30, 60] mm wristwatch range");
    }
    if (in.case_height_mm < 8.0 || in.case_height_mm > 18.0) {
        r.add("DFM-CASE-HEIGHT", "error",
              "diver_case_full_compound: case_height " +
              std::to_string(in.case_height_mm) +
              " mm outside typical [8, 18] mm dive watch range");
    }
    if (in.bezel_outer_dia_mm > in.case_dia_mm) {
        r.add("DFM-BEZEL-DIA", "error",
              "diver_case_full_compound: bezel_outer_dia (" +
              std::to_string(in.bezel_outer_dia_mm) +
              ") must be <= case_dia");
    }
    if (in.lug_width_mm != 20.0 && in.lug_width_mm != 22.0 &&
        in.lug_width_mm != 24.0) {
        r.add("DFM-LUG-WIDTH", "error",
              "diver_case_full_compound: lug_width must be 20, 22, or 24 mm");
    }
    if (in.crown_position != "3" && in.crown_position != "4") {
        r.add("DFM-CROWN-POS", "error",
              "diver_case_full_compound: crown_position must be \"3\" or \"4\"");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Compose 6 existing skills sequentially (each apply produces a new
// Workpiece consumed by the next).  All sub-skill operations are real
// OCCT pr::cut / pr::fuse — performed inside each sub-skill.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "diver_case_full_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Crown angular position: "3" o'clock = 0°, "4" o'clock = -30° (i.e., 330°).
    const double crownAngleDeg = (in.crown_position == "3") ? 0.0 : 330.0;
    const double caseR = in.case_dia_mm / 2.0;
    const double caseH = in.case_height_mm;
    const gp_Ax1 caseAxis(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));

    // Stage 0: start from the incoming workpiece (assumed cylindrical stock).
    std::shared_ptr<Workpiece> current =
        std::make_shared<Workpiece>(wp.shape(), wp.material());

    // ── Stage 1: integrated curved lugs (4) ──────────────────────────────
    {
        lug_integrated_curved::Input lugIn;
        lugIn.case_side_face        = FaceByNormal{ gp_Dir(1, 0, 0) };
        lugIn.case_axis             = caseAxis;
        lugIn.case_outer_radius_mm  = caseR;
        lugIn.lug_count             = 4;
        lugIn.lug_angle_deg         = 80.0;
        lugIn.lug_width_mm          = in.lug_width_mm * 0.18;   // axial extent
        lugIn.lug_thickness_mm      = 2.5;
        lugIn.lug_length_mm         = 7.0;
        lugIn.spring_bar_dia_mm     = 1.5;
        auto out = lug_integrated_curved::apply(*current, lugIn);
        current = out.workpiece;
    }

    // ── Stage 2: diving bezel seat (ISO 6425, 120-click) ─────────────────
    {
        diving_bezel_unidirectional::Input bzIn;
        bzIn.case_top_face      = FaceByNormal{ gp_Dir(0, 0, 1) };
        bzIn.case_axis          = caseAxis;
        bzIn.bezel_outer_dia_mm = in.bezel_outer_dia_mm;
        bzIn.bezel_inner_dia_mm = in.bezel_outer_dia_mm - 5.0;
        bzIn.bezel_height_mm    = 1.5;
        bzIn.click_count        = 120;
        bzIn.click_depth_mm     = 0.4;
        bzIn.click_notch_dia_mm = 0.5;
        auto out = diving_bezel_unidirectional::apply(*current, bzIn);
        current = out.workpiece;
    }

    // ── Stage 3: sapphire glass seat ─────────────────────────────────────
    {
        sapphire_glass_seat::Input glIn;
        glIn.case_face          = FaceByNormal{ gp_Dir(0, 0, 1) };
        glIn.center_x_mm        = 0.0;
        glIn.center_y_mm        = 0.0;
        glIn.glass_dia_mm       = in.bezel_outer_dia_mm - 8.0;
        // Shoulder (ledge) must clear the O-ring groove: sub-skill DFM requires
        // ledge_depth > o_ring_cs (1.0) + o_ring_z_offset (0.5) + 0.4 = 1.9 mm.
        // Use 2.5 mm for a physically realistic glass seat with 0.6 mm margin.
        glIn.ledge_depth_mm     = 2.5;
        glIn.inner_bore_dia_mm  = 0.0;   // auto
        glIn.inner_bore_depth_mm = 0.0;  // auto
        glIn.o_ring_cs_mm       = 1.0;
        glIn.o_ring_depth_mm    = 0.4;
        glIn.o_ring_z_offset_mm = 0.5;
        auto out = sapphire_glass_seat::apply(*current, glIn);
        current = out.workpiece;
    }

    // ── Stage 4: crown guard shoulder ────────────────────────────────────
    {
        crown_guard_shoulder_compound::Input cgIn;
        cgIn.case_side_face         = FaceByNormal{ gp_Dir(1, 0, 0) };
        cgIn.case_axis              = caseAxis;
        cgIn.case_outer_radius_mm   = caseR;
        cgIn.guard_angle_deg        = crownAngleDeg;
        cgIn.guard_height_mm        = std::min(4.0, caseH * 0.45);
        cgIn.guard_width_mm         = 6.0;
        cgIn.guard_thickness_mm     = 2.0;
        cgIn.crown_clearance_dia_mm = 2.5;
        cgIn.crown_z_mm             = caseH / 2.0;
        auto out = crown_guard_shoulder_compound::apply(*current, cgIn);
        current = out.workpiece;
    }

    // ── Stage 5: crown stem cavity ───────────────────────────────────────
    {
        crown_stem_cavity_compound::Input csIn;
        csIn.case_axis            = caseAxis;
        csIn.case_outer_radius_mm = caseR;
        csIn.angle_deg            = crownAngleDeg;
        csIn.port_center_z_mm     = caseH / 2.0;
        csIn.cavity_dia_mm        = 3.0;
        csIn.cavity_depth_mm      = 2.5;
        csIn.stem_dia_mm          = 1.5;
        csIn.stem_total_length_mm = 8.0;
        csIn.o_ring_cs_mm         = 1.0;
        csIn.o_ring_depth_mm      = 0.4;
        csIn.o_ring_offset_mm     = 1.0;
        auto out = crown_stem_cavity_compound::apply(*current, csIn);
        current = out.workpiece;
    }

    // ── Stage 6: screw-down caseback (AS568 -116 O-ring) ─────────────────
    {
        case_back_screw_down::Input cbIn;
        cbIn.case_bottom_face        = FaceByNormal{ gp_Dir(0, 0, -1) };
        cbIn.case_axis               = caseAxis;
        cbIn.back_dia_mm             = in.case_dia_mm - 4.0;
        cbIn.thread_pitch_mm         = 0.6;
        cbIn.thread_depth_mm         = 0.5;
        cbIn.thread_band_axial_mm    = 3.5;   // > -116 CS (2.62) + 0.3 clearance (DFM-ORING-FIT)
        cbIn.o_ring_groove_position  = "external";
        cbIn.o_ring_size_key         = "-116";
        auto out = case_back_screw_down::apply(*current, cbIn);
        current = out.workpiece;
    }

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "case_dia_mm",        in.case_dia_mm },
        { "case_height_mm",     in.case_height_mm },
        { "bezel_outer_dia_mm", in.bezel_outer_dia_mm },
        { "lug_width_mm",       in.lug_width_mm },
        { "crown_position",     in.crown_position },
    };
    json pattern = {
        { "kind",               kSkillId },
        { "is_compound",        true },
        { "is_watch_assembly",  true },
        { "watch_style",        "diver" },
        { "subfeature_count",   6 },
        { "subfeatures", json::array({
            { { "op", "lug_integrated_curved" },          { "count", 4 } },
            { { "op", "diving_bezel_unidirectional" },    { "click_count", 120 } },
            { { "op", "sapphire_glass_seat" } },
            { { "op", "crown_guard_shoulder_compound" },  { "angle_deg", crownAngleDeg } },
            { { "op", "crown_stem_cavity_compound" },     { "angle_deg", crownAngleDeg } },
            { { "op", "case_back_screw_down" },           { "as568_dash", "-116" } },
        }) },
        { "case_dia_mm",         in.case_dia_mm },
        { "case_height_mm",      in.case_height_mm },
        { "bezel_outer_dia_mm",  in.bezel_outer_dia_mm },
        { "lug_width_mm",        in.lug_width_mm },
        { "crown_position",      in.crown_position },
        { "crown_angle_deg",     crownAngleDeg },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "assembly_chain";
    tooling.tool_dia_mm       = in.case_dia_mm;
    tooling.tool_length_mm    = in.case_height_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 220.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 = M_PI * caseR * caseR * caseH * 0.3;
    tooling.est_cycle_time_s  = 120.0;
    tooling.extra = {
        { "feature_standard", "ISO 6425 dive watch assembly" },
        { "sub_skill_chain", json::array({
            "lug_integrated_curved",
            "diving_bezel_unidirectional",
            "sapphire_glass_seat",
            "crown_guard_shoulder_compound",
            "crown_stem_cavity_compound",
            "case_back_screw_down",
        }) },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(current->shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::diver_case_full_compound applied: dia={} h={} bezel={} lug={} crown={}",
        in.case_dia_mm, in.case_height_mm, in.bezel_outer_dia_mm,
        in.lug_width_mm, in.crown_position);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 0.5;   // very-high-order compound: replay only
        r.matched_geometry = {
            { "source",            "metadata_replay" },
            { "is_compound",       true },
            { "is_watch_assembly", true },
            { "watch_style",       "diver" },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::diver_case_full_compound
