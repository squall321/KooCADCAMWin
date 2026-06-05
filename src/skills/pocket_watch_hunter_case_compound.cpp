// @lat: [[engine/skills#pocket_watch_hunter_case_compound]]

#include "pocket_watch_hunter_case_compound.hpp"

#include "Workpiece.hpp"
#include "case_back_screw_down.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"
#include "sapphire_glass_seat.hpp"

#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::pocket_watch_hunter_case_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    (void)wp;
    DFMReport r;

    if (in.case_dia_mm <= 0.0 || in.case_thickness_mm <= 0.0 ||
        in.bow_attachment_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "pocket_watch_hunter_case_compound: dimensions must be > 0");
        return r;
    }
    if (in.case_dia_mm < 40.0 || in.case_dia_mm > 60.0) {
        r.add("DFM-CASE-DIA", "error",
              "pocket_watch_hunter_case_compound: case_dia " +
              std::to_string(in.case_dia_mm) +
              " mm outside pocket watch range [40, 60] mm");
    }
    if (in.case_thickness_mm < 8.0 || in.case_thickness_mm > 20.0) {
        r.add("DFM-CASE-THICK", "error",
              "pocket_watch_hunter_case_compound: case_thickness " +
              std::to_string(in.case_thickness_mm) +
              " mm outside [8, 20] mm");
    }
    if (in.hinge_position != "12_oclock") {
        r.add("DFM-HINGE-POS", "error",
              "pocket_watch_hunter_case_compound: hinge_position must be \"12_oclock\"");
    }
    if (in.bow_attachment_dia_mm < 4.0 || in.bow_attachment_dia_mm > 12.0) {
        r.add("DFM-BOW-DIA", "error",
              "pocket_watch_hunter_case_compound: bow_attachment_dia " +
              std::to_string(in.bow_attachment_dia_mm) +
              " mm outside [4, 12] mm");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "pocket_watch_hunter_case_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double caseR = in.case_dia_mm / 2.0;
    const double caseT = in.case_thickness_mm;
    const gp_Ax1 caseAxis(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    (void)topZ;
    constexpr double kOverhang = 0.05;

    std::shared_ptr<Workpiece> current =
        std::make_shared<Workpiece>(wp.shape(), wp.material());

    // ── Stage 2: hinge groove at 12 o'clock + hinge pin hole ─────────────
    {
        // 12 o'clock = +Y direction (angle 90°).
        const gp_Dir radialOut(0.0, 1.0, 0.0);
        const gp_Dir radialIn(0.0, -1.0, 0.0);
        // Groove: small rectangular slot on the case OD at 12 o'clock.
        const double grooveW = 4.0;     // tangential
        const double grooveAxial = 2.0; // axial extent
        const double grooveDepth = 1.5; // radial depth
        const gp_Pnt grooveOrigin(
            -grooveW / 2.0,
            caseR - grooveDepth + 0.05,
            caseT / 2.0 - grooveAxial / 2.0);
        const gp_Ax2 grooveAx(grooveOrigin, gp_Dir(0, 0, 1));
        const TopoDS_Shape hingeGroove = pr::box(
            grooveAx, grooveW, grooveDepth + kOverhang, grooveAxial);

        // Hinge pin hole through groove tangentially (X axis).
        const gp_Pnt pinStart(-grooveW / 2.0 - 0.5, caseR - 0.5, caseT / 2.0);
        const gp_Ax2 pinAx(pinStart, gp_Dir(1, 0, 0));
        const TopoDS_Shape hingePin = pr::cylinder(pinAx, 0.6, grooveW + 1.0);

        std::vector<TopoDS_Shape> cutters = { hingeGroove, hingePin };
        const auto newShape = pr::cutMany(current->shape(), cutters);
        current = std::make_shared<Workpiece>(newShape, wp.material());
    }

    // ── Stage 3: bow ring attachment at top ──────────────────────────────
    {
        // Fuse a small cylindrical lug at +Y, +Z (top of case).
        const double bowR = in.bow_attachment_dia_mm / 2.0;
        const double bowExtend = 5.0;   // radial protrusion past case OD
        // Position lug centered axially at top-Y.
        const gp_Pnt bowCenter(0.0, caseR + 0.05, caseT / 2.0);
        const gp_Ax2 bowAx(bowCenter, gp_Dir(0, 1, 0));
        const TopoDS_Shape bowLug = pr::cylinder(bowAx, bowR, bowExtend);
        const TopoDS_Shape fused = pr::fuse(current->shape(), bowLug);

        // Drill a through-hole for the chain pin axially through lug (Z axis).
        const gp_Pnt pinStart(0.0, caseR + bowExtend * 0.5, -kOverhang);
        const gp_Ax2 pinAx(pinStart, gp_Dir(0, 0, 1));
        const TopoDS_Shape pinHole = pr::cylinder(
            pinAx, bowR * 0.45, caseT + 2.0 * kOverhang);
        const TopoDS_Shape newShape = pr::cut(fused, pinHole);
        current = std::make_shared<Workpiece>(newShape, wp.material());
    }

    // ── Stage 4: sapphire glass seat (crystal pocket) ────────────────────
    {
        sapphire_glass_seat::Input glIn;
        glIn.case_face          = FaceByNormal{ gp_Dir(0, 0, 1) };
        glIn.center_x_mm        = 0.0;
        glIn.center_y_mm        = 0.0;
        glIn.glass_dia_mm       = in.case_dia_mm - 6.0;
        // Shoulder height must clear the O-ring groove:
        //   groove_width(cs=1.0) + z_offset(0.5) + 0.4 margin = 1.9 mm.
        // Use 2.3 mm for a 0.4 mm DFM margin (still « case_thickness-1).
        glIn.ledge_depth_mm     = 2.3;
        glIn.inner_bore_dia_mm  = 0.0;   // auto
        glIn.inner_bore_depth_mm = 0.0;  // auto
        glIn.o_ring_cs_mm       = 1.0;
        glIn.o_ring_depth_mm    = 0.4;
        glIn.o_ring_z_offset_mm = 0.5;
        auto out = sapphire_glass_seat::apply(*current, glIn);
        current = out.workpiece;
    }

    // ── Stage 5: 3 movement mounting holes ───────────────────────────────
    {
        const double mountR = 0.7;
        const double mountCircleR = caseR * 0.35;
        std::vector<TopoDS_Shape> mountHoles;
        mountHoles.reserve(3);
        for (int i = 0; i < 3; ++i) {
            const double a = (i * 120.0 + 30.0) * M_PI / 180.0;
            const gp_Pnt p(mountCircleR * std::cos(a),
                           mountCircleR * std::sin(a),
                           zMin - kOverhang);
            const gp_Ax2 ax(p, gp_Dir(0, 0, 1));
            mountHoles.push_back(pr::cylinder(ax, mountR, caseT * 0.4));
        }
        const auto newShape = pr::cutMany(current->shape(), mountHoles);
        current = std::make_shared<Workpiece>(newShape, wp.material());
    }

    // ── Stage 6: caseback threaded retainer ──────────────────────────────
    {
        case_back_screw_down::Input cbIn;
        cbIn.case_bottom_face       = FaceByNormal{ gp_Dir(0, 0, -1) };
        cbIn.case_axis              = caseAxis;
        cbIn.back_dia_mm            = in.case_dia_mm - 4.0;
        cbIn.thread_pitch_mm        = 0.6;
        cbIn.thread_depth_mm        = 1.1;     // flat groove: (depth+1.0) - groove_w/2 >= 0.5 (DFM-ORING-CLEAR)
        cbIn.thread_band_axial_mm   = 1.8;
        cbIn.o_ring_groove_position = "flat";
        cbIn.o_ring_size_key        = "-016";  // narrower groove fits flat-caseback edge clearance
        auto out = case_back_screw_down::apply(*current, cbIn);
        current = out.workpiece;
    }

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "case_dia_mm",           in.case_dia_mm },
        { "case_thickness_mm",     in.case_thickness_mm },
        { "hinge_position",        in.hinge_position },
        { "bow_attachment_dia_mm", in.bow_attachment_dia_mm },
    };
    json pattern = {
        { "kind",               kSkillId },
        { "is_compound",        true },
        { "is_watch_assembly",  true },
        { "watch_style",        "pocket_hunter" },
        { "subfeature_count",   6 },
        { "subfeatures", json::array({
            { { "op", "hinge_groove_with_pin" },        { "angle_deg", 90.0 } },
            { { "op", "bow_ring_attachment" },          { "dia_mm", in.bow_attachment_dia_mm } },
            { { "op", "sapphire_glass_seat" } },
            { { "op", "movement_mounting_holes" },      { "count", 3 } },
            { { "op", "case_back_screw_down" },         { "as568_dash", "-016" } },
        }) },
        { "case_dia_mm",        in.case_dia_mm },
        { "case_thickness_mm",  in.case_thickness_mm },
        { "hinge_position",     in.hinge_position },
        { "bow_attachment_dia_mm", in.bow_attachment_dia_mm },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "assembly_chain";
    tooling.tool_dia_mm       = in.case_dia_mm;
    tooling.tool_length_mm    = in.case_thickness_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 = M_PI * caseR * caseR * caseT * 0.2;
    tooling.est_cycle_time_s  = 150.0;
    tooling.extra = {
        { "feature_standard", "Hunter-style pocket watch (1860-1920s classic)" },
        { "sub_skill_chain", json::array({
            "hinge_groove_with_pin",
            "bow_ring_attachment",
            "sapphire_glass_seat",
            "movement_mounting_holes",
            "case_back_screw_down",
        }) },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(current->shape(), wp.material());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::pocket_watch_hunter_case_compound applied: dia={} t={} hinge={} bow={}",
        in.case_dia_mm, in.case_thickness_mm, in.hinge_position,
        in.bow_attachment_dia_mm);

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
        r.confidence       = 0.5;
        r.matched_geometry = {
            { "source",            "metadata_replay" },
            { "is_compound",       true },
            { "is_watch_assembly", true },
            { "watch_style",       "pocket_hunter" },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::pocket_watch_hunter_case_compound
