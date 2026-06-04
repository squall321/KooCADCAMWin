// @lat: [[engine/skills#valve_seat_insert_compound]]

#include "valve_seat_insert_compound.hpp"

#include "Workpiece.hpp"
#include "_iso286_fits.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::valve_seat_insert_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    if (s.IsNull()) return 0.0;
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "valve_seat_insert_compound: face_id out of range");
        return r;
    }
    if (!(in.valve_head_dia_mm > 0.0) ||
        !(in.seat_press_fit_dia_mm > 0.0) ||
        !(in.seat_depth_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "valve_seat_insert_compound: all dimensions must be > 0");
        return r;
    }
    if (in.valve_head_dia_mm < 20.0 || in.valve_head_dia_mm > 60.0) {
        r.add("DFM-VALVE-HEAD-DIA", "error",
              "valve_seat_insert_compound: valve_head_dia_mm " +
              std::to_string(in.valve_head_dia_mm) +
              " mm outside [20, 60] mm (SAE J intake/exhaust valve range)");
    }
    if (in.seat_press_fit_dia_mm <= in.valve_head_dia_mm) {
        r.add("DFM-SEAT-OD", "error",
              "valve_seat_insert_compound: seat_press_fit_dia_mm (" +
              std::to_string(in.seat_press_fit_dia_mm) +
              ") must be > valve_head_dia_mm (" +
              std::to_string(in.valve_head_dia_mm) + ")");
    }
    if (in.top_angle_deg    <= 0.0 || in.top_angle_deg    >= 90.0 ||
        in.seat_angle_deg   <= 0.0 || in.seat_angle_deg   >= 90.0 ||
        in.throat_angle_deg <= 0.0 || in.throat_angle_deg >= 90.0) {
        r.add("DFM-SEAT-ANGLE", "error",
              "valve_seat_insert_compound: all seat angles must be in (0, 90) deg");
    }
    if (in.seat_depth_mm > 0.6 * in.seat_press_fit_dia_mm) {
        r.add("DFM-SEAT-DEPTH", "warning",
              "valve_seat_insert_compound: seat_depth/seat_dia ratio > 0.6 — "
              "unusually deep insert pocket");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "valve_seat_insert_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const gp_Dir n     = wp.faceNormal(in.face_id);
    const double baseZ = faceC.Z();

    // ── P7 press fit bore (HOLE basis). seat_press_fit_dia_mm is the
    // nominal stated by the caller; the actual bore is in [p7_min, p7_max].
    // We machine to the nominal (mid-band would be more precise but the
    // pattern records both bounds for downstream inspection).
    const double p7Max = iso286::p7_max_mm(in.seat_press_fit_dia_mm);
    const double p7Min = iso286::p7_min_mm(in.seat_press_fit_dia_mm);

    const double seatR    = in.seat_press_fit_dia_mm / 2.0;
    const double valveR   = in.valve_head_dia_mm     / 2.0;
    constexpr double kEmbed = 0.05;

    // Drill DOWN into the face: tool axis points opposite of outward normal.
    const gp_Dir drillDir(-n.X(), -n.Y(), -n.Z());

    // ── Sub-feature 1: cylindrical pocket for the insert (P7 bore) ───────
    const gp_Pnt pocketOrigin(in.center_x_mm, in.center_y_mm, baseZ + kEmbed);
    const gp_Ax2 pocketAx(pocketOrigin, drillDir);
    const TopoDS_Shape pocket =
        pr::cylinder(pocketAx, seatR, in.seat_depth_mm + kEmbed);

    // ── Sub-features 2/3/4: three angled conical reliefs (cone frusta) ───
    // The three cones STACK along the bore axis from the entry face downward:
    //   top cone:    radius seatR -> valveR   over h_top   (steepest near 30deg)
    //   seat cone:   radius valveR -> valveR*0.9 thin over h_seat (45 deg face)
    //   throat cone: radius valveR*0.9 -> valveR*0.7 over h_throat (opens out)
    // We pick small heights proportional to seat angles so that the apply
    // changes geometry visibly while staying inside the pocket.
    //
    // h = (delta_r) / tan(angle).  Tighter angles cut shallower for the same
    // radial drop.  We bound h to a fraction of seat_depth so cones stay inside
    // the press-fit pocket.
    auto safeTan = [](double deg) {
        const double rad = deg * M_PI / 180.0;
        return std::max(std::tan(rad), 0.05);
    };

    const double topH    = std::min(in.seat_depth_mm * 0.35,
                                    (seatR - valveR) / safeTan(in.top_angle_deg));
    const double seatH   = std::min(in.seat_depth_mm * 0.20,
                                    (valveR * 0.1) / safeTan(in.seat_angle_deg));
    const double throatH = std::min(in.seat_depth_mm * 0.30,
                                    (valveR * 0.2) / safeTan(in.throat_angle_deg));

    // Stack cones starting at the entry face going DOWN.
    // Cone frustum needs axis at base + a height vector.
    const gp_Pnt topConeOrigin   (in.center_x_mm, in.center_y_mm, baseZ + kEmbed);
    const gp_Ax2 topConeAx       (topConeOrigin, drillDir);
    const TopoDS_Shape topCone   =
        pr::coneFrustum(topConeAx, seatR, valveR + 0.5, std::max(topH, 0.5));

    const gp_Pnt seatConeOrigin  (in.center_x_mm, in.center_y_mm,
                                  baseZ - topH);
    const gp_Ax2 seatConeAx      (seatConeOrigin, drillDir);
    const TopoDS_Shape seatCone  =
        pr::coneFrustum(seatConeAx, valveR + 0.5, valveR * 0.9,
                        std::max(seatH, 0.3));

    const gp_Pnt throatOrigin    (in.center_x_mm, in.center_y_mm,
                                  baseZ - topH - seatH);
    const gp_Ax2 throatAx        (throatOrigin, drillDir);
    const TopoDS_Shape throatCone =
        pr::coneFrustum(throatAx, valveR * 0.9, valveR * 0.7,
                        std::max(throatH, 0.5));

    // Fuse cutters into one compound and cut once.
    const TopoDS_Shape pocketAndTop   = pr::fuse(pocket, topCone);
    const TopoDS_Shape plusSeat       = pr::fuse(pocketAndTop, seatCone);
    const TopoDS_Shape fusedAll       = pr::fuse(plusSeat, throatCone);

    const double volBefore = volumeOf(wp.shape());
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fusedAll);
    const double volAfter  = volumeOf(newShape);
    const double removedVol = volBefore - volAfter;

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "face_id",              in.face_id },
        { "center_x_mm",          in.center_x_mm },
        { "center_y_mm",          in.center_y_mm },
        { "valve_head_dia_mm",    in.valve_head_dia_mm },
        { "seat_press_fit_dia_mm", in.seat_press_fit_dia_mm },
        { "seat_depth_mm",        in.seat_depth_mm },
        { "top_angle_deg",        in.top_angle_deg },
        { "seat_angle_deg",       in.seat_angle_deg },
        { "throat_angle_deg",     in.throat_angle_deg },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "engine_feature_type",        "valve_seat_insert" },
        { "subfeature_count",           4 },
        { "valve_head_dia_mm",          in.valve_head_dia_mm },
        { "seat_press_fit_dia_nom_mm",  in.seat_press_fit_dia_mm },
        { "seat_press_fit_p7_max_mm",   p7Max },
        { "seat_press_fit_p7_min_mm",   p7Min },
        { "seat_depth_mm",              in.seat_depth_mm },
        { "top_angle_deg",              in.top_angle_deg },
        { "seat_angle_deg",             in.seat_angle_deg },
        { "throat_angle_deg",           in.throat_angle_deg },
        { "derived_volume_removed_mm3", removedVol },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "valve_seat_cutter_3angle";
    tooling.tool_dia_mm       = in.seat_press_fit_dia_mm;
    tooling.tool_length_mm    = in.seat_depth_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 6;
    tooling.cutting_speed_sfm = 350.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  = std::max(8.0, in.seat_depth_mm * 0.6 + 5.0);
    tooling.extra = {
        { "feature_standard",  "SAE J cylinder head valve seat insert" },
        { "press_fit",         "ISO 286 P7 hole-basis interference" },
        { "p7_bore_max_mm",    p7Max },
        { "p7_bore_min_mm",    p7Min },
        { "angles_deg",        { in.top_angle_deg, in.seat_angle_deg, in.throat_angle_deg } },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::valve_seat_insert_compound applied: "
                  "valve={}mm seat_OD={}mm depth={}mm",
                  in.valve_head_dia_mm, in.seat_press_fit_dia_mm,
                  in.seat_depth_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition — metadata replay ────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        out.push_back(RecognizedFeature{
            kSkillId, f.params, 1.0,
            { { "source",               "metadata_replay" },
              { "is_compound",          true },
              { "engine_feature_type",  "valve_seat_insert" },
              { "subfeature_count",     4 } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::valve_seat_insert_compound
