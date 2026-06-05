// @lat: [[engine/skills#mc4_connector_seat_pair]]

#include "mc4_connector_seat_pair.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::mc4_connector_seat_pair {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "mc4_connector_seat_pair: face_id out of range");
    }

    if (!(in.connector_dia_mm > 0.0) || !(in.pair_pitch_mm > 0.0) ||
        !(in.seat_depth_mm > 0.0) ||
        !(in.strain_relief_slot_w_mm > 0.0) ||
        !(in.strain_relief_slot_l_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "mc4_connector_seat_pair: all dimensions must be > 0");
        return r;
    }

    if (in.connector_dia_mm < 5.0 || in.connector_dia_mm > 12.0) {
        r.add("DFM-MC4-DIA", "error",
              "mc4_connector_seat_pair: connector_dia " +
              std::to_string(in.connector_dia_mm) +
              " mm outside MC4 family range [5, 12] mm");
    }

    if (in.pair_pitch_mm < in.connector_dia_mm + 2.0) {
        r.add("DFM-MC4-PITCH", "error",
              "mc4_connector_seat_pair: pair_pitch must exceed "
              "connector_dia by >= 2 mm (bore wall)");
    }

    // Strain-relief slot must not overlap the partner connector.
    // Slot extends radially outward of each connector by slot_l/2 + slot_w
    // tangentially.  Slot centred on connector → it must fit within
    // (pair_pitch - connector_dia)/2 clearance on one side.
    const double clearance = (in.pair_pitch_mm - in.connector_dia_mm) / 2.0;
    if (in.strain_relief_slot_l_mm * 0.5 > clearance) {
        r.add("DFM-MC4-SLOT-OVERLAP", "warning",
              "mc4_connector_seat_pair: strain-relief slot length may "
              "interfere between paired connectors");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "mc4_connector_seat_pair DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const gp_Dir n     = wp.faceNormal(in.face_id);
    const double baseZ = faceC.Z();

    // Drill direction = INTO housing (opposite outward normal).
    const gp_Dir drillDir(-n.X(), -n.Y(), -n.Z());

    const double connR  = in.connector_dia_mm / 2.0;
    constexpr double kEmbed = 0.05;
    const double cutDepth = in.seat_depth_mm + kEmbed;
    const double slotW    = in.strain_relief_slot_w_mm;
    const double slotL    = in.strain_relief_slot_l_mm;

    const double cx1 = in.pair_origin_x_mm;
    const double cy1 = in.pair_origin_y_mm;
    const double cx2 = cx1 + in.pair_pitch_mm;
    const double cy2 = cy1;

    // ── 1. CUT connector bore #1 ─────────────────────────────────────────
    const gp_Pnt bore1Origin(cx1, cy1, baseZ + kEmbed);
    const gp_Ax2 bore1Ax(bore1Origin, drillDir);
    const TopoDS_Shape bore1 = pr::cylinder(bore1Ax, connR, cutDepth);
    const TopoDS_Shape s1 = pr::cut(wp.shape(), bore1);

    // ── 2. CUT strain-relief slot #1 at connector #1 ─────────────────────
    // Slot oriented along +Y axis on the face, centred on connector.
    const gp_Pnt slot1Origin(
        cx1 - slotW / 2.0,
        cy1 - slotL / 2.0,
        baseZ + kEmbed);
    const gp_Ax2 slot1Ax(slot1Origin, drillDir);
    // box(ax, dx, dy, dz): dx along +X (slotW), dy along +Y (slotL),
    //                     dz axial = drillDir (cutDepth)
    const TopoDS_Shape slot1 = pr::box(slot1Ax, slotW, slotL, cutDepth);
    const TopoDS_Shape s2 = pr::cut(s1, slot1);

    // ── 3. CUT connector bore #2 ─────────────────────────────────────────
    const gp_Pnt bore2Origin(cx2, cy2, baseZ + kEmbed);
    const gp_Ax2 bore2Ax(bore2Origin, drillDir);
    const TopoDS_Shape bore2 = pr::cylinder(bore2Ax, connR, cutDepth);
    const TopoDS_Shape s3 = pr::cut(s2, bore2);

    // ── 4. CUT strain-relief slot #2 at connector #2 ─────────────────────
    const gp_Pnt slot2Origin(
        cx2 - slotW / 2.0,
        cy2 - slotL / 2.0,
        baseZ + kEmbed);
    const gp_Ax2 slot2Ax(slot2Origin, drillDir);
    const TopoDS_Shape slot2 = pr::box(slot2Ax, slotW, slotL, cutDepth);
    const TopoDS_Shape newShape = pr::cut(s3, slot2);

    // ── Volumes ──────────────────────────────────────────────────────────
    const double boreVol = M_PI * connR * connR * in.seat_depth_mm;
    const double slotVol = slotW * slotL * in.seat_depth_mm;
    const double removedVol = 2.0 * (boreVol + slotVol);

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "face_id",                 in.face_id },
        { "pair_origin_x_mm",        in.pair_origin_x_mm },
        { "pair_origin_y_mm",        in.pair_origin_y_mm },
        { "pair_pitch_mm",           in.pair_pitch_mm },
        { "connector_dia_mm",        in.connector_dia_mm },
        { "seat_depth_mm",           in.seat_depth_mm },
        { "strain_relief_slot_w_mm", in.strain_relief_slot_w_mm },
        { "strain_relief_slot_l_mm", in.strain_relief_slot_l_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "is_solar_feature",    true },
        { "solar_feature_type",  "mc4_connector_pair" },
        { "subfeature_count",    4 },
        { "connector_dia_mm",    in.connector_dia_mm },
        { "pair_pitch_mm",       in.pair_pitch_mm },
        { "derived_volume_removed_mm3", removedVol },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill;end_mill";
    tooling.tool_dia_mm       = in.connector_dia_mm;
    tooling.tool_length_mm    = in.seat_depth_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  = std::max(4.0, removedVol / 120.0);
    tooling.extra = {
        { "feature_standard", "MC4 PV connector pair (IEC 62852)" },
        { "removed_volume_mm3", removedVol },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::mc4_connector_seat_pair applied: connD={} pitch={}",
        in.connector_dia_mm, in.pair_pitch_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition: metadata + cylinder-pair geometric fallback ─────────────

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
            { "solar_feature_type", "mc4_connector_pair" },
        };
        out.push_back(r);
    }

    // Geometric fallback: look for exactly 2 axial cylinders of equal radius
    // in MC4 range, separated by a sensible pitch.
    if (out.empty()) {
        struct Cyl { double radius; gp_Pnt loc; };
        std::vector<Cyl> mc4Cyls;
        for (int i = 0; i < wp.faceCount(); ++i) {
            if (!wp.isFaceCylinder(i)) continue;
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Cylinder c = s.Cylinder();
            const double r = c.Radius();
            if (r >= 2.5 && r <= 6.0) {
                mc4Cyls.push_back({ r, c.Axis().Location() });
            }
        }
        if (mc4Cyls.size() == 2) {
            const double pitch = std::hypot(
                mc4Cyls[0].loc.X() - mc4Cyls[1].loc.X(),
                mc4Cyls[0].loc.Y() - mc4Cyls[1].loc.Y());
            const double rAvg = 0.5 * (mc4Cyls[0].radius + mc4Cyls[1].radius);
            json recovered = {
                { "connector_dia_mm", 2.0 * rAvg },
                { "pair_pitch_mm",    pitch },
            };
            json matched = {
                { "source",            "geometric_fallback" },
                { "pair_count",        2 },
                { "observed_radius_mm", rAvg },
            };
            out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
        }
    }
    return out;
}

}  // namespace koocadcam::skill::mc4_connector_seat_pair
