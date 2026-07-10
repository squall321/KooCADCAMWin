// @lat: [[engine/skills#counterbore_ring_pattern]]

#include "counterbore_ring_pattern.hpp"

#include "Workpiece.hpp"
#include "counterbore.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::counterbore_ring_pattern {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& /*wp*/, const Input& in)
{
    DFMReport r;
    if (in.count < 3)
        r.add("DFM-INPUT", "error",
              "counterbore_ring_pattern needs >= 3 instances (got " +
              std::to_string(in.count) + ")");
    // Mirror the composed counterbore atom's DFM-002 floor: counterbore::apply
    // hard-throws below 0.8 mm, so the ring must fail validation the same way
    // (validate/apply contract — validate() must predict every apply() throw).
    if (in.pilot_dia_mm < 0.8)
        r.add("DFM-002", "error",
              "counterbore ring pilot diameter " + std::to_string(in.pilot_dia_mm) +
              " mm < min 0.8 mm");
    if (in.pilot_dia_mm <= 0.0)
        r.add("DFM-INPUT", "error", "counterbore ring pilot_dia_mm must be > 0");
    if (in.seat_dia_mm <= in.pilot_dia_mm)
        r.add("DFM-INPUT", "error",
              "counterbore ring seat_dia_mm must exceed pilot_dia_mm");
    if (in.seat_depth_mm <= 0.0 || in.pilot_depth_mm <= in.seat_depth_mm)
        r.add("DFM-INPUT", "error",
              "counterbore ring needs 0 < seat_depth_mm < pilot_depth_mm");
    if (in.bolt_circle_dia_mm <= in.seat_dia_mm)
        r.add("DFM-INPUT", "error",
              "counterbore ring bolt_circle_dia_mm must exceed seat_dia_mm");
    if (std::abs(in.axis_dir.Z()) < 0.99)
        r.add("DFM-INPUT", "error",
              "counterbore_ring_pattern supports a vertical (±Z) axis only");

    // Seat-to-seat edge clearance along the pitch circle (the SEATS are the
    // widest cut, so they gate the density — mirrors bolt_circle's DFM-003).
    if (in.count >= 3 && in.bolt_circle_dia_mm > 0.0) {
        const double R     = in.bolt_circle_dia_mm / 2.0;
        const double chord = 2.0 * R * std::sin(M_PI / in.count);
        const double gap   = chord - in.seat_dia_mm;
        if (gap < 1.5)
            r.add("DFM-003", "warning",
                  "counterbore-ring seat-to-seat gap " + std::to_string(gap) +
                  " mm < 1.5 mm (seats too dense for the pitch circle)");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    const DFMReport r = validate(wp, in);
    if (!r.passed)
        throw SkillError("counterbore_ring_pattern::apply — invalid input (DFM error)");

    const double R = in.bolt_circle_dia_mm / 2.0;

    // Compose counterbore::apply per instance so the two-diameter cut, entry
    // resolution and the chain contract are inherited from the proven atom.
    // Keep each member's signature: the atom stamps its resolved 3-D entry
    // point (position_x/y/z_mm), which the compound re-exports for CAM.
    std::shared_ptr<Workpiece> current;
    const Workpiece* src = &wp;
    json holeCenters = json::array();   // per-member [x, y, z] entry points
    double firstEntryZ = 0.0;           // the ring's shared entry plane
    for (int i = 0; i < in.count; ++i) {
        const double ang =
            (in.start_angle_deg + i * 360.0 / in.count) * M_PI / 180.0;
        counterbore::Input cin;
        cin.entry_face     = in.entry_face;
        cin.position_x_mm  = in.center_x_mm + R * std::cos(ang);
        cin.position_y_mm  = in.center_y_mm + R * std::sin(ang);
        cin.axis_dir       = in.axis_dir;
        cin.pilot_dia_mm   = in.pilot_dia_mm;
        cin.pilot_depth_mm = in.pilot_depth_mm;
        cin.seat_dia_mm    = in.seat_dia_mm;
        cin.seat_depth_mm  = in.seat_depth_mm;
        const SkillOutput member = counterbore::apply(*src, cin);
        const json& mp = member.signature.params;
        holeCenters.push_back({ mp.value("position_x_mm", 0.0),
                                mp.value("position_y_mm", 0.0),
                                mp.value("position_z_mm", 0.0) });
        if (i == 0) firstEntryZ = mp.value("position_z_mm", 0.0);
        current = member.workpiece;
        src = current.get();
    }

    // Stamp the compound signature on top of the per-instance signatures.
    // position_z_mm (the resolved entry plane — the ring shares one flat entry
    // face, so the first member's entry Z stands for all) and hole_centers are
    // the CAM contract: after an Executor replay only THIS signature survives,
    // and downstream toolpath generation must not fall back to Z=0.
    json params = {
        { "count",               in.count },
        { "bolt_circle_dia_mm",  in.bolt_circle_dia_mm },
        { "center_x_mm",         in.center_x_mm },
        { "center_y_mm",         in.center_y_mm },
        { "position_z_mm",       firstEntryZ },
        { "axis_dir",            { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "pilot_dia_mm",        in.pilot_dia_mm },
        { "pilot_depth_mm",      in.pilot_depth_mm },
        { "seat_dia_mm",         in.seat_dia_mm },
        { "seat_depth_mm",       in.seat_depth_mm },
        { "start_angle_deg",     in.start_angle_deg },
        { "hole_centers",        holeCenters },
    };
    json pattern = {
        { "kind",        kSkillId },
        { "is_compound", true },
        { "count",       in.count },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "drill;counterbore";
    tooling.tool_dia_mm       = in.seat_dia_mm;   // largest engagement (atom convention)
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    const double pilotR = in.pilot_dia_mm / 2.0, seatR = in.seat_dia_mm / 2.0;
    tooling.stock_removed_mm3 =
        in.count * (M_PI * seatR * seatR * in.seat_depth_mm +
                    M_PI * pilotR * pilotR * (in.pilot_depth_mm - in.seat_depth_mm));

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    current->addFeature(sig);

    spdlog::debug("skill::counterbore_ring_pattern applied: {} cbores "
                  "pilot={} seat={} on PCD={}",
                  in.count, in.pilot_dia_mm, in.seat_dia_mm, in.bolt_circle_dia_mm);

    return SkillOutput{ current, sig };
}

// ── Recognition (handled by the koo_re grammar) ──────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& /*wp*/)
{
    return {};   // see re::recognizeCompounds -> matchCounterboreRings
}

}  // namespace koocadcam::skill::counterbore_ring_pattern
