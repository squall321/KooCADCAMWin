// @lat: [[engine/skills#countersink_ring_pattern]]

#include "countersink_ring_pattern.hpp"

#include "Workpiece.hpp"
#include "countersink.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::countersink_ring_pattern {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& /*wp*/, const Input& in)
{
    DFMReport r;
    if (in.count < 3)
        r.add("DFM-INPUT", "error",
              "countersink_ring_pattern needs >= 3 instances (got " +
              std::to_string(in.count) + ")");
    // Mirror EVERY error gate of the composed countersink atom: countersink::
    // apply hard-throws on any of them, so the ring must fail validation the
    // same way (validate/apply contract — validate() must predict every
    // apply() throw).
    if (in.pilot_dia_mm < 0.8)
        r.add("DFM-002", "error",
              "countersink ring pilot diameter " + std::to_string(in.pilot_dia_mm) +
              " mm < min 0.8 mm");
    if (in.pilot_dia_mm <= 0.0)
        r.add("DFM-INPUT", "error", "countersink ring pilot_dia_mm must be > 0");
    if (in.pilot_depth_mm <= 0.0)
        r.add("DFM-INPUT", "error", "countersink ring pilot_depth_mm must be > 0");
    if (in.cone_top_dia_mm <= in.pilot_dia_mm)
        r.add("DFM-COUNTERSINK-GEOM", "error",
              "countersink ring cone_top_dia_mm must exceed pilot_dia_mm");
    if (in.cone_angle_deg < 45.0 || in.cone_angle_deg > 120.0)
        r.add("DFM-COUNTERSINK-ANGLE", "error",
              "countersink ring cone angle " + std::to_string(in.cone_angle_deg) +
              " deg outside ISO 7721 / ASME B18.6.3 / ISO 13715 envelope [45, 120]");
    // Atom gate: the cone-pilot junction must sit ABOVE the pilot bottom
    // (else the feature degenerates).  Reuse the atom's own derivation.
    {
        countersink::Input atom;
        atom.pilot_dia_mm    = in.pilot_dia_mm;
        atom.pilot_depth_mm  = in.pilot_depth_mm;
        atom.cone_top_dia_mm = in.cone_top_dia_mm;
        atom.cone_angle_deg  = in.cone_angle_deg;
        const double cd = countersink::computeConeDepth(atom);
        if (cd > 0.0 && cd >= in.pilot_depth_mm)
            r.add("DFM-COUNTERSINK-GEOM", "error",
                  "countersink ring cone depth " + std::to_string(cd) +
                  " >= pilot depth " + std::to_string(in.pilot_depth_mm) +
                  " — pilot must extend past the cone");
    }
    if (in.bolt_circle_dia_mm <= in.cone_top_dia_mm)
        r.add("DFM-INPUT", "error",
              "countersink ring bolt_circle_dia_mm must exceed cone_top_dia_mm");
    if (std::abs(in.axis_dir.Z()) < 0.99)
        r.add("DFM-INPUT", "error",
              "countersink_ring_pattern supports a vertical (±Z) axis only");

    // Cone-to-cone edge clearance along the pitch circle (the CONES are the
    // widest cut, so they gate the density — mirrors counterbore_ring's DFM-003).
    if (in.count >= 3 && in.bolt_circle_dia_mm > 0.0) {
        const double R     = in.bolt_circle_dia_mm / 2.0;
        const double chord = 2.0 * R * std::sin(M_PI / in.count);
        const double gap   = chord - in.cone_top_dia_mm;
        if (gap < 1.5)
            r.add("DFM-003", "warning",
                  "countersink-ring cone-to-cone gap " + std::to_string(gap) +
                  " mm < 1.5 mm (cones too dense for the pitch circle)");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    const DFMReport r = validate(wp, in);
    if (!r.passed)
        throw SkillError("countersink_ring_pattern::apply — invalid input (DFM error)");

    const double R = in.bolt_circle_dia_mm / 2.0;

    // Compose countersink::apply per instance so the pilot + cone cut, entry
    // resolution and the chain contract are inherited from the proven atom.
    // Keep each member's signature: the atom stamps its resolved 3-D entry
    // point (position_x/y/z_mm), which the compound re-exports for CAM.
    std::shared_ptr<Workpiece> current;
    const Workpiece* src = &wp;
    json holeCenters = json::array();   // per-member [x, y, z] entry points
    double firstEntryZ    = 0.0;        // the ring's shared entry plane
    double firstConeDepth = 0.0;        // atom-derived (identical members)
    for (int i = 0; i < in.count; ++i) {
        const double ang =
            (in.start_angle_deg + i * 360.0 / in.count) * M_PI / 180.0;
        countersink::Input cin;
        cin.entry_face      = in.entry_face;
        cin.position_x_mm   = in.center_x_mm + R * std::cos(ang);
        cin.position_y_mm   = in.center_y_mm + R * std::sin(ang);
        cin.axis_dir        = in.axis_dir;
        cin.pilot_dia_mm    = in.pilot_dia_mm;
        cin.pilot_depth_mm  = in.pilot_depth_mm;
        cin.cone_top_dia_mm = in.cone_top_dia_mm;
        cin.cone_angle_deg  = in.cone_angle_deg;
        const SkillOutput member = countersink::apply(*src, cin);
        const json& mp = member.signature.params;
        holeCenters.push_back({ mp.value("position_x_mm", 0.0),
                                mp.value("position_y_mm", 0.0),
                                mp.value("position_z_mm", 0.0) });
        if (i == 0) {
            firstEntryZ    = mp.value("position_z_mm", 0.0);
            firstConeDepth = mp.value("cone_depth_mm", 0.0);
        }
        current = member.workpiece;
        src = current.get();
    }

    // Stamp the compound signature on top of the per-instance signatures.
    // position_z_mm (the resolved entry plane — the ring shares one flat entry
    // face, so the first member's entry Z stands for all), cone_depth_mm (the
    // atom-derived chamfer depth, identical across members) and hole_centers
    // are the CAM contract: after an Executor replay only THIS signature
    // survives, and downstream toolpath generation must not fall back to Z=0.
    json params = {
        { "count",               in.count },
        { "bolt_circle_dia_mm",  in.bolt_circle_dia_mm },
        { "center_x_mm",         in.center_x_mm },
        { "center_y_mm",         in.center_y_mm },
        { "position_z_mm",       firstEntryZ },
        { "axis_dir",            { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "pilot_dia_mm",        in.pilot_dia_mm },
        { "pilot_depth_mm",      in.pilot_depth_mm },
        { "cone_top_dia_mm",     in.cone_top_dia_mm },
        { "cone_angle_deg",      in.cone_angle_deg },
        { "cone_depth_mm",       firstConeDepth },
        { "start_angle_deg",     in.start_angle_deg },
        { "hole_centers",        holeCenters },
    };
    json pattern = {
        { "kind",        kSkillId },
        { "is_compound", true },
        { "count",       in.count },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "drill;countersink";
    tooling.tool_dia_mm       = in.cone_top_dia_mm;   // largest engagement (atom convention)
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    // Per instance (the atom's own formula, regrouped): pilot cylinder below
    // the cone + the cone frustum — frustum volume = (π h / 3)(R² + Rr + r²).
    const double pilotR   = in.pilot_dia_mm    / 2.0;
    const double coneTopR = in.cone_top_dia_mm / 2.0;
    const double frustVol = (M_PI * firstConeDepth / 3.0) *
                            (coneTopR * coneTopR + coneTopR * pilotR + pilotR * pilotR);
    tooling.stock_removed_mm3 =
        in.count * (M_PI * pilotR * pilotR * (in.pilot_depth_mm - firstConeDepth) +
                    frustVol);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    current->addFeature(sig);

    spdlog::debug("skill::countersink_ring_pattern applied: {} csinks "
                  "pilot={} cone={} on PCD={}",
                  in.count, in.pilot_dia_mm, in.cone_top_dia_mm, in.bolt_circle_dia_mm);

    return SkillOutput{ current, sig };
}

// ── Recognition (handled by the koo_re grammar) ──────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& /*wp*/)
{
    return {};   // see re::recognizeCompounds -> matchCountersinkRings
}

}  // namespace koocadcam::skill::countersink_ring_pattern
