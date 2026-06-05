// @lat: [[engine/skills#brass_valve_port_compound]]

#include "brass_valve_port_compound.hpp"

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

#include <algorithm>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::brass_valve_port_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.valve_bore_dia_mm <= 0.0 || in.slide_port_dia_mm <= 0.0
        || in.slide_port_radial_distance_mm <= 0.0
        || in.slide_port_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "brass_valve_port_compound: positive dimensions required");
    }
    if (in.slide_port_count < 2 || in.slide_port_count > 6) {
        r.add("DFM-PORT-COUNT", "error",
              "brass_valve_port_compound: slide_port_count "
              + std::to_string(in.slide_port_count)
              + " not in [2, 6] (trumpet=3, trombone=2, tuba up to 6)");
    }
    if (in.slide_port_pitch_deg <= 0.0 || in.slide_port_pitch_deg > 180.0) {
        r.add("DFM-PITCH", "error",
              "brass_valve_port_compound: slide_port_pitch_deg must be in (0, 180]");
    }
    // Slide ports must clear the central bore: their inner edge (radial - dia/2)
    // must be > valve_bore_radius.
    const double radialInner =
        in.slide_port_radial_distance_mm - in.slide_port_dia_mm / 2.0;
    if (radialInner <= in.valve_bore_dia_mm / 2.0) {
        r.add("DFM-RADIUS", "error",
              "brass_valve_port_compound: slide port intersects central bore "
              "(radial_distance - port_dia/2 <= valve_bore_radius)");
    }
    if (!(std::abs(in.valve_axis_dir.X()) < 1e-6
          && std::abs(in.valve_axis_dir.Y()) < 1e-6
          && in.valve_axis_dir.Z() > 0)) {
        r.add("DFM-AXIS", "error",
              "brass_valve_port_compound: only valve_axis_dir = +Z is supported");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "brass_valve_port_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    constexpr double kOver = 0.1;
    const double cx = in.center_x_mm;
    const double cy = in.center_y_mm;
    const double stockH = zMax - zMin;
    const double boreR = in.valve_bore_dia_mm / 2.0;

    // ── 1) Central valve bore — full-through axial cylinder cut ──────────
    const gp_Pnt boreOrigin(cx, cy, zMin - kOver);
    const gp_Ax2 boreAx(boreOrigin, gp::DZ());
    const TopoDS_Shape boreTool = pr::cylinder(boreAx, boreR,
        stockH + 2.0 * kOver);
    TopoDS_Shape current = pr::cut(wp.shape(), boreTool);

    // ── 2..) N slide ports — each is a HORIZONTAL cylinder cut radially.
    //         Port axis lies in the XY plane, perpendicular to valve_axis.
    //         Position: theta_i = i * pitch_deg.
    const double portR  = in.slide_port_dia_mm / 2.0;
    const double radial = in.slide_port_radial_distance_mm;
    const double portZ  = (zMin + zMax) / 2.0;   // mid-height of stock

    for (int i = 0; i < in.slide_port_count; ++i) {
        const double thetaDeg = static_cast<double>(i) * in.slide_port_pitch_deg;
        const double theta = thetaDeg * M_PI / 180.0;
        const double cosT = std::cos(theta);
        const double sinT = std::sin(theta);

        // Slide port axis = outward radial direction (from centre to outside).
        const gp_Dir portDir(cosT, sinT, 0.0);

        // Start point: at radial distance + depth (so the cylinder cuts from
        // OUTSIDE inward by slide_port_depth_mm).  But OCCT cylinder extrudes
        // along axis from origin; we want a tube whose far end is at
        // (cx + radial*cosT, cy + radial*sinT, portZ) and whose near end is
        // closer to centre by slide_port_depth_mm.
        // Origin = (cx + (radial - depth)*cosT, cy + (radial - depth)*sinT, portZ)
        // Axis dir = +portDir, Length = depth + kOver (overhang outward).
        const double startR = radial - in.slide_port_depth_mm;
        const gp_Pnt portOrigin(
            cx + startR * cosT,
            cy + startR * sinT,
            portZ);
        const gp_Ax2 portAx(portOrigin, portDir);
        const TopoDS_Shape portTool = pr::cylinder(portAx, portR,
            in.slide_port_depth_mm + kOver);
        current = pr::cut(current, portTool);
    }

    // Volumes for tooling estimate.
    const double vBore = M_PI * boreR * boreR * stockH;
    const double vPort = M_PI * portR * portR * in.slide_port_depth_mm;
    const double volRemoved = vBore + in.slide_port_count * vPort;

    json params = {
        { "center_x_mm",                  cx },
        { "center_y_mm",                  cy },
        { "valve_axis_dir",               { in.valve_axis_dir.X(),
                                            in.valve_axis_dir.Y(),
                                            in.valve_axis_dir.Z() } },
        { "valve_bore_dia_mm",            in.valve_bore_dia_mm },
        { "slide_port_count",             in.slide_port_count },
        { "slide_port_dia_mm",            in.slide_port_dia_mm },
        { "slide_port_pitch_deg",         in.slide_port_pitch_deg },
        { "slide_port_radial_distance_mm", in.slide_port_radial_distance_mm },
        { "slide_port_depth_mm",          in.slide_port_depth_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "audio_feature_type",  "brass_valve_port" },
        { "subfeature_count",    1 + in.slide_port_count },
        { "valve_bore_dia_mm",   in.valve_bore_dia_mm },
        { "slide_port_count",    in.slide_port_count },
        { "slide_port_dia_mm",   in.slide_port_dia_mm },
        { "slide_port_pitch_deg", in.slide_port_pitch_deg },
        { "derived_volume_removed_mm3", volRemoved },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "drill;ream;side_mill";
    tooling.tool_dia_mm       = std::max(in.valve_bore_dia_mm, in.slide_port_dia_mm);
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 300.0;   // brass is very machinable
    tooling.feed_per_tooth_mm = 0.08;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(20.0,
        15.0 + 5.0 * static_cast<double>(in.slide_port_count));
    tooling.extra = {
        { "standard",          "brass instrument valve casing port" },
        { "instrument_family", "trumpet/trombone/tuba" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::brass_valve_port_compound: bore={} ports={}",
                  in.valve_bore_dia_mm, in.slide_port_count);

    return SkillOutput{ wpNew, sig };
}

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric: 1 axial cylinder (central bore) + N horizontal cylinders.
    int axialCyls = 0;
    int radialCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Dir d = s.Cylinder().Axis().Direction();
            if (std::abs(d.Z()) > 0.9) ++axialCyls;
            else if (std::abs(d.Z()) < 0.1) ++radialCyls;
        } catch (...) {}
    }
    if (axialCyls >= 1 && radialCyls >= 2) {
        json recovered = {
            { "valve_bore_estimate", true },
            { "slide_port_count_estimate", radialCyls },
        };
        json matched = { { "source", "geometric_brass_valve_pattern" },
                         { "axial_cyl_count",  axialCyls },
                         { "radial_cyl_count", radialCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.55, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::brass_valve_port_compound
