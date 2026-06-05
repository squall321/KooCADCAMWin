// @lat: [[engine/skills#internal_cable_port_compound]]

#include "internal_cable_port_compound.hpp"

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

namespace koocadcam::skill::internal_cable_port_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.port_dia_mm <= 0.0 || in.grommet_groove_dia_mm <= 0.0 ||
        in.grommet_groove_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "internal_cable_port_compound: all dimensions must be > 0");
        return r;
    }

    if (in.entry_angle_deg < 0.0 || in.entry_angle_deg > 60.0) {
        r.add("DFM-ANGLE", "error",
              "internal_cable_port_compound: entry_angle_deg " +
              std::to_string(in.entry_angle_deg) +
              " out of [0, 60] (steep ports tear out)");
    }

    if (in.grommet_groove_dia_mm <= in.port_dia_mm) {
        r.add("DFM-GROOVE", "error",
              "internal_cable_port_compound: grommet_groove_dia_mm " +
              std::to_string(in.grommet_groove_dia_mm) +
              " must exceed port_dia_mm " + std::to_string(in.port_dia_mm));
    }

    // Grommet groove must fit inside the stock around the port mouth.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double grooveR = in.grommet_groove_dia_mm / 2.0;
    const double cx = in.center_xy.X();
    const double cy = in.center_xy.Y();
    if (cx - grooveR < xMin - 1e-3 || cx + grooveR > xMax + 1e-3 ||
        cy - grooveR < yMin - 1e-3 || cy + grooveR > yMax + 1e-3) {
        r.add("DFM-STOCK", "error",
              "internal_cable_port_compound: grommet groove overruns the stock");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "internal_cable_port_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ  = zMax;
    const double thick = zMax - zMin;
    const double cx    = in.center_xy.X();
    const double cy    = in.center_xy.Y();

    // ── 1) Angled cable port bore (big feature, cut first) ───────────────
    // Tilt from vertical by entry_angle_deg in the XZ plane; the bore axis
    // points down-and-inward.  Length covers the slanted wall thickness plus
    // overshoot so it breaks through both surfaces.
    const double portR = in.port_dia_mm / 2.0;
    const double ang   = in.entry_angle_deg * M_PI / 180.0;
    const gp_Dir boreDir(std::sin(ang), 0.0, -std::cos(ang));
    // Slanted run length: wall thickness / cos(angle), with overshoot both ends.
    const double boreLen = thick / std::max(0.2, std::cos(ang)) + 2.0 * portR + 2.0 * kOver;
    // Start the bore axis above the top face along the (reversed) bore dir.
    const gp_Pnt boreStart(cx - kOver * boreDir.X(),
                           cy - kOver * boreDir.Y(),
                           topZ - kOver * boreDir.Z());
    const gp_Ax2 boreAx(boreStart, boreDir);
    TopoDS_Shape current = pr::cut(
        wp.shape(),
        pr::cylinder(boreAx, portR, boreLen));

    // ── 2) Grommet retention groove (annular ring around the mouth) ──────
    const double grooveOuterR = in.grommet_groove_dia_mm / 2.0;
    const double grooveInnerR = portR + (grooveOuterR - portR) * 0.4;
    const double grooveDepth  = in.grommet_groove_depth_mm;
    const gp_Pnt grooveStart(cx, cy, topZ - grooveDepth);
    const gp_Ax2 grooveAx(grooveStart, gp::DZ());
    current = pr::cut(
        current,
        pr::annularRing(grooveAx, grooveOuterR, grooveInnerR, grooveDepth + kOver));

    // ── Derived volume (analytic) ────────────────────────────────────────
    const double vPort = M_PI * portR * portR * (thick / std::max(0.2, std::cos(ang)));
    const double vGroove = M_PI *
        (grooveOuterR * grooveOuterR - grooveInnerR * grooveInnerR) * grooveDepth;
    const double volRemoved = vPort + vGroove;

    json params = {
        { "center_xy",             { in.center_xy.X(),
                                     in.center_xy.Y(),
                                     in.center_xy.Z() } },
        { "port_dia_mm",           in.port_dia_mm },
        { "grommet_groove_dia_mm", in.grommet_groove_dia_mm },
        { "grommet_groove_depth_mm", in.grommet_groove_depth_mm },
        { "entry_angle_deg",       in.entry_angle_deg },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "bicycle_feature_type",       "internal_cable_routing_port" },
        { "subfeature_count",           2 },
        { "port_dia_mm",                in.port_dia_mm },
        { "grommet_groove_dia_mm",      in.grommet_groove_dia_mm },
        { "entry_angle_deg",            in.entry_angle_deg },
        { "derived_groove_inner_dia_mm", 2.0 * grooveInnerR },
        { "derived_bore_length_mm",     boreLen },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "internal cable routing port" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill;groove_mill";
    tooling.tool_dia_mm       = in.port_dia_mm;
    tooling.tool_length_mm    = boreLen + 10.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 220.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = 35.0;
    tooling.extra = {
        { "bicycle_feature_type", "internal_cable_routing_port" },
        { "entry_angle_deg",      in.entry_angle_deg },
        { "standard",             "internal routing" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::internal_cable_port_compound: port Ø{} groove Ø{} angle {}",
                  in.port_dia_mm, in.grommet_groove_dia_mm, in.entry_angle_deg);

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
        r.confidence       = 1.0;
        r.matched_geometry = {
            { "source",               "metadata_replay" },
            { "is_compound",          true },
            { "bicycle_feature_type", "internal_cable_routing_port" },
        };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric fallback: a small port cylinder plus a slightly larger
    // groove-ring cylinder coaxial near the surface.
    int portCyls   = 0;
    int grooveCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const double radius = s.Cylinder().Radius();
            if (radius >= 2.0 && radius <= 4.5) ++portCyls;
            else if (radius > 4.5 && radius <= 7.0) ++grooveCyls;
        } catch (...) {}
    }
    if (portCyls >= 1 && grooveCyls >= 1) {
        json recovered = { { "port_dia_mm",           6.0 },
                           { "grommet_groove_dia_mm", 10.0 } };
        json matched   = { { "source",      "geometric_port_groove_pattern" },
                           { "port_cyls",   portCyls },
                           { "groove_cyls", grooveCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::internal_cable_port_compound
