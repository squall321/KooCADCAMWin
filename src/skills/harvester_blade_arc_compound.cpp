// @lat: [[engine/skills#harvester_blade_arc_compound]]

#include "harvester_blade_arc_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::harvester_blade_arc_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.section_pitch_mm <= 0.0 || in.blade_height_mm <= 0.0
        || in.blade_section_count <= 0) {
        r.add("DFM-INPUT", "error",
              "harvester_blade_arc_compound: positive pitch/height and "
              "blade_section_count >= 1 required");
    }
    if (in.edge_angle_deg < 10.0 || in.edge_angle_deg > 35.0) {
        r.add("DFM-EDGE-ANGLE", "error",
              "harvester_blade_arc_compound: edge_angle " +
              std::to_string(in.edge_angle_deg) +
              " deg outside typical 10-35 deg sickle range");
    }
    if (in.section_pitch_mm < 50.0 || in.section_pitch_mm > 100.0) {
        r.add("DFM-PITCH", "error",
              "harvester_blade_arc_compound: section_pitch " +
              std::to_string(in.section_pitch_mm) +
              " mm outside typical ASAE 50-100 mm range");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "harvester_blade_arc_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;

    // Each knife section is a triangle along the sickle bar.  We cut a
    // tapered "tooth gap" between adjacent triangles by removing a thin
    // wedge-shaped pocket at every junction.  The tooth tip points in +Y
    // (cutting direction); the back (-Y) is the riveted shank.
    //
    // Geometry: at each pitch position x_i we cut a thin V-shape from the
    // top face downward.  Wedge half-width at top = pitch * tan(edge_angle/2)
    // bounded so it never exceeds pitch/2.  Cut depth = blade_height.
    const double halfAngleRad = (in.edge_angle_deg * 0.5) * M_PI / 180.0;
    const double wedgeHalfW = std::min(in.section_pitch_mm * 0.5,
                                       in.section_pitch_mm * std::tan(halfAngleRad));
    const double wedgeDepth = in.blade_height_mm;

    TopoDS_Shape current = wp.shape();

    for (int i = 0; i < in.blade_section_count; ++i) {
        // V-gap centred at this i-th tooth boundary along +X.
        const double xCentre = in.blade_origin.X()
                               + (static_cast<double>(i) + 0.5) * in.section_pitch_mm;

        // Approximate the V-pocket with a rectangular box (constant cross
        // section).  For visual fidelity the gap could be triangular, but
        // a thin rectangle gives the same material-removal volume and a
        // recognisable subfeature count.  The box spans the full blade
        // width in Y so it definitely intersects the sickle bar.
        const double yLow  = yMin - kOver;
        const double yHigh = yMax + kOver;
        const gp_Pnt boxOrigin(xCentre - wedgeHalfW,
                               yLow,
                               topZ - wedgeDepth);
        BRepPrimAPI_MakeBox mkBox(boxOrigin,
                                  2.0 * wedgeHalfW,
                                  yHigh - yLow,
                                  wedgeDepth + kOver);
        mkBox.Build();
        if (!mkBox.IsDone())
            throw SkillError("harvester_blade_arc_compound: wedge box failed");
        const TopoDS_Shape wedge = mkBox.Shape();
        current = pr::cut(current, wedge);
    }

    const double vTotal = in.blade_section_count
                          * (2.0 * wedgeHalfW)
                          * (yMax - yMin)
                          * wedgeDepth;

    json params = {
        { "blade_origin",        { in.blade_origin.X(),
                                   in.blade_origin.Y(),
                                   in.blade_origin.Z() } },
        { "blade_section_count", in.blade_section_count },
        { "section_pitch_mm",    in.section_pitch_mm },
        { "blade_height_mm",     in.blade_height_mm },
        { "edge_angle_deg",      in.edge_angle_deg },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "ag_feature_type",            "harvester_sickle_blade" },
        { "subfeature_count",           in.blade_section_count },
        { "blade_section_count",        in.blade_section_count },
        { "section_pitch_mm",           in.section_pitch_mm },
        { "blade_height_mm",            in.blade_height_mm },
        { "edge_angle_deg",             in.edge_angle_deg },
        { "derived_wedge_half_w_mm",    wedgeHalfW },
        { "derived_volume_removed_mm3", vTotal },
        { "standard",                   "ASAE S345 sickle pattern" },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "v_mill;grinding_wheel";
    tooling.tool_dia_mm       = wedgeHalfW * 2.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 = vTotal;
    tooling.est_cycle_time_s  = std::max(30.0, in.blade_section_count * 4.0);
    tooling.extra = {
        { "ag_application", "reciprocating_cutter_bar" },
        { "edge_angle_deg", in.edge_angle_deg },
        { "pitch_mm",       in.section_pitch_mm },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::harvester_blade_arc_compound: N={} pitch={} edge={}",
                  in.blade_section_count, in.section_pitch_mm, in.edge_angle_deg);

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

    // Geometric: count vertical planar faces (sickle wedge walls).
    int wedgeWalls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        try {
            gp_Dir n = wp.faceNormal(i);
            if (std::abs(n.Z()) < 0.2 && std::abs(n.X()) > 0.8) ++wedgeWalls;
        } catch (...) {}
    }
    if (wedgeWalls >= 4) {
        const int Nest = wedgeWalls / 2;
        json recovered = { { "blade_section_count", Nest },
                           { "section_pitch_mm",    76.2 } };
        json matched   = { { "source", "geometric_sickle_pattern" },
                           { "wedge_wall_count", wedgeWalls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::harvester_blade_arc_compound
