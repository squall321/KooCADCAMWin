// @lat: [[engine/skills#v_belt_pulley_groove]]

#include "v_belt_pulley_groove.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::v_belt_pulley_groove {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

constexpr double kOver = 0.1;

struct SectionSpec {
    const char* section;       // "A", "B", "SPZ"
    double      top_width_mm;   // groove top opening width (belt top width)
    double      pitch_mm;       // axial groove-to-groove spacing
};

constexpr SectionSpec kSections[] {
    { "A",   13.0, 15.0 },
    { "B",   17.0, 19.0 },
    { "SPZ", 10.0, 12.0 },
};

const SectionSpec* findSection(const std::string& s)
{
    for (const auto& sp : kSections) if (s == sp.section) return &sp;
    return nullptr;
}

}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.pulley_od_mm <= 0.0 || in.groove_depth_mm <= 0.0 ||
        in.groove_count <= 0) {
        r.add("DFM-INPUT", "error",
              "v_belt_pulley_groove: od/depth/groove_count must be > 0");
        return r;
    }

    const SectionSpec* sec = findSection(in.belt_section);
    if (!sec) {
        r.add("DFM-PT-SECTION", "error",
              "v_belt_pulley_groove: belt_section '" + in.belt_section +
              "' invalid (must be A | B | SPZ)");
    }

    if (in.groove_angle_deg < 34.0 || in.groove_angle_deg > 38.0) {
        r.add("DFM-PT-ANGLE", "error",
              "v_belt_pulley_groove: groove_angle_deg (" +
              std::to_string(in.groove_angle_deg) +
              ") must be in [34, 38]");
    }

    // Require the OD radius to comfortably exceed the groove depth so a hub
    // wall remains under the groove roots.
    if (in.groove_depth_mm >= in.pulley_od_mm / 2.0 - 2.0) {
        r.add("DFM-PT-WIDTH", "error",
              "v_belt_pulley_groove: groove_depth_mm (" +
              std::to_string(in.groove_depth_mm) +
              ") too deep for pulley_od_mm (" +
              std::to_string(in.pulley_od_mm) + ")");
    }

    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "v_belt_pulley_groove DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const SectionSpec* sec = findSection(in.belt_section);
    if (!sec) throw SkillError("v_belt_pulley_groove: section lookup failed");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double cx = in.center_xy.X();
    const double cy = in.center_xy.Y();
    const double odR  = in.pulley_od_mm / 2.0;

    // Groove geometry: root radius = odR - groove_depth.  The V opening at
    // the OD spans the belt-section top width; its axial half-width is half
    // of that opening.  The wedge narrows linearly to the root, giving the
    // characteristic V flank (groove_angle_deg is the nominal spec angle,
    // carried into the signature).
    const double rootR    = odR - in.groove_depth_mm;
    const double halfWidth = sec->top_width_mm / 2.0;

    // Stack the grooves axially, centered on the face mid-plane.
    const double faceMidZ = (zMin + zMax) / 2.0;
    const double pitch    = sec->pitch_mm;
    const double firstZc  = faceMidZ -
                            (static_cast<double>(in.groove_count - 1) / 2.0) * pitch;

    TopoDS_Shape current = wp.shape();
    for (int i = 0; i < in.groove_count; ++i) {
        const double zc = firstZc + static_cast<double>(i) * pitch;
        // The groove is the ANNULAR V shell on the rim: the cylinder band at
        // the OD minus the V-shaped "kept metal" bicone (narrowest at the
        // root plane z=zc, widening to odR at the band extremes).  Building
        // it as band − bicone gives a single wedge-ring cutter whose inner
        // wall is the V profile; cut SEQUENTIALLY from the disc.
        const gp_Ax2 bandAx(gp_Pnt(cx, cy, zc - halfWidth), gp::DZ());
        const TopoDS_Shape band =
            pr::cylinder(bandAx, odR + kOver, 2.0 * halfWidth);
        const gp_Ax2 axUp(gp_Pnt(cx, cy, zc), gp::DZ());
        const TopoDS_Shape coneUp =
            pr::coneFrustum(axUp, rootR, odR + kOver, halfWidth);
        const gp_Ax2 axDn(gp_Pnt(cx, cy, zc), gp_Dir(0.0, 0.0, -1.0));
        const TopoDS_Shape coneDn =
            pr::coneFrustum(axDn, rootR, odR + kOver, halfWidth);
        const TopoDS_Shape keptBicone = pr::fuse(coneUp, coneDn);
        const TopoDS_Shape grooveTool = pr::cut(band, keptBicone);  // V shell
        current = pr::cut(current, grooveTool);                     // sequential
    }

    // Derived volume — annular V wedge per groove approximated as a revolved
    // triangle: mean radius × cross-section area (depth × full opening width),
    // summed over N grooves.
    const double meanR  = (odR + rootR) / 2.0;
    const double xArea  = 0.5 * in.groove_depth_mm * (2.0 * halfWidth);
    const double vGroove = 2.0 * M_PI * meanR * xArea;
    const double volRemoved = static_cast<double>(in.groove_count) * vGroove;

    json params = {
        { "center_xy",        { in.center_xy.X(),
                                in.center_xy.Y(),
                                in.center_xy.Z() } },
        { "pulley_od_mm",     in.pulley_od_mm },
        { "belt_section",     in.belt_section },
        { "groove_count",     in.groove_count },
        { "groove_angle_deg", in.groove_angle_deg },
        { "groove_depth_mm",  in.groove_depth_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "powertrans_feature_type",    "v_belt_sheave_grooves" },
        { "subfeature_count",           in.groove_count },
        { "belt_section",               in.belt_section },
        { "groove_count",               in.groove_count },
        { "groove_angle_deg",           in.groove_angle_deg },
        { "derived_groove_top_width_mm", sec->top_width_mm },
        { "derived_root_radius_mm",     rootR },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "ISO 4183 / RMA IP-20" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "form_groove_tool;profile_lathe";
    tooling.tool_dia_mm       = sec->top_width_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 1;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.08;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = 15.0 * static_cast<double>(in.groove_count);
    tooling.extra = {
        { "powertrans_application", "v_belt_sheave" },
        { "belt_section",          in.belt_section },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::v_belt_pulley_groove: section={} grooves={} faces {}→{}",
                  in.belt_section, in.groove_count,
                  wp.faceCount(), wpNew->faceCount());

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

    // Geometric fallback: count conical faces around the OD (V-flanks).
    int coneFaces = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        try {
            BRepAdaptor_Surface s(wp.face(i));
            if (s.GetType() == GeomAbs_Cone) ++coneFaces;
        } catch (...) {}
    }
    if (coneFaces >= 2) {
        json recovered = { { "groove_count", coneFaces / 2 },
                           { "belt_section", "A" } };
        json matched   = { { "source",     "geometric_v_flank_cones" },
                           { "cone_faces", coneFaces } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::v_belt_pulley_groove
