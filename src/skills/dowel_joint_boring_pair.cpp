// @lat: [[engine/skills#dowel_joint_boring_pair]]

#include "dowel_joint_boring_pair.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::dowel_joint_boring_pair {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!(in.dowel_dia_mm > 0.0) || !(in.pitch_mm > 0.0) ||
        !(in.dowel_depth_mm > 0.0) || !(in.glue_groove_width_mm > 0.0) ||
        !(in.glue_groove_depth_mm > 0.0) || in.dowel_count < 2) {
        r.add("DFM-INPUT", "error",
              "dowel_joint_boring_pair: dims must be > 0 and dowel_count >= 2");
        return r;
    }
    if (in.pitch_mm <= in.dowel_dia_mm) {
        r.add("DFM-PITCH", "error",
              "dowel_joint_boring_pair: pitch_mm " +
              std::to_string(in.pitch_mm) +
              " must exceed dowel_dia_mm " + std::to_string(in.dowel_dia_mm) +
              " (dowel holes would overlap)");
    }
    if (in.glue_groove_width_mm >= in.dowel_dia_mm) {
        r.add("DFM-GROOVE", "error",
              "dowel_joint_boring_pair: glue_groove_width_mm " +
              std::to_string(in.glue_groove_width_mm) +
              " must be < dowel_dia_mm " + std::to_string(in.dowel_dia_mm) +
              " (groove must not undercut the dowels)");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "dowel_joint_boring_pair DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double ox = in.row_origin.X();
    const double oy = in.row_origin.Y();

    const double rowLen = in.pitch_mm * static_cast<double>(in.dowel_count - 1);
    const double dowelR = in.dowel_dia_mm / 2.0;

    // ── 1) Glue groove — long shallow box along +X (large feature FIRST) ──
    // Spans from the first to the last dowel centre, centred in Y on the row.
    const double grooveLen = rowLen + in.dowel_dia_mm;  // bridge past end dowels
    const gp_Pnt grooveOrigin(ox - in.dowel_dia_mm / 2.0,
                              oy - in.glue_groove_width_mm / 2.0,
                              topZ - in.glue_groove_depth_mm);
    const gp_Ax2 grooveAx(grooveOrigin, gp::DZ());
    TopoDS_Shape current = pr::cut(
        wp.shape(),
        pr::box(grooveAx, grooveLen, in.glue_groove_width_mm,
                in.glue_groove_depth_mm + kOver));

    // ── 2..N+1) Dowel holes along +X (SEQUENTIAL cuts, coaxial w/ groove) ─
    for (int i = 0; i < in.dowel_count; ++i) {
        const double hx = ox + static_cast<double>(i) * in.pitch_mm;
        const gp_Pnt hStart(hx, oy, topZ - in.dowel_depth_mm);
        const gp_Ax2 hAx(hStart, gp::DZ());
        current = pr::cut(current, pr::cylinder(hAx, dowelR, in.dowel_depth_mm + kOver));
    }

    const int subfeatures = in.dowel_count + 1;
    const double vGroove =
        grooveLen * in.glue_groove_width_mm * in.glue_groove_depth_mm;
    const double vDowels =
        M_PI * dowelR * dowelR * in.dowel_depth_mm * static_cast<double>(in.dowel_count);
    const double volRemoved = vGroove + vDowels;

    json params = {
        { "row_origin",           { in.row_origin.X(),
                                    in.row_origin.Y(),
                                    in.row_origin.Z() } },
        { "dowel_dia_mm",         in.dowel_dia_mm },
        { "dowel_count",          in.dowel_count },
        { "pitch_mm",             in.pitch_mm },
        { "dowel_depth_mm",       in.dowel_depth_mm },
        { "glue_groove_width_mm", in.glue_groove_width_mm },
        { "glue_groove_depth_mm", in.glue_groove_depth_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "furniture_feature_type",     "dowel_joint_with_glue_groove" },
        { "subfeature_count",           subfeatures },
        { "dowel_count",                in.dowel_count },
        { "derived_dowel_dia_mm",       in.dowel_dia_mm },
        { "derived_pitch_mm",           in.pitch_mm },
        { "derived_dowel_depth_mm",     in.dowel_depth_mm },
        { "derived_groove_width_mm",    in.glue_groove_width_mm },
        { "derived_groove_length_mm",   grooveLen },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "edge dowel joint" },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "drill;slot_mill";
    tooling.tool_dia_mm       = in.dowel_dia_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 170.0;
    tooling.feed_per_tooth_mm = 0.06;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(14.0, 3.0 * in.dowel_count + rowLen / 8.0);
    tooling.extra = {
        { "furniture_application", "edge_dowel_joint" },
        { "standard",              "edge dowel joint" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::dowel_joint_boring_pair: dowels={} pitch={} groove={}x{}",
                  in.dowel_count, in.pitch_mm,
                  in.glue_groove_width_mm, in.glue_groove_depth_mm);

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

    // Geometric: a row of dowel cylinders (~4 mm radius) plus a shallow
    // groove (planar walls, no extra cylinders) bridging them.
    int dowelCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const double radius = s.Cylinder().Radius();
            if (radius >= 2.5 && radius <= 7.0) ++dowelCyls;
        } catch (...) {}
    }
    if (dowelCyls >= 2) {
        json recovered = { { "dowel_dia_mm",         8.0 },
                           { "dowel_count",          dowelCyls },
                           { "pitch_mm",             64.0 },
                           { "dowel_depth_mm",       15.0 },
                           { "glue_groove_width_mm", 3.0 },
                           { "glue_groove_depth_mm", 1.0 } };
        json matched   = { { "source",     "geometric_dowel_glue_groove" },
                           { "dowel_cyls", dowelCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::dowel_joint_boring_pair
