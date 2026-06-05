// @lat: [[engine/skills#guitar_truss_rod_channel]]

#include "guitar_truss_rod_channel.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::guitar_truss_rod_channel {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.rod_length_mm <= 0.0 || in.rod_channel_width_mm <= 0.0
        || in.rod_channel_depth_mm <= 0.0 || in.nut_recess_dia_mm <= 0.0
        || in.nut_recess_depth_mm <= 0.0 || in.end_anchor_recess_w_mm <= 0.0
        || in.end_anchor_recess_l_mm <= 0.0
        || in.end_anchor_recess_d_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "guitar_truss_rod_channel: all dimensions must be > 0");
    }
    if (in.rod_channel_width_mm < 4.0 || in.rod_channel_width_mm > 9.0) {
        r.add("DFM-CHANNEL", "error",
              "guitar_truss_rod_channel: channel width "
              + std::to_string(in.rod_channel_width_mm)
              + " mm not in typical [4, 9] mm range");
    }
    if (in.rod_channel_depth_mm < 4.0 || in.rod_channel_depth_mm > 12.0) {
        r.add("DFM-CHANNEL", "error",
              "guitar_truss_rod_channel: channel depth "
              + std::to_string(in.rod_channel_depth_mm)
              + " mm not in typical [4, 12] mm range");
    }
    if (in.rod_length_mm < 300.0 || in.rod_length_mm > 700.0) {
        r.add("DFM-LENGTH", "error",
              "guitar_truss_rod_channel: rod_length "
              + std::to_string(in.rod_length_mm)
              + " mm not in typical [300, 700] mm range");
    }
    if (in.nut_recess_dia_mm > in.rod_channel_width_mm + 2.0) {
        r.add("DFM-NUT", "error",
              "guitar_truss_rod_channel: nut_recess_dia must be <= channel_width + 2 mm");
    }
    if (in.end_anchor_recess_w_mm > in.rod_channel_width_mm + 3.0) {
        r.add("DFM-ANCHOR", "error",
              "guitar_truss_rod_channel: anchor recess width must be "
              "<= channel_width + 3 mm");
    }
    // Only +X axis is supported for the simple slot tool orientation.
    if (!(std::abs(in.rod_axis_dir.X()) > 0.9
          && std::abs(in.rod_axis_dir.Y()) < 0.1
          && std::abs(in.rod_axis_dir.Z()) < 0.1)) {
        r.add("DFM-AXIS", "error",
              "guitar_truss_rod_channel: only rod_axis_dir = +X is supported");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "guitar_truss_rod_channel DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    constexpr double kOver = 0.1;

    const double x0 = in.rod_axis_origin.X();
    const double y0 = in.rod_axis_origin.Y();
    // Channel is cut DOWN from the top face (zMax) by rod_channel_depth_mm.
    const double zTop = zMax;
    const double chanW = in.rod_channel_width_mm;
    const double chanD = in.rod_channel_depth_mm;
    const double chanL = in.rod_length_mm;

    // ── 1) Straight channel slot — rectangular box centred on rod_axis_origin
    //       along +X.  Cuts down from top face by chanD.
    const gp_Pnt chanOrigin(x0, y0 - chanW / 2.0, zTop - chanD);
    const gp_Ax2 chanAx(chanOrigin, gp::DZ());
    const TopoDS_Shape channelTool = pr::box(chanAx,
        chanL,              // DX  — along rod axis
        chanW,              // DY  — channel width
        chanD + kOver);     // DZ  — depth + overhang up

    TopoDS_Shape current = pr::cut(wp.shape(), channelTool);

    // ── 2) Nut hex recess at NUT end (x0) — cylinder cut deeper than channel.
    //       Centred on rod axis at the start (x0).
    const double nutR = in.nut_recess_dia_mm / 2.0;
    const gp_Pnt nutOrigin(x0, y0, zTop + kOver);
    const gp_Ax2 nutAx(nutOrigin, gp_Dir(0, 0, -1));
    const TopoDS_Shape nutTool = pr::cylinder(nutAx, nutR,
        in.nut_recess_depth_mm + 2.0 * kOver);
    current = pr::cut(current, nutTool);

    // ── 3) Anchor block recess at FAR end (x0 + chanL).  Slightly larger
    //       rectangular pocket cut below the channel floor to seat the
    //       end-anchor block.
    const double anchorX = x0 + chanL - in.end_anchor_recess_l_mm;
    const gp_Pnt anchorOrigin(
        anchorX,
        y0 - in.end_anchor_recess_w_mm / 2.0,
        zTop - in.end_anchor_recess_d_mm);
    const gp_Ax2 anchorAx(anchorOrigin, gp::DZ());
    const TopoDS_Shape anchorTool = pr::box(anchorAx,
        in.end_anchor_recess_l_mm,
        in.end_anchor_recess_w_mm,
        in.end_anchor_recess_d_mm + kOver);
    current = pr::cut(current, anchorTool);

    // Approx total volume removed for tooling estimate.
    const double vChan   = chanL * chanW * chanD;
    const double vNut    = M_PI * nutR * nutR * in.nut_recess_depth_mm;
    const double vAnchor = in.end_anchor_recess_l_mm
                         * in.end_anchor_recess_w_mm
                         * in.end_anchor_recess_d_mm;
    const double volRemoved = vChan + vNut + vAnchor;

    json params = {
        { "rod_axis_origin",      { x0, y0, in.rod_axis_origin.Z() } },
        { "rod_axis_dir",         { in.rod_axis_dir.X(),
                                    in.rod_axis_dir.Y(),
                                    in.rod_axis_dir.Z() } },
        { "rod_length_mm",        in.rod_length_mm },
        { "rod_channel_width_mm", in.rod_channel_width_mm },
        { "rod_channel_depth_mm", in.rod_channel_depth_mm },
        { "nut_recess_dia_mm",    in.nut_recess_dia_mm },
        { "nut_recess_depth_mm",  in.nut_recess_depth_mm },
        { "end_anchor_recess_w_mm", in.end_anchor_recess_w_mm },
        { "end_anchor_recess_l_mm", in.end_anchor_recess_l_mm },
        { "end_anchor_recess_d_mm", in.end_anchor_recess_d_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "audio_feature_type",  "guitar_truss_rod_channel" },
        { "subfeature_count",    3 },
        { "rod_length_mm",       in.rod_length_mm },
        { "rod_channel_width_mm", in.rod_channel_width_mm },
        { "rod_channel_depth_mm", in.rod_channel_depth_mm },
        { "nut_recess_dia_mm",   in.nut_recess_dia_mm },
        { "derived_volume_removed_mm3", volRemoved },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill;drill;end_mill";
    tooling.tool_dia_mm       = in.rod_channel_width_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 350.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(30.0, in.rod_length_mm * 0.05 + 10.0);
    tooling.extra = {
        { "standard",         "guitar luthier truss rod cavity" },
        { "subfeatures",      "channel+nut_recess+anchor_recess" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::guitar_truss_rod_channel: len={} chan={}×{}",
                  in.rod_length_mm, in.rod_channel_width_mm,
                  in.rod_channel_depth_mm);

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

    // Geometric heuristic: detect a long straight slot ON top of a flat block
    // by counting downward-facing planar floor faces and a single small
    // cylindrical face (nut recess).  Low confidence fallback.
    int planarFaces = 0;
    int smallCyls   = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (wp.isFacePlanar(i)) {
            ++planarFaces;
        } else if (wp.isFaceCylinder(i)) {
            try {
                BRepAdaptor_Surface s(wp.face(i));
                if (s.Cylinder().Radius() <= 5.0) ++smallCyls;
            } catch (...) {}
        }
    }
    if (planarFaces >= 8 && smallCyls >= 1) {
        json recovered = { { "candidate", "truss_rod_channel" } };
        json matched   = { { "source", "geometric_slot_with_recess" },
                           { "planar_faces", planarFaces },
                           { "small_cyl_count", smallCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.45, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::guitar_truss_rod_channel
