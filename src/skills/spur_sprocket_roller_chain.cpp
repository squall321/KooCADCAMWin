// @lat: [[engine/skills#spur_sprocket_roller_chain]]

#include "spur_sprocket_roller_chain.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::spur_sprocket_roller_chain {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.chain_pitch_mm <= 0.0 || in.roller_dia_mm <= 0.0 ||
        in.blank_outer_dia_mm <= 0.0 || in.face_width_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "spur_sprocket_roller_chain: all dimensions must be > 0");
        return r;
    }

    if (in.tooth_count < 9) {
        r.add("DFM-PT-TEETH", "error",
              "spur_sprocket_roller_chain: tooth_count (" +
              std::to_string(in.tooth_count) +
              ") must be >= 9 (ANSI minimum practical sprocket)");
    }

    if (in.roller_dia_mm >= in.chain_pitch_mm) {
        r.add("DFM-PT-ROLLER", "error",
              "spur_sprocket_roller_chain: roller_dia_mm (" +
              std::to_string(in.roller_dia_mm) +
              ") must be < chain_pitch_mm (" +
              std::to_string(in.chain_pitch_mm) + ")");
    }

    // Pitch circle: PCD = pitch / sin(pi / N).
    if (in.tooth_count >= 9) {
        const double pcd = in.chain_pitch_mm /
                           std::sin(M_PI / static_cast<double>(in.tooth_count));
        if (in.blank_outer_dia_mm <= pcd) {
            r.add("DFM-PT-BLANK", "error",
                  "spur_sprocket_roller_chain: blank_outer_dia_mm (" +
                  std::to_string(in.blank_outer_dia_mm) +
                  ") must exceed pitch circle diameter (" +
                  std::to_string(pcd) + ") for a tooth tip to remain");
        }
    }

    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "spur_sprocket_roller_chain DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double cx = in.center_xy.X();
    const double cy = in.center_xy.Y();
    const double thru = (zMax - zMin) + 2.0 * kOver;

    // Pitch circle radius where rollers seat.
    const double pcd  = in.chain_pitch_mm /
                        std::sin(M_PI / static_cast<double>(in.tooth_count));
    const double pcr  = pcd / 2.0;
    // Roller seat pocket radius — the roller half-diameter plus a small
    // working clearance (ANSI seating curve approximation).
    const double seatR = in.roller_dia_mm / 2.0 + 0.05 * in.chain_pitch_mm;

    // Tooth-gap template at theta=0: a full-depth cylinder pocket centered
    // on the pitch circle in +X.  Cut sequentially via SetRotation.
    const gp_Pnt seatTpl(cx + pcr, cy, zMin - kOver);
    const gp_Ax2 seatAx(seatTpl, gp::DZ());
    const TopoDS_Shape seatTemplate = pr::cylinder(seatAx, seatR, thru);

    const gp_Ax1 rotAxis(gp_Pnt(cx, cy, zMax), gp::DZ());
    TopoDS_Shape current = wp.shape();
    for (int i = 0; i < in.tooth_count; ++i) {
        const double theta = (2.0 * M_PI * static_cast<double>(i)) /
                             static_cast<double>(in.tooth_count);
        gp_Trsf rot;
        rot.SetRotation(rotAxis, theta);
        BRepBuilderAPI_Transform xform(seatTemplate, rot, true);
        if (!xform.IsDone())
            throw SkillError("spur_sprocket_roller_chain: tooth rotation failed");
        current = pr::cut(current, xform.Shape());  // sequential — no compound
    }

    const double diskThk = (zMax - zMin);
    const double vSeat   = M_PI * seatR * seatR * diskThk;
    const double volRemoved = static_cast<double>(in.tooth_count) * vSeat;

    json params = {
        { "center_xy",          { in.center_xy.X(),
                                  in.center_xy.Y(),
                                  in.center_xy.Z() } },
        { "chain_pitch_mm",     in.chain_pitch_mm },
        { "roller_dia_mm",      in.roller_dia_mm },
        { "tooth_count",        in.tooth_count },
        { "blank_outer_dia_mm", in.blank_outer_dia_mm },
        { "face_width_mm",      in.face_width_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "powertrans_feature_type",    "roller_chain_sprocket_teeth" },
        { "subfeature_count",           in.tooth_count },
        { "chain_pitch_mm",             in.chain_pitch_mm },
        { "roller_dia_mm",              in.roller_dia_mm },
        { "tooth_count",                in.tooth_count },
        { "derived_pitch_circle_dia_mm", pcd },
        { "derived_seat_radius_mm",     seatR },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "ANSI/ASME B29.1" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "form_mill;hob";
    tooling.tool_dia_mm       = 2.0 * seatR;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 180.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = 6.0 * static_cast<double>(in.tooth_count);
    tooling.extra = {
        { "powertrans_application", "roller_chain_sprocket" },
        { "standard",              "ANSI/ASME B29.1" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::spur_sprocket_roller_chain: pitch={} N={} faces {}→{}",
                  in.chain_pitch_mm, in.tooth_count,
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

    // Geometric fallback: count vertical cylindrical pockets near the rim.
    int seatCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Cylinder c = s.Cylinder();
            if (std::abs(c.Axis().Direction().Z()) < 0.9) continue;
            const double radius = c.Radius();
            if (radius >= 1.0 && radius <= 8.0) ++seatCyls;
        } catch (...) {}
    }
    if (seatCyls >= 9) {
        json recovered = { { "tooth_count",    seatCyls },
                           { "chain_pitch_mm", 12.70 } };
        json matched   = { { "source",     "geometric_roller_seat_ring" },
                           { "seat_cyls",  seatCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::spur_sprocket_roller_chain
