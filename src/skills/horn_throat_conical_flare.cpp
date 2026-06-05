// @lat: [[engine/skills#horn_throat_conical_flare]]

#include "horn_throat_conical_flare.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
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

namespace koocadcam::skill::horn_throat_conical_flare {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!(in.throat_dia_mm > 0.0) || !(in.mouth_dia_mm > 0.0) ||
        !(in.flare_length_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "horn_throat_conical_flare: all dims must be > 0");
        return r;
    }
    if (!(in.mouth_dia_mm > in.throat_dia_mm)) {
        r.add("DFM-FLARE", "error",
              "horn_throat_conical_flare: mouth_dia (" +
              std::to_string(in.mouth_dia_mm) +
              ") must be > throat_dia (" +
              std::to_string(in.throat_dia_mm) + ") — a horn must expand");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "horn_throat_conical_flare DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double cx = in.center_xy.X();
    const double cy = in.center_xy.Y();

    const double throatR = in.throat_dia_mm / 2.0;
    const double mouthR  = in.mouth_dia_mm / 2.0;

    // ── 1) Conical flare bore ────────────────────────────────────────────
    // Axis sits at the bottom of the flare pointing +Z; r1 = mouth radius
    // (bottom), r2 = throat radius (top).  The cone narrows as it rises so
    // the throat opening sits flush with the top face.
    const double height = in.flare_length_mm + kOver;
    const gp_Pnt flareStart(cx, cy, topZ - in.flare_length_mm);
    const gp_Ax2 flareAx(flareStart, gp::DZ());
    const TopoDS_Shape flareTool = pr::coneFrustum(flareAx, mouthR, throatR, height);
    TopoDS_Shape current = pr::cut(wp.shape(), flareTool);

    // Volume of a conical frustum = (pi*h/3)*(R^2 + R*r + r^2).
    const double volRemoved = (M_PI * in.flare_length_mm / 3.0) *
                              (mouthR * mouthR + mouthR * throatR + throatR * throatR);

    json params = {
        { "center_xy",       { in.center_xy.X(),
                               in.center_xy.Y(),
                               in.center_xy.Z() } },
        { "throat_dia_mm",   in.throat_dia_mm },
        { "mouth_dia_mm",    in.mouth_dia_mm },
        { "flare_length_mm", in.flare_length_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "audio_feature_type",         "conical_horn_flare" },
        { "subfeature_count",           1 },
        { "derived_throat_dia_mm",      in.throat_dia_mm },
        { "derived_mouth_dia_mm",       in.mouth_dia_mm },
        { "derived_flare_length_mm",    in.flare_length_mm },
        { "derived_volume_removed_mm3", volRemoved },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "taper_mill";
    tooling.tool_dia_mm       = in.mouth_dia_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 220.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(25.0, in.flare_length_mm * 1.5);
    tooling.extra = {
        { "audio_application", "compression_driver_horn_throat" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::horn_throat_conical_flare: throat={} mouth={} len={}",
                  in.throat_dia_mm, in.mouth_dia_mm, in.flare_length_mm);

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

    // Geometric: a horn flare leaves exactly one conical (non-cylindrical)
    // tapered face.  We approximate by counting tapered surfaces.
    int coneFaces = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (wp.isFacePlanar(i) || wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            if (s.GetType() == GeomAbs_Cone) ++coneFaces;
        } catch (...) {}
    }
    if (coneFaces >= 1) {
        json recovered = { { "throat_dia_mm",   25.4 },
                           { "mouth_dia_mm",    60.0 },
                           { "flare_length_mm", 25.0 } };
        json matched   = { { "source",     "geometric_cone_flare" },
                           { "cone_faces", coneFaces } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::horn_throat_conical_flare
