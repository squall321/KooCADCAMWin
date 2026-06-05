// @lat: [[engine/skills#flange_coupling_bolt_circle]]

#include "flange_coupling_bolt_circle.hpp"

#include "Workpiece.hpp"
#include "_iso286_fits.hpp"
#include "_keyway_table.hpp"
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

#include <algorithm>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::flange_coupling_bolt_circle {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.bore_dia_mm <= 0.0 || in.key_length_mm <= 0.0 ||
        in.bolt_circle_dia_mm <= 0.0 || in.bolt_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "flange_coupling_bolt_circle: all dimensions must be > 0");
        return r;
    }

    if (!keyway::findDin6885Band(in.bore_dia_mm)) {
        r.add("DFM-PT-KEY", "error",
              "flange_coupling_bolt_circle: bore_dia_mm (" +
              std::to_string(in.bore_dia_mm) +
              ") outside DIN 6885 parallel-key table range");
    }

    if (in.bolt_count < 3) {
        r.add("DFM-PT-BOLTS", "error",
              "flange_coupling_bolt_circle: bolt_count (" +
              std::to_string(in.bolt_count) +
              ") must be >= 3");
    }

    if (in.bolt_circle_dia_mm <= in.bore_dia_mm) {
        r.add("DFM-PT-PCD", "error",
              "flange_coupling_bolt_circle: bolt_circle_dia_mm (" +
              std::to_string(in.bolt_circle_dia_mm) +
              ") must exceed bore_dia_mm (" +
              std::to_string(in.bore_dia_mm) +
              ") so bolts clear the bore");
    }

    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "flange_coupling_bolt_circle DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const keyway::Din6885ParallelBand* key =
        keyway::findDin6885Band(in.bore_dia_mm);
    if (!key) throw SkillError("flange_coupling_bolt_circle: key lookup failed");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double cx = in.center_xy.X();
    const double cy = in.center_xy.Y();
    const double thru = (zMax - zMin) + 2.0 * kOver;

    // ── 1) Central H7 bore (big feature first) ─────────────────────────
    const double boreActual = iso286::h7_max_mm(in.bore_dia_mm);
    const double boreR = boreActual / 2.0;
    const gp_Ax2 boreAx(gp_Pnt(cx, cy, zMin - kOver), gp::DZ());
    TopoDS_Shape current = pr::cut(wp.shape(), pr::cylinder(boreAx, boreR, thru));

    // ── 2) DIN 6885 keyway slot (box from bore outward into hub) ───────
    // Width = key_width_mm along Y; the hub keyway depth (t2) measured from
    // the bore wall radially outward.  Length axial = key_length_mm.
    const double keyW   = key->key_width_mm;
    const double keyDep = key->hub_keyway_depth_mm;     // radial into hub
    const double keyLen = in.key_length_mm;
    // Box main dir = +Z (axial), XDir = +X (radial outward).  Origin chosen
    // so the slot starts inside the bore wall and extends radially by
    // (boreR + keyDep), centered on the bore in Y, top-aligned axially.
    const double slotRadial = boreR + keyDep;
    const gp_Pnt keyOrigin(cx, cy - keyW / 2.0, topZ - keyLen);
    const gp_Ax2 keyAx(keyOrigin, gp::DZ());
    current = pr::cut(current, pr::box(keyAx, slotRadial, keyW, keyLen + kOver));

    // ── 3..N+2) Bolt holes on the bolt circle (SetRotation, sequential) ─
    const double pcdR  = in.bolt_circle_dia_mm / 2.0;
    const double boltR = in.bolt_dia_mm / 2.0;
    const gp_Pnt boltTpl(cx + pcdR, cy, zMin - kOver);
    const gp_Ax2 boltAxTpl(boltTpl, gp::DZ());
    const TopoDS_Shape boltTemplate = pr::cylinder(boltAxTpl, boltR, thru);

    const gp_Ax1 rotAxis(gp_Pnt(cx, cy, topZ), gp::DZ());
    for (int i = 0; i < in.bolt_count; ++i) {
        const double theta = (2.0 * M_PI * static_cast<double>(i)) /
                             static_cast<double>(in.bolt_count);
        gp_Trsf rot;
        rot.SetRotation(rotAxis, theta);
        BRepBuilderAPI_Transform xform(boltTemplate, rot, true);
        if (!xform.IsDone())
            throw SkillError("flange_coupling_bolt_circle: bolt rotation failed");
        current = pr::cut(current, xform.Shape());  // sequential — no compound
    }

    const double plateThk = (zMax - zMin);
    const double vBore = M_PI * boreR * boreR * plateThk;
    const double vKey  = slotRadial * keyW * keyLen;
    const double vBolts = static_cast<double>(in.bolt_count) *
                          M_PI * boltR * boltR * plateThk;
    const double volRemoved = vBore + vKey + vBolts;

    const int subfeatureCount = 1 + 1 + in.bolt_count;

    json bolts_json = json::array();
    for (int i = 0; i < in.bolt_count; ++i) {
        const double theta = (2.0 * M_PI * static_cast<double>(i)) /
                             static_cast<double>(in.bolt_count);
        bolts_json.push_back({
            { "x_mm",      cx + pcdR * std::cos(theta) },
            { "y_mm",      cy + pcdR * std::sin(theta) },
            { "theta_rad", theta },
        });
    }

    json params = {
        { "center_xy",          { in.center_xy.X(),
                                  in.center_xy.Y(),
                                  in.center_xy.Z() } },
        { "bore_dia_mm",        in.bore_dia_mm },
        { "key_length_mm",      in.key_length_mm },
        { "bolt_circle_dia_mm", in.bolt_circle_dia_mm },
        { "bolt_count",         in.bolt_count },
        { "bolt_dia_mm",        in.bolt_dia_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "powertrans_feature_type",    "rigid_flange_coupling" },
        { "subfeature_count",           subfeatureCount },
        { "bore_dia_mm",                in.bore_dia_mm },
        { "bolt_circle_dia_mm",         in.bolt_circle_dia_mm },
        { "bolt_count",                 in.bolt_count },
        { "bolt_positions",             bolts_json },
        { "derived_bore_h7_max_mm",     boreActual },
        { "derived_key_width_mm",       key->key_width_mm },
        { "derived_hub_keyway_depth_mm", key->hub_keyway_depth_mm },
        { "derived_key_tolerance_grade", key->tolerance_grade },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "ISO 286 (H7) + DIN 6885" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "boring_bar;broach;drill";
    tooling.tool_dia_mm       = std::max(in.bore_dia_mm, in.bolt_dia_mm);
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = 60.0 + 5.0 * static_cast<double>(in.bolt_count);
    tooling.extra = {
        { "powertrans_application", "rigid_flange_coupling" },
        { "key_tolerance_grade",   key->tolerance_grade },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::flange_coupling_bolt_circle: bore={} bolts={} faces {}→{}",
                  in.bore_dia_mm, in.bolt_count,
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

    // Geometric fallback: a central bore plus >= 3 bolt cylinders.
    int boreCyls = 0;
    int boltCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Cylinder c = s.Cylinder();
            if (std::abs(c.Axis().Direction().Z()) < 0.9) continue;
            const double radius = c.Radius();
            if (radius >= 8.0) ++boreCyls;
            else if (radius >= 1.5 && radius < 8.0) ++boltCyls;
        } catch (...) {}
    }
    if (boreCyls >= 1 && boltCyls >= 3) {
        json recovered = { { "bore_dia_mm", 25.0 },
                           { "bolt_count",  boltCyls } };
        json matched   = { { "source",     "geometric_flange_coupling" },
                           { "bore_cyls",  boreCyls },
                           { "bolt_cyls",  boltCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::flange_coupling_bolt_circle
