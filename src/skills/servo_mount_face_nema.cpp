// @lat: [[engine/skills#servo_mount_face_nema]]

#include "servo_mount_face_nema.hpp"

#include "Workpiece.hpp"
#include "_iso_thread_table.hpp"
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

namespace koocadcam::skill::servo_mount_face_nema {

namespace pr = koocadcam::engine::prim;
namespace tt = koocadcam::skill::thread_table;
using nlohmann::json;

namespace {

constexpr double kOver = 0.1;

struct NemaSpec {
    const char* nema_size;     // "17", "23", "34"
    double      bolt_square_mm; // corner-to-corner bolt square pitch
    double      pilot_dia_mm;   // motor spigot register diameter
};

constexpr NemaSpec kNema[] {
    { "17", 31.00, 22.0 },
    { "23", 47.14, 38.1 },
    { "34", 69.60, 73.0 },
};

const NemaSpec* findNema(const std::string& size)
{
    for (const auto& s : kNema) if (size == s.nema_size) return &s;
    return nullptr;
}

}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.pilot_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "servo_mount_face_nema: pilot_dia_mm must be > 0");
    }

    if (!findNema(in.nema_size)) {
        r.add("DFM-NEMA", "error",
              "servo_mount_face_nema: nema_size '" + in.nema_size +
              "' invalid (must be 17 | 23 | 34)");
    }

    if (!tt::findMetric(in.bolt_thread_key)) {
        r.add("DFM-THREAD", "error",
              "servo_mount_face_nema: bolt_thread_key '" +
              in.bolt_thread_key + "' not in central metric thread table");
    }

    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "servo_mount_face_nema DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const NemaSpec* nema = findNema(in.nema_size);
    if (!nema) throw SkillError("servo_mount_face_nema: NEMA lookup failed");
    const tt::MetricThreadSpec* thread = tt::findMetric(in.bolt_thread_key);
    if (!thread) throw SkillError("servo_mount_face_nema: thread lookup failed");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double cx = in.center_xy.X();
    const double cy = in.center_xy.Y();
    const double thru = (zMax - zMin) + 2.0 * kOver;

    // ── 1) Central pilot boss bore — biggest feature first ─────────────
    const double pilotR = in.pilot_dia_mm / 2.0;
    const gp_Ax2 pilotAx(gp_Pnt(cx, cy, zMin - kOver), gp::DZ());
    TopoDS_Shape current = pr::cut(wp.shape(), pr::cylinder(pilotAx, pilotR, thru));

    // ── 2..5) 4 corner bolt holes on the NEMA bolt square ──────────────
    const double half = nema->bolt_square_mm / 2.0;
    const double boltR = thread->clearance_medium_mm / 2.0;
    const double corners[4][2] = {
        { +half, +half }, { -half, +half }, { -half, -half }, { +half, -half }
    };
    json holes_json = json::array();
    for (const auto& cnr : corners) {
        const double hx = cx + cnr[0];
        const double hy = cy + cnr[1];
        const gp_Ax2 holeAx(gp_Pnt(hx, hy, zMin - kOver), gp::DZ());
        current = pr::cut(current, pr::cylinder(holeAx, boltR, thru));  // sequential
        holes_json.push_back({ { "x_mm", hx }, { "y_mm", hy } });
    }

    const TopoDS_Shape newShape = current;

    // ── Derived volume (analytic) ──────────────────────────────────────
    const double plateThk = (zMax - zMin);
    const double vPilot = M_PI * pilotR * pilotR * plateThk;
    const double vBolts = 4.0 * M_PI * boltR * boltR * plateThk;
    const double volRemoved = vPilot + vBolts;

    const int subfeatureCount = 1 + 4;

    json params = {
        { "center_xy",       { in.center_xy.X(),
                               in.center_xy.Y(),
                               in.center_xy.Z() } },
        { "nema_size",       in.nema_size },
        { "pilot_dia_mm",    in.pilot_dia_mm },
        { "bolt_thread_key", in.bolt_thread_key },
    };

    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "robotics_feature_type",      "nema_servo_mount_face" },
        { "subfeature_count",           subfeatureCount },
        { "nema_size",                  in.nema_size },
        { "pilot_dia_mm",               in.pilot_dia_mm },
        { "bolt_thread_key",            in.bolt_thread_key },
        { "bolt_positions",             holes_json },
        { "derived_bolt_square_mm",     nema->bolt_square_mm },
        { "derived_bolt_clearance_mm",  thread->clearance_medium_mm },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "NEMA motor mount face pattern" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "boring_bar;drill";
    tooling.tool_dia_mm       = std::max(in.pilot_dia_mm, thread->clearance_medium_mm);
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 230.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = 50.0;
    tooling.extra = {
        { "robotics_application", "nema_servo_mount" },
        { "removed_volume_mm3",   volRemoved },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::servo_mount_face_nema: nema={} pilot={} faces {}→{}",
                  in.nema_size, in.pilot_dia_mm,
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

    // Geometric fallback: one large +Z pilot bore plus exactly-ish 4 bolt
    // cylinders.
    int pilotCyls = 0;
    int boltCyls  = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Cylinder c = s.Cylinder();
            if (std::abs(c.Axis().Direction().Z()) < 0.9) continue;
            const double radius = c.Radius();
            if (radius >= 9.0) ++pilotCyls;
            else if (radius >= 1.5 && radius < 9.0) ++boltCyls;
        } catch (...) {}
    }
    if (pilotCyls >= 1 && boltCyls >= 4) {
        json recovered = { { "nema_size",    "23" },
                           { "pilot_dia_mm", 38.1 } };
        json matched   = { { "source",     "geometric_nema_mount" },
                           { "pilot_cyls", pilotCyls },
                           { "bolt_cyls",  boltCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::servo_mount_face_nema
