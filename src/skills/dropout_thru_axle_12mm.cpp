// @lat: [[engine/skills#dropout_thru_axle_12mm]]

#include "dropout_thru_axle_12mm.hpp"

#include "_iso_thread_table.hpp"
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

namespace koocadcam::skill::dropout_thru_axle_12mm {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
constexpr double kAxleStdMm = 12.0;
}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.axle_dia_mm <= 0.0 || in.hanger_bolt_dia_mm <= 0.0 ||
        in.hanger_offset_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "dropout_thru_axle_12mm: all dimensions must be > 0");
        return r;
    }

    if (std::abs(in.axle_dia_mm - kAxleStdMm) > 0.3) {
        r.add("DFM-AXLE", "error",
              "dropout_thru_axle_12mm: axle_dia_mm " +
              std::to_string(in.axle_dia_mm) +
              " is not the 12 mm thru-axle standard");
    }

    const auto* spec = thread_table::findMetric(in.thread_key);
    if (!spec) {
        r.add("DFM-THREAD", "error",
              "dropout_thru_axle_12mm: thread_key '" + in.thread_key +
              "' not in central ISO M-thread table");
    }

    // Hanger hole must stay inside the stock below the axle bore.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double hangerY = in.center_xy.Y() - in.hanger_offset_mm;
    const double hangerR = in.hanger_bolt_dia_mm / 2.0;
    if (hangerY - hangerR < yMin + 1e-3) {
        r.add("DFM-STOCK", "error",
              "dropout_thru_axle_12mm: hanger bolt hole at offset " +
              std::to_string(in.hanger_offset_mm) +
              " mm runs off the bottom of the stock");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "dropout_thru_axle_12mm DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto* spec = thread_table::findMetric(in.thread_key);
    if (!spec) throw SkillError("dropout_thru_axle_12mm: thread lookup failed");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double botZ = zMin;
    const double thick = zMax - zMin;
    const double cx   = in.center_xy.X();
    const double cy   = in.center_xy.Y();

    // The axle line is split into two STACKED coaxial sections along Z:
    //   - clearance bore (12 mm) in the upper half
    //   - threaded pilot bore (tap pilot dia) in the lower half
    const double axleR  = in.axle_dia_mm / 2.0;
    const double pilotR = spec->tap_pilot_dia_mm / 2.0;
    const double clearLen = thick * 0.5;

    // ── 1) 12 mm thru-axle clearance bore (upper section) ────────────────
    const gp_Pnt clearStart(cx, cy, topZ - clearLen);
    const gp_Ax2 clearAx(clearStart, gp::DZ());
    TopoDS_Shape current = pr::cut(
        wp.shape(),
        pr::cylinder(clearAx, axleR, clearLen + kOver));

    // ── 2) Threaded engagement bore (lower section, coaxial, stacked) ────
    const double threadTopZ = topZ - clearLen;       // meets clearance bore
    const double threadLen  = threadTopZ - botZ;
    const gp_Pnt threadStart(cx, cy, botZ - kOver);
    const gp_Ax2 threadAx(threadStart, gp::DZ());
    current = pr::cut(
        current,
        pr::cylinder(threadAx, pilotR, threadLen + 2.0 * kOver));

    // ── 3) Derailleur-hanger bolt hole (offset below, through) ───────────
    const double hangerR = in.hanger_bolt_dia_mm / 2.0;
    const double hangerY = cy - in.hanger_offset_mm;
    const gp_Pnt hangerStart(cx, hangerY, botZ - kOver);
    const gp_Ax2 hangerAx(hangerStart, gp::DZ());
    current = pr::cut(
        current,
        pr::cylinder(hangerAx, hangerR, thick + 2.0 * kOver));

    // ── Derived volume (analytic) ────────────────────────────────────────
    const double vClear  = M_PI * axleR * axleR * clearLen;
    const double vThread = M_PI * pilotR * pilotR * threadLen;
    const double vHanger = M_PI * hangerR * hangerR * thick;
    const double volRemoved = vClear + vThread + vHanger;

    json params = {
        { "center_xy",         { in.center_xy.X(),
                                 in.center_xy.Y(),
                                 in.center_xy.Z() } },
        { "axle_dia_mm",       in.axle_dia_mm },
        { "thread_key",        in.thread_key },
        { "hanger_bolt_dia_mm", in.hanger_bolt_dia_mm },
        { "hanger_offset_mm",  in.hanger_offset_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "bicycle_feature_type",       "rear_dropout_thru_axle_12mm" },
        { "subfeature_count",           3 },
        { "axle_dia_mm",                in.axle_dia_mm },
        { "thread_key",                 in.thread_key },
        { "hanger_bolt_dia_mm",         in.hanger_bolt_dia_mm },
        { "derived_thread_major_mm",    spec->nominal_dia_mm },
        { "derived_pilot_dia_mm",       spec->tap_pilot_dia_mm },
        { "derived_hanger_y_mm",        hangerY },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "12 mm thru-axle dropout" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill;thread_tap";
    tooling.tool_dia_mm       = in.axle_dia_mm;
    tooling.tool_length_mm    = thick + 15.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = 55.0;
    tooling.extra = {
        { "bicycle_feature_type", "rear_dropout_thru_axle_12mm" },
        { "thread_key",           in.thread_key },
        { "standard",             "12 mm thru-axle" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::dropout_thru_axle_12mm: axle Ø{} thread {} hanger Ø{}",
                  in.axle_dia_mm, in.thread_key, in.hanger_bolt_dia_mm);

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
            { "bicycle_feature_type", "rear_dropout_thru_axle_12mm" },
        };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric fallback: a ~6 mm-radius axle bore plus a smaller hanger
    // bolt hole, axes both +Z.
    int axleCyls   = 0;
    int hangerCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const double radius = s.Cylinder().Radius();
            if (radius >= 5.0 && radius <= 7.0) ++axleCyls;
            else if (radius >= 1.5 && radius < 5.0) ++hangerCyls;
        } catch (...) {}
    }
    if (axleCyls >= 1 && hangerCyls >= 1) {
        json recovered = { { "axle_dia_mm", kAxleStdMm },
                           { "thread_key",  "M12" } };
        json matched   = { { "source",      "geometric_axle_hanger_pattern" },
                           { "axle_cyls",   axleCyls },
                           { "hanger_cyls", hangerCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::dropout_thru_axle_12mm
