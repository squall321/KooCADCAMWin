// @lat: [[engine/skills#bottle_cage_boss_m5]]

#include "bottle_cage_boss_m5.hpp"

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

namespace koocadcam::skill::bottle_cage_boss_m5 {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
constexpr double kSpacingStdMm = 64.0;
constexpr double kSeatDepthMm  = 2.0;   // riv-nut flange seat depth
}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.boss_spacing_mm <= 0.0 || in.rivnut_seat_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "bottle_cage_boss_m5: all dimensions must be > 0");
        return r;
    }

    if (std::abs(in.boss_spacing_mm - kSpacingStdMm) > 1.0) {
        r.add("DFM-SPACING", "error",
              "bottle_cage_boss_m5: boss_spacing_mm " +
              std::to_string(in.boss_spacing_mm) +
              " is not the 64 mm ISO 4210 standard");
    }

    const auto* spec = thread_table::findMetric(in.thread_key);
    if (!spec) {
        r.add("DFM-THREAD", "error",
              "bottle_cage_boss_m5: thread_key '" + in.thread_key +
              "' not in central ISO M-thread table");
    } else if (in.rivnut_seat_dia_mm <= spec->tap_pilot_dia_mm) {
        r.add("DFM-SEAT", "error",
              "bottle_cage_boss_m5: rivnut_seat_dia_mm " +
              std::to_string(in.rivnut_seat_dia_mm) +
              " must exceed the M-thread pilot diameter " +
              std::to_string(spec->tap_pilot_dia_mm));
    }

    // Both bosses must fit inside the stock along X.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double seatR = in.rivnut_seat_dia_mm / 2.0;
    const double bx    = in.row_origin.X() + in.boss_spacing_mm;
    if (in.row_origin.X() - seatR < xMin - 1e-3 || bx + seatR > xMax + 1e-3) {
        r.add("DFM-SEAT", "error",
              "bottle_cage_boss_m5: boss pair overruns the stock in X");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "bottle_cage_boss_m5 DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto* spec = thread_table::findMetric(in.thread_key);
    if (!spec) throw SkillError("bottle_cage_boss_m5: thread lookup failed");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ  = zMax;
    const double thick = zMax - zMin;
    const double ay    = in.row_origin.Y();
    const double ax    = in.row_origin.X();
    const double bx    = ax + in.boss_spacing_mm;

    const double seatR  = in.rivnut_seat_dia_mm / 2.0;
    const double pilotR = spec->tap_pilot_dia_mm / 2.0;
    const double seatDepth = std::min(kSeatDepthMm, thick * 0.5);

    TopoDS_Shape current = wp.shape();

    // ── 1,2) Riv-nut seat counterbores (wide, shallow) — bosses A & B ─────
    for (const double cx : { ax, bx }) {
        const gp_Pnt seatStart(cx, ay, topZ - seatDepth);
        const gp_Ax2 seatAx(seatStart, gp::DZ());
        current = pr::cut(current,
            pr::cylinder(seatAx, seatR, seatDepth + kOver));
    }

    // ── 3,4) Tapped M5 pilots (narrow, deep — STACKED below each seat) ────
    const double pilotTopZ = topZ - seatDepth;       // meets seat bottom
    const double pilotLen  = pilotTopZ - zMin;
    for (const double cx : { ax, bx }) {
        const gp_Pnt pilotStart(cx, ay, zMin - kOver);
        const gp_Ax2 pilotAx(pilotStart, gp::DZ());
        current = pr::cut(current,
            pr::cylinder(pilotAx, pilotR, pilotLen + 2.0 * kOver));
    }

    // ── Derived volume (analytic) ────────────────────────────────────────
    const double vSeat  = M_PI * seatR * seatR * seatDepth;
    const double vPilot = M_PI * pilotR * pilotR * pilotLen;
    const double volRemoved = 2.0 * vSeat + 2.0 * vPilot;

    json params = {
        { "row_origin",        { in.row_origin.X(),
                                 in.row_origin.Y(),
                                 in.row_origin.Z() } },
        { "boss_spacing_mm",   in.boss_spacing_mm },
        { "thread_key",        in.thread_key },
        { "rivnut_seat_dia_mm", in.rivnut_seat_dia_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "bicycle_feature_type",       "bottle_cage_boss_pair_m5" },
        { "subfeature_count",           4 },
        { "boss_spacing_mm",            in.boss_spacing_mm },
        { "thread_key",                 in.thread_key },
        { "rivnut_seat_dia_mm",         in.rivnut_seat_dia_mm },
        { "derived_thread_major_mm",    spec->nominal_dia_mm },
        { "derived_pilot_dia_mm",       spec->tap_pilot_dia_mm },
        { "derived_boss_b_x_mm",        bx },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "ISO 4210 bottle-cage M5 x 64 mm" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "counterbore;drill;thread_tap";
    tooling.tool_dia_mm       = in.rivnut_seat_dia_mm;
    tooling.tool_length_mm    = thick + 10.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = 40.0;
    tooling.extra = {
        { "bicycle_feature_type", "bottle_cage_boss_pair_m5" },
        { "thread_key",           in.thread_key },
        { "standard",             "ISO 4210" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::bottle_cage_boss_m5: spacing={} thread={} seat Ø{}",
                  in.boss_spacing_mm, in.thread_key, in.rivnut_seat_dia_mm);

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
            { "bicycle_feature_type", "bottle_cage_boss_pair_m5" },
        };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric fallback: two riv-nut seats (~4 mm radius) plus two M5 pilots
    // (~2 mm radius), all +Z.
    int seatCyls  = 0;
    int pilotCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const double radius = s.Cylinder().Radius();
            if (radius >= 3.5 && radius <= 5.5) ++seatCyls;
            else if (radius >= 1.8 && radius < 3.0) ++pilotCyls;
        } catch (...) {}
    }
    if (seatCyls >= 2 && pilotCyls >= 2) {
        json recovered = { { "boss_spacing_mm", kSpacingStdMm },
                           { "thread_key",      "M5" } };
        json matched   = { { "source",     "geometric_boss_pair_pattern" },
                           { "seat_cyls",  seatCyls },
                           { "pilot_cyls", pilotCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::bottle_cage_boss_m5
