// @lat: [[engine/skills#shaft_collar_clamp_split]]

#include "shaft_collar_clamp_split.hpp"

#include "Workpiece.hpp"
#include "_iso286_fits.hpp"
#include "_iso_thread_table.hpp"
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

namespace koocadcam::skill::shaft_collar_clamp_split {

namespace pr = koocadcam::engine::prim;
namespace tt = koocadcam::skill::thread_table;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.bore_dia_mm <= 0.0 || in.collar_od_mm <= 0.0 ||
        in.split_width_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "shaft_collar_clamp_split: bore/od/split must be > 0");
        return r;
    }

    if (in.clamp_thread_key.empty()) {
        r.add("DFM-PT-THREAD", "error",
              "shaft_collar_clamp_split: clamp_thread_key is empty");
    } else if (!tt::findMetric(in.clamp_thread_key)) {
        r.add("DFM-PT-THREAD", "error",
              "shaft_collar_clamp_split: clamp_thread_key '" +
              in.clamp_thread_key +
              "' not in central metric thread table");
    }

    const double wallThk = (in.collar_od_mm - in.bore_dia_mm) / 2.0;
    if (in.collar_od_mm <= in.bore_dia_mm || wallThk < 2.0) {
        r.add("DFM-PT-WALL", "error",
              "shaft_collar_clamp_split: wall thickness " +
              std::to_string(wallThk) +
              " mm < 2 mm (collar_od too close to bore_dia)");
    }

    if (in.split_width_mm >= wallThk) {
        r.add("DFM-PT-SPLIT", "error",
              "shaft_collar_clamp_split: split_width_mm (" +
              std::to_string(in.split_width_mm) +
              ") must be < wall thickness (" +
              std::to_string(wallThk) + ")");
    }

    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "shaft_collar_clamp_split DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const tt::MetricThreadSpec* thr = tt::findMetric(in.clamp_thread_key);
    if (!thr) throw SkillError("shaft_collar_clamp_split: thread lookup failed");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double cx = in.axis_origin.X();
    const double cy = in.axis_origin.Y();
    const double thru = (zMax - zMin) + 2.0 * kOver;
    const double odR  = in.collar_od_mm / 2.0;

    // ── 1) Central H7 bore (big feature first) ─────────────────────────
    const double boreActual = iso286::h7_max_mm(in.bore_dia_mm);
    const double boreR = boreActual / 2.0;
    const gp_Ax2 boreAx(gp_Pnt(cx, cy, zMin - kOver), gp::DZ());
    TopoDS_Shape current = pr::cut(wp.shape(), pr::cylinder(boreAx, boreR, thru));

    // ── 2) Axial split slot — thin radial box through the +X wall ──────
    // Box main dir = +Z (axial, full collar length), XDir = +X (radial),
    // so DX = radial extent (bore→OD+over), DY = split_width, DZ = length.
    const double slotRadial = odR + kOver;  // from center out past the OD
    const double splitW     = in.split_width_mm;
    const gp_Pnt slotOrigin(cx, cy - splitW / 2.0, zMin - kOver);
    const gp_Ax2 slotAx(slotOrigin, gp::DZ());
    current = pr::cut(current, pr::box(slotAx, slotRadial, splitW, thru));

    // ── 3) Transverse clamp-screw pilot bore (perpendicular to axis) ───
    // Drilled along +Y at mid-height, on the +X side, crossing the split so
    // the screw pulls the two clamp ears together.  Axis = +Y direction.
    const double clampR  = thr->tap_pilot_dia_mm / 2.0;
    const double midZ    = (zMin + zMax) / 2.0;
    const double earX    = cx + (boreR + odR) / 2.0;  // through the +X ear
    const gp_Pnt clampStart(earX, cy - odR - kOver, midZ);
    const gp_Ax2 clampAx(clampStart, gp_Dir(0.0, 1.0, 0.0));
    const double clampLen = 2.0 * odR + 2.0 * kOver;
    current = pr::cut(current, pr::cylinder(clampAx, clampR, clampLen));

    const double plateThk = (zMax - zMin);
    const double vBore  = M_PI * boreR * boreR * plateThk;
    const double vSplit = slotRadial * splitW * plateThk;
    const double vClamp = M_PI * clampR * clampR * clampLen;
    const double volRemoved = vBore + vSplit + vClamp;

    json params = {
        { "axis_origin",      { in.axis_origin.X(),
                                in.axis_origin.Y(),
                                in.axis_origin.Z() } },
        { "bore_dia_mm",      in.bore_dia_mm },
        { "collar_od_mm",     in.collar_od_mm },
        { "split_width_mm",   in.split_width_mm },
        { "clamp_thread_key", in.clamp_thread_key },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "powertrans_feature_type",    "clamp_split_shaft_collar" },
        { "subfeature_count",           3 },
        { "bore_dia_mm",                in.bore_dia_mm },
        { "collar_od_mm",               in.collar_od_mm },
        { "split_width_mm",             in.split_width_mm },
        { "clamp_thread_key",           in.clamp_thread_key },
        { "derived_bore_h7_max_mm",     boreActual },
        { "derived_clamp_pilot_dia_mm", thr->tap_pilot_dia_mm },
        { "derived_clamp_nominal_dia_mm", thr->nominal_dia_mm },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "ISO 286 (H7) + ISO 965 (pilot)" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "boring_bar;slot_mill;drill;tap";
    tooling.tool_dia_mm       = std::max(in.bore_dia_mm, thr->nominal_dia_mm);
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 180.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = 75.0;
    tooling.extra = {
        { "powertrans_application", "clamp_split_shaft_collar" },
        { "clamp_thread_key",      in.clamp_thread_key },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::shaft_collar_clamp_split: bore={} od={} thread={} faces {}→{}",
                  in.bore_dia_mm, in.collar_od_mm, in.clamp_thread_key,
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

    // Geometric fallback: a central axial bore plus a transverse (non-Z)
    // clamp-screw cylinder.
    int axialBore   = 0;
    int transverse  = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Cylinder c = s.Cylinder();
            const double az = std::abs(c.Axis().Direction().Z());
            if (az > 0.9 && c.Radius() >= 3.0) ++axialBore;
            else if (az < 0.3) ++transverse;
        } catch (...) {}
    }
    if (axialBore >= 1 && transverse >= 1) {
        json recovered = { { "bore_dia_mm",      20.0 },
                           { "clamp_thread_key", "M6" } };
        json matched   = { { "source",        "geometric_clamp_collar" },
                           { "axial_bore",    axialBore },
                           { "transverse",    transverse } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::shaft_collar_clamp_split
