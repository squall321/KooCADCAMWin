// @lat: [[engine/skills#thru_hull_seacock_flange]]

#include "thru_hull_seacock_flange.hpp"

#include "Workpiece.hpp"
#include "_as568_table.hpp"
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

namespace koocadcam::skill::thru_hull_seacock_flange {

namespace pr = koocadcam::engine::prim;
namespace as = koocadcam::skill::as568;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
constexpr int    kBoltCount = 3;
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.hull_bore_dia_mm <= 0.0 || in.flange_dia_mm <= 0.0 ||
        in.bolt_circle_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "thru_hull_seacock_flange: all dimensions must be > 0");
    }

    if (in.o_ring_size_key.empty()) {
        r.add("DFM-AS568", "error",
              "thru_hull_seacock_flange: o_ring_size_key is empty");
    } else if (!as::findDash(in.o_ring_size_key)) {
        r.add("DFM-AS568", "error",
              "thru_hull_seacock_flange: o_ring_size_key '" +
              in.o_ring_size_key +
              "' not present in central _as568_table.hpp");
    }

    if (in.bolt_circle_dia_mm <= in.hull_bore_dia_mm) {
        r.add("DFM-MARINE-PCD", "error",
              "thru_hull_seacock_flange: bolt_circle_dia_mm must exceed "
              "hull_bore_dia_mm (bolts would intersect the bore)");
    }
    if (in.bolt_circle_dia_mm >= in.flange_dia_mm) {
        r.add("DFM-MARINE-PCD", "error",
              "thru_hull_seacock_flange: bolt_circle_dia_mm must be < "
              "flange_dia_mm (bolts must land within the flange seat)");
    }

    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "thru_hull_seacock_flange DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const as::DashSpec* ring = as::findDash(in.o_ring_size_key);
    if (!ring) throw SkillError("thru_hull_seacock_flange: AS568 lookup failed");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double cx = in.center_xy.X();
    const double cy = in.center_xy.Y();

    // ── 1) Thru-hull central through-bore ──────────────────────────────
    const double boreR = in.hull_bore_dia_mm / 2.0;
    const double thru   = (zMax - zMin) + 2.0 * kOver;
    const gp_Ax2 boreAx(gp_Pnt(cx, cy, zMin - kOver), gp::DZ());
    TopoDS_Shape current = pr::cut(wp.shape(),
                                   pr::cylinder(boreAx, boreR, thru));

    // ── 2) Flange seat counterbore (wider, shallow seat on top face) ───
    const double seatR     = in.flange_dia_mm / 2.0;
    const double seatDepth = std::max(2.0, ring->groove_depth_mm + 1.5);
    const gp_Ax2 seatAx(gp_Pnt(cx, cy, topZ - seatDepth), gp::DZ());
    current = pr::cut(current, pr::cylinder(seatAx, seatR, seatDepth + kOver));

    // ── 3) AS568 O-ring face groove on the flange seat ─────────────────
    // Groove sits between the bore and the bolt circle.
    const double grooveID = in.hull_bore_dia_mm + 3.0;
    const double grooveOD = grooveID + 2.0 * ring->groove_width_mm;
    const double grooveD  = ring->groove_depth_mm;
    const gp_Ax2 grooveAx(gp_Pnt(cx, cy, topZ - seatDepth - grooveD), gp::DZ());
    current = pr::cut(current,
                      pr::annularRing(grooveAx, grooveOD / 2.0,
                                      grooveID / 2.0, grooveD + kOver));

    // ── 4..6) Three bolt holes on PCD via SEQUENTIAL SetRotation ───────
    const double pcdR  = in.bolt_circle_dia_mm / 2.0;
    const double boltR = std::max(2.0, in.hull_bore_dia_mm * 0.12);
    const gp_Pnt boltEntryTpl(cx + pcdR, cy, zMin - kOver);
    const gp_Ax2 boltAxTpl(boltEntryTpl, gp::DZ());
    const TopoDS_Shape boltTemplate = pr::cylinder(boltAxTpl, boltR, thru);

    const gp_Ax1 rotAxis(gp_Pnt(cx, cy, topZ), gp::DZ());
    for (int i = 0; i < kBoltCount; ++i) {
        const double theta = (2.0 * M_PI * static_cast<double>(i)) /
                             static_cast<double>(kBoltCount);  // 0,120,240 deg
        gp_Trsf rot;
        rot.SetRotation(rotAxis, theta);
        BRepBuilderAPI_Transform xform(boltTemplate, rot, true);
        if (!xform.IsDone())
            throw SkillError("thru_hull_seacock_flange: bolt rotation failed");
        current = pr::cut(current, xform.Shape());  // sequential — no compound
    }

    const TopoDS_Shape newShape = current;

    // ── Derived volume (analytic) ──────────────────────────────────────
    const double plateThk = (zMax - zMin);
    const double vBore   = M_PI * boreR * boreR * plateThk;
    const double vSeat   = M_PI * (seatR * seatR - boreR * boreR) * seatDepth;
    const double vGroove = M_PI *
        ((grooveOD * 0.5) * (grooveOD * 0.5) -
         (grooveID * 0.5) * (grooveID * 0.5)) * grooveD;
    const double vBolts  = static_cast<double>(kBoltCount) *
                           M_PI * boltR * boltR * plateThk;
    const double volRemoved = vBore + std::max(0.0, vSeat) + vGroove + vBolts;

    json bolts_json = json::array();
    for (int i = 0; i < kBoltCount; ++i) {
        const double theta = (2.0 * M_PI * static_cast<double>(i)) /
                             static_cast<double>(kBoltCount);
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
        { "hull_bore_dia_mm",   in.hull_bore_dia_mm },
        { "flange_dia_mm",      in.flange_dia_mm },
        { "bolt_circle_dia_mm", in.bolt_circle_dia_mm },
        { "o_ring_size_key",    in.o_ring_size_key },
    };

    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "marine_feature_type",        "thru_hull_seacock_flange" },
        { "subfeature_count",           6 },
        { "hull_bore_dia_mm",           in.hull_bore_dia_mm },
        { "flange_dia_mm",              in.flange_dia_mm },
        { "bolt_circle_dia_mm",         in.bolt_circle_dia_mm },
        { "o_ring_dash",                in.o_ring_size_key },
        { "bolt_count",                 kBoltCount },
        { "bolt_positions",             bolts_json },
        { "derived_seat_depth_mm",      seatDepth },
        { "derived_groove_id_mm",       grooveID },
        { "derived_groove_od_mm",       grooveOD },
        { "derived_bolt_hole_dia_mm",   2.0 * boltR },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "AS568 + marine seacock practice" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill;counterbore;groove_mill";
    tooling.tool_dia_mm       = std::max(in.flange_dia_mm, in.hull_bore_dia_mm);
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = 70.0 + 5.0 * static_cast<double>(kBoltCount);
    tooling.extra = {
        { "marine_application", "thru_hull_seacock" },
        { "removed_volume_mm3", volRemoved },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::thru_hull_seacock_flange: bore={} flange={} pcd={} faces {}→{}",
                  in.hull_bore_dia_mm, in.flange_dia_mm,
                  in.bolt_circle_dia_mm, wp.faceCount(), wpNew->faceCount());

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

    // Geometric fallback: one large central +Z bore plus >= 3 small bolt
    // cylinders on a common PCD.
    int centralBore = 0;
    int boltCyls    = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Cylinder c = s.Cylinder();
            if (std::abs(c.Axis().Direction().Z()) < 0.9) continue;
            const double radius = c.Radius();
            if (radius >= 10.0) ++centralBore;
            else if (radius >= 2.0 && radius < 10.0) ++boltCyls;
        } catch (...) {}
    }
    if (centralBore >= 1 && boltCyls >= 3) {
        json recovered = { { "hull_bore_dia_mm", 38.0 },
                           { "o_ring_size_key",  "-116" } };
        json matched   = { { "source",     "geometric_flange_pattern" },
                           { "bore_cyls",  centralBore },
                           { "bolt_cyls",  boltCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::thru_hull_seacock_flange
