// @lat: [[engine/skills#int_retaining_ring_groove_din472]]

#include "int_retaining_ring_groove_din472.hpp"

#include "_retaining_rings.hpp"
#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
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
#include <limits>
#include <memory>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::int_retaining_ring_groove_din472 {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;
using retaining_rings::findDin472;
using retaining_rings::Din472Spec;

namespace {

// Find the smallest-radius cylindrical face along Z axis (= the bore).
// Returns radius <= 0 if no bore found.
double smallestBoreRadius(const Workpiece& wp)
{
    double bestR = std::numeric_limits<double>::max();
    bool found = false;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface s(wp.face(i));
        const gp_Cylinder c = s.Cylinder();
        const gp_Dir d = c.Axis().Direction();
        if (std::abs(std::abs(d.Z()) - 1.0) > 1e-3) continue;
        // Axis must be near the part axis (0,0,*).
        const gp_Pnt p = c.Axis().Location();
        if (std::sqrt(p.X() * p.X() + p.Y() * p.Y()) > 1.0) continue;
        if (c.Radius() < bestR) { bestR = c.Radius(); found = true; }
    }
    return found ? bestR : -1.0;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.ring_size_key.empty()) {
        r.add("DFM-DIN472", "error",
              "int_retaining_ring_groove_din472: ring_size_key is empty");
        return r;
    }
    const Din472Spec* spec = findDin472(in.ring_size_key);
    if (!spec) {
        r.add("DFM-DIN472", "error",
              "int_retaining_ring_groove_din472: unknown DIN 472 size '" +
              in.ring_size_key + "'");
        return r;
    }

    if (wp.shape().IsNull()) {
        r.add("DFM-INPUT", "error",
              "int_retaining_ring_groove_din472: workpiece is null");
        return r;
    }

    // Need an existing bore.
    const double boreR = smallestBoreRadius(wp);
    if (boreR <= 0.0) {
        r.add("DFM-DIN472-BORE", "error",
              "int_retaining_ring_groove_din472: no internal cylindrical "
              "bore detected on workpiece (drill it first)");
        return r;
    }
    // Bore Ø should roughly match nominal (within 30%).
    const double err = std::abs(2.0 * boreR - spec->bore_dia_mm) /
                       spec->bore_dia_mm;
    if (err > 0.30) {
        r.add("DFM-DIN472-FIT", "warning",
              "int_retaining_ring_groove_din472: bore Ø " +
              std::to_string(2.0 * boreR) +
              " differs from DIN 472 nominal " +
              std::to_string(spec->bore_dia_mm) + " by >30%");
    }

    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double zLow  = in.bore_z_position_mm - spec->groove_width_mm / 2.0;
    const double zHigh = in.bore_z_position_mm + spec->groove_width_mm / 2.0;
    if (zLow < bb.zMin - 1e-3 || zHigh > bb.zMax + 1e-3) {
        r.add("DFM-DIN472-Z", "error",
              "int_retaining_ring_groove_din472: groove Z zone outside stock Z");
    }

    auto faceId = wp.resolve(in.bore_face_id);
    if (!faceId) {
        r.add("DFM-INPUT", "warning",
              "int_retaining_ring_groove_din472: bore_face_id unresolved "
              "(falling back to smallest cylindrical face)");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "int_retaining_ring_groove_din472 DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const Din472Spec* spec = findDin472(in.ring_size_key);
    const double boreR = smallestBoreRadius(wp);
    const double grooveR = boreR + spec->groove_depth_mm;

    constexpr double kOverhangR = 0.01;   // tiny overlap on inner side
    const double startZ = in.bore_z_position_mm - spec->groove_width_mm / 2.0;
    const double height = spec->groove_width_mm;
    const gp_Ax2 ax(gp_Pnt(0.0, 0.0, startZ), gp::DZ());
    // Annular cutter: outer = grooveR, inner = boreR − tiny (so it
    // overlaps the existing bore wall and cleanly cuts only into the
    // surrounding material).
    const TopoDS_Shape cutter = pr::annularRing(ax,
                                                grooveR,
                                                std::max(0.05, boreR - kOverhangR),
                                                height);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    const double ringVol =
        M_PI * (grooveR * grooveR - boreR * boreR) * spec->groove_width_mm;

    json params = {
        { "bore_face_id_kind",     "datum" },
        { "bore_z_position_mm",    in.bore_z_position_mm },
        { "ring_size_key",         in.ring_size_key },
    };
    json derived = {
        { "bore_dia_mm",           spec->bore_dia_mm },
        { "groove_dia_mm",         spec->groove_dia_mm },
        { "groove_width_mm",       spec->groove_width_mm },
        { "groove_depth_mm",       spec->groove_depth_mm },
        { "ring_thickness_mm",     spec->ring_thickness_mm },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "ring_standard",         kRingStandard },
        { "ring_size_key",         in.ring_size_key },
        { "derived_dims",          derived },
        { "ring_volume_mm3",       ringVol },
        { "center_z_mm",           in.bore_z_position_mm },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "boring_bar_groove_insert";
    tooling.tool_dia_mm       = spec->groove_width_mm;
    tooling.tool_length_mm    = spec->groove_depth_mm + 10.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 1;
    tooling.cutting_speed_sfm = 350.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 = ringVol;
    tooling.est_cycle_time_s  = std::max(1.5, spec->groove_depth_mm / 3.0);
    tooling.extra = {
        { "standard",      kRingStandard },
        { "ring_size_key", in.ring_size_key },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::int_retaining_ring_groove_din472 applied: size={} d={} w={}",
                  in.ring_size_key, spec->groove_depth_mm, spec->groove_width_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (const auto& feat : wp.features()) {
        if (feat.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = feat.params;
        r.confidence       = 1.0;
        r.matched_geometry = {
            { "source",  "metadata_replay" },
            { "pattern", feat.pattern },
        };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric fallback: find paired cylindrical faces (bore + larger
    // recess) along Z; recess radius − bore radius = groove depth.
    if (wp.shape().IsNull()) return out;
    struct CylR { int idx; double r; };
    std::vector<CylR> cyls;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface s(wp.face(i));
        const gp_Cylinder c = s.Cylinder();
        const gp_Dir d = c.Axis().Direction();
        if (std::abs(std::abs(d.Z()) - 1.0) > 1e-3) continue;
        const gp_Pnt p = c.Axis().Location();
        if (std::sqrt(p.X() * p.X() + p.Y() * p.Y()) > 1.0) continue;
        cyls.push_back({ i, c.Radius() });
    }
    if (cyls.size() < 2) return out;

    // Sort by radius ascending; pair (smallest = bore) with next larger.
    std::sort(cyls.begin(), cyls.end(),
              [](const CylR& a, const CylR& b) { return a.r < b.r; });
    const double boreR = cyls.front().r;
    for (size_t i = 1; i < cyls.size(); ++i) {
        const double grooveR = cyls[i].r;
        const double depthEst = grooveR - boreR;
        if (depthEst <= 0.05) continue;
        std::string bestKey;
        double bestErr = 1e9;
        for (const auto& s : retaining_rings::kDin472) {
            const double err = std::abs(s.groove_depth_mm - depthEst);
            if (err < bestErr) { bestErr = err; bestKey = s.size_key; }
        }
        if (bestKey.empty() || bestErr > 0.5) continue;

        json recovered = {
            { "ring_size_key",      bestKey },
            { "bore_z_position_mm", 0.0 },
        };
        json matched = {
            { "source",            "geometric_fallback" },
            { "bore_face_id",      cyls.front().idx },
            { "groove_face_id",    cyls[i].idx },
            { "groove_depth_est",  depthEst },
            { "depth_match_err",   bestErr },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.55, matched });
        break;
    }
    return out;
}

}  // namespace koocadcam::skill::int_retaining_ring_groove_din472
