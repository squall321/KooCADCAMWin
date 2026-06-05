// @lat: [[engine/skills#tractor_pto_spline_compound]]

#include "tractor_pto_spline_compound.hpp"

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

#include <algorithm>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::tractor_pto_spline_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

struct PtoTypeSpec {
    const char* type_key;
    int         spline_count;
    double      nominal_dia_mm;
    double      rpm;
    double      slot_width_ratio;     // slot width / chord-from-N
    double      slot_depth_ratio;     // slot depth / radius
};

constexpr PtoTypeSpec kPtoTypes[] {
    // ASAE S203.14: Type1 = 6 spline 1-3/8" (34.92mm) @540 RPM, deep involute
    { "type1_6spline",  6,  34.92,  540.0, 0.50, 0.10 },
    // Type 2 = 21 spline 1-3/8" (34.92mm) @1000 RPM, fine straight-sided
    { "type2_21spline", 21, 34.92, 1000.0, 0.50, 0.06 },
    // Type 3 = 20 spline 1-3/4" (44.45mm) @1000 RPM, fine involute
    { "type3_20spline", 20, 44.45, 1000.0, 0.50, 0.06 },
};

const PtoTypeSpec* findPto(const std::string& key)
{
    for (const auto& s : kPtoTypes) {
        if (key == s.type_key) return &s;
    }
    return nullptr;
}

}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.shaft_dia_mm <= 0.0 || in.spline_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "tractor_pto_spline_compound: positive shaft_dia and "
              "spline_length required");
    }
    const PtoTypeSpec* spec = findPto(in.pto_type);
    if (!spec) {
        r.add("DFM-PTO-TYPE", "error",
              "tractor_pto_spline_compound: pto_type '" + in.pto_type +
              "' invalid (must be type1_6spline | type2_21spline | type3_20spline)");
    } else {
        if (in.shaft_dia_mm > 0.0
            && std::abs(in.shaft_dia_mm - spec->nominal_dia_mm) > 1.0) {
            r.add("DFM-PTO-DIA", "error",
                  "tractor_pto_spline_compound: shaft_dia " +
                  std::to_string(in.shaft_dia_mm) +
                  " mm does not match ASAE nominal " +
                  std::to_string(spec->nominal_dia_mm) + " mm for " +
                  in.pto_type);
        }
    }
    if (in.spline_length_mm > 0.0 && in.spline_length_mm < 10.0) {
        r.add("DFM-PTO-LEN", "error",
              "tractor_pto_spline_compound: spline_length < 10 mm too short");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "tractor_pto_spline_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const PtoTypeSpec* spec = findPto(in.pto_type);
    if (!spec) throw SkillError("tractor_pto_spline_compound: PTO lookup failed");

    const int    N       = spec->spline_count;
    const double shaftR  = in.shaft_dia_mm / 2.0;
    const double slotW   = (2.0 * M_PI * shaftR / static_cast<double>(N))
                           * spec->slot_width_ratio;
    const double slotD   = shaftR * spec->slot_depth_ratio;
    const double overhang = 0.1;

    // Build the BASE slot cutter at theta = 0 (along +X), then SEQUENTIALLY
    // apply gp_Trsf::SetRotation(z-axis, theta_i) and pr::cut for each slot.
    // The base cutter is a small radial cylinder positioned so it bites the
    // shaft OD at the rotated angle.
    const double rCut = std::max(slotW * 0.5, slotD * 0.5) + 0.05;
    const double rPos = shaftR + overhang - rCut;

    const gp_Pnt baseCutterCenter(
        in.shaft_origin.X() + rPos,
        in.shaft_origin.Y(),
        in.shaft_origin.Z() - overhang);
    const gp_Ax2 baseAx(baseCutterCenter, gp::DZ());
    const TopoDS_Shape baseCutter = pr::cylinder(
        baseAx, rCut, in.spline_length_mm + 2.0 * overhang);

    const gp_Ax1 zAxis(in.shaft_origin, gp::DZ());

    TopoDS_Shape current = wp.shape();
    for (int i = 0; i < N; ++i) {
        const double theta = (2.0 * M_PI * static_cast<double>(i))
                             / static_cast<double>(N);
        gp_Trsf t;
        t.SetRotation(zAxis, theta);
        BRepBuilderAPI_Transform mover(baseCutter, t, /*copy=*/true);
        if (!mover.IsDone())
            throw SkillError("tractor_pto_spline_compound: rotation transform failed");
        const TopoDS_Shape rotatedCutter = mover.Shape();
        // SEQUENTIAL pr::cut — one tool at a time, no compound.
        current = pr::cut(current, rotatedCutter);
    }

    const double volRemoved = N * (M_PI * rCut * rCut) * in.spline_length_mm * 0.5;

    json params = {
        { "pto_type",         in.pto_type },
        { "shaft_dia_mm",     in.shaft_dia_mm },
        { "spline_length_mm", in.spline_length_mm },
        { "shaft_origin",     { in.shaft_origin.X(),
                                in.shaft_origin.Y(),
                                in.shaft_origin.Z() } },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "ag_feature_type",            "tractor_pto_spline" },
        { "subfeature_count",           N },
        { "pto_type",                   in.pto_type },
        { "spline_count",               N },
        { "nominal_dia_mm",             spec->nominal_dia_mm },
        { "rpm",                        spec->rpm },
        { "spline_length_mm",           in.spline_length_mm },
        { "derived_slot_width_mm",      slotW },
        { "derived_slot_depth_mm",      slotD },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "ASAE S203" },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "spline_hob";
    tooling.tool_dia_mm       = in.shaft_dia_mm;
    tooling.tool_material     = "hss";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 80.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(60.0, 30.0 + N * 2.0);
    tooling.extra = {
        { "ag_application", "tractor_pto" },
        { "standard",       "ASAE S203" },
        { "rpm",            spec->rpm },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::tractor_pto_spline_compound: type={} N={} dia={}",
                  in.pto_type, N, in.shaft_dia_mm);

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

    // Geometric: count cylindrical flank faces.  Each spline slot leaves
    // 1 cylindrical face (the cutter footprint).  For N=6 → ~6, N=20/21 → ~20.
    int splineCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const double radius = s.Cylinder().Radius();
            if (radius >= 0.5 && radius <= 5.0) ++splineCyls;
        } catch (...) {}
    }
    if (splineCyls >= 5) {
        std::string typeGuess = "type1_6spline";
        if (splineCyls >= 18) typeGuess = "type3_20spline";
        if (splineCyls >= 20) typeGuess = "type2_21spline";
        json recovered = { { "pto_type", typeGuess },
                           { "spline_count_estimated", splineCyls } };
        json matched   = { { "source", "geometric_spline_count" },
                           { "cyl_face_count", splineCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::tractor_pto_spline_compound
