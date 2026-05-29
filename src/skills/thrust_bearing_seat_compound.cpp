// @lat: [[engine/skills#thrust_bearing_seat_compound]]

#include "thrust_bearing_seat_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::thrust_bearing_seat_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.outer_dia_mm <= 0.0)
        r.add("DFM-INPUT", "error", std::string(kSkillId) +
              ": outer_dia_mm must be > 0");
    if (in.inner_dia_mm <= 0.0)
        r.add("DFM-INPUT", "error", std::string(kSkillId) +
              ": inner_dia_mm must be > 0");
    if (in.face_depth_mm <= 0.0)
        r.add("DFM-INPUT", "error", std::string(kSkillId) +
              ": face_depth_mm must be > 0");
    if (in.seat_depth_mm <= 0.0)
        r.add("DFM-INPUT", "error", std::string(kSkillId) +
              ": seat_depth_mm must be > 0");

    if (in.outer_dia_mm > 0.0 && in.inner_dia_mm > 0.0 &&
        in.inner_dia_mm >= in.outer_dia_mm) {
        r.add("DFM-SEAT-GEOM", "error", std::string(kSkillId) +
              ": inner_dia (" + std::to_string(in.inner_dia_mm) +
              ") must be < outer_dia (" + std::to_string(in.outer_dia_mm) + ")");
    }

    const double race_width = in.outer_dia_mm - in.inner_dia_mm;
    if (race_width > 0.0 && race_width < 1.6) {
        r.add("DFM-002", "error", std::string(kSkillId) +
              ": race width (od-id) " + std::to_string(race_width) +
              " mm < min 1.6 mm — insufficient bearing-race width");
    }

    if (in.inner_dia_mm > 0.0 && in.inner_dia_mm < 3.0) {
        r.add("DFM-LOCKNUT", "error", std::string(kSkillId) +
              ": inner_dia " + std::to_string(in.inner_dia_mm) +
              " mm < 3.0 mm (M3 minimum lock-nut shaft thread)");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = std::string(kSkillId) + " DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError(std::string(kSkillId) +
                                   ": entry_face datum unresolved");

    // Derived raceway geometry.
    const double race_width = in.outer_dia_mm - in.inner_dia_mm;
    const double raceway_groove_width  = race_width / 6.0;
    const double raceway_groove_depth  = race_width / 8.0;
    const double outerR = in.outer_dia_mm / 2.0;
    const double innerR = in.inner_dia_mm / 2.0;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    constexpr double kEntryOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;
    gp_Pnt entry(in.position_x_mm, in.position_y_mm, zMax + kEntryOverhang);
    if (!(std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6 && adir.Z() < 0)) {
        const double bboxDiag = std::sqrt(
            (xMax - xMin) * (xMax - xMin) +
            (yMax - yMin) * (yMax - yMin) +
            (zMax - zMin) * (zMax - zMin));
        entry = gp_Pnt(
            in.position_x_mm - adir.X() * (bboxDiag + kEntryOverhang),
            in.position_y_mm - adir.Y() * (bboxDiag + kEntryOverhang),
            (zMin + zMax) / 2.0 - adir.Z() * (bboxDiag + kEntryOverhang));
    }

    // Sub-feature 1: shaft-end face cut — short outer cylinder of
    // face_depth_mm at outer_dia.  Models the face turning pass.
    const gp_Ax2 axOuter(entry, adir);
    const TopoDS_Shape faceCut =
        pr::cylinder(axOuter, outerR, in.face_depth_mm + kEntryOverhang);

    // Sub-feature 2: raceway ring groove sitting just below the face.
    const gp_Pnt grooveOrigin(
        entry.X() + adir.X() * in.face_depth_mm,
        entry.Y() + adir.Y() * in.face_depth_mm,
        entry.Z() + adir.Z() * in.face_depth_mm);
    const gp_Ax2 axGroove(grooveOrigin, adir);
    // Raceway groove sits between innerR + 1 race-half and outerR - 1
    // race-half.  We use a thin annular ring just inside the outer race
    // (mid-radius = (innerR+outerR)/2) with the standard width/depth.
    const double midR = (innerR + outerR) / 2.0;
    const TopoDS_Shape raceway =
        pr::annularRing(axGroove,
                        midR + raceway_groove_depth,
                        midR - raceway_groove_depth,
                        raceway_groove_width);

    // Sub-feature 3: lock-nut pilot diameter — deeper cylinder of inner_dia
    // extending through the seat region.
    const TopoDS_Shape locknutPilot =
        pr::cylinder(axOuter, innerR,
                     in.face_depth_mm + in.seat_depth_mm + kEntryOverhang);

    // Sub-feature 4: thrust shoulder is the residual face produced where
    // the outer-cylinder face cut ends and the raw stock continues — it is
    // an EMERGENT face from sub-features 1+3 and not a separate tool.  We
    // log it in the signature as a derived sub-feature.

    const TopoDS_Shape fused1 = pr::fuse(faceCut,  raceway);
    const TopoDS_Shape fused  = pr::fuse(fused1,   locknutPilot);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    json params = {
        { "entry_face_id",  *entryId },
        { "position_x_mm",  in.position_x_mm },
        { "position_y_mm",  in.position_y_mm },
        { "axis_dir",       { adir.X(), adir.Y(), adir.Z() } },
        { "outer_dia_mm",   in.outer_dia_mm },
        { "inner_dia_mm",   in.inner_dia_mm },
        { "face_depth_mm",  in.face_depth_mm },
        { "seat_depth_mm",  in.seat_depth_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "subfeature_count",           4 },
        { "outer_dia_mm",               in.outer_dia_mm },
        { "inner_dia_mm",               in.inner_dia_mm },
        { "face_depth_mm",              in.face_depth_mm },
        { "seat_depth_mm",              in.seat_depth_mm },
        { "raceway_groove_width_mm",    raceway_groove_width },
        { "raceway_groove_depth_mm",    raceway_groove_depth },
        { "raceway_groove_mid_dia_mm",  2.0 * midR },
        { "race_width_mm",              race_width },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type     = "facing_tool;groove_tool;boring_bar";
    tooling.tool_dia_mm   = in.outer_dia_mm;
    tooling.tool_material = "carbide";
    tooling.flute_count   = 1;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.06;
    tooling.stock_removed_mm3 =
        M_PI * outerR * outerR * in.face_depth_mm +
        M_PI * innerR * innerR * in.seat_depth_mm;
    tooling.est_cycle_time_s = std::max(4.0, in.seat_depth_mm / 20.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "facing_tool" },
              { "dia_mm", in.outer_dia_mm },
              { "depth_mm", in.face_depth_mm } },
            { { "tool_type", "groove_tool" },
              { "groove_mid_dia_mm", 2.0 * midR },
              { "width_mm", raceway_groove_width },
              { "depth_mm", raceway_groove_depth },
              { "note", "Timken thrust raceway" } },
            { { "tool_type", "boring_bar" },
              { "dia_mm", in.inner_dia_mm },
              { "depth_mm", in.face_depth_mm + in.seat_depth_mm },
              { "note", "lock-nut pilot — thread cut separately" } },
        } },
        { "derived_subfeatures", json::array({
            "shaft_end_face",
            "raceway_ring_groove",
            "locknut_pilot_dia",
            "thrust_shoulder",
        }) },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::thrust_bearing_seat_compound applied: od={} id={}",
                  in.outer_dia_mm, in.inner_dia_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::thrust_bearing_seat_compound
