// @lat: [[engine/skills#zif_connector_seat]]

#include "zif_connector_seat.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Shape.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace koocadcam::skill::zif_connector_seat {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    if (s.IsNull()) return 0.0;
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "zif_connector_seat: face_id out of range");
    }
    if (!(in.fpc_width_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "zif_connector_seat: fpc_width_mm must be > 0");
    }
    if (!(in.connector_height_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "zif_connector_seat: connector_height_mm must be > 0");
    }
    if (in.latch_clearance_mm < 0.0) {
        r.add("DFM-INPUT", "error",
              "zif_connector_seat: latch_clearance_mm must be ≥ 0");
    }
    if (in.fpc_width_mm > 0.0 &&
        (in.fpc_width_mm < 1.5 || in.fpc_width_mm > 15.0)) {
        r.add("DFM-ZIF-RANGE", "warning",
              "ZIF fpc_width " + std::to_string(in.fpc_width_mm) +
              " outside typical [1.5, 15] mm");
    }
    if (in.connector_height_mm > 0.0 &&
        (in.connector_height_mm < 0.5 || in.connector_height_mm > 1.2)) {
        r.add("DFM-ZIF-RANGE", "warning",
              "ZIF height " + std::to_string(in.connector_height_mm) +
              " outside typical [0.5, 1.2] mm");
    }
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "zif_connector_seat DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const double baseZ = faceC.Z();
    const double cx    = in.position_x_mm;
    const double cy    = in.position_y_mm;
    constexpr double kEmbed = 0.05;
    constexpr double kFpcTol = 0.5;          // 0.5 mm clearance per spec
    constexpr double kConnLen = 8.0;         // typical ZIF connector body length

    // ZIF body length (along FPC insertion axis = X)
    const double mainLen = kConnLen;
    const double mainWid = in.fpc_width_mm + kFpcTol;
    const double mainDep = in.connector_height_mm;

    // Latch hinge clearance pocket: slightly wider, shallow & above the pocket
    const double latchLen = mainLen * 0.4;
    const double latchWid = mainWid + 0.4;
    const double latchDep = std::max(0.05, in.latch_clearance_mm);

    // Lead-in chamfer pocket at one end (FPC entry)
    const double leadLen = std::max(0.1, mainLen * 0.2);
    const double leadWid = mainWid;
    const double leadDep = std::max(0.05, mainDep * 0.3);

    const double volBefore = volumeOf(wp.shape());

    // The 3 sub-pockets overlap (latch sits on top of main, lead-in at end).
    // For overlapping cutters BRepAlgoAPI_Cut against a compound is unreliable
    // in OCCT 8.0, so we apply the cuts sequentially.  Counter-bookkeeping
    // tracks the per-step volume removed.
    TopoDS_Shape working = wp.shape();
    int subCount = 0;

    // Op 1: main FPC pocket
    {
        const gp_Pnt origin(cx - mainLen / 2.0, cy - mainWid / 2.0,
                            baseZ - mainDep);
        const gp_Ax2 ax(origin, gp::DZ());
        const TopoDS_Shape tool = pr::box(ax, mainLen, mainWid,
                                          mainDep + kEmbed);
        working = pr::cut(working, tool);
        ++subCount;
    }

    // Op 2: latch clearance pocket (slightly wider, shallow, on top)
    {
        const gp_Pnt origin(cx - latchLen / 2.0,
                            cy - latchWid / 2.0,
                            baseZ - latchDep);
        const gp_Ax2 ax(origin, gp::DZ());
        const TopoDS_Shape tool = pr::box(ax, latchLen, latchWid,
                                          latchDep + kEmbed);
        working = pr::cut(working, tool);
        ++subCount;
    }

    // Op 3: lead-in at +X end of main pocket
    {
        const gp_Pnt origin(cx + mainLen / 2.0 - leadLen / 2.0,
                            cy - leadWid / 2.0,
                            baseZ - leadDep);
        const gp_Ax2 ax(origin, gp::DZ());
        const TopoDS_Shape tool = pr::box(ax, leadLen, leadWid,
                                          leadDep + kEmbed);
        working = pr::cut(working, tool);
        ++subCount;
    }

    TopoDS_Shape newShape = working;
    if (newShape.IsNull()) {
        throw SkillError("zif_connector_seat: cut returned null");
    }
    const double volAfter   = volumeOf(newShape);
    const double volRemoved = volBefore - volAfter;

    json params = {
        { "face_id",             in.face_id },
        { "position_x_mm",       in.position_x_mm },
        { "position_y_mm",       in.position_y_mm },
        { "fpc_width_mm",        in.fpc_width_mm },
        { "connector_height_mm", in.connector_height_mm },
        { "latch_clearance_mm",  in.latch_clearance_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "is_phone_feature",           true },
        { "subfeature_count",           subCount },
        { "derived_volume_removed_mm3", volRemoved },
        { "fpc_width_mm",               in.fpc_width_mm },
        { "connector_height_mm",        in.connector_height_mm },
        { "latch_clearance_mm",         in.latch_clearance_mm },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = std::max(0.5, in.fpc_width_mm * 0.2);
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = volRemoved / 100.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::zif_connector_seat: fpc={} h={} vol-removed={}",
                  in.fpc_width_mm, in.connector_height_mm, volRemoved);

    return SkillOutput{ wpNew, sig };
}

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 0.92;
        rf.matched_geometry = f.pattern;
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::zif_connector_seat
