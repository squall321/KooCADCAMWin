// @lat: [[engine/skills#c_mount_lens_thread]]

#include "c_mount_lens_thread.hpp"

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
#include <array>
#include <cmath>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::c_mount_lens_thread {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

struct Row { const char* style; double od; double pitch; double flange; };

// flange distance per ISO/JIS/CCTV mount standards.
// C-mount: 1"-32 UN (32 TPI → 25.4/32 = 0.79375 mm), pilot ≈ 24.4 mm.
// CS-mount: same thread as C, flange 12.5 mm.
// M42×1, M52×0.75 (T-mount family).
constexpr std::array<Row, 4> kMounts {{
    { "C",   25.4, 25.4 / 32.0, 17.526 },
    { "CS",  25.4, 25.4 / 32.0, 12.500 },
    { "M42", 42.0, 1.00,        45.460 },   // typical T2/M42 flange ~45.46 mm
    { "M52", 52.0, 0.75,        46.000 },   // photographic M52 lens filter standard
}};

double volumeOf(const TopoDS_Shape& s)
{
    if (s.IsNull()) return 0.0;
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

}  // namespace

bool lookupMount(const std::string& style, MountSpec& out)
{
    for (const auto& r : kMounts) {
        if (style == r.style) {
            out.od_mm = r.od;
            out.pitch_mm = r.pitch;
            out.flange_distance_mm = r.flange;
            // Pilot dia = thread minor diameter ≈ OD - 1.082×pitch (ISO 68 form).
            out.pilot_dia_mm = r.od - 1.082 * r.pitch;
            return true;
        }
    }
    return false;
}

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "c_mount_lens_thread: face_id out of range");
    }
    MountSpec sp{};
    if (!lookupMount(in.mount_style, sp)) {
        r.add("DFM-MOUNT-STYLE", "error",
              "c_mount_lens_thread: unknown mount_style '" + in.mount_style +
              "' (must be one of C / CS / M42 / M52)");
    }
    if (in.face_id >= 0 && in.face_id < wp.faceCount()) {
        if (!wp.isFacePlanar(in.face_id)) {
            r.add("DFM-FACE", "error",
                  "c_mount_lens_thread: target face must be planar");
        }
    }
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "c_mount_lens_thread DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    MountSpec sp{};
    lookupMount(in.mount_style, sp);

    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const double baseZ = faceC.Z();
    const double cx    = in.center_x_mm;
    const double cy    = in.center_y_mm;
    constexpr double kEmbed = 0.05;

    // Sub 1: pilot bore (full OD/2 on the OD), depth = flange distance (acts
    // as the threaded receptacle bore).
    const double pilotR    = sp.pilot_dia_mm / 2.0;
    const double odR       = sp.od_mm / 2.0;
    const double boreDepth = sp.flange_distance_mm;

    std::vector<TopoDS_Shape> cutters;
    cutters.reserve(3);
    {
        const gp_Pnt c(cx, cy, baseZ - boreDepth);
        const gp_Ax2 ax(c, gp::DZ());
        cutters.push_back(pr::cylinder(ax, odR, boreDepth + kEmbed));
    }

    // Sub 2: cosmetic thread band — shallow annular ring around the bore wall
    // (outer radius = odR + 0.05 representing thread crest, inner = pilotR,
    // depth = boreDepth × 0.75 — shows up as a distinct geometric feature).
    const double bandDepth = boreDepth * 0.75;
    {
        const gp_Pnt c(cx, cy, baseZ - bandDepth);
        const gp_Ax2 ax(c, gp::DZ());
        try {
            const TopoDS_Shape ring = pr::annularRing(
                ax, odR + 0.10, pilotR, bandDepth + kEmbed);
            cutters.push_back(ring);
        } catch (const Standard_Failure& ex) {
            spdlog::debug("c_mount_lens_thread: thread band failed — {}", ex.what());
        }
    }

    // Sub 3: flange recess pad — a shallow planar pocket at the flange face
    // (slightly wider than OD, height ~0.4 mm) to register the lens flange.
    const double flangeRecess = 0.4;
    {
        const gp_Pnt c(cx, cy, baseZ - flangeRecess);
        const gp_Ax2 ax(c, gp::DZ());
        const TopoDS_Shape pad = pr::cylinder(
            ax, odR + 1.5, flangeRecess + kEmbed);
        cutters.push_back(pad);
    }

    const double volBefore = volumeOf(wp.shape());
    // NOTE: pr::cutMany with overlapping/interpenetrating tool solids on a box
    // base silently returns the input unchanged in OCCT 8.0 (the bore cylinder
    // contains the cosmetic thread band, and the flange pad overlaps both at
    // the top).  We sequentialise the cuts to avoid the compound-tool failure
    // mode: ~9 cm^3 are removed per call (pilot bore dominates).
    TopoDS_Shape newShape = wp.shape();
    for (const auto& tool : cutters) {
        if (tool.IsNull()) continue;
        newShape = pr::cut(newShape, tool);
    }
    if (newShape.IsNull()) {
        throw SkillError("c_mount_lens_thread: cutMany returned null");
    }
    const double volAfter   = volumeOf(newShape);
    const double volRemoved = volBefore - volAfter;

    json params = {
        { "face_id",     in.face_id },
        { "center_x_mm", in.center_x_mm },
        { "center_y_mm", in.center_y_mm },
        { "mount_style", in.mount_style },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "optical_feature_type",       "lens_thread_mount" },
        { "mount_style",                in.mount_style },
        { "od_mm",                      sp.od_mm },
        { "pitch_mm",                   sp.pitch_mm },
        { "pilot_dia_mm",               sp.pilot_dia_mm },
        { "flange_distance_mm",         sp.flange_distance_mm },
        { "subfeature_count",           static_cast<int>(cutters.size()) },
        { "derived_volume_removed_mm3", volRemoved },
    };
    ToolingMeta tooling;
    tooling.tool_type        = "drill;thread_mill;face_mill";
    tooling.tool_dia_mm      = sp.od_mm;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = volRemoved / 80.0 + 30.0;
    tooling.extra = {
        { "thread_form", in.mount_style },
        { "tpi", (in.mount_style == "C" || in.mount_style == "CS") ? 32 : 0 },
        { "spec_source", "ISO/JIS lens mount standards" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);
    spdlog::debug("skill::c_mount_lens_thread: style={} OD={} flange={}",
                  in.mount_style, sp.od_mm, sp.flange_distance_mm);
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
        rf.confidence       = 0.93;
        rf.matched_geometry = f.pattern;
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::c_mount_lens_thread
