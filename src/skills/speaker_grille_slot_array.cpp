// @lat: [[engine/skills#speaker_grille_slot_array]]

#include "speaker_grille_slot_array.hpp"

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

namespace koocadcam::skill::speaker_grille_slot_array {

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
              "speaker_grille_slot_array: face_id out of range");
    }
    if (in.slot_count <= 0) {
        r.add("DFM-INPUT", "error",
              "speaker_grille_slot_array: slot_count must be > 0");
    }
    if (!(in.slot_length_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "speaker_grille_slot_array: slot_length_mm must be > 0");
    }
    if (!(in.slot_width_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "speaker_grille_slot_array: slot_width_mm must be > 0");
    }
    if (!(in.pitch_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "speaker_grille_slot_array: pitch_mm must be > 0");
    }
    if (!(in.depth_through > 0.0)) {
        r.add("DFM-INPUT", "error",
              "speaker_grille_slot_array: depth_through must be > 0");
    }
    if (in.pitch_mm > 0.0 && in.slot_width_mm > 0.0 &&
        in.pitch_mm <= in.slot_width_mm) {
        r.add("DFM-SLOT-OVERLAP", "error",
              "speaker_grille_slot_array: pitch_mm (" +
              std::to_string(in.pitch_mm) +
              ") <= slot_width_mm (" + std::to_string(in.slot_width_mm) +
              ") — adjacent slots would overlap");
    }
    if (in.slot_count > 0 && (in.slot_count < 3 || in.slot_count > 30)) {
        r.add("DFM-SLOT-COUNT", "warning",
              "speaker slot_count " + std::to_string(in.slot_count) +
              " outside typical phone range [3, 30]");
    }
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "speaker_grille_slot_array DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const double baseZ = faceC.Z();
    const double cx    = in.center_x_mm;
    const double cy    = in.center_y_mm;
    constexpr double kEmbed = 0.05;
    const double Lw    = in.slot_width_mm;     // X
    const double Ll    = in.slot_length_mm;    // Y (slots oriented along Y)
    const double D     = in.depth_through;
    const int    N     = in.slot_count;

    std::vector<TopoDS_Shape> cutters;
    cutters.reserve(static_cast<size_t>(N));

    const double startOffset = -(static_cast<double>(N - 1) * in.pitch_mm) / 2.0;

    for (int i = 0; i < N; ++i) {
        const double sx = cx + startOffset + i * in.pitch_mm;
        const gp_Pnt origin(sx - Lw / 2.0, cy - Ll / 2.0, baseZ - D);
        const gp_Ax2 ax(origin, gp::DZ());
        try {
            cutters.push_back(pr::box(ax, Lw, Ll, D + 2.0 * kEmbed));
        } catch (const Standard_Failure& ex) {
            spdlog::debug("speaker_grille_slot_array: slot {} failed — {}",
                          i, ex.what());
        }
    }

    if (cutters.empty()) {
        throw SkillError(
            "speaker_grille_slot_array: no slots produced");
    }

    const double volBefore = volumeOf(wp.shape());
    TopoDS_Shape newShape  = pr::cutMany(wp.shape(), cutters);
    if (newShape.IsNull()) {
        throw SkillError("speaker_grille_slot_array: cutMany returned null");
    }
    const double volAfter   = volumeOf(newShape);
    const double volRemoved = volBefore - volAfter;

    const double estPer   = Lw * Ll * D;
    const double estTotal = estPer * static_cast<double>(cutters.size());

    json params = {
        { "face_id",        in.face_id },
        { "center_x_mm",    in.center_x_mm },
        { "center_y_mm",    in.center_y_mm },
        { "slot_count",     in.slot_count },
        { "slot_length_mm", in.slot_length_mm },
        { "slot_width_mm",  in.slot_width_mm },
        { "pitch_mm",       in.pitch_mm },
        { "depth_through",  in.depth_through },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "is_phone_feature",           true },
        { "is_periodic_pattern",        true },
        { "subfeature_count",           static_cast<int>(cutters.size()) },
        { "slot_count",                 in.slot_count },
        { "pitch_mm",                   in.pitch_mm },
        { "slot_length_mm",             in.slot_length_mm },
        { "slot_width_mm",              in.slot_width_mm },
        { "derived_volume_removed_mm3", volRemoved },
        { "estimated_total_mm3",        estTotal },
        { "estimated_per_slot_mm3",     estPer },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = std::max(0.3, in.slot_width_mm * 0.7);
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = volRemoved / 60.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::speaker_grille_slot_array: N={} pitch={} vol-removed={}",
                  N, in.pitch_mm, volRemoved);

    return SkillOutput{ wpNew, sig };
}

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // History replay first
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 0.95;
        rf.matched_geometry = f.pattern;
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::speaker_grille_slot_array
