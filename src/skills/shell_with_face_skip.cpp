// @lat: [[engine/skills#shell_with_face_skip]]

#include "shell_with_face_skip.hpp"

#include "Workpiece.hpp"

#include <BRepGProp.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepOffset_Mode.hxx>
#include <GProp_GProps.hxx>
#include <GeomAbs_JoinType.hxx>
#include <NCollection_List.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::shell_with_face_skip {

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

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!(in.wall_thickness_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "shell_with_face_skip: wall_thickness_mm must be > 0 (got " +
              std::to_string(in.wall_thickness_mm) + ")");
    }

    // Each requested open face id must be in range.
    const int nFaces = wp.faceCount();
    for (int id : in.open_face_ids) {
        if (id < 0 || id >= nFaces) {
            r.add("DFM-INPUT", "error",
                  "shell_with_face_skip: open_face_id " + std::to_string(id) +
                  " out of range (faceCount=" + std::to_string(nFaces) + ")");
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "shell_with_face_skip DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double volBefore = volumeOf(wp.shape());

    // Collect the list of faces to remove (skip) before offsetting.
    NCollection_List<TopoDS_Shape> facesToRemove;
    for (int id : in.open_face_ids) {
        facesToRemove.Append(wp.face(id));
    }

    // MakeThickSolidByJoin: offset = -wall_thickness (inward to make a shell).
    TopoDS_Shape newShape;
    bool ok = false;
    try {
        BRepOffsetAPI_MakeThickSolid thicker;
        thicker.MakeThickSolidByJoin(
            wp.shape(),
            facesToRemove,
            -in.wall_thickness_mm,        // offset (negative = inward)
            1.0e-3,                       // tolerance
            BRepOffset_Skin,              // mode
            false,                        // intersection
            false,                        // selfInter
            GeomAbs_Arc                   // join type
        );
        thicker.Build();
        if (thicker.IsDone()) {
            newShape = thicker.Shape();
            ok = !newShape.IsNull();
        }
    } catch (const Standard_Failure& ex) {
        spdlog::debug("shell_with_face_skip: OCCT threw — {}", ex.what());
    } catch (const std::exception& ex) {
        spdlog::debug("shell_with_face_skip: std::exception — {}", ex.what());
    }

    if (!ok) {
        throw SkillError(
            "DFM-SHELL-FAILED: BRepOffsetAPI_MakeThickSolid could not build "
            "(wall_thickness=" + std::to_string(in.wall_thickness_mm) +
            ", open_faces=" + std::to_string(in.open_face_ids.size()) + ")");
    }

    const double volAfter = volumeOf(newShape);
    const double removed  = volBefore - volAfter;

    json params = {
        { "wall_thickness_mm", in.wall_thickness_mm },
        { "open_face_ids",     in.open_face_ids },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "parameters",            params },
        { "derived_volume",        removed },
        { "open_face_count",       static_cast<int>(in.open_face_ids.size()) },
        { "volume_before_mm3",     volBefore },
        { "volume_after_mm3",      volAfter },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "shell_feature";
    tooling.stock_removed_mm3 = removed;
    tooling.est_cycle_time_s  = std::max(5.0, removed / 800.0);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::shell_with_face_skip: wall={} open_faces={} vol {}→{}",
                  in.wall_thickness_mm, in.open_face_ids.size(),
                  volBefore, volAfter);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Geometric heuristic: many "interior" planar faces (facing inward) suggest
// a hollow shell.  Primary path is history replay.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 0.95;
        rf.matched_geometry = f.pattern;
        out.push_back(rf);
    }
    if (!out.empty()) return out;

    // Geometric fallback: count internal planar faces.  A shell has roughly
    // 2× the planar face count of the original solid (outer + inner walls).
    int planarFaces = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (wp.isFacePlanar(i)) ++planarFaces;
    }
    if (planarFaces < 6) return out;
    RecognizedFeature rf;
    rf.skill_id = kSkillId;
    rf.recovered_params = {
        { "wall_thickness_mm", 0.0 },
        { "open_face_ids",     json::array() },
    };
    rf.confidence       = 0.35;
    rf.matched_geometry = json{
        { "source",       "geometry" },
        { "planar_faces", planarFaces },
    };
    out.push_back(rf);
    return out;
}

}  // namespace koocadcam::skill::shell_with_face_skip
