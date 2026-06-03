// @lat: [[engine/skills#add_draft_to_face]]

#include "add_draft_to_face.hpp"

#include "Workpiece.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <BRepOffsetAPI_DraftAngle.hxx>
#include <GProp_GProps.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::add_draft_to_face {

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

    if (in.draft_angle_deg < 0.5 - 1e-9 ||
        in.draft_angle_deg > 15.0 + 1e-9) {
        r.add("DFM-DRAFT-RANGE", "error",
              "add_draft_to_face: draft_angle_deg " +
              std::to_string(in.draft_angle_deg) +
              " outside [0.5, 15.0] deg");
    }

    const int n = wp.faceCount();
    if (in.source_face_id < 0 || in.source_face_id >= n) {
        r.add("DFM-INPUT", "error",
              "add_draft_to_face: source_face_id out of range");
    }
    if (in.base_face_id < 0 || in.base_face_id >= n) {
        r.add("DFM-INPUT", "error",
              "add_draft_to_face: base_face_id out of range");
    }
    if (in.source_face_id == in.base_face_id && in.source_face_id >= 0) {
        r.add("DFM-INPUT", "error",
              "add_draft_to_face: source and base face must differ");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "add_draft_to_face DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const TopoDS_Face& src  = wp.face(in.source_face_id);
    const TopoDS_Face& base = wp.face(in.base_face_id);

    // Pull direction = base face outward normal.  Neutral plane = base face
    // surface plane.
    BRepAdaptor_Surface baseSurf(base);
    if (baseSurf.GetType() != GeomAbs_Plane) {
        throw SkillError(
            "DFM-DRAFT-FAILED: add_draft_to_face: base face must be planar");
    }
    const gp_Pln neutralPlane = baseSurf.Plane();
    gp_Dir pullDir = neutralPlane.Axis().Direction();
    if (base.Orientation() == TopAbs_REVERSED) pullDir.Reverse();

    const double angleRad = in.draft_angle_deg * M_PI / 180.0;
    const double volBefore = volumeOf(wp.shape());

    TopoDS_Shape newShape;
    bool ok = false;
    bool addDone = false;
    try {
        BRepOffsetAPI_DraftAngle drafter(wp.shape());
        drafter.Add(src, pullDir, angleRad, neutralPlane);
        addDone = drafter.AddDone();
        if (addDone) {
            drafter.Build();
            if (drafter.IsDone()) {
                newShape = drafter.Shape();
                ok = !newShape.IsNull();
            }
        }
    } catch (const Standard_Failure& ex) {
        spdlog::debug("add_draft_to_face: OCCT threw — {}",
                      ex.what());
    } catch (const std::exception& ex) {
        spdlog::debug("add_draft_to_face: std::exception — {}", ex.what());
    }

    if (!ok) {
        throw SkillError(
            std::string("DFM-DRAFT-FAILED: BRepOffsetAPI_DraftAngle could not "
                        "build (AddDone=") + (addDone ? "true" : "false") +
            ", angle_deg=" + std::to_string(in.draft_angle_deg) + ")");
    }

    const double volAfter = volumeOf(newShape);

    json params = {
        { "source_face_id",  in.source_face_id },
        { "base_face_id",    in.base_face_id },
        { "draft_angle_deg", in.draft_angle_deg },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "parameters",            params },
        { "derived_volume",        volAfter - volBefore },
        { "volume_before_mm3",     volBefore },
        { "volume_after_mm3",      volAfter },
        { "pull_dir",              { pullDir.X(), pullDir.Y(), pullDir.Z() } },
        { "applied_geometry",      true },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "mold_feature";
    tooling.stock_removed_mm3 = volBefore - volAfter;
    tooling.est_cycle_time_s  = 0.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::add_draft_to_face: src={} base={} angle={} vol {}→{}",
                  in.source_face_id, in.base_face_id, in.draft_angle_deg,
                  volBefore, volAfter);

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
        rf.confidence       = 0.95;
        rf.matched_geometry = f.pattern;
        out.push_back(rf);
    }
    if (!out.empty()) return out;

    // Geometric fallback: scan for non-axis-aligned planar faces.  If we find
    // a planar face whose normal has a nontrivial Z component AND non-trivial
    // XY component (i.e. tilted, not orthogonal), it is likely drafted.
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        const double az = std::abs(n.Z());
        const double axy = std::sqrt(n.X() * n.X() + n.Y() * n.Y());
        if (az < 0.05 || az > 0.95) continue;       // axis-aligned skip
        if (axy < 0.05) continue;
        // Estimate draft angle from |n.Z|:  vertical → 0°, tilted → arctan…
        const double estDeg =
            std::atan2(az, axy) * 180.0 / M_PI;     // angle off-vertical
        RecognizedFeature rf;
        rf.skill_id = kSkillId;
        rf.recovered_params = {
            { "source_face_id",  i },
            { "base_face_id",    -1 },
            { "draft_angle_deg", estDeg },
        };
        rf.confidence       = 0.45;
        rf.matched_geometry = json{
            { "source",    "geometry" },
            { "face_id",   i },
            { "normal",    { n.X(), n.Y(), n.Z() } },
        };
        out.push_back(rf);
        if (out.size() >= 3) break;
    }
    return out;
}

}  // namespace koocadcam::skill::add_draft_to_face
