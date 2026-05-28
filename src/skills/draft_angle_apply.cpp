// @lat: [[engine/skills#draft_angle_apply]]

#include "draft_angle_apply.hpp"

#include "Workpiece.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Dir.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>

namespace koocadcam::skill::draft_angle_apply {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.draft_deg < 1.0 - 1e-9 || in.draft_deg > 5.0 + 1e-9) {
        r.add("DFM-DRAFT-RANGE", "error",
              "draft_angle_apply draft_deg " + std::to_string(in.draft_deg) +
              " outside [1.0, 5.0] deg");
    }

    // Resolve the target face and check that its normal is approximately in
    // the XY plane (i.e. the face is "vertical" — eligible for drafting).
    auto faceId = wp.resolve(in.target_face);
    if (!faceId) {
        r.add("DFM-INPUT", "error",
              "draft_angle_apply target_face datum unresolved");
    } else {
        try {
            BRepAdaptor_Surface surf(wp.face(*faceId));
            if (surf.GetType() != GeomAbs_Plane) {
                r.add("DFM-DRAFT-VERT", "warning",
                      "draft_angle_apply target_face is non-planar — "
                      "draft metadata recorded but verticality not verified");
            } else {
                const gp_Dir n = surf.Plane().Axis().Direction();
                if (std::abs(n.Z()) > 0.2) {
                    r.add("DFM-DRAFT-VERT", "error",
                          "draft_angle_apply target_face normal not in XY "
                          "plane (|n.Z|=" + std::to_string(std::abs(n.Z())) +
                          " > 0.2) — not a vertical face");
                }
            }
        } catch (...) {
            r.add("DFM-INPUT", "warning",
                  "draft_angle_apply could not inspect target_face surface");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Metadata-only: we do NOT modify the geometry.  The skill exists so
// downstream process plans / CAM ops know which face needs how much draft.
// (See header comment for the design rationale.)

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "draft_angle_apply DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto faceId = wp.resolve(in.target_face);
    if (!faceId) {
        throw SkillError("draft_angle_apply: target_face datum unresolved");
    }

    // Capture the face normal for downstream replay (so a real geometric
    // applier in a later slice can rebuild without re-resolving the datum).
    json normalArr = json::array({ 0.0, 0.0, 0.0 });
    try {
        BRepAdaptor_Surface surf(wp.face(*faceId));
        if (surf.GetType() == GeomAbs_Plane) {
            const gp_Dir n = surf.Plane().Axis().Direction();
            normalArr = json::array({ n.X(), n.Y(), n.Z() });
        }
    } catch (...) { /* leave zero */ }

    json params = {
        { "target_face_kind", "resolved_id" },
        { "face_id",          *faceId },
        { "draft_deg",        in.draft_deg },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "face_id",           *faceId },
        { "draft_deg",         in.draft_deg },
        { "face_normal",       normalArr },
        { "applied_geometry",  false },     // metadata-only marker
    };

    ToolingMeta tooling;
    tooling.tool_type         = "mold_feature";
    tooling.stock_removed_mm3 = 0.0;        // no material change
    tooling.est_cycle_time_s  = 0.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    // Shape is unchanged — clone the workpiece's shape into a new instance so
    // the feature is recorded in a fresh Workpiece (matches the contract:
    // apply() always returns a NEW Workpiece).
    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    // Carry over existing features then append this one.
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::draft_angle_apply (metadata-only) applied: face_id={} deg={}",
                  *faceId, in.draft_deg);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// History replay only.  Since apply() leaves the shape untouched, there is
// no geometric trace to recover from a "naked" workpiece — recognition
// without history is impossible by design.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "face_id",   f.params.value("face_id",   -1) },
            { "draft_deg", f.params.value("draft_deg", 0.0) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 0.95, json{ { "source", "feature_history" } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::draft_angle_apply
