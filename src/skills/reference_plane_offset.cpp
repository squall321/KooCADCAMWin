// @lat: [[engine/skills#reference_plane_offset]]
//
// Metadata-only construction geometry: workpiece shape is unchanged, the
// reference plane (origin = faceCenter + n * offset_mm, normal = faceNormal)
// is captured in the FeatureSignature for downstream skills to reference by
// `named_id`.

#include "reference_plane_offset.hpp"

#include "Workpiece.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>

namespace koocadcam::skill::reference_plane_offset {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.name.empty()) {
        r.add("DFM-INPUT", "error",
              "reference_plane_offset: name must be non-empty");
    }
    if (std::abs(in.offset_mm) < 1e-6) {
        r.add("DFM-INPUT", "error",
              "reference_plane_offset: |offset_mm| must be >= 1e-6 "
              "(got " + std::to_string(in.offset_mm) + ")");
    }
    if (in.source_face_id < 0 || in.source_face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "reference_plane_offset: source_face_id " +
              std::to_string(in.source_face_id) + " out of range [0," +
              std::to_string(wp.faceCount()) + ")");
        return r;
    }
    if (!wp.isFacePlanar(in.source_face_id)) {
        r.add("DFM-REF-NONPLANAR", "error",
              "reference_plane_offset: source face " +
              std::to_string(in.source_face_id) + " is not planar");
    }
    return r;
}

// ── Synthesis (metadata only) ────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "reference_plane_offset DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const int    srcId   = in.source_face_id;
    const gp_Dir n       = wp.faceNormal(srcId);
    const gp_Pnt c       = wp.faceCenter(srcId);
    const double srcArea = wp.faceArea(srcId);

    // Reference plane origin = source face center + offset_mm * normal.
    const double ox = c.X() + n.X() * in.offset_mm;
    const double oy = c.Y() + n.Y() * in.offset_mm;
    const double oz = c.Z() + n.Z() * in.offset_mm;

    json params = {
        { "source_face_id", srcId },
        { "offset_mm",      in.offset_mm },
        { "name",           in.name },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "is_reference",      true },
        { "datum_class",       "plane" },
        { "named_id",          in.name },
        { "source_face_id",    srcId },
        { "source_face_area",  srcArea },
        { "offset_mm",         in.offset_mm },
        { "plane_origin_xyz",  { ox, oy, oz } },
        { "plane_normal_xyz",  { n.X(), n.Y(), n.Z() } },
        { "geometry_changed",  false },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "reference_datum";
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 0.0;
    tooling.extra = {
        { "datum_class",     "plane" },
        { "geometric_no_op", true },
        { "named_id",        in.name },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    // Clone workpiece verbatim — shape unchanged; carry feature history.
    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::reference_plane_offset: name={} face={} off={}",
                  in.name, srcId, in.offset_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Reference planes leave NO B-Rep trace.  We can only recover prior datums
// by replaying signatures.  Confidence is 1.0 because the recovered params
// are byte-equal to the user input.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "feature_history" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::reference_plane_offset
