// @lat: [[engine/skills#reference_point_on_face_uv]]
//
// Metadata-only reference point: workpiece shape unchanged.  Point resolved
// via BRepAdaptor_Surface::Value on the face's underlying surface, with
// input u/v normalized to the surface's parameter range.

#include "reference_point_on_face_uv.hpp"

#include "Workpiece.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace koocadcam::skill::reference_point_on_face_uv {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.name.empty()) {
        r.add("DFM-INPUT", "error",
              "reference_point_on_face_uv: name must be non-empty");
    }
    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "reference_point_on_face_uv: face_id " +
              std::to_string(in.face_id) + " out of range [0," +
              std::to_string(wp.faceCount()) + ")");
        return r;
    }
    if (in.u < 0.0 || in.u > 1.0) {
        r.add("DFM-INPUT", "error",
              "reference_point_on_face_uv: u = " + std::to_string(in.u) +
              " out of [0, 1]");
    }
    if (in.v < 0.0 || in.v > 1.0) {
        r.add("DFM-INPUT", "error",
              "reference_point_on_face_uv: v = " + std::to_string(in.v) +
              " out of [0, 1]");
    }
    return r;
}

// ── Synthesis (metadata only) ────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "reference_point_on_face_uv DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const TopoDS_Face& face = wp.face(in.face_id);

    double x = 0.0, y = 0.0, z = 0.0;
    bool resolved = false;
    try {
        BRepAdaptor_Surface adaptor(face);
        const double uMin = adaptor.FirstUParameter();
        const double uMax = adaptor.LastUParameter();
        const double vMin = adaptor.FirstVParameter();
        const double vMax = adaptor.LastVParameter();
        const double uPar = uMin + in.u * (uMax - uMin);
        const double vPar = vMin + in.v * (vMax - vMin);
        const gp_Pnt pt   = adaptor.Value(uPar, vPar);
        x = pt.X(); y = pt.Y(); z = pt.Z();
        resolved = true;
    } catch (const Standard_Failure& ex) {
        spdlog::debug("reference_point_on_face_uv: surface eval failed — {}",
                      ex.what());
    }

    json params = {
        { "face_id", in.face_id },
        { "u",       in.u },
        { "v",       in.v },
        { "name",    in.name },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "is_reference",      true },
        { "datum_class",       "point" },
        { "named_id",          in.name },
        { "face_id",           in.face_id },
        { "u_norm",            in.u },
        { "v_norm",            in.v },
        { "point_xyz",         { x, y, z } },
        { "resolved",          resolved },
        { "geometry_changed",  false },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "reference_datum";
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 0.0;
    tooling.extra = {
        { "datum_class",     "point" },
        { "geometric_no_op", true },
        { "named_id",        in.name },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::reference_point_on_face_uv: name={} face={} u={} v={}"
                  " -> ({},{},{}) resolved={}",
                  in.name, in.face_id, in.u, in.v, x, y, z, resolved);

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
        rf.matched_geometry = { { "source", "feature_history" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::reference_point_on_face_uv
