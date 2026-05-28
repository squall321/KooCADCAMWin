// @lat: [[engine/skills#burnish]]

#include "burnish.hpp"

#include "Workpiece.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::burnish {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.pressure_kn <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": pressure_kn must be > 0");
    } else if (in.pressure_kn < 0.1) {
        r.add("DFM-BURNISH-PRESSURE", "error",
              std::string(kSkillId) + ": pressure_kn " +
              std::to_string(in.pressure_kn) +
              " < 0.1 kN — too light to deform asperities");
    } else if (in.pressure_kn > 20.0) {
        r.add("DFM-BURNISH-PRESSURE", "error",
              std::string(kSkillId) + ": pressure_kn " +
              std::to_string(in.pressure_kn) +
              " > 20 kN — risks subsurface fatigue / tool damage");
    }

    if (in.target_ra_um <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": target_ra_um must be > 0");
    } else if (in.target_ra_um < 0.05) {
        r.add("DFM-BURNISH-RA", "error",
              std::string(kSkillId) + ": target_ra_um " +
              std::to_string(in.target_ra_um) +
              " < 0.05 μm — below typical burnish achievable floor");
    } else if (in.target_ra_um > 3.2) {
        r.add("DFM-BURNISH-RA", "error",
              std::string(kSkillId) + ": target_ra_um " +
              std::to_string(in.target_ra_um) +
              " > 3.2 μm — burnish target should be < input Ra; "
              "consider a coarser intermediate process");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "burnish DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("burnish: entry_face datum unresolved");

    GProp_GProps props;
    BRepGProp::SurfaceProperties(wp.face(*entryId), props);
    const double faceArea = props.Mass();

    json params = {
        { "entry_face_id", *entryId },
        { "pressure_kn",   in.pressure_kn },
        { "target_ra_um",  in.target_ra_um },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "target_face_id",       *entryId },
        { "pressure_kn",          in.pressure_kn },
        { "target_ra_um",         in.target_ra_um },
        { "target_face_area_mm2", faceArea },
        { "geometry_changed",     false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "burnishing_roller";
    tooling.tool_dia_mm       = 10.0;        // typical roller diameter
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "hardened_steel_polished";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;     // cold-smear, no removal
    // ~ 1 minute per 200 mm²; floor at 5 s.
    tooling.est_cycle_time_s  = std::max(5.0, faceArea / 200.0 * 60.0);
    tooling.extra = {
        { "pressure_kn",  in.pressure_kn },
        { "target_ra_um", in.target_ra_um },
        { "process",      "roller_burnish" },
        { "effect",       "Ra ↓, surface work-hardened, compressive σ" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::burnish applied: face={} P={}kN target_Ra={}μm",
                  *entryId, in.pressure_kn, in.target_ra_um);

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

}  // namespace koocadcam::skill::burnish
