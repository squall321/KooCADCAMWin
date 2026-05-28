// @lat: [[engine/skills#anti_wear_treatment]]

#include "anti_wear_treatment.hpp"

#include "Workpiece.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::anti_wear_treatment {

using nlohmann::json;

namespace {

bool isKnownTechnique(const std::string& t)
{
    return t == "laser_hardening"
        || t == "induction_hardening"
        || t == "surface_texturing"
        || t == "ion_implantation";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.depth_um <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": depth_um must be > 0");
    } else if (in.depth_um < 1.0) {
        r.add("DFM-ANTIWEAR-DEPTH", "error",
              std::string(kSkillId) + ": depth_um " +
              std::to_string(in.depth_um) +
              " < 1.0 μm — below practical anti-wear penetration");
    } else if (in.depth_um > 2000.0) {
        r.add("DFM-ANTIWEAR-DEPTH", "error",
              std::string(kSkillId) + ": depth_um " +
              std::to_string(in.depth_um) +
              " > 2000 μm — exceeds surface-treatment ceiling "
              "(consider through-hardening instead)");
    }

    if (in.hardness_increase_pct <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": hardness_increase_pct must be > 0");
    } else if (in.hardness_increase_pct < 5.0) {
        r.add("DFM-ANTIWEAR-HARDNESS", "error",
              std::string(kSkillId) + ": hardness_increase_pct " +
              std::to_string(in.hardness_increase_pct) +
              " < 5 % — below noise floor of HV measurement");
    } else if (in.hardness_increase_pct > 500.0) {
        r.add("DFM-ANTIWEAR-HARDNESS", "error",
              std::string(kSkillId) + ": hardness_increase_pct " +
              std::to_string(in.hardness_increase_pct) +
              " > 500 % — unrealistic gain claim");
    }

    if (!isKnownTechnique(in.technique)) {
        r.add("DFM-ANTIWEAR-TECHNIQUE", "info",
              std::string(kSkillId) + ": technique '" + in.technique +
              "' not in known set — expected laser_hardening | "
              "induction_hardening | surface_texturing | ion_implantation");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "anti_wear_treatment DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError(
        "anti_wear_treatment: entry_face datum unresolved");

    GProp_GProps props;
    BRepGProp::SurfaceProperties(wp.face(*entryId), props);
    const double faceArea = props.Mass();

    json params = {
        { "entry_face_id",         *entryId },
        { "technique",             in.technique },
        { "depth_um",              in.depth_um },
        { "hardness_increase_pct", in.hardness_increase_pct },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "target_face_id",        *entryId },
        { "technique",             in.technique },
        { "depth_um",              in.depth_um },
        { "hardness_increase_pct", in.hardness_increase_pct },
        { "target_face_area_mm2",  faceArea },
        { "geometry_changed",      false },
    };

    ToolingMeta tooling;
    tooling.tool_type =
        in.technique == "laser_hardening"     ? "laser_hardening_head" :
        in.technique == "induction_hardening" ? "induction_coil"       :
        in.technique == "surface_texturing"   ? "ultrafast_laser_head" :
        in.technique == "ion_implantation"    ? "ion_implantation_gun" :
                                                "anti_wear_apparatus";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = in.technique;
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;          // texturing-only / diffusion
    // ~ 30 s per 100 mm² for laser scan; floor at 5 s.
    tooling.est_cycle_time_s  = std::max(5.0, faceArea / 100.0 * 30.0);
    tooling.extra = {
        { "technique",             in.technique },
        { "depth_um",              in.depth_um },
        { "hardness_increase_pct", in.hardness_increase_pct },
        { "process",               std::string("anti_wear_") + in.technique },
        { "effect",                "surface hardness ↑, wear resistance ↑" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::anti_wear_treatment applied: face={} tech={} "
                  "depth={}μm Δhardness={}%",
                  *entryId, in.technique, in.depth_um,
                  in.hardness_increase_pct);

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

}  // namespace koocadcam::skill::anti_wear_treatment
