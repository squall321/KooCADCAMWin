// @lat: [[engine/skills#drill_through_hole]]

#include "drill_through_hole.hpp"

#include "Workpiece.hpp"
#include "drill_hole.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>

namespace koocadcam::skill::drill_through_hole {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────
//
// We compose drill_hole's validate() (so DFM-002 etc fire identically) and
// then add a thickness check specific to through holes — there must be
// material along axis_dir for the hole to traverse.

DFMReport validate(const Workpiece& wp, const Input& in)
{
    // Mirror drill_hole's checks via a synthesized Input.
    drill_hole::Input dh;
    dh.entry_face    = in.entry_face;
    dh.position_x_mm = in.position_x_mm;
    dh.position_y_mm = in.position_y_mm;
    dh.axis_dir      = in.axis_dir;
    dh.diameter_mm   = in.diameter_mm;
    dh.depth_mm      = 0.0;             // unused: through_hole=true ignores depth
    dh.through_hole  = true;
    DFMReport r = drill_hole::validate(wp, dh);

    // Extra: verify the workpiece has material to traverse along axis_dir.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const gp_Dir adir = in.axis_dir;
    const double extent =
        std::abs(adir.X()) * (xMax - xMin) +
        std::abs(adir.Y()) * (yMax - yMin) +
        std::abs(adir.Z()) * (zMax - zMin);
    if (extent <= 0.0) {
        r.add("DFM-THICKNESS", "error",
              "drill_through_hole: workpiece has zero extent along axis_dir");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Delegate to drill_hole::apply with through_hole=true, then OVERRIDE the
// resulting signature's skill_id and pattern.kind so this feature is
// catalogued as a "drill_through_hole" — even though the topology is
// produced by drill_hole internally.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    // Our own DFM gate first (covers DFM-002 + thickness).
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "drill_through_hole DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Compose: build the drill_hole input with through_hole=true.
    drill_hole::Input dh;
    dh.entry_face    = in.entry_face;
    dh.position_x_mm = in.position_x_mm;
    dh.position_y_mm = in.position_y_mm;
    dh.axis_dir      = in.axis_dir;
    dh.diameter_mm   = in.diameter_mm;
    dh.depth_mm      = 0.0;            // ignored when through_hole=true
    dh.through_hole  = true;

    SkillOutput drillOut = drill_hole::apply(wp, dh);

    // Re-stamp the signature so the catalog records this as a
    // "drill_through_hole" intent rather than a plain drill_hole.
    FeatureSignature sig = drillOut.signature;
    sig.skill_id        = kSkillId;
    sig.pattern["kind"] = kSkillId;
    // Echo our own input params verbatim (no depth_mm in this skill).
    sig.params = {
        { "position_x_mm", in.position_x_mm },
        { "position_y_mm", in.position_y_mm },
        { "axis_dir",      { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "diameter_mm",   in.diameter_mm },
        { "through_hole",  true },
    };

    // The drill_hole apply() already appended its own signature to the
    // returned workpiece's feature history.  We append our re-stamped
    // signature on top — the inner drill_hole entry documents the
    // underlying topology, the outer drill_through_hole entry captures
    // the higher-level intent.  Workpiece exposes addFeature but no
    // pop/clear, so this two-entry layout is intentional.
    auto wpNew = drillOut.workpiece;
    wpNew->addFeature(sig);

    spdlog::debug("skill::drill_through_hole applied: dia={} (composed via drill_hole)",
                  in.diameter_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// drill_hole::recognize finds every cylinder-as-hole pattern and tags
// each with a `through_hole` boolean.  We filter to keep only through
// candidates and re-stamp them with this skill's id.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    auto drillCands = drill_hole::recognize(wp);
    for (const auto& c : drillCands) {
        const auto it = c.recovered_params.find("through_hole");
        if (it == c.recovered_params.end()) continue;
        if (!it->get<bool>()) continue;

        // Build a recovered_params view that drops depth_mm (irrelevant for
        // a through hole — the depth is determined by the workpiece).
        json rp = c.recovered_params;
        rp.erase("depth_mm");

        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = rp;
        r.confidence       = 0.95;
        r.matched_geometry = c.matched_geometry;
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::drill_through_hole
