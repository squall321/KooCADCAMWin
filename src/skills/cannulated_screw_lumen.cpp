// @lat: [[engine/skills#cannulated_screw_lumen]]

#include "cannulated_screw_lumen.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>

namespace koocadcam::skill::cannulated_screw_lumen {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.outer_dia_mm <= 0.0) {
        r.add("DFM-CANN-INPUT", "error",
              "cannulated_screw_lumen: outer_dia_mm must be > 0");
    }
    if (in.lumen_dia_mm < 0.5) {
        r.add("DFM-CANN-LUMEN-MIN", "error",
              "cannulated_screw_lumen: lumen_dia " +
              std::to_string(in.lumen_dia_mm) +
              " mm < 0.5 mm (smallest practical K-wire path)");
    }
    if (in.outer_dia_mm > 0.0 &&
        in.lumen_dia_mm >= in.outer_dia_mm * 0.5) {
        r.add("DFM-CANN-LUMEN", "error",
              "cannulated_screw_lumen: lumen_dia " +
              std::to_string(in.lumen_dia_mm) +
              " mm must be < 0.5 × outer_dia (" +
              std::to_string(in.outer_dia_mm * 0.5) +
              " mm) — wall thickness insufficient");
    }
    if (in.head_dia_mm <= in.outer_dia_mm) {
        r.add("DFM-CANN-HEAD", "error",
              "cannulated_screw_lumen: head_dia " +
              std::to_string(in.head_dia_mm) +
              " mm must exceed outer_dia " +
              std::to_string(in.outer_dia_mm) + " mm");
    }
    if (in.length_mm <= in.head_depth_mm + 1.0) {
        r.add("DFM-CANN-LENGTH", "error",
              "cannulated_screw_lumen: length " +
              std::to_string(in.length_mm) +
              " mm must exceed head_depth + 1 mm");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "cannulated_screw_lumen DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("cannulated_screw_lumen: entry_face unresolved");

    const gp_Dir adir   = in.screw_axis_dir;
    const gp_Pnt origin = in.screw_axis_origin;

    const double kOverhang = 0.05;

    // Sub 1: through lumen along the screw axis.
    const gp_Pnt lumenStart(
        origin.X() - adir.X() * kOverhang,
        origin.Y() - adir.Y() * kOverhang,
        origin.Z() - adir.Z() * kOverhang);
    const gp_Ax2 lumenAx(lumenStart, adir);
    const TopoDS_Shape lumenTool =
        pr::cylinder(lumenAx, in.lumen_dia_mm / 2.0,
                     in.length_mm + 2.0 * kOverhang);

    // Sub 2: head recess at the entry face.
    const gp_Ax2 headAx(lumenStart, adir);
    const TopoDS_Shape headTool =
        pr::cylinder(headAx, in.head_dia_mm / 2.0,
                     in.head_depth_mm + kOverhang);

    // Fuse concentric cutters, single cut.
    const TopoDS_Shape fused   = pr::fuse(lumenTool, headTool);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    // Signature
    json params = {
        { "entry_face_id",       *entryId },
        { "screw_axis_dir",      { adir.X(), adir.Y(), adir.Z() } },
        { "screw_axis_origin",   { origin.X(), origin.Y(), origin.Z() } },
        { "outer_dia_mm",        in.outer_dia_mm },
        { "lumen_dia_mm",        in.lumen_dia_mm },
        { "length_mm",           in.length_mm },
        { "head_dia_mm",         in.head_dia_mm },
        { "head_depth_mm",       in.head_depth_mm },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "subfeature_count",      2 },
        { "medical_feature_type",  "cannulated_bone_screw" },
        { "screw_size",            in.outer_dia_mm },
        { "outer_dia_mm",          in.outer_dia_mm },
        { "lumen_dia_mm",          in.lumen_dia_mm },
        { "head_dia_mm",           in.head_dia_mm },
        { "wall_thickness_mm",     (in.outer_dia_mm - in.lumen_dia_mm) / 2.0 },
        { "lumen_through",         true },
        { "axis_dir",              { adir.X(), adir.Y(), adir.Z() } },
        { "implant_standard",      "ISO_5832_ASTM_F543" },
    };

    ToolingMeta tooling;
    tooling.tool_type   = "gun_drill;endmill";
    tooling.tool_dia_mm = in.lumen_dia_mm;
    tooling.tool_length_mm = in.length_mm * 1.2 + 5.0;
    tooling.tool_material  = "carbide";
    tooling.flute_count    = 1;
    tooling.cutting_speed_sfm = 150.0;
    tooling.feed_per_tooth_mm = 0.02;
    const double lR = in.lumen_dia_mm / 2.0;
    const double hR = in.head_dia_mm / 2.0;
    tooling.stock_removed_mm3 = M_PI * lR * lR * in.length_mm
                              + M_PI * hR * hR * in.head_depth_mm;
    tooling.est_cycle_time_s  = std::max(1.0, in.length_mm / 20.0);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::cannulated_screw_lumen applied: outer {} lumen {} length {}",
                  in.outer_dia_mm, in.lumen_dia_mm, in.length_mm);

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
        rf.matched_geometry = { { "source", "metadata" },
                                { "is_compound", true },
                                { "medical_feature_type",
                                  "cannulated_bone_screw" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::cannulated_screw_lumen
