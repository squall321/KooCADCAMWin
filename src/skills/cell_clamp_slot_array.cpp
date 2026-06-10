// @lat: [[engine/skills#cell_clamp_slot_array]]

#include "cell_clamp_slot_array.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::cell_clamp_slot_array {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "cell_clamp_slot_array: face_id out of range");
    }
    if (!(in.cell_dia_mm > 0.0) || !(in.cell_pitch_x_mm > 0.0) ||
        !(in.cell_pitch_y_mm > 0.0) ||
        !(in.clamp_slot_width_mm > 0.0) || !(in.clamp_slot_depth_mm > 0.0) ||
        in.cell_count_x <= 0 || in.cell_count_y <= 0) {
        r.add("DFM-INPUT", "error",
              "cell_clamp_slot_array: all dimensions and counts must be > 0");
        return r;
    }

    if (in.cell_dia_mm < 10.0 || in.cell_dia_mm > 30.0) {
        r.add("DFM-CELL-DIA", "error",
              "cell_clamp_slot_array: cell_dia " +
              std::to_string(in.cell_dia_mm) +
              " mm outside [10, 30] mm typical cylindrical cell range");
    }

    if (in.cell_pitch_x_mm < in.cell_dia_mm + 0.5 ||
        in.cell_pitch_y_mm < in.cell_dia_mm + 0.5) {
        r.add("DFM-CELL-PITCH", "error",
              "cell_clamp_slot_array: pitch must exceed cell_dia by ≥ 0.5 mm");
    }

    if (in.clamp_slot_width_mm < 0.5) {
        r.add("DFM-SLOT-WIDTH", "error",
              "cell_clamp_slot_array: clamp_slot_width " +
              std::to_string(in.clamp_slot_width_mm) +
              " mm < 0.5 mm minimum (uncuttable)");
    }

    if (in.clamp_slot_width_mm > in.cell_dia_mm * 0.25) {
        r.add("DFM-SLOT-OVERWIDE", "warning",
              "cell_clamp_slot_array: clamp_slot_width > 25% of cell_dia "
              "is unusual for spring clamps");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "cell_clamp_slot_array DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const gp_Dir n     = wp.faceNormal(in.face_id);
    const double baseZ = faceC.Z();

    // Drill direction = INTO the housing (opposite outward normal).
    const gp_Dir drillDir(-n.X(), -n.Y(), -n.Z());

    const double cellR        = in.cell_dia_mm / 2.0;
    const double slotW        = in.clamp_slot_width_mm;
    const double slotDepth    = in.clamp_slot_depth_mm;
    // Slot length = slotW + radial extension outward from cell wall.
    // We make it 1.6 × slotW long radially: from (cellR - 0.2 × slotW)
    // outward to (cellR + slotW).  Provides spring engagement.
    const double slotLengthRadial = slotW * 1.5 + cellR * 0.05;

    constexpr double kEmbed = 0.05;
    const double pocketDepth = slotDepth + kEmbed;

    std::vector<TopoDS_Shape> cutters;
    cutters.reserve(static_cast<size_t>(in.cell_count_x * in.cell_count_y * 5));

    double totalRemoved = 0.0;

    for (int iy = 0; iy < in.cell_count_y; ++iy) {
        for (int ix = 0; ix < in.cell_count_x; ++ix) {
            const double cx = in.origin_x_mm + ix * in.cell_pitch_x_mm;
            const double cy = in.origin_y_mm + iy * in.cell_pitch_y_mm;

            // ── 1. Cylindrical cell pocket ───────────────────────────────
            const gp_Pnt pocketOrigin(cx, cy, baseZ + kEmbed);
            const gp_Ax2 pocketAx(pocketOrigin, drillDir);
            cutters.push_back(pr::cylinder(pocketAx, cellR, pocketDepth));
            totalRemoved += M_PI * cellR * cellR * slotDepth;

            // ── 2. 4 axial clamp slots N/S/E/W ───────────────────────────
            // Each slot is a small box centred on the cell wall (at radius
            // cellR), straddling the wall by slotLengthRadial.  Slot box is
            // slotW wide tangential × slotLengthRadial long radial × slotDepth
            // deep axial.
            //
            // We use box(ax, dx, dy, dz) where Vmain = drillDir (axial), so
            // DZ = slotDepth.
            // Vx = radial outward direction (per slot orientation).
            const struct {
                double rx, ry;
            } dirs[4] = {
                { +1.0,  0.0 },  // +X
                { -1.0,  0.0 },  // -X
                {  0.0, +1.0 },  // +Y
                {  0.0, -1.0 },  // -Y
            };
            for (int k = 0; k < 4; ++k) {
                const gp_Dir radialOut(dirs[k].rx, dirs[k].ry, 0.0);
                // Tangential = Z × radial; we use (-ry, rx, 0) so DY positive
                // lies tangentially.
                const gp_Dir tang(-dirs[k].ry, dirs[k].rx, 0.0);
                // Slot extends from (cellR - 0.2*slotW) outward to
                // (cellR + slotLengthRadial - 0.2*slotW)
                const double radialStart = cellR - 0.2 * slotW;
                const gp_Pnt slotOrigin(
                    cx + radialOut.X() * radialStart - tang.X() * (slotW / 2.0),
                    cy + radialOut.Y() * radialStart - tang.Y() * (slotW / 2.0),
                    baseZ + kEmbed);
                const gp_Ax2 slotAx(slotOrigin, drillDir, radialOut);
                // DX along Vx = radial, DY tangential, DZ axial (into housing).
                cutters.push_back(pr::box(
                    slotAx, slotLengthRadial, slotW, slotDepth));
                totalRemoved += slotLengthRadial * slotW * slotDepth;
            }
        }
    }

    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), cutters);

    const int cellCount = in.cell_count_x * in.cell_count_y;

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "face_id",             in.face_id },
        { "origin_x_mm",         in.origin_x_mm },
        { "origin_y_mm",         in.origin_y_mm },
        { "cell_dia_mm",         in.cell_dia_mm },
        { "cell_count_x",        in.cell_count_x },
        { "cell_count_y",        in.cell_count_y },
        { "cell_pitch_x_mm",     in.cell_pitch_x_mm },
        { "cell_pitch_y_mm",     in.cell_pitch_y_mm },
        { "clamp_slot_width_mm", in.clamp_slot_width_mm },
        { "clamp_slot_depth_mm", in.clamp_slot_depth_mm },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "is_compound",      true },
        { "is_ev_feature",    true },
        { "ev_feature_type",  "cell_clamp_array" },
        { "cell_count",       cellCount },
        { "subfeature_count", cellCount * 5 },
        { "cell_dia_mm",      in.cell_dia_mm },
        { "cell_format",      (in.cell_dia_mm > 19.5) ? "21700_or_larger"
                                                      : "18650_class" },
        { "derived_volume_removed_mm3", totalRemoved },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill;end_mill";
    tooling.tool_dia_mm       = in.cell_dia_mm;
    tooling.tool_length_mm    = slotDepth + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = totalRemoved;
    tooling.est_cycle_time_s  = std::max(5.0, totalRemoved / 200.0);
    tooling.extra = {
        { "feature_standard", "EV cell clamp array (cylindrical 18650/21700)" },
        { "cell_count",       cellCount },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::cell_clamp_slot_array applied: nx={} ny={} cellD={}",
        in.cell_count_x, in.cell_count_y, in.cell_dia_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition: metadata replay + cylindrical-array geometric fallback ──

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = {
            { "source",          "metadata_replay" },
            { "is_compound",     true },
            { "ev_feature_type", "cell_clamp_array" },
        };
        out.push_back(r);
    }

    // Geometric fallback: count distinct cylindrical faces that look like
    // 18650/21700 cell pockets (axial extent significant, radius ∈ [9, 11]
    // or [10, 11.5]).  If we see at least 4 in a regular pattern, emit one
    // low-confidence candidate.
    if (out.empty()) {
        int cylCount = 0;
        double obsRadius = 0.0;
        for (int i = 0; i < wp.faceCount(); ++i) {
            if (!wp.isFaceCylinder(i)) continue;
            BRepAdaptor_Surface surf(wp.face(i));
            const double r = surf.Cylinder().Radius();
            if (r > 5.0 && r < 15.5) {
                cylCount++;
                obsRadius = r;
            }
        }
        if (cylCount >= 4) {
            json recovered = {
                { "cell_dia_mm", 2.0 * obsRadius },
                { "cell_count_x", static_cast<int>(std::sqrt(double(cylCount))) },
                { "cell_count_y", static_cast<int>(std::sqrt(double(cylCount))) },
            };
            json matched = {
                { "source",            "geometric_fallback" },
                { "cyl_pocket_count",  cylCount },
                { "observed_radius_mm", obsRadius },
            };
            out.push_back(RecognizedFeature{ kSkillId, recovered, 0.5, matched });
        }
    }
    return out;
}

}  // namespace koocadcam::skill::cell_clamp_slot_array
