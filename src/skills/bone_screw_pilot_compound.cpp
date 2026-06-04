// @lat: [[engine/skills#bone_screw_pilot_compound]]

#include "bone_screw_pilot_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <array>
#include <cmath>

namespace koocadcam::skill::bone_screw_pilot_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── AO/ASIF cortical screw → pilot drill ─────────────────────────────────
namespace {

struct PilotEntry {
    const char* size_id;        // "2.0" .. "6.5"
    double      nominal_dia_mm; // screw OD
    double      pilot_dia_mm;   // AO/ASIF pilot
};

constexpr std::array<PilotEntry, 7> kPilots {{
    { "2.0", 2.0, 1.5 },
    { "2.4", 2.4, 1.8 },
    { "2.7", 2.7, 2.0 },
    { "3.5", 3.5, 2.5 },
    { "4.0", 4.0, 2.9 },
    { "4.5", 4.5, 3.2 },
    { "6.5", 6.5, 4.5 },
}};

const PilotEntry* lookup(const std::string& s) {
    for (const auto& e : kPilots) if (s == e.size_id) return &e;
    return nullptr;
}

}  // namespace

double pilotDiaFor(const std::string& screw_size_id)
{
    const auto* e = lookup(screw_size_id);
    return e ? e->pilot_dia_mm : 0.0;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.screw_size_id.empty()) {
        r.add("DFM-BONESCREW-SIZE", "error",
              "bone_screw_pilot_compound: screw_size_id is empty");
        return r;
    }
    const auto* e = lookup(in.screw_size_id);
    if (!e) {
        r.add("DFM-BONESCREW-SIZE", "error",
              "bone_screw_pilot_compound: screw_size_id '" + in.screw_size_id +
              "' not in AO/ASIF cortical set {2.0, 2.4, 2.7, 3.5, 4.0, 4.5, 6.5}");
        return r;
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-BONESCREW-DEPTH", "error",
              "bone_screw_pilot_compound: depth_mm must be > 0");
    }
    if (in.countersink_dia_mm <= e->pilot_dia_mm) {
        r.add("DFM-BONESCREW-COUNTERSINK", "error",
              "bone_screw_pilot_compound: countersink_dia " +
              std::to_string(in.countersink_dia_mm) +
              " mm must exceed pilot_dia " +
              std::to_string(e->pilot_dia_mm) + " mm");
    }
    if (in.chamfer_dia_mm <= in.countersink_dia_mm) {
        r.add("DFM-BONESCREW-CHAMFER", "error",
              "bone_screw_pilot_compound: chamfer_dia " +
              std::to_string(in.chamfer_dia_mm) +
              " mm must exceed countersink_dia " +
              std::to_string(in.countersink_dia_mm) + " mm");
    }

    // Stock-thickness sanity: pilot must fit.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness = zMax - zMin;
    if (thickness > 0.0 && in.depth_mm > thickness + 1e-6) {
        r.add("DFM-BONESCREW-DEPTH", "warning",
              "bone_screw_pilot_compound: depth " + std::to_string(in.depth_mm) +
              " mm exceeds stock thickness " + std::to_string(thickness));
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "bone_screw_pilot_compound DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto* e = lookup(in.screw_size_id);
    const double pilotDia = e->pilot_dia_mm;
    const double cskTopR  = in.countersink_dia_mm / 2.0;
    const double chamTopR = in.chamfer_dia_mm / 2.0;

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("bone_screw_pilot_compound: entry_face unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const double kOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    // Default (general-axis case)
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));
    gp_Pnt toolStartCyl(
        in.position_x_mm - adir.X() * (bboxDiag + kOverhang),
        in.position_y_mm - adir.Y() * (bboxDiag + kOverhang),
        (zMin + zMax) / 2.0 - adir.Z() * (bboxDiag + kOverhang));
    gp_Pnt entryPlanePoint(toolStartCyl);

    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        const double entryZ = (adir.Z() < 0) ? zMax : zMin;
        toolStartCyl    = gp_Pnt(in.position_x_mm, in.position_y_mm,
                                 entryZ - adir.Z() * kOverhang);
        entryPlanePoint = gp_Pnt(in.position_x_mm, in.position_y_mm, entryZ);
    }

    const gp_Ax2 pilotAx (toolStartCyl,    adir);
    const gp_Ax2 cskAx   (entryPlanePoint, adir);

    // Sub 1: pilot bore.
    const TopoDS_Shape pilotTool =
        pr::cylinder(pilotAx, pilotDia / 2.0, in.depth_mm + kOverhang);

    // Sub 2: 90° countersink cone.  At included 90° → half-angle 45°,
    // cone depth = (top_dia - pilot_dia)/2 / tan(45°) = (top_dia - pilot_dia)/2.
    const double cskAngleDeg = 90.0;
    const double cskDepth    = (in.countersink_dia_mm - pilotDia) / 2.0
                             / std::tan(cskAngleDeg * 0.5 * M_PI / 180.0);
    const TopoDS_Shape cskTool =
        pr::coneFrustum(cskAx, cskTopR, pilotDia / 2.0, cskDepth);

    // Sub 3: shallow chamfer cone above the countersink (lead-in / de-burr).
    // 45° chamfer between chamfer_dia (top) and countersink_dia (bottom of
    // chamfer = top of countersink).  Chamfer sits OUTSIDE entry plane,
    // pushing slightly above (negative along adir) so it carves the corner.
    const double chamferDepth = (in.chamfer_dia_mm - in.countersink_dia_mm) / 2.0;
    const gp_Pnt chamferOrigin(
        entryPlanePoint.X() - adir.X() * chamferDepth,
        entryPlanePoint.Y() - adir.Y() * chamferDepth,
        entryPlanePoint.Z() - adir.Z() * chamferDepth);
    const gp_Ax2 chamferAx(chamferOrigin, adir);
    const TopoDS_Shape chamferTool =
        pr::coneFrustum(chamferAx, chamTopR, cskTopR, chamferDepth);

    // Fuse all three concentric cutters into a single connected tool,
    // then perform one cut.
    TopoDS_Shape fused = pr::fuse(pilotTool, cskTool);
    fused = pr::fuse(fused, chamferTool);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    // Signature
    json params = {
        { "entry_face_id",      *entryId },
        { "position_x_mm",      in.position_x_mm },
        { "position_y_mm",      in.position_y_mm },
        { "axis_dir",           { adir.X(), adir.Y(), adir.Z() } },
        { "screw_size_id",      in.screw_size_id },
        { "depth_mm",           in.depth_mm },
        { "countersink_dia_mm", in.countersink_dia_mm },
        { "chamfer_dia_mm",     in.chamfer_dia_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    3 },
        { "medical_feature_type","cortical_bone_screw_pilot" },
        { "screw_size",          in.screw_size_id },
        { "screw_nominal_dia_mm",e->nominal_dia_mm },
        { "pilot_dia_mm",        pilotDia },
        { "countersink_dia_mm",  in.countersink_dia_mm },
        { "countersink_depth_mm",cskDepth },
        { "countersink_angle_deg", cskAngleDeg },
        { "chamfer_dia_mm",      in.chamfer_dia_mm },
        { "chamfer_depth_mm",    chamferDepth },
        { "axis_dir",            { adir.X(), adir.Y(), adir.Z() } },
        { "implant_standard",    "ISO_5836_AOASIF" },
    };

    ToolingMeta tooling;
    tooling.tool_type   = "drill;countersink;chamfer";
    tooling.tool_dia_mm = in.chamfer_dia_mm;
    tooling.tool_length_mm = in.depth_mm * 1.5 + 5.0;
    tooling.tool_material  = "carbide";
    tooling.flute_count    = 2;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.03;
    const double pR = pilotDia / 2.0;
    const double pilotVol = M_PI * pR * pR * in.depth_mm;
    const double cskVol   = (M_PI * cskDepth / 3.0)
                          * (cskTopR * cskTopR + cskTopR * pR + pR * pR)
                          - M_PI * pR * pR * cskDepth;
    const double chamVol  = (M_PI * chamferDepth / 3.0)
                          * (chamTopR * chamTopR + chamTopR * cskTopR + cskTopR * cskTopR)
                          - M_PI * cskTopR * cskTopR * chamferDepth;
    tooling.stock_removed_mm3 = pilotVol + std::max(0.0, cskVol) + std::max(0.0, chamVol);
    tooling.est_cycle_time_s  = std::max(1.0, in.depth_mm / 30.0);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::bone_screw_pilot_compound applied: size {} pilot {} depth {}",
                  in.screw_size_id, pilotDia, in.depth_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Metadata replay (compound feature, hard to geometric-fingerprint
// uniquely against a plain countersink).  Geometric fallback returns
// nothing — by-design the metadata path is the contract.

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
                                  "cortical_bone_screw_pilot" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::bone_screw_pilot_compound
