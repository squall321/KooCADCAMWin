// @lat: [[engine/skills#friction_stir]]

#include "friction_stir.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <map>

namespace koocadcam::skill::friction_stir {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.tool_shoulder_dia_mm < 6.0) {
        r.add("DFM-FSW-DIA", "error",
              "friction_stir tool_shoulder_dia_mm " +
              std::to_string(in.tool_shoulder_dia_mm) +
              " < 6.0 mm (sub-spec FSW pin tool)");
    }
    if (in.tool_rotation_rpm < 200.0) {
        r.add("DFM-FSW-RPM", "error",
              "friction_stir tool_rotation_rpm " +
              std::to_string(in.tool_rotation_rpm) +
              " < 200 (insufficient frictional heating)");
    }
    if (in.tool_rotation_rpm > 3000.0) {
        r.add("DFM-FSW-RPM", "error",
              "friction_stir tool_rotation_rpm " +
              std::to_string(in.tool_rotation_rpm) +
              " > 3000 (excessive tool wear, defect risk)");
    }
    if (in.traverse_speed_mm_per_s <= 0.0) {
        r.add("DFM-INPUT", "error",
              "friction_stir traverse_speed_mm_per_s must be > 0");
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "friction_stir depth_mm must be > 0");
    }
    if (in.depth_mm > 0.0 && in.depth_mm < 0.05) {
        r.add("DFM-FSW-DEPTH", "error",
              "friction_stir depth_mm " + std::to_string(in.depth_mm) +
              " < 0.05 mm (tool barely touches the surface)");
    }
    if (in.depth_mm > 0.5) {
        r.add("DFM-FSW-DEPTH", "warning",
              "friction_stir depth_mm " + std::to_string(in.depth_mm) +
              " > 0.5 mm — surface flash will form; consider larger shoulder");
    }
    const double seamLen = std::hypot(in.end_x_mm - in.start_x_mm,
                                      in.end_y_mm - in.start_y_mm);
    if (seamLen <= 1e-6) {
        r.add("DFM-INPUT", "error", "friction_stir start and end must differ");
    }
    if (in.tool_shoulder_dia_mm > 0.0 && seamLen > 0.0 &&
        seamLen <= in.tool_shoulder_dia_mm) {
        r.add("DFM-FSW-LEN", "warning",
              "friction_stir length " + std::to_string(seamLen) +
              " ≤ tool_shoulder_dia " +
              std::to_string(in.tool_shoulder_dia_mm) +
              " — single-spot plunge weld instead");
    }
    // Material compatibility — INFO only.
    if (in.material != "aluminum" && in.material != "magnesium" &&
        in.material != "steel") {
        r.add("DFM-FSW-MAT", "info",
              "friction_stir material '" + in.material +
              "' not in {aluminum, magnesium, steel} — verify weldability");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Friction stir is the ONLY weld in this family that REMOVES material — the
// rotating shoulder leaves a shallow groove.  We model it with a thin
// stadium-shaped subtractive cutter (same shape as mill_slot but very
// shallow).  No filler is deposited.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "friction_stir DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("friction_stir: entry_face datum unresolved");
    if (!wp.isFacePlanar(*entryId))
        throw SkillError("friction_stir: entry_face must be planar");

    const gp_Dir outNorm = wp.faceNormal(*entryId);
    const gp_Pnt faceCtr = wp.faceCenter(*entryId);

    const double radius = in.tool_shoulder_dia_mm / 2.0;
    const double kOverhang = 0.05;
    // Cutter penetrates INWARD = opposite of outNorm.
    const double cutterH = in.depth_mm + kOverhang;
    const double baseZ   = faceCtr.Z();

    // Cutter ORIGIN is OUTSIDE the surface by `kOverhang` so the Boolean cut
    // is clean.  Cutter direction = -outNorm so DZ goes INTO the material.
    const gp_Dir inwardDir(-outNorm.X(), -outNorm.Y(), -outNorm.Z());

    const gp_Pnt sBase(in.start_x_mm + outNorm.X() * kOverhang,
                       in.start_y_mm + outNorm.Y() * kOverhang,
                       baseZ          + outNorm.Z() * kOverhang);
    const gp_Pnt eBase(in.end_x_mm   + outNorm.X() * kOverhang,
                       in.end_y_mm   + outNorm.Y() * kOverhang,
                       baseZ          + outNorm.Z() * kOverhang);

    const gp_Ax2 axStart(sBase, inwardDir);
    const gp_Ax2 axEnd  (eBase, inwardDir);
    const TopoDS_Shape cylStart = pr::cylinder(axStart, radius, cutterH);
    const TopoDS_Shape cylEnd   = pr::cylinder(axEnd,   radius, cutterH);

    const gp_Vec dir2D(in.end_x_mm - in.start_x_mm,
                       in.end_y_mm - in.start_y_mm,
                       0.0);
    const double seamLen = dir2D.Magnitude();
    const gp_Dir xLoc(dir2D.X() / seamLen, dir2D.Y() / seamLen, 0.0);
    const gp_Dir yLoc(-xLoc.Y(), xLoc.X(), 0.0);

    // Box origin sits at the BOTTOM of the cut so DZ along outNorm extends
    // upward through the surface (mirror of mill_slot, where box DZ extends
    // up from z = zMax - depth).  YDir = outNorm × xLoc = yLoc by right-hand
    // rule for outNorm = +Z, so the box spans [-r, +r] along yLoc.
    const gp_Pnt boxOrigin(
        in.start_x_mm - radius * yLoc.X() - outNorm.X() * in.depth_mm,
        in.start_y_mm - radius * yLoc.Y() - outNorm.Y() * in.depth_mm,
        baseZ          - outNorm.Z() * in.depth_mm);
    const gp_Ax2 boxAx(boxOrigin, outNorm, xLoc);
    const TopoDS_Shape boxConn = pr::box(boxAx,
                                         seamLen,
                                         in.tool_shoulder_dia_mm,
                                         cutterH);

    const TopoDS_Shape cutter = pr::fuseMany(cylStart, { cylEnd, boxConn });
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json params = {
        { "entry_face_id",          *entryId },
        { "start_x_mm",             in.start_x_mm },
        { "start_y_mm",             in.start_y_mm },
        { "end_x_mm",               in.end_x_mm },
        { "end_y_mm",               in.end_y_mm },
        { "tool_shoulder_dia_mm",   in.tool_shoulder_dia_mm },
        { "tool_rotation_rpm",      in.tool_rotation_rpm },
        { "traverse_speed_mm_per_s", in.traverse_speed_mm_per_s },
        { "depth_mm",               in.depth_mm },
        { "material",               in.material },
        { "entry_face_normal", { outNorm.X(), outNorm.Y(), outNorm.Z() } },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "end_cylinder_count",         2 },
        { "long_wall_planar_count",     2 },
        { "bottom_planar_face",         true },
        { "tool_shoulder_dia_mm",       in.tool_shoulder_dia_mm },
        { "depth_mm",                   in.depth_mm },
        { "seam_length_mm",             seamLen },
        { "tool_rotation_rpm",          in.tool_rotation_rpm },
        // KEY DIFFERENTIATOR: subtractive — no filler deposited.
        { "additive",                   false },
        { "subtractive_shallow_trace",  true },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "fsw_pin_tool";
    tooling.tool_dia_mm       = in.tool_shoulder_dia_mm;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "h13_tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    // Volume REMOVED (positive — same convention as drill_hole / mill_slot).
    const double removedVol =
        (M_PI * radius * radius + seamLen * in.tool_shoulder_dia_mm) * in.depth_mm;
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  =
        std::max(2.0, seamLen / std::max(1e-3, in.traverse_speed_mm_per_s));
    tooling.extra = {
        { "process",                "friction_stir_weld" },
        { "material",               in.material },
        { "tool_rotation_rpm",      in.tool_rotation_rpm },
        { "traverse_speed_mm_per_s", in.traverse_speed_mm_per_s },
        { "solid_state",            true },
        { "filler",                 "none" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::friction_stir applied: len={} shoulder={} depth={} faces {}→{}",
                  seamLen, in.tool_shoulder_dia_mm, in.depth_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Friction stir leaves a shallow SUBTRACTIVE stadium trace — topologically
// identical to mill_slot.  Differentiator is the unusually-shallow depth
// (< 0.5 mm) combined with a relatively LARGE shoulder diameter (≥ 6 mm).
// We still rely on metadata when available — the geometric path is a
// fallback that gives lower confidence.

namespace {

using EdgeFaceMap = NCollection_IndexedDataMap<
    TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>;

EdgeFaceMap buildEdgeFaceMap(const TopoDS_Shape& shape)
{
    EdgeFaceMap m;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, m);
    return m;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Metadata replay first.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    if (!out.empty()) return out;

    // Geometric fallback: look for shallow stadium depressions.
    const auto edgeFaces = buildEdgeFaceMap(wp.shape());

    struct CylEntry {
        int     cylIdx;
        double  radius;
        gp_Pnt  axisBase;
        double  zHi, zLo;
    };
    std::map<int, std::vector<CylEntry>> bottomGroups;

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        const TopoDS_Face& cf = wp.face(fIdx);
        BRepAdaptor_Surface surf(cf);
        const gp_Cylinder cyl = surf.Cylinder();
        const gp_Dir adir = cyl.Axis().Direction();
        if (std::abs(std::abs(adir.Z()) - 1.0) > 1e-2) continue;
        const double rad = cyl.Radius();
        if (rad * 2.0 < 6.0)  continue;   // shoulder dia ≥ 6 mm
        if (rad * 2.0 > 30.0) continue;   // outside FSW practical range

        int bottomFaceIdx = -1;
        double zHi = -1e30, zLo = 1e30;
        for (TopExp_Explorer exp(cf, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Pnt mid = crv.Value(
                (crv.FirstParameter() + crv.LastParameter()) / 2.0);
            zHi = std::max(zHi, mid.Z());
            zLo = std::min(zLo, mid.Z());
            if (!edgeFaces.Contains(e)) continue;
            const auto& adj = edgeFaces.FindFromKey(e);
            for (NCollection_List<TopoDS_Shape>::Iterator it(adj); it.More(); it.Next()) {
                const TopoDS_Face& af = TopoDS::Face(it.Value());
                if (af.IsSame(cf)) continue;
                if (BRepAdaptor_Surface(af).GetType() != GeomAbs_Plane) continue;
                GProp_GProps gp;
                BRepGProp::SurfaceProperties(af, gp);
                // SUBTRACTIVE: bottom planar face is small (≈ slot bottom),
                // distinguishes depression from a protrusion.
                const double areaCap = M_PI * rad * rad * 5.0;
                if (gp.Mass() < areaCap) {
                    for (int j = 0; j < wp.faceCount(); ++j) {
                        if (wp.face(j).IsSame(af)) {
                            bottomFaceIdx = j;
                            break;
                        }
                    }
                }
            }
        }
        if (bottomFaceIdx < 0) continue;
        bottomGroups[bottomFaceIdx].push_back({
            fIdx, rad, cyl.Axis().Location(), zHi, zLo
        });
    }

    for (const auto& [botIdx, entries] : bottomGroups) {
        if (entries.size() != 2) continue;
        if (std::abs(entries[0].radius - entries[1].radius) > 1e-3) continue;
        const double r = entries[0].radius;
        const double depth = std::abs(entries[0].zHi - entries[0].zLo);
        if (depth <= 1e-6) continue;
        if (depth > 0.5) continue;            // too deep for FSW
        if (depth / (2.0 * r) > 0.05) continue;  // FSW signature: shallow trace

        json recovered = {
            { "start_x_mm",              entries[0].axisBase.X() },
            { "start_y_mm",              entries[0].axisBase.Y() },
            { "end_x_mm",                entries[1].axisBase.X() },
            { "end_y_mm",                entries[1].axisBase.Y() },
            { "tool_shoulder_dia_mm",    2.0 * r },
            { "depth_mm",                depth },
            { "tool_rotation_rpm",       1200.0 },
            { "traverse_speed_mm_per_s", 5.0 },
            { "material",                "aluminum" },
            { "entry_face_normal", { 0.0, 0.0, 1.0 } },
        };
        json matched = {
            { "bottom_face_id",    botIdx },
            { "end_cylinder_ids",  { entries[0].cylIdx, entries[1].cylIdx } },
            { "depth_to_dia",      depth / (2.0 * r) },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.55, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::friction_stir
