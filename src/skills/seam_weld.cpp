// @lat: [[engine/skills#seam_weld]]

#include "seam_weld.hpp"

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
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

namespace koocadcam::skill::seam_weld {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.bead_width_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "seam_weld bead_width_mm must be > 0");
    }
    if (in.bead_height_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "seam_weld bead_height_mm must be > 0");
    }
    if (in.bead_width_mm > 0.0 && in.bead_width_mm < 3.0) {
        r.add("DFM-WELD-WIDTH", "error",
              "seam_weld bead_width_mm " + std::to_string(in.bead_width_mm) +
              " < 3 mm");
    }
    if (in.bead_width_mm > 12.0) {
        r.add("DFM-WELD-WIDTH", "error",
              "seam_weld bead_width_mm " + std::to_string(in.bead_width_mm) +
              " > 12 mm");
    }
    if (in.bead_width_mm > 0.0 &&
        in.bead_height_mm > in.bead_width_mm * 0.4) {
        r.add("DFM-WELD-RATIO", "error",
              "seam_weld bead_height " + std::to_string(in.bead_height_mm) +
              " > 0.4 × bead_width " + std::to_string(in.bead_width_mm) +
              " — multi-pass build-up, not a single seam pass");
    }
    const double seamLen = std::hypot(in.end_x_mm - in.start_x_mm,
                                      in.end_y_mm - in.start_y_mm);
    if (seamLen <= 1e-6) {
        r.add("DFM-INPUT", "error", "seam_weld start and end must differ");
    }
    if (in.bead_width_mm > 0.0 && seamLen > 0.0 && seamLen <= in.bead_width_mm) {
        r.add("DFM-WELD-LEN", "warning",
              "seam_weld length " + std::to_string(seamLen) +
              " ≤ bead_width " + std::to_string(in.bead_width_mm) +
              " — consider spot_weld instead");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "seam_weld DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("seam_weld: entry_face datum unresolved");
    if (!wp.isFacePlanar(*entryId))
        throw SkillError("seam_weld: entry_face must be planar");

    // Slice 1: support arbitrary planar face but the easy case (face normal
    // along ±Z) is the only one fully exercised by tests.
    const gp_Dir outNorm = wp.faceNormal(*entryId);
    const gp_Pnt faceCtr = wp.faceCenter(*entryId);

    // Bead lives in the entry-face plane.  We support arbitrary face normals
    // but the (x,y) inputs are interpreted as world XY (same convention as
    // mill_slot).  For non-axis-aligned faces the user should pre-project.
    const double radius   = in.bead_width_mm / 2.0;
    const double kSink    = in.bead_height_mm * 0.01;
    const double beadH    = in.bead_height_mm + kSink;

    // For the typical +Z top face, entry plane Z == faceCtr.Z().  For arbitrary
    // faces we still use faceCtr's projection as the base plane elevation
    // along the normal.
    const double baseZ = faceCtr.Z();

    // Build stadium bead = cylinder_start + box + cylinder_end, then fuse.
    // Each cylinder's axis is along outNorm; bottoms sit kSink below the face.
    const gp_Pnt sBase(in.start_x_mm - outNorm.X() * kSink,
                       in.start_y_mm - outNorm.Y() * kSink,
                       baseZ          - outNorm.Z() * kSink);
    const gp_Pnt eBase(in.end_x_mm   - outNorm.X() * kSink,
                       in.end_y_mm   - outNorm.Y() * kSink,
                       baseZ          - outNorm.Z() * kSink);

    const gp_Ax2 axStart(sBase, outNorm);
    const gp_Ax2 axEnd  (eBase, outNorm);
    const TopoDS_Shape cylStart = pr::cylinder(axStart, radius, beadH);
    const TopoDS_Shape cylEnd   = pr::cylinder(axEnd,   radius, beadH);

    // Connecting box along start→end vector.  We follow mill_slot's recipe.
    const gp_Vec dir2D(in.end_x_mm - in.start_x_mm,
                       in.end_y_mm - in.start_y_mm,
                       0.0);
    const double seamLen = dir2D.Magnitude();
    const gp_Dir xLoc(dir2D.X() / seamLen, dir2D.Y() / seamLen, 0.0);
    const gp_Dir yLoc(-xLoc.Y(), xLoc.X(), 0.0);

    // gp_Ax2(P, V=local Z, Vx=local X).  V = outNorm pushes box +Z = outward.
    // Origin = (start - radius * yLoc) in XY, base Z - sink in Z.
    const gp_Pnt boxOrigin(
        in.start_x_mm - radius * yLoc.X() - outNorm.X() * kSink,
        in.start_y_mm - radius * yLoc.Y() - outNorm.Y() * kSink,
        baseZ          - outNorm.Z() * kSink);
    const gp_Ax2 boxAx(boxOrigin, outNorm, xLoc);
    const TopoDS_Shape boxConn = pr::box(boxAx,
                                         seamLen,         // along xLoc
                                         in.bead_width_mm, // along yLoc
                                         beadH);          // along outNorm

    // Fuse pieces into a single stadium bead first (avoids cascading fuses).
    const TopoDS_Shape bead = pr::fuseMany(cylStart, { cylEnd, boxConn });
    const TopoDS_Shape newShape = pr::fuse(wp.shape(), bead);

    json params = {
        { "entry_face_id",   *entryId },
        { "start_x_mm",      in.start_x_mm },
        { "start_y_mm",      in.start_y_mm },
        { "end_x_mm",        in.end_x_mm },
        { "end_y_mm",        in.end_y_mm },
        { "bead_width_mm",   in.bead_width_mm },
        { "bead_height_mm",  in.bead_height_mm },
        { "material",        in.material },
        { "entry_face_normal", { outNorm.X(), outNorm.Y(), outNorm.Z() } },
    };
    json pattern = {
        { "kind",                    kSkillId },
        { "end_cylinder_count",      2 },
        { "long_wall_planar_count",  2 },
        { "top_planar_face",         true },
        { "bead_width_mm",           in.bead_width_mm },
        { "bead_height_mm",          in.bead_height_mm },
        { "seam_length_mm",          seamLen },
        { "additive",                true },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "seam_welder";
    tooling.tool_dia_mm       = in.bead_width_mm;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "copper_alloy";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    // Volume ADDED (record as negative — same convention as spot_weld).
    const double beadVol =
        (M_PI * radius * radius + seamLen * in.bead_width_mm) * in.bead_height_mm;
    tooling.stock_removed_mm3 = -beadVol;
    tooling.est_cycle_time_s  = std::max(1.0, seamLen / 50.0);  // ~50 mm/s
    tooling.extra = {
        { "process",   "resistance_seam_weld" },
        { "material",  in.material },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::seam_weld applied: len={} w={} h={} faces {}→{}",
                  seamLen, in.bead_width_mm, in.bead_height_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

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

    // Metadata replay path.
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

    // Geometric path: mirror of mill_slot recognize but expecting an
    // ADDITIVE stadium ABOVE the parent face.
    const auto edgeFaces = buildEdgeFaceMap(wp.shape());

    struct CylEntry {
        int     cylIdx;
        double  radius;
        gp_Pnt  axisBase;
        double  zHi, zLo;
    };
    // Group cylinders by shared top planar face index.
    std::map<int, std::vector<CylEntry>> topGroups;

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        const TopoDS_Face& cf = wp.face(fIdx);
        BRepAdaptor_Surface surf(cf);
        const gp_Cylinder cyl = surf.Cylinder();
        const gp_Dir adir = cyl.Axis().Direction();
        if (std::abs(std::abs(adir.Z()) - 1.0) > 1e-2) continue;
        const double rad = cyl.Radius();
        if (rad * 2.0 > 12.0) continue;  // outside weld width range
        if (rad * 2.0 < 2.5)  continue;

        int topFaceIdx = -1;
        double zHi = -1e30, zLo = 1e30;
        double topFaceArea = 0.0;
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
                // SMALL planar face — the bead's TOP, distinguishes additive
                // bead from a milled slot (which has a LARGE bottom face).
                const double areaCap = M_PI * rad * rad * 5.0;
                if (gp.Mass() < areaCap) {
                    // Find this face's index in wp.
                    for (int j = 0; j < wp.faceCount(); ++j) {
                        if (wp.face(j).IsSame(af)) {
                            topFaceIdx  = j;
                            topFaceArea = gp.Mass();
                            break;
                        }
                    }
                }
            }
        }
        if (topFaceIdx < 0) continue;
        (void)topFaceArea;
        topGroups[topFaceIdx].push_back({
            fIdx, rad, cyl.Axis().Location(), zHi, zLo
        });
    }

    for (const auto& [topIdx, entries] : topGroups) {
        if (entries.size() != 2) continue;
        if (std::abs(entries[0].radius - entries[1].radius) > 1e-3) continue;
        const double r = entries[0].radius;
        const double height = std::abs(entries[0].zHi - entries[0].zLo);
        if (height <= 1e-6) continue;
        if (height / (2.0 * r) > 0.5) continue;

        json recovered = {
            { "start_x_mm",      entries[0].axisBase.X() },
            { "start_y_mm",      entries[0].axisBase.Y() },
            { "end_x_mm",        entries[1].axisBase.X() },
            { "end_y_mm",        entries[1].axisBase.Y() },
            { "bead_width_mm",   2.0 * r },
            { "bead_height_mm",  height },
            { "material",        "carbon_steel" },
            { "entry_face_normal", { 0.0, 0.0, 1.0 } },
        };
        json matched = {
            { "top_face_id",       topIdx },
            { "end_cylinder_ids",  { entries[0].cylIdx, entries[1].cylIdx } },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.70, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::seam_weld
