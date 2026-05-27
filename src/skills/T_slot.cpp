// @lat: [[engine/skills#T_slot]]

#include "T_slot.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

namespace koocadcam::skill::T_slot {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.neck_width_mm < 0.8) {
        r.add("DFM-002", "error",
              "T_slot neck_width " + std::to_string(in.neck_width_mm) +
              " mm < min 0.8 mm");
    }
    if (in.flange_width_mm < 0.8) {
        r.add("DFM-002", "error",
              "T_slot flange_width " + std::to_string(in.flange_width_mm) +
              " mm < min 0.8 mm");
    }
    if (in.neck_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "neck_depth_mm must be > 0");
    }
    if (in.flange_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "flange_depth_mm must be > 0");
    }
    if (in.flange_width_mm <= in.neck_width_mm) {
        r.add("DFM-TSLOT", "error",
              "T_slot flange_width (" + std::to_string(in.flange_width_mm) +
              ") must be > neck_width (" + std::to_string(in.neck_width_mm) +
              ") — otherwise this is a plain keyway");
    }
    const double slotLen = std::hypot(in.end_x_mm - in.start_x_mm,
                                      in.end_y_mm - in.start_y_mm);
    if (slotLen <= 1e-6) {
        r.add("DFM-INPUT", "error", "start and end must differ (slot length > 0)");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "T_slot DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("T_slot: entry_face datum unresolved");

    if (std::abs(in.axis_dir.X()) > 1e-6 || std::abs(in.axis_dir.Y()) > 1e-6
        || in.axis_dir.Z() >= 0) {
        throw SkillError("T_slot: only axis_dir = -Z is supported in slice 1");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;

    // Slot length and local axes.
    const gp_Vec dir2D(in.end_x_mm - in.start_x_mm, in.end_y_mm - in.start_y_mm, 0.0);
    const double slotLen = dir2D.Magnitude();
    const gp_Dir xLoc(dir2D.X() / slotLen, dir2D.Y() / slotLen, 0.0);
    const gp_Dir yLoc(-xLoc.Y(), xLoc.X(), 0.0);

    const double zEntry  = zMax;
    const double zNeckBot = zEntry - in.neck_depth_mm;             // top of flange
    const double zFlangeBot = zNeckBot - in.flange_depth_mm;       // floor of T-slot

    // Box 1 — NECK: width neck_width_mm × length slotLen × height (neck_depth + overhang).
    // The +overhang on top makes the Boolean cut clean.
    const gp_Pnt neckOrigin(
        in.start_x_mm - in.neck_width_mm / 2.0 * yLoc.X(),
        in.start_y_mm - in.neck_width_mm / 2.0 * yLoc.Y(),
        zNeckBot);
    const gp_Ax2 neckAx(neckOrigin, gp_Dir(0.0, 0.0, 1.0), xLoc);
    const TopoDS_Shape neckBox = pr::box(
        neckAx, slotLen, in.neck_width_mm, in.neck_depth_mm + kOverhang);

    // Box 2 — FLANGE: width flange_width_mm × length slotLen × height flange_depth_mm.
    // The flange sits BELOW the neck (zFlangeBot ↑ zNeckBot).
    const gp_Pnt flangeOrigin(
        in.start_x_mm - in.flange_width_mm / 2.0 * yLoc.X(),
        in.start_y_mm - in.flange_width_mm / 2.0 * yLoc.Y(),
        zFlangeBot);
    const gp_Ax2 flangeAx(flangeOrigin, gp_Dir(0.0, 0.0, 1.0), xLoc);
    const TopoDS_Shape flangeBox = pr::box(
        flangeAx, slotLen, in.flange_width_mm, in.flange_depth_mm);

    // Fuse the two boxes into a single T-prism, then cut.
    const TopoDS_Shape cutter = pr::fuse(neckBox, flangeBox);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json params = {
        { "entry_face_id",      *entryId },
        { "start_x_mm",         in.start_x_mm },
        { "start_y_mm",         in.start_y_mm },
        { "end_x_mm",           in.end_x_mm },
        { "end_y_mm",           in.end_y_mm },
        { "axis_dir",           { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "neck_width_mm",      in.neck_width_mm },
        { "flange_width_mm",    in.flange_width_mm },
        { "neck_depth_mm",      in.neck_depth_mm },
        { "flange_depth_mm",    in.flange_depth_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "neck_wall_count",            2 },
        { "flange_wall_count",          2 },
        { "bottom_planar_face",         true },
        { "step_face_count",            2 },
        { "neck_width_mm",              in.neck_width_mm },
        { "flange_width_mm",            in.flange_width_mm },
        { "neck_depth_mm",              in.neck_depth_mm },
        { "flange_depth_mm",            in.flange_depth_mm },
        { "length_mm",                  slotLen },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "t_slot_cutter";  // dedicated T-slot cutter
    tooling.tool_dia_mm       = in.flange_width_mm;
    tooling.tool_length_mm    = in.neck_depth_mm + in.flange_depth_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 350.0;
    tooling.feed_per_tooth_mm = 0.025;
    tooling.stock_removed_mm3 =
        (in.neck_width_mm * in.neck_depth_mm
         + in.flange_width_mm * in.flange_depth_mm) * slotLen;
    tooling.est_cycle_time_s  = std::max(3.0,
        slotLen * 0.2
        + (in.neck_depth_mm + in.flange_depth_mm) * 0.5);
    tooling.extra["operation_note"] =
        "Pre-mill the neck with end-mill, then plunge T-slot cutter for flange";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::T_slot applied: neck={}x{} flange={}x{} len={} faces {}→{}",
                  in.neck_width_mm, in.neck_depth_mm,
                  in.flange_width_mm, in.flange_depth_mm, slotLen,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Pattern: two stepped slot levels.  We detect by clustering planar
// vertical walls by their face-center Z mid-range, looking for at least 2
// distinct Z-bands (the neck and the flange).

namespace {

using EdgeFaceMap = NCollection_IndexedDataMap<
    TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>;

EdgeFaceMap buildEdgeFaceMap(const TopoDS_Shape& shape)
{
    EdgeFaceMap m;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, m);
    return m;
}

// Geometric planar-face matcher — STEP round-trip safe.
int findPlanarFaceIndex(const Workpiece& wp, const TopoDS_Face& af)
{
    BRepAdaptor_Surface as(af);
    if (as.GetType() != GeomAbs_Plane) return -1;
    const gp_Pln plnA = as.Plane();
    const gp_Dir nA   = plnA.Axis().Direction();
    const gp_Pnt pA   = plnA.Location();

    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        BRepAdaptor_Surface bs(wp.face(i));
        const gp_Pln plnB = bs.Plane();
        const gp_Dir nB   = plnB.Axis().Direction();
        if (std::abs(std::abs(nA.Dot(nB)) - 1.0) > 1e-4) continue;
        const gp_Pnt pB = plnB.Location();
        const double dx = pA.X() - pB.X();
        const double dy = pA.Y() - pB.Y();
        const double dz = pA.Z() - pB.Z();
        const double dist = std::abs(dx * nB.X() + dy * nB.Y() + dz * nB.Z());
        if (dist > 1e-3) continue;
        return i;
    }
    return -1;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const auto edgeFaces = buildEdgeFaceMap(wp.shape());

    auto faceIndex = [&](const TopoDS_Face& f) -> int {
        return findPlanarFaceIndex(wp, f);
    };

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    // For each horizontal-normal bottom face (potential T-slot floor),
    // gather adjacent vertical walls and look for the T-step pattern.
    for (int bIdx = 0; bIdx < wp.faceCount(); ++bIdx) {
        if (!wp.isFacePlanar(bIdx)) continue;
        gp_Dir nb;
        try { nb = wp.faceNormal(bIdx); } catch (...) { continue; }
        if (std::abs(nb.Z() - 1.0) > 1e-2) continue;
        const gp_Pnt bc = wp.faceCenter(bIdx);
        if (std::abs(bc.Z() - zMax) < 1e-3) continue;  // workpiece top
        const double zBottom = bc.Z();

        // Collect adjacent vertical walls (4 expected: 2 flange-level wide,
        // 2 neck-level narrow).
        std::vector<int> walls;
        std::vector<int> stepFaces;  // horizontal step faces between neck/flange
        for (TopExp_Explorer exp(wp.face(bIdx), TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            if (!edgeFaces.Contains(e)) continue;
            const auto& adj = edgeFaces.FindFromKey(e);
            for (NCollection_List<TopoDS_Shape>::Iterator it(adj); it.More(); it.Next()) {
                const TopoDS_Face& af = TopoDS::Face(it.Value());
                if (af.IsSame(wp.face(bIdx))) continue;
                const int ai = faceIndex(af);
                if (ai < 0 || !wp.isFacePlanar(ai)) continue;
                gp_Dir an;
                try { an = wp.faceNormal(ai); } catch (...) { continue; }
                if (std::abs(an.Z()) < 1e-2) {
                    // Vertical wall (flange wall).
                    if (std::find(walls.begin(), walls.end(), ai) == walls.end())
                        walls.push_back(ai);
                }
            }
        }

        // For T-slot we need to detect step faces that connect flange-level
        // walls to neck-level walls.  Scan all planar faces with a downward
        // (or upward) horizontal-band signature at an intermediate Z.
        for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
            if (fIdx == bIdx) continue;
            if (!wp.isFacePlanar(fIdx)) continue;
            gp_Dir fn;
            try { fn = wp.faceNormal(fIdx); } catch (...) { continue; }
            // Step face normal points DOWN (-Z) — into the T-slot from above.
            if (std::abs(fn.Z() + 1.0) > 1e-2) continue;
            const gp_Pnt fc = wp.faceCenter(fIdx);
            // Must be above the bottom and below the workpiece top.
            if (fc.Z() <= zBottom + 1e-3) continue;
            if (fc.Z() >= zMax     - 1e-3) continue;
            stepFaces.push_back(fIdx);
        }

        // Need at least 2 step faces (one each side of the neck) for a true T-slot.
        if (stepFaces.size() < 2) continue;

        // Cluster step faces by Z (within tolerance) to find a single neck level.
        std::sort(stepFaces.begin(), stepFaces.end(), [&](int a, int b){
            return wp.faceCenter(a).Z() < wp.faceCenter(b).Z();
        });
        const double zStep = wp.faceCenter(stepFaces.front()).Z();
        std::vector<int> stepsAtLevel;
        for (int sid : stepFaces) {
            if (std::abs(wp.faceCenter(sid).Z() - zStep) < 0.5)
                stepsAtLevel.push_back(sid);
        }
        if (stepsAtLevel.size() < 2) continue;

        // Recover dimensions.
        const double flangeDepth = zStep - zBottom;
        const double neckDepth   = zMax - zStep;

        // Estimate neck_width and flange_width from face areas approximately.
        // Width = distance between paired walls.  Use the bounding box of the
        // wall set instead.
        double maxW = 0.0, minW = 1e30;
        gp_Pnt startCap, endCap;
        bool capsFound = false;
        for (size_t i = 0; i < walls.size(); ++i) {
            for (size_t j = i + 1; j < walls.size(); ++j) {
                gp_Dir ni, nj;
                try { ni = wp.faceNormal(walls[i]); nj = wp.faceNormal(walls[j]); }
                catch (...) { continue; }
                const double dot = std::abs(ni.X()*nj.X() + ni.Y()*nj.Y() + ni.Z()*nj.Z());
                if (dot < 0.95) continue;  // not parallel
                const gp_Pnt pi = wp.faceCenter(walls[i]);
                const gp_Pnt pj = wp.faceCenter(walls[j]);
                const double dist = std::hypot(pi.X() - pj.X(), pi.Y() - pj.Y());
                if (dist > maxW) { maxW = dist; }
                if (dist < minW) {
                    minW = dist;
                    // Capture the perpendicular axis from the narrow pair's midpoint
                    // as the slot direction.
                    startCap = pi;
                    endCap   = pj;
                    capsFound = true;
                }
            }
        }
        if (!capsFound) continue;

        // The slot midpoint and direction can be derived from the bottom face
        // bounding box.  As a simplification, use the bottom face's bbox for
        // start/end approximations.
        // The T-slot recognizer here is intentionally heuristic.
        json recovered = {
            { "start_x_mm",         bc.X() - (maxW / 2.0) },
            { "start_y_mm",         bc.Y() },
            { "end_x_mm",           bc.X() + (maxW / 2.0) },
            { "end_y_mm",           bc.Y() },
            { "axis_dir",           { 0.0, 0.0, -1.0 } },
            { "neck_width_mm",      minW },
            { "flange_width_mm",    maxW },
            { "neck_depth_mm",      neckDepth },
            { "flange_depth_mm",    flangeDepth },
        };
        json matched = {
            { "bottom_face_id",     bIdx },
            { "step_face_ids",      json(stepsAtLevel) },
            { "wall_face_ids",      json(walls) },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.65, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::T_slot
