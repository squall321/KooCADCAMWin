// @lat: [[engine/skills#multi_step_bore]]

#include "multi_step_bore.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRep_Tool.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pln.hxx>

#include <limits>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::multi_step_bore {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.steps.size() < 3) {
        r.add("DFM-INPUT", "error",
              "multi_step_bore requires ≥ 3 steps (got " +
              std::to_string(in.steps.size()) +
              ") — use bore_with_shelf for 2-step bores");
    }

    double prevDia = std::numeric_limits<double>::infinity();
    double totalDepth = 0.0;
    for (size_t i = 0; i < in.steps.size(); ++i) {
        const auto& s = in.steps[i];
        if (s.dia_mm <= 0.0) {
            r.add("DFM-INPUT", "error",
                  "step " + std::to_string(i) + " diameter must be > 0");
        }
        if (s.depth_mm <= 0.0) {
            r.add("DFM-INPUT", "error",
                  "step " + std::to_string(i) + " depth must be > 0");
        }
        if (s.dia_mm > 0.0 && s.dia_mm < 0.8) {
            r.add("DFM-002", "error",
                  "step " + std::to_string(i) + " dia " + std::to_string(s.dia_mm) +
                  " mm < min 0.8 mm");
        }
        if (s.dia_mm > 0.0 && i > 0 && s.dia_mm >= prevDia) {
            r.add("DFM-MSB-ORDER", "error",
                  "step " + std::to_string(i) + " dia " + std::to_string(s.dia_mm) +
                  " mm is not strictly less than previous step (" +
                  std::to_string(prevDia) + " mm) — diameters must decrease");
        }
        prevDia = s.dia_mm;
        if (s.depth_mm > 0.0) totalDepth += s.depth_mm;
    }

    if (totalDepth > 30.0) {
        r.add("DFM-MSB-DEPTH", "warning",
              "total step depth " + std::to_string(totalDepth) +
              " mm > 30 mm — verify workpiece thickness allows this");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "multi_step_bore DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("multi_step_bore: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const double kEntryOverhang = 0.05;
    const double kStepOverlap   = 0.05;     // overlap between adjacent steps
    const gp_Dir adir = in.axis_dir;

    // True entry = where the axis pierces the resolved entry-face plane (any
    // axis; a prior bbox guess put the cutter a whole bboxDiag off for tilted).
    const gp_Pnt entryPt =
        entryPointOnFacePlane(wp, *entryId, in.position_x_mm, in.position_y_mm,
                              adir, zMin, zMax);
    // Launch origin at the entry face (just outside, along reversed axis).
    const gp_Pnt entryStart(entryPt.X() - adir.X() * kEntryOverhang,
                            entryPt.Y() - adir.Y() * kEntryOverhang,
                            entryPt.Z() - adir.Z() * kEntryOverhang);

    // Build N cylinders.  Step 0 starts at the entry; each subsequent step
    // starts (overlap before) at the cumulative depth of the previous steps.
    std::vector<TopoDS_Shape> cylinders;
    cylinders.reserve(in.steps.size());

    double cumulative = 0.0;
    for (size_t i = 0; i < in.steps.size(); ++i) {
        const auto& s = in.steps[i];

        const double advance = (i == 0)
            ? 0.0
            : (cumulative - kStepOverlap);

        // Heights: step 0 must include the kEntryOverhang we baked into the
        // start; subsequent steps add the overlap on top of their depth.
        const double height = (i == 0)
            ? (s.depth_mm + kEntryOverhang)
            : (s.depth_mm + kStepOverlap);

        const gp_Pnt origin(
            entryStart.X() + adir.X() * advance,
            entryStart.Y() + adir.Y() * advance,
            entryStart.Z() + adir.Z() * advance);

        const gp_Ax2 ax(origin, adir);
        cylinders.push_back(pr::cylinder(ax, s.dia_mm / 2.0, height));

        cumulative += s.depth_mm;
    }

    // IMPORTANT: OCCT 8.0 `BRepAlgoAPI_Cut` on a single COMPOUND of N
    // overlapping concentric cylinders misses interior overlap (the
    // compound's surfaces self-intersect inside the wide cylinder).  Fuse
    // all cylinders into ONE solid first, then issue a single cut.  This
    // is the established workaround for the multi-step concentric pattern.
    if (cylinders.empty()) {
        throw SkillError("multi_step_bore: cylinder list empty after build");
    }
    TopoDS_Shape fused = cylinders.front();
    {
        std::vector<TopoDS_Shape> rest(cylinders.begin() + 1, cylinders.end());
        fused = pr::fuseMany(fused, rest);
    }
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    // Build the steps JSON.
    json stepsJson = json::array();
    for (const auto& s : in.steps) {
        stepsJson.push_back({ { "dia_mm", s.dia_mm }, { "depth_mm", s.depth_mm } });
    }

    json params = {
        { "entry_face_kind", "resolved_id" },
        { "entry_face_id",   *entryId },
        { "position_x_mm",   in.position_x_mm },
        { "position_y_mm",   in.position_y_mm },
        { "position_z_mm",   entryPt.Z() },
        { "axis_dir",        { adir.X(), adir.Y(), adir.Z() } },
        { "steps",           stepsJson },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     static_cast<int>(in.steps.size()) },
        { "annular_planar_face_count",  static_cast<int>(in.steps.size()) - 1 },
        { "bottom_planar_face_present", true },
        { "step_count",                 static_cast<int>(in.steps.size()) },
        { "steps",                      stepsJson },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
    };

    // Tooling: a (typically multiple) carbide step-bore tool, or sequential
    // boring bars.  We summarise the smallest step's tool.
    double minDia = in.steps.front().dia_mm;
    double totalDepth = 0.0;
    double totalVol = 0.0;
    for (const auto& s : in.steps) {
        minDia = std::min(minDia, s.dia_mm);
        totalDepth += s.depth_mm;
        totalVol += M_PI * (s.dia_mm * 0.5) * (s.dia_mm * 0.5) * s.depth_mm;
    }

    ToolingMeta tooling;
    tooling.tool_type        = "boring_bar";
    tooling.tool_dia_mm      = minDia;        // limited by the narrowest step
    tooling.tool_length_mm   = totalDepth * 1.4 + 15.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 1;
    tooling.cutting_speed_sfm = 110.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = totalVol;
    tooling.est_cycle_time_s  = std::max(5.0, totalDepth / 20.0)
                              + 4.0 * static_cast<double>(in.steps.size());  // tool changes / passes
    tooling.extra = json{
        { "step_count", static_cast<int>(in.steps.size()) },
        { "steps",      stepsJson },
    };

    FeatureSignature sig {
        kSkillId,
        params,
        pattern,
        tooling,
    };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::multi_step_bore applied: steps={} total_depth={} faces {}→{}",
                  in.steps.size(), totalDepth,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Strategy:
//   1. Collect cylindrical faces, compute their axes & extreme circle Zs.
//   2. Group by shared axis (parallel + collinear).
//   3. Sort group members along the axis from entry side to deepest.
//   4. Verify strictly-decreasing diameters and that adjacent cylinders
//      "touch" via an annular planar shelf.
//   5. If the chain length is ≥ 3, emit a candidate.

namespace {

using EdgeFaceMap = NCollection_IndexedDataMap<
    TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>;

EdgeFaceMap buildEdgeFaceMap(const TopoDS_Shape& shape)
{
    EdgeFaceMap m;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, m);
    return m;
}

struct CylInfo {
    int    faceIdx;
    double radius;
    gp_Ax1 axis;
    gp_Pnt topCenter;       // along the canonical axis direction, the larger projection
    gp_Pnt bottomCenter;
    double topProj;
    double bottomProj;
};

// Geometric matcher: locate `af` in the workpiece's face enumeration by
// matching its plane (parallel normal + coincident point) — NOT by IsSame()
// TShape identity, which breaks after STEP round-trip.  Same approach as
// bore_with_shelf::findPlanarFaceIndex.
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
    const TopoDS_Shape& shape = wp.shape();
    const auto edgeFaces = buildEdgeFaceMap(shape);

    // 1. Collect cylindrical faces.
    std::vector<CylInfo> cyls;
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double radius = cyl.Radius();
        const gp_Ax1 axis = cyl.Axis();

        std::vector<gp_Pnt> centers;
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            if (std::abs(std::abs(c.Axis().Direction().Dot(axis.Direction())) - 1.0) > 1e-3)
                continue;
            if (std::abs(c.Radius() - radius) > 1e-3) continue;
            centers.push_back(c.Location());
        }
        if (centers.size() < 2) continue;

        const gp_Dir adir = axis.Direction();
        auto projOnAxis = [&](const gp_Pnt& p) {
            return (p.X() - axis.Location().X()) * adir.X() +
                   (p.Y() - axis.Location().Y()) * adir.Y() +
                   (p.Z() - axis.Location().Z()) * adir.Z();
        };
        auto cmp = [&](const gp_Pnt& a, const gp_Pnt& b) {
            return projOnAxis(a) < projOnAxis(b);
        };
        const auto minIt = std::min_element(centers.begin(), centers.end(), cmp);
        const auto maxIt = std::max_element(centers.begin(), centers.end(), cmp);

        CylInfo ci;
        ci.faceIdx      = fIdx;
        ci.radius       = radius;
        ci.axis         = axis;
        ci.topCenter    = *maxIt;
        ci.bottomCenter = *minIt;
        ci.topProj      = projOnAxis(*maxIt);
        ci.bottomProj   = projOnAxis(*minIt);
        cyls.push_back(ci);
    }

    if (cyls.size() < 3) return out;   // need at least 3 cylinders for an MSB

    // 2. Group by axis: two cylinders are "co-axial" iff
    //    - parallel directions (|dot|=1), and
    //    - origin-to-origin vector is along the axis (no perpendicular
    //      component).
    std::vector<bool> visited(cyls.size(), false);
    for (size_t i = 0; i < cyls.size(); ++i) {
        if (visited[i]) continue;

        std::vector<int> group;
        group.push_back(static_cast<int>(i));
        visited[i] = true;

        const gp_Ax1 baseAxis = cyls[i].axis;
        const gp_Dir baseDir  = baseAxis.Direction();
        const gp_Vec baseDirV(baseDir);

        for (size_t j = i + 1; j < cyls.size(); ++j) {
            if (visited[j]) continue;
            const gp_Dir d2 = cyls[j].axis.Direction();
            if (std::abs(std::abs(d2.Dot(baseDir)) - 1.0) > 1e-3) continue;
            const gp_Vec ao(baseAxis.Location(), cyls[j].axis.Location());
            const gp_Vec perp = ao - baseDirV * ao.Dot(baseDirV);
            if (perp.Magnitude() > 5e-3) continue;
            group.push_back(static_cast<int>(j));
            visited[j] = true;
        }

        if (group.size() < 3) continue;

        // Each CylInfo's topProj/bottomProj is relative to its OWN axis
        // location, so cross-cylinder comparison requires a shared origin.
        // Use the FIRST member's axis as the canonical reference.
        const gp_Ax1 canonAxis = cyls[group.front()].axis;
        const gp_Dir canonDir  = canonAxis.Direction();
        auto canonProj = [&](const gp_Pnt& p) {
            return (p.X() - canonAxis.Location().X()) * canonDir.X() +
                   (p.Y() - canonAxis.Location().Y()) * canonDir.Y() +
                   (p.Z() - canonAxis.Location().Z()) * canonDir.Z();
        };
        // Compute per-cylinder canonical {top,bot} projections.  "Top" =
        // smaller proj (= entry side along +canonDir = into the material).
        // We pre-store both in a parallel vector indexed by group position.
        struct GroupProj { double topC; double botC; };
        std::vector<GroupProj> gp(group.size());
        for (size_t k = 0; k < group.size(); ++k) {
            const CylInfo& ci = cyls[group[k]];
            const double pa = canonProj(ci.topCenter);
            const double pb = canonProj(ci.bottomCenter);
            // Entry-side = smaller projection (closer to canon origin = the
            // material entry face along canonDir).
            gp[k].topC = std::min(pa, pb);
            gp[k].botC = std::max(pa, pb);
        }

        // 3. Sort the group by canonical TOP projection ascending (entry →
        //    deepest).  After sorting, gp[] no longer aligns with group[];
        //    re-sort both in lockstep.
        std::vector<size_t> ord(group.size());
        for (size_t k = 0; k < ord.size(); ++k) ord[k] = k;
        std::sort(ord.begin(), ord.end(), [&](size_t a, size_t b) {
            return gp[a].topC < gp[b].topC;
        });
        std::vector<int>       sortedGroup; sortedGroup.reserve(group.size());
        std::vector<GroupProj> sortedGp;    sortedGp.reserve(group.size());
        for (size_t idx : ord) {
            sortedGroup.push_back(group[idx]);
            sortedGp.push_back(gp[idx]);
        }
        group = std::move(sortedGroup);
        gp    = std::move(sortedGp);

        // 4. Verify strictly decreasing diameters AND that adjacent cylinders
        //    are connected by an annular planar shelf.
        bool valid = true;
        std::vector<int> annularShelves;   // face indices, one per shelf
        for (size_t k = 0; k + 1 < group.size(); ++k) {
            const CylInfo& a = cyls[group[k]];     // upper (wider, expected)
            const CylInfo& b = cyls[group[k + 1]]; // lower (narrower, expected)

            if (a.radius <= b.radius + 1e-4) { valid = false; break; }

            // Step a's lower extreme should match step b's upper extreme
            // (within meeting tolerance) in canonical projection.
            const double meetingTol = 0.5;
            const bool meet = std::abs(gp[k].botC - gp[k + 1].topC) < meetingTol;
            if (!meet) { valid = false; break; }

            // Find the annular planar face that bridges the two cylinders
            // (it is adjacent to both via shared circular edges).  Use the
            // geometric matcher so this works after STEP round-trip.
            int shelfId = -1;
            const TopoDS_Face& faceA = wp.face(a.faceIdx);
            for (TopExp_Explorer expA(faceA, TopAbs_EDGE); expA.More(); expA.Next()) {
                const TopoDS_Edge& e = TopoDS::Edge(expA.Current());
                BRepAdaptor_Curve crv(e);
                if (crv.GetType() != GeomAbs_Circle) continue;
                if (!edgeFaces.Contains(e)) continue;
                const auto& adj = edgeFaces.FindFromKey(e);
                for (NCollection_List<TopoDS_Shape>::Iterator it(adj); it.More(); it.Next()) {
                    const TopoDS_Face& af = TopoDS::Face(it.Value());
                    if (af.IsSame(faceA)) continue;
                    if (BRepAdaptor_Surface(af).GetType() != GeomAbs_Plane) continue;
                    const int planarIdx = findPlanarFaceIndex(wp, af);
                    if (planarIdx >= 0) { shelfId = planarIdx; break; }
                }
                if (shelfId >= 0) break;
            }
            if (shelfId < 0) { valid = false; break; }
            annularShelves.push_back(shelfId);
        }
        if (!valid) continue;

        // 5. Emit the candidate.  Depth = canonical projection span (gp[k]).
        json stepsJson = json::array();
        for (size_t k = 0; k < group.size(); ++k) {
            const CylInfo& c = cyls[group[k]];
            const double depth = std::abs(gp[k].botC - gp[k].topC);
            stepsJson.push_back({
                { "dia_mm",   2.0 * c.radius },
                { "depth_mm", depth },
            });
        }

        // Entry point = whichever endpoint of the first cylinder has the
        // smaller canonical projection (= entry-face side).  Deepest point =
        // whichever endpoint of the last cylinder has the larger projection.
        const CylInfo& first = cyls[group.front()];
        const CylInfo& last  = cyls[group.back()];
        const double firstTopP = canonProj(first.topCenter);
        const double firstBotP = canonProj(first.bottomCenter);
        const gp_Pnt entryPt  = (firstTopP < firstBotP) ? first.topCenter : first.bottomCenter;
        const double lastTopP = canonProj(last.topCenter);
        const double lastBotP = canonProj(last.bottomCenter);
        const gp_Pnt deepPt   = (lastTopP > lastBotP) ? last.topCenter : last.bottomCenter;

        gp_Vec drillVec(entryPt, deepPt);
        if (drillVec.Magnitude() < 1e-6) continue;
        drillVec.Normalize();

        json recovered = {
            { "position_x_mm", entryPt.X() },
            { "position_y_mm", entryPt.Y() },
            { "axis_dir",      { drillVec.X(), drillVec.Y(), drillVec.Z() } },
            { "steps",         stepsJson },
        };
        json matched = {
            { "step_count",     static_cast<int>(group.size()) },
            { "cyl_face_ids",   json::array() },
            { "shelf_face_ids", json::array() },
            { "entry_top",     { entryPt.X(), entryPt.Y(), entryPt.Z() } },
            { "deepest_bot",   { deepPt.X(),  deepPt.Y(),  deepPt.Z()  } },
        };
        for (int idx : group) matched["cyl_face_ids"].push_back(cyls[idx].faceIdx);
        for (int s   : annularShelves) matched["shelf_face_ids"].push_back(s);

        out.push_back(RecognizedFeature{
            kSkillId, recovered, /*confidence*/ 0.85, matched
        });
    }
    return out;
}

}  // namespace koocadcam::skill::multi_step_bore
