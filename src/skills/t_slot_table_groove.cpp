// @lat: [[engine/skills#t_slot_table_groove]]

#include "t_slot_table_groove.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRep_Tool.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <Standard_Failure.hxx>
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
#include <limits>

namespace koocadcam::skill::t_slot_table_groove {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.top_w_mm < 0.8) {
        r.add("DFM-002", "error",
              "t_slot_table_groove top_w " + std::to_string(in.top_w_mm) +
              " mm < min 0.8 mm");
    }
    if (in.bot_w_mm < 0.8) {
        r.add("DFM-002", "error",
              "t_slot_table_groove bot_w " + std::to_string(in.bot_w_mm) +
              " mm < min 0.8 mm");
    }
    if (in.top_d_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "t_slot_table_groove top_d_mm must be > 0");
    }
    if (in.bot_d_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "t_slot_table_groove bot_d_mm must be > 0");
    }
    if (in.bot_w_mm <= in.top_w_mm) {
        r.add("DFM-TSLOT", "error",
              "t_slot_table_groove bot_w (" + std::to_string(in.bot_w_mm) +
              ") must be > top_w (" + std::to_string(in.top_w_mm) +
              ") — else this is a plain keyway, not a T-slot");
    }

    // DIN 650 table T-slot typical ratio bot_w / top_w ∈ [1.5, 3.5].
    if (in.top_w_mm > 0.0 && in.bot_w_mm > 0.0) {
        const double ratio = in.bot_w_mm / in.top_w_mm;
        if (ratio < 1.5 || ratio > 3.5) {
            r.add("DFM-DIN650", "warning",
                  "t_slot_table_groove bot_w/top_w ratio " +
                  std::to_string(ratio) +
                  " outside DIN 650 typical [1.5, 3.5]");
        }
    }

    const double slotLen = std::hypot(in.end_x_mm - in.start_x_mm,
                                      in.end_y_mm - in.start_y_mm);
    if (slotLen <= 1e-6) {
        r.add("DFM-INPUT", "error",
              "t_slot_table_groove start and end must differ (slot length > 0)");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "t_slot_table_groove DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("t_slot_table_groove: entry_face datum unresolved");

    if (std::abs(in.axis_dir.X()) > 1e-6 || std::abs(in.axis_dir.Y()) > 1e-6
        || in.axis_dir.Z() >= 0) {
        throw SkillError("t_slot_table_groove: only axis_dir = -Z supported");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;

    // Local frame along slot path.
    const gp_Vec dir2D(in.end_x_mm - in.start_x_mm,
                       in.end_y_mm - in.start_y_mm, 0.0);
    const double slotLen = dir2D.Magnitude();
    const gp_Dir xLoc(dir2D.X() / slotLen, dir2D.Y() / slotLen, 0.0);
    const gp_Dir yLoc(-xLoc.Y(), xLoc.X(), 0.0);

    const double zEntry  = zMax;
    const double zTopBot = zEntry - in.top_d_mm;            // floor of top slot
    const double zBotBot = zTopBot - in.bot_d_mm;           // floor of bottom slot

    // ── Sub-feature 1: top narrow slot ─────────────────────────────────
    //
    // Box of width top_w (along yLoc), depth top_d + kOverhang (along +Z),
    // length slotLen (along xLoc).  Origin = at start, offset by
    // -top_w/2 along yLoc, at z=zTopBot.  V = +Z so DZ extrudes up to
    // zTopBot + top_d + overhang = zEntry + overhang.
    const gp_Pnt topOrigin(
        in.start_x_mm - in.top_w_mm / 2.0 * yLoc.X(),
        in.start_y_mm - in.top_w_mm / 2.0 * yLoc.Y(),
        zTopBot);
    const gp_Ax2 topAx(topOrigin, gp_Dir(0.0, 0.0, 1.0), xLoc);
    const TopoDS_Shape topBox = pr::box(
        topAx, slotLen, in.top_w_mm, in.top_d_mm + kOverhang);

    // ── Sub-feature 2: bottom wider slot ───────────────────────────────
    //
    // Same idea but width = bot_w, depth = bot_d, at z=zBotBot up to zTopBot.
    const gp_Pnt botOrigin(
        in.start_x_mm - in.bot_w_mm / 2.0 * yLoc.X(),
        in.start_y_mm - in.bot_w_mm / 2.0 * yLoc.Y(),
        zBotBot);
    const gp_Ax2 botAx(botOrigin, gp_Dir(0.0, 0.0, 1.0), xLoc);
    const TopoDS_Shape botBox = pr::box(
        botAx, slotLen, in.bot_w_mm, in.bot_d_mm);

    // Fuse the two boxes into a single T-prism, then single cut (same as
    // T_slot.cpp's pattern).
    TopoDS_Shape fused;
    try {
        fused = pr::fuse(topBox, botBox);
    } catch (const Standard_Failure& e) {
        throw SkillError(std::string("t_slot_table_groove: fuse failed: ") + e.what());
    }

    TopoDS_Shape newShape;
    try {
        newShape = pr::cut(wp.shape(), fused);
    } catch (const Standard_Failure& e) {
        throw SkillError(std::string("t_slot_table_groove: cut failed: ") + e.what());
    }

    json params = {
        { "entry_face_id", *entryId },
        { "start_x_mm",    in.start_x_mm },
        { "start_y_mm",    in.start_y_mm },
        { "end_x_mm",      in.end_x_mm },
        { "end_y_mm",      in.end_y_mm },
        { "axis_dir",      { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "top_w_mm",      in.top_w_mm },
        { "top_d_mm",      in.top_d_mm },
        { "bot_w_mm",      in.bot_w_mm },
        { "bot_d_mm",      in.bot_d_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    2 },
        { "top_w_mm",            in.top_w_mm },
        { "top_d_mm",            in.top_d_mm },
        { "bot_w_mm",            in.bot_w_mm },
        { "bot_d_mm",            in.bot_d_mm },
        { "total_depth_mm",      in.top_d_mm + in.bot_d_mm },
        { "length_mm",           slotLen },
        { "bot_to_top_ratio",    in.bot_w_mm / std::max(1e-6, in.top_w_mm) },
        { "standard",            "DIN 650 / ISO 299 table T-slot" },
    };
    ToolingMeta tooling;
    tooling.tool_type   = "endmill;t_slot_cutter";
    tooling.tool_dia_mm = in.bot_w_mm;
    tooling.tool_material = "carbide";
    tooling.flute_count   = 4;
    tooling.cutting_speed_sfm = 350.0;
    tooling.feed_per_tooth_mm = 0.025;
    tooling.stock_removed_mm3 =
        (in.top_w_mm * in.top_d_mm + in.bot_w_mm * in.bot_d_mm) * slotLen;
    tooling.est_cycle_time_s = std::max(5.0,
        slotLen * 0.2 + (in.top_d_mm + in.bot_d_mm) * 0.5);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "endmill" }, { "tool_dia_mm", in.top_w_mm },
              { "depth_mm", in.top_d_mm }, { "note", "rough top slot" } },
            { { "tool_type", "t_slot_cutter" }, { "tool_dia_mm", in.bot_w_mm },
              { "depth_mm", in.bot_d_mm }, { "note", "plunge T-cutter for bottom channel" } },
        } },
        { "standard", "DIN 650 / ISO 299" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::t_slot_table_groove applied: top={}x{} bot={}x{} len={} faces {}→{}",
                  in.top_w_mm, in.top_d_mm,
                  in.bot_w_mm, in.bot_d_mm, slotLen,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition: metadata replay + geometric fallback ────────────────────
//
// Geometric signature of a table T-slot groove (DIN 650):
//   - bottom planar face with +Z normal (the floor) below zMax;
//   - at least 2 step faces with -Z normal at an intermediate Z (neck shoulders);
//   - bot_w / top_w ratio in DIN 650 typical band [1.5, 3.5].
// This mirrors T_slot.cpp's recognizer but emits the table-specific skill_id
// only when the ratio falls within the DIN 650 family; otherwise we leave
// the match to T_slot.cpp.

namespace {

using EdgeFaceMap = NCollection_IndexedDataMap<
    TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>;

EdgeFaceMap buildEdgeFaceMap(const TopoDS_Shape& shape)
{
    EdgeFaceMap m;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, m);
    return m;
}

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

std::vector<RecognizedFeature> geometric_fallback(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const auto edgeFaces = buildEdgeFaceMap(wp.shape());
    auto faceIndex = [&](const TopoDS_Face& f){ return findPlanarFaceIndex(wp, f); };

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    for (int bIdx = 0; bIdx < wp.faceCount(); ++bIdx) {
        if (!wp.isFacePlanar(bIdx)) continue;
        gp_Dir nb;
        try { nb = wp.faceNormal(bIdx); } catch (...) { continue; }
        if (std::abs(nb.Z() - 1.0) > 1e-2) continue;
        const gp_Pnt bc = wp.faceCenter(bIdx);
        if (std::abs(bc.Z() - zMax) < 1e-3) continue;
        const double zBottom = bc.Z();

        std::vector<int> walls;
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
                if (std::abs(an.Z()) < 1e-2)
                    if (std::find(walls.begin(), walls.end(), ai) == walls.end())
                        walls.push_back(ai);
            }
        }

        std::vector<int> stepFaces;
        for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
            if (fIdx == bIdx) continue;
            if (!wp.isFacePlanar(fIdx)) continue;
            gp_Dir fn;
            try { fn = wp.faceNormal(fIdx); } catch (...) { continue; }
            if (std::abs(fn.Z() + 1.0) > 1e-2) continue;
            const gp_Pnt fc = wp.faceCenter(fIdx);
            if (fc.Z() <= zBottom + 1e-3) continue;
            if (fc.Z() >= zMax     - 1e-3) continue;
            stepFaces.push_back(fIdx);
        }
        if (stepFaces.size() < 2) continue;
        std::sort(stepFaces.begin(), stepFaces.end(),
                  [&](int a, int b){ return wp.faceCenter(a).Z() < wp.faceCenter(b).Z(); });
        const double zStep = wp.faceCenter(stepFaces.front()).Z();
        std::vector<int> stepsAtLevel;
        for (int sid : stepFaces) {
            if (std::abs(wp.faceCenter(sid).Z() - zStep) < 0.5)
                stepsAtLevel.push_back(sid);
        }
        if (stepsAtLevel.size() < 2) continue;

        const double flangeDepth = zStep - zBottom;
        const double neckDepth   = zMax  - zStep;

        double maxW = 0.0, minW = std::numeric_limits<double>::max();
        for (size_t i = 0; i < walls.size(); ++i) {
            for (size_t j = i + 1; j < walls.size(); ++j) {
                gp_Dir ni, nj;
                try { ni = wp.faceNormal(walls[i]); nj = wp.faceNormal(walls[j]); }
                catch (...) { continue; }
                const double dot = std::abs(ni.X()*nj.X() + ni.Y()*nj.Y() + ni.Z()*nj.Z());
                if (dot < 0.95) continue;
                const gp_Pnt pi = wp.faceCenter(walls[i]);
                const gp_Pnt pj = wp.faceCenter(walls[j]);
                const double dist = std::hypot(pi.X() - pj.X(), pi.Y() - pj.Y());
                if (dist > maxW) maxW = dist;
                if (dist < minW) minW = dist;
            }
        }
        if (minW <= 0.0 || maxW <= minW) continue;

        // DIN 650 ratio gate distinguishes table T-slot from generic T_slot.
        const double ratio = maxW / minW;
        if (ratio < 1.5 || ratio > 3.5) continue;

        json recovered = {
            { "start_x_mm", bc.X() - (maxW / 2.0) },
            { "start_y_mm", bc.Y() },
            { "end_x_mm",   bc.X() + (maxW / 2.0) },
            { "end_y_mm",   bc.Y() },
            { "axis_dir",   { 0.0, 0.0, -1.0 } },
            { "top_w_mm",   minW },
            { "top_d_mm",   neckDepth },
            { "bot_w_mm",   maxW },
            { "bot_d_mm",   flangeDepth },
        };
        json matched = {
            { "source",        "geometric_fallback" },
            { "bottom_face_id", bIdx },
            { "step_face_ids",  json(stepsAtLevel) },
            { "wall_face_ids",  json(walls) },
            { "din650_ratio",   ratio },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.55, matched });
    }
    return out;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "feature_history_replay" } };
        out.push_back(r);
    }
    if (out.empty()) {
        out = geometric_fallback(wp);
    }
    return out;
}

}  // namespace koocadcam::skill::t_slot_table_groove
