// @lat: [[engine/skills#undercut_milling]]

#include "undercut_milling.hpp"

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
#include <vector>

namespace koocadcam::skill::undercut_milling {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.pocket_width_mm < 0.8) {
        r.add("DFM-002", "error",
              "undercut_milling pocket_width " + std::to_string(in.pocket_width_mm) +
              " mm < min 0.8 mm");
    }
    if (in.pocket_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "pocket_depth_mm must be > 0");
    }
    if (in.pocket_height_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "pocket_height_mm must be > 0");
    }

    // Always emit an info-level finding flagging the undercut.
    r.add("DFM-005", "info",
          "undercut detected — requires non-3-axis CAM (T-cutter / "
          "lollipop tool / 5-axis machining centre)");

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

namespace {

// Build a local right-handed frame for a planar SIDE face:
//   z_loc = the face's outward normal (pointing AWAY from the workpiece)
//   y_loc = world +Z, projected onto the face plane
//           (i.e. up-along-the-face direction)
//   x_loc = z_loc × y_loc  (CCW completion, lies in face plane)
//
// If the face IS horizontal (its normal is along ±Z), we fall back to
// y_loc = world +Y to avoid degenerate cross product.
struct LocalFrame
{
    gp_Dir xLoc;   // along face, horizontal-ish
    gp_Dir yLoc;   // along face, vertical-ish
    gp_Dir zLoc;   // outward normal of face (away from material)
};

LocalFrame buildFrameForFace(const gp_Dir& outwardNormal)
{
    LocalFrame f;
    f.zLoc = outwardNormal;
    // Pick world +Z as the "up" direction unless the face is horizontal.
    gp_Dir upGuess(0.0, 0.0, 1.0);
    const double dotZ = f.zLoc.X()*upGuess.X() + f.zLoc.Y()*upGuess.Y() + f.zLoc.Z()*upGuess.Z();
    if (std::abs(std::abs(dotZ) - 1.0) < 1e-3) {
        // Face is horizontal — use world +Y as the up.
        upGuess = gp_Dir(0.0, 1.0, 0.0);
    }
    // Project upGuess onto plane perpendicular to zLoc.
    gp_Vec upVec(upGuess.X(), upGuess.Y(), upGuess.Z());
    gp_Vec zVec(f.zLoc.X(), f.zLoc.Y(), f.zLoc.Z());
    gp_Vec yVec = upVec - zVec * upVec.Dot(zVec);
    yVec.Normalize();
    f.yLoc = gp_Dir(yVec);
    // x = z × y → right-handed CCW frame.  But we want positive area when
    // sweeping y over x toward z; build manually:
    gp_Vec xVec = yVec.Crossed(zVec);  // y × z = x in right-handed system
    xVec.Normalize();
    f.xLoc = gp_Dir(xVec);
    return f;
}

}  // namespace

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    // We collect findings (including DFM-005 info) but only fail on error.
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "undercut_milling DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("undercut_milling: entry_face datum unresolved");

    // Compute the face center and inward normal.
    // Note: workpiece.faceNormal() returns the OUTWARD normal of the face.
    gp_Pnt faceCenter;
    gp_Dir outwardN;
    try {
        faceCenter = wp.faceCenter(*entryId);
        outwardN   = wp.faceNormal(*entryId);
    } catch (...) {
        throw SkillError("undercut_milling: entry_face geometry unreadable");
    }

    // Local frame on the face.
    const LocalFrame frame = buildFrameForFace(outwardN);

    // Pocket center position on the face surface:
    //   pocketCenter = faceCenter + position_y_mm * yLoc + position_z_mm * xLoc
    //   (We use position_y for the vertical-along-face direction, and
    //    position_z for the horizontal-along-face direction so the input
    //    field naming is the user's preference.)
    //
    // The naming in Input is somewhat conventional: position_y_mm and
    // position_z_mm are "along the face plane" in the user's mental model
    // of the entry face's local YZ.  Map them through our frame's yLoc/xLoc.
    const gp_Pnt onSurface(
        faceCenter.X() + in.position_y_mm * frame.yLoc.X() + in.position_z_mm * frame.xLoc.X(),
        faceCenter.Y() + in.position_y_mm * frame.yLoc.Y() + in.position_z_mm * frame.xLoc.Y(),
        faceCenter.Z() + in.position_y_mm * frame.yLoc.Z() + in.position_z_mm * frame.xLoc.Z());

    // Build a box that enters through the face.
    //   - DZ extends along the INWARD normal (= -frame.zLoc) by pocket_depth_mm + overhang
    //   - DX extends along frame.xLoc by pocket_width_mm
    //   - DY extends along frame.yLoc by pocket_height_mm
    //
    // gp_Ax2(P, V=mainDir, Vx=xDir) maps DZ → mainDir, DX → xDir, DY = mainDir × xDir.
    // We pick mainDir = inwardN, xDir = frame.xLoc, so:
    //   DZ direction (inwardN) is the depth axis,
    //   DX direction (xLoc)    is the width axis,
    //   DY direction (yLoc)    is the height axis.
    //
    // To get yLoc as DY direction we need mainDir × xLoc = yLoc → inwardN × xLoc = yLoc.
    // Since frame is right-handed with x = y × z, we have z × x = y (in right-handed
    // basis). So inwardN = -zLoc, and (-zLoc) × xLoc = -(zLoc × xLoc) = -yLoc.
    // That would invert the DY direction.  To compensate, we shift the box origin to
    // negate the offset along yLoc (i.e. start the box at -pocket_height_mm/2 along yLoc).
    const gp_Dir inwardN(-frame.zLoc.X(), -frame.zLoc.Y(), -frame.zLoc.Z());

    // Origin: centered on (xLoc, yLoc) plane, at face surface (or slightly
    // outside, along -inwardN, for clean Boolean entry).
    const double kOverhang = 0.05;
    const double depth = in.pocket_depth_mm + kOverhang;

    // Calculate (mainDir × xDir) actual direction to know how DY moves.
    gp_Vec mainV(inwardN.X(), inwardN.Y(), inwardN.Z());
    gp_Vec xV(frame.xLoc.X(), frame.xLoc.Y(), frame.xLoc.Z());
    gp_Vec dyDirActual = mainV.Crossed(xV);
    if (dyDirActual.Magnitude() < 1e-9)
        throw SkillError("undercut_milling: degenerate local frame");
    dyDirActual.Normalize();
    const gp_Dir dyDir(dyDirActual);

    // Origin = surfacePoint shifted by:
    //   -pocket_width_mm/2 along xLoc (so DX centres on width)
    //   -pocket_height_mm/2 along dyDir (so DY centres on height)
    //   +kOverhang along -inwardN (so DZ entry starts just outside the body)
    const gp_Pnt boxOrigin(
        onSurface.X() - in.pocket_width_mm / 2.0 * frame.xLoc.X()
                       - in.pocket_height_mm / 2.0 * dyDir.X()
                       - kOverhang * inwardN.X(),
        onSurface.Y() - in.pocket_width_mm / 2.0 * frame.xLoc.Y()
                       - in.pocket_height_mm / 2.0 * dyDir.Y()
                       - kOverhang * inwardN.Y(),
        onSurface.Z() - in.pocket_width_mm / 2.0 * frame.xLoc.Z()
                       - in.pocket_height_mm / 2.0 * dyDir.Z()
                       - kOverhang * inwardN.Z());

    const gp_Ax2 boxAx(boxOrigin, inwardN, frame.xLoc);
    const TopoDS_Shape cutter = pr::box(
        boxAx, in.pocket_width_mm, in.pocket_height_mm, depth);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json params = {
        { "entry_face_id",      *entryId },
        { "position_y_mm",      in.position_y_mm },
        { "position_z_mm",      in.position_z_mm },
        { "pocket_width_mm",    in.pocket_width_mm },
        { "pocket_depth_mm",    in.pocket_depth_mm },
        { "pocket_height_mm",   in.pocket_height_mm },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "planar_wall_count",    4 },
        { "bottom_planar_face",   true },
        { "access_axis",          { inwardN.X(), inwardN.Y(), inwardN.Z() } },
        { "is_top_accessible",    false },
        { "pocket_width_mm",      in.pocket_width_mm },
        { "pocket_depth_mm",      in.pocket_depth_mm },
        { "pocket_height_mm",     in.pocket_height_mm },
    };
    // Pass DFM findings into tooling.extra for downstream CAPP.
    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "t_cutter";    // or "lollipop" or 5-axis end-mill
    tooling.tool_dia_mm       = in.pocket_width_mm;
    tooling.tool_length_mm    = in.pocket_depth_mm + 10.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 200.0;          // undercuts run slower
    tooling.feed_per_tooth_mm = 0.020;
    tooling.stock_removed_mm3 =
        in.pocket_width_mm * in.pocket_depth_mm * in.pocket_height_mm;
    tooling.est_cycle_time_s  = std::max(5.0,
        tooling.stock_removed_mm3 / 2000.0);     // slower than normal milling
    tooling.extra["dfm_findings"] = findings;
    tooling.extra["machining_constraint"] =
        "Requires non-3-axis CAM (T-cutter / lollipop / 5-axis)";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::undercut_milling applied: w={} d={} h={} entry_face={} faces {}→{}",
                  in.pocket_width_mm, in.pocket_depth_mm, in.pocket_height_mm,
                  *entryId, wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Heuristic: find pocket-like features whose deepest face has a normal that
// does NOT point in the +Z direction.  A standard top-face pocket has a
// bottom whose normal points UP (+Z); an undercut has a bottom whose
// outward normal is NOT vertical.
//
// More concretely: look for a planar face whose outward normal is roughly
// HORIZONTAL (|nz| < 0.3) and that is surrounded by other planar faces
// forming a closed pocket on the inside of the workpiece.

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

// Geometric planar-face matcher — STEP round-trip safe.
static int findPlanarFaceIndex(const Workpiece& wp, const TopoDS_Face& af)
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

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const auto edgeFaces = buildEdgeFaceMap(wp.shape());

    auto faceIndex = [&](const TopoDS_Face& f) -> int {
        return findPlanarFaceIndex(wp, f);
    };

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    // For each planar face that could be the BACK of an undercut pocket
    // (i.e. a face whose outward normal is roughly horizontal and that is
    // INSIDE the workpiece — not the workpiece's own outer boundary).
    for (int bIdx = 0; bIdx < wp.faceCount(); ++bIdx) {
        if (!wp.isFacePlanar(bIdx)) continue;
        gp_Dir nb;
        try { nb = wp.faceNormal(bIdx); } catch (...) { continue; }
        // Horizontal normal: |nz| small.  This is the back wall of the
        // undercut pocket.
        if (std::abs(nb.Z()) > 0.3) continue;

        const gp_Pnt bc = wp.faceCenter(bIdx);
        // Reject if the face is ON the outer boundary of the workpiece.
        // Heuristic: face center is at one of the X-extreme planes of the
        // bbox → it IS the outer face, not the back of a pocket.
        const double tol = 1e-2;
        const bool onXBoundary =
            std::abs(bc.X() - xMin) < tol || std::abs(bc.X() - xMax) < tol;
        const bool onYBoundary =
            std::abs(bc.Y() - yMin) < tol || std::abs(bc.Y() - yMax) < tol;
        if (onXBoundary || onYBoundary) continue;

        // Collect adjacent planar walls.
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
                if (std::find(walls.begin(), walls.end(), ai) == walls.end())
                    walls.push_back(ai);
            }
        }
        if (walls.size() < 3) continue;  // need pocket-like enclosure

        // Recover approximate dimensions from the back face area.
        const double area = wp.faceArea(bIdx);
        const double aspect = 1.0;  // unknown; assume roughly square
        const double approxW = std::sqrt(area * aspect);
        const double approxH = area / std::max(1e-6, approxW);

        // pocket_depth: unknown precisely (no good way without ray casting);
        // estimate as half of bbox extent along the access direction.
        const gp_Dir inwardN(-nb.X(), -nb.Y(), -nb.Z());
        const double extentX = xMax - xMin;
        const double extentY = yMax - yMin;
        const double approxD = std::min(extentX, extentY) / 3.0;  // very rough

        json recovered = {
            { "position_y_mm",      0.0 },
            { "position_z_mm",      0.0 },
            { "pocket_width_mm",    approxW },
            { "pocket_depth_mm",    approxD },
            { "pocket_height_mm",   approxH },
        };
        json matched = {
            { "back_face_id",   bIdx },
            { "wall_face_ids",  json(walls) },
            { "access_axis",    { inwardN.X(), inwardN.Y(), inwardN.Z() } },
        };
        // Heuristic confidence: low because dimensions are very approximate.
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.55, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::undercut_milling
