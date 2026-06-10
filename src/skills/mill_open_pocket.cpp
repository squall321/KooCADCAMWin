// @lat: [[engine/skills#mill_open_pocket]]

#include "mill_open_pocket.hpp"

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
#include <gp_Cylinder.hxx>
#include <gp_Pln.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <map>

namespace koocadcam::skill::mill_open_pocket {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.length_mm <= 0.0 || in.width_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "length_mm and width_mm must be > 0");
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "depth_mm must be > 0");
    }
    if (in.corner_r_mm < 0.0) {
        r.add("DFM-INPUT", "error", "corner_r_mm must be >= 0");
    }
    if (in.corner_r_mm > 0.0 && in.corner_r_mm < 0.2) {
        r.add("DFM-004", "error",
              "corner_r " + std::to_string(in.corner_r_mm) +
              " mm < min R 0.2 mm");
    }
    // Width must accommodate both back-corner fillets.
    if (in.corner_r_mm > 0.0 && 2.0 * in.corner_r_mm >= in.width_mm - 1e-6) {
        r.add("DFM-INPUT", "error",
              "2 × corner_r must be < width_mm (back-corner fillets collapse)");
    }
    if (in.corner_r_mm > 0.0 && in.corner_r_mm >= in.length_mm - 1e-6) {
        r.add("DFM-INPUT", "error",
              "corner_r must be < length_mm (back-corner fillet collapses)");
    }
    if (in.depth_mm > 5.0) {
        r.add("DFM-006", "warning",
              "depth_mm " + std::to_string(in.depth_mm) +
              " > 5 — verify against workpiece thickness");
    }
    // The open_direction must lie in the entry face's plane.  For the slice-1
    // restriction (entry face along ±Z), open_direction must be ⊥ Z.
    if (std::abs(in.open_direction.Z()) > 1e-3) {
        r.add("DFM-INPUT", "error",
              "open_direction must lie in the entry face plane (z=0 component)");
    }
    if (std::abs(in.axis_dir.X()) > 1e-6 ||
        std::abs(in.axis_dir.Y()) > 1e-6 ||
        in.axis_dir.Z() >= 0.0) {
        r.add("DFM-INPUT", "error",
              "slice-1: only axis_dir = -Z is supported");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "mill_open_pocket DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("mill_open_pocket: entry_face datum unresolved");

    // Slice-1 implementation strategy:
    //
    //   We use `roundedRectPocketTool` to make a cutter whose footprint is
    //   length_extended × width.  The footprint is OFFSET past the workpiece
    //   boundary in `open_direction` by an overhang so the Boolean cut leaves
    //   no closing wall on that side.  The back-corner fillets at radius
    //   corner_r_mm survive (they are away from the open edge).  The
    //   FRONT-corner fillets (those at the open edge) would also be created
    //   by roundedRectPocketTool — but because that portion of the cutter
    //   extends past the workpiece, those fillets sit OUTSIDE the workpiece
    //   and the Boolean cut produces clean open edges with NO fillet on the
    //   open side.  (Net topology: 3 walls + 2 back-corner cylinders, as
    //   advertised by the signature.)
    //
    //   open_direction is in the entry-face plane.  In slice-1 (axis = -Z,
    //   entry face = +Z top), this is an XY direction; the cutter rectangle
    //   is aligned with open_direction (length axis = open_direction).

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const double kOverhang = 0.05;
    // How far the cutter extends past the open edge.  Use bbox diagonal to
    // guarantee we punch through any side wall in slice-1.
    const double openOverhang = std::max(
        2.0,
        std::sqrt((xMax - xMin) * (xMax - xMin) + (yMax - yMin) * (yMax - yMin)));

    // The footprint axis-along-open: length_mm INTO the workpiece +
    // openOverhang OUT the side.  The pocket's CLOSED end center sits at
    // (position − length·open_direction) — the FAR side of the notch.
    //
    // We build the cutter footprint as an axis-aligned rectangle by
    // PROJECTING open_direction onto the principal axis it's closest to.
    // (Slice-1 restricts open_direction to lie in the XY plane.)
    //
    // Choose between X-aligned and Y-aligned based on which component of
    // open_direction is dominant.
    const bool openAlongX = std::abs(in.open_direction.X()) >
                            std::abs(in.open_direction.Y());

    double cutterSx = 0.0, cutterSy = 0.0;
    double bcX = 0.0, bcY = 0.0;   // bottom-center XY

    if (openAlongX) {
        // Length runs along X.  Width along Y.
        cutterSx = in.length_mm + openOverhang;
        cutterSy = in.width_mm;
        // Position is the centre of the open edge.  If open_direction is +X,
        // the pocket interior is at lower X than `position_x_mm`.  The
        // cutter rectangle centre along X is:
        //   position_x_mm - (length/2) · open_dir.X() + (openOverhang/2) · open_dir.X()
        // = position_x_mm + ((openOverhang - length)/2) · sign(open_dir.X())
        const double sx = (in.open_direction.X() > 0) ? 1.0 : -1.0;
        bcX = in.position_x_mm + sx * (openOverhang - in.length_mm) / 2.0;
        bcY = in.position_y_mm;
    } else {
        cutterSx = in.width_mm;
        cutterSy = in.length_mm + openOverhang;
        const double sy = (in.open_direction.Y() > 0) ? 1.0 : -1.0;
        bcX = in.position_x_mm;
        bcY = in.position_y_mm + sy * (openOverhang - in.length_mm) / 2.0;
    }

    const gp_Pnt bottomCenter(bcX, bcY, zMax - in.depth_mm);
    const TopoDS_Shape cutter = pr::roundedRectPocketTool(
        bottomCenter, cutterSx, cutterSy,
        in.depth_mm + kOverhang, in.corner_r_mm);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    // Build signature.
    json params = {
        { "entry_face_id",   *entryId },
        { "position_x_mm",   in.position_x_mm },
        { "position_y_mm",   in.position_y_mm },
        { "axis_dir",        { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "open_direction",  { in.open_direction.X(), in.open_direction.Y(),
                               in.open_direction.Z() } },
        { "length_mm",       in.length_mm },
        { "width_mm",        in.width_mm },
        { "depth_mm",        in.depth_mm },
        { "corner_r_mm",     in.corner_r_mm },
    };
    json pattern = {
        { "kind",                     kSkillId },
        // 3 walls (back + 2 sides), NOT 4.
        { "planar_wall_count",        3 },
        // 2 back-corner fillets, NOT 4.
        { "corner_cylinder_count",    in.corner_r_mm > 0.0 ? 2 : 0 },
        { "bottom_planar_face",       true },
        { "length_mm",                in.length_mm },
        { "width_mm",                 in.width_mm },
        { "corner_r_mm",              in.corner_r_mm },
        { "open_direction",           { in.open_direction.X(), in.open_direction.Y(),
                                        in.open_direction.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = (in.corner_r_mm > 0.0) ? in.corner_r_mm * 2.0 : 6.0;
    tooling.tool_length_mm    = in.depth_mm * 1.5 + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 600.0;
    tooling.feed_per_tooth_mm = 0.025;
    tooling.stock_removed_mm3 = in.length_mm * in.width_mm * in.depth_mm;
    tooling.est_cycle_time_s  = std::max(1.0,
        (in.length_mm * in.width_mm * in.depth_mm) / 5000.0);
    tooling.extra = {
        { "edge_exits_workpiece", true },
        { "open_side_strategy",   "tool_overhang_past_boundary" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::mill_open_pocket applied: {}×{}×{} cornerR={} faces {}→{}",
                  in.length_mm, in.width_mm, in.depth_mm, in.corner_r_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Pattern: 2 vertical-axis cylindrical CORNER FILLETS sharing a common
// planar BOTTOM face.  (Closed pockets have 4 corners; open ones have 2.)
//
// We mirror mill_rect_pocket's bottom-face grouping but accept groups of
// exactly 2 (vs exactly 4 for closed pockets).

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

// Geometric matcher: find a workpiece face index whose plane coincides
// with `af`'s plane.  STEP round-trip safe (no IsSame TShape identity).
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

    struct CornerEntry {
        int    cylFaceIdx;
        double radius;
        gp_Pnt axisBase;
        gp_Dir axisDir;
        double zTop;
        double zBot;
    };

    std::map<int, std::vector<CornerEntry>> bottomGroups;

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        const TopoDS_Face& cf = wp.face(fIdx);
        BRepAdaptor_Surface surf(cf);
        const gp_Cylinder cyl = surf.Cylinder();
        const gp_Dir adir = cyl.Axis().Direction();
        if (std::abs(std::abs(adir.Z()) - 1.0) > 1e-2) continue;

        int bottomFaceIdx = -1;
        double zHi = -1e30, zLo = 1e30;
        for (TopExp_Explorer exp(cf, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Pnt mid = crv.Value((crv.FirstParameter() + crv.LastParameter()) / 2.0);
            zHi = std::max(zHi, mid.Z());
            zLo = std::min(zLo, mid.Z());
            if (!edgeFaces.Contains(e)) continue;
            const auto& adj = edgeFaces.FindFromKey(e);
            for (NCollection_List<TopoDS_Shape>::Iterator it(adj); it.More(); it.Next()) {
                const TopoDS_Face& af = TopoDS::Face(it.Value());
                if (af.IsSame(cf)) continue;
                if (BRepAdaptor_Surface(af).GetType() != GeomAbs_Plane) continue;
                const int idx = findPlanarFaceIndex(wp, af);
                if (idx >= 0) { bottomFaceIdx = idx; }
            }
        }
        if (bottomFaceIdx < 0) continue;
        bottomGroups[bottomFaceIdx].push_back({
            fIdx, cyl.Radius(), cyl.Axis().Location(), adir, zHi, zLo
        });
    }

    // Exactly 2 corner cylinders sharing a bottom face → open pocket.
    for (const auto& [bottomIdx, entries] : bottomGroups) {
        if (entries.size() != 2) continue;

        // Both corner cylinders should share a Z range (same pocket).
        const double zHi0 = entries[0].zTop, zLo0 = entries[0].zBot;
        const double zHi1 = entries[1].zTop, zLo1 = entries[1].zBot;
        if (std::abs(zHi0 - zHi1) > 0.1) continue;
        if (std::abs(zLo0 - zLo1) > 0.1) continue;

        const double r = (entries[0].radius + entries[1].radius) / 2.0;
        const double zHi = (zHi0 + zHi1) / 2.0;
        const double zLo = (zLo0 + zLo1) / 2.0;
        const double depth = std::abs(zHi - zLo);

        // The 2 corners form one EDGE of the pocket (the back).  The
        // midpoint of the corner-axis pair is the "back-edge midpoint".
        const gp_Pnt& a = entries[0].axisBase;
        const gp_Pnt& b = entries[1].axisBase;
        const double backCx = (a.X() + b.X()) / 2.0;
        const double backCy = (a.Y() + b.Y()) / 2.0;
        // width = distance between the 2 corner centres + 2 r (the radii
        // sit at the corner inside, so corner_centre = backCorner − r·towards-interior).
        const double cornerSpan = std::sqrt(
            (a.X() - b.X()) * (a.X() - b.X()) +
            (a.Y() - b.Y()) * (a.Y() - b.Y()));
        const double width = cornerSpan + 2.0 * r;

        // Determine open_direction: the line connecting the two corners is
        // ALONG the back edge.  The OPEN edge is parallel to this line.
        // open_direction is perpendicular to the back-corner line.  Without
        // prior context we cannot infer SIGN (which side is open), so we
        // pick the +X or +Y direction perpendicular to the back-corner line
        // and emit the magnitude.
        gp_Vec backLine(a, b);
        if (backLine.Magnitude() < 1e-9) continue;
        backLine.Normalize();
        // Perpendicular in XY plane: rotate 90°.
        const gp_Dir openDir(-backLine.Y(), backLine.X(), 0.0);

        // length is unrecoverable without finding the open-edge plane, which
        // requires walking the topology.  As an approximation, the back wall
        // sits in the plane perpendicular to openDir passing through (backCx,
        // backCy).  Without scanning all planar walls we report length=0 as
        // a sentinel (caller can post-process if needed).
        const double length = 0.0;

        json recovered = {
            { "position_x_mm",   backCx },
            { "position_y_mm",   backCy },
            { "axis_dir",        { 0.0, 0.0, -1.0 } },
            { "open_direction",  { openDir.X(), openDir.Y(), 0.0 } },
            { "length_mm",       length },
            { "width_mm",        width },
            { "depth_mm",        depth },
            { "corner_r_mm",     r },
        };
        json matched = {
            { "bottom_face_id",        bottomIdx },
            { "corner_cylinder_ids",   { entries[0].cylFaceIdx, entries[1].cylFaceIdx } },
            { "note",                  "length_mm and open-side orientation "
                                       "require full topology walk; reported as "
                                       "best-effort from corner pair" },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.70, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::mill_open_pocket
