// @lat: [[engine/skills#sheet_punch]]

#include "sheet_punch.hpp"

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

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace koocadcam::skill::sheet_punch {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

double sheetThickness(const Workpiece& wp)
{
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    return std::min({ xMax - xMin, yMax - yMin, zMax - zMin });
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "sheet_punch dia_mm must be > 0");
    }

    const double t = sheetThickness(wp);
    if (t > 5.0) {
        r.add("DFM-SP-SHEET", "error",
              "sheet_punch: smallest stock extent " + std::to_string(t) +
              " mm > 5 mm — not sheet metal");
    }
    if (t <= 0.0) {
        r.add("DFM-SP-SHEET", "error", "sheet_punch: degenerate stock thickness");
    }

    if (in.dia_mm > 0.0 && t > 0.0 && in.dia_mm < 1.5 * t) {
        r.add("DFM-SP-MIN-DIA", "error",
              "sheet_punch dia " + std::to_string(in.dia_mm) +
              " mm < 1.5 × sheet_thickness (" + std::to_string(1.5 * t) +
              " mm) — punch min ratio violated");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "sheet_punch DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("sheet_punch: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    const double kOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    // Position the cutter cylinder above the entry face along -axis_dir,
    // then extend it across the full sheet so both top and bottom are
    // pierced (through cut).
    gp_Pnt toolStart;
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        if (adir.Z() < 0) {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMax + kOverhang);
        } else {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMin - kOverhang);
        }
    } else {
        // Fallback (non-axial punch): start outside bbox along -axis_dir.
        const double margin = bboxDiag + 1.0;
        toolStart = gp_Pnt(
            in.position_x_mm - adir.X() * margin,
            in.position_y_mm - adir.Y() * margin,
            (zMin + zMax) / 2.0 - adir.Z() * margin);
    }

    const double toolHeight = bboxDiag + 2.0 * kOverhang;
    const gp_Ax2 toolAx(toolStart, adir);
    const TopoDS_Shape cutter = pr::cylinder(toolAx, in.dia_mm / 2.0, toolHeight);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    // Signature
    const double t = sheetThickness(wp);
    json params = {
        { "entry_face_id", *entryId },
        { "position_x_mm", in.position_x_mm },
        { "position_y_mm", in.position_y_mm },
        { "axis_dir",      { adir.X(), adir.Y(), adir.Z() } },
        { "dia_mm",        in.dia_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     1 },
        { "circular_edge_count",        2 },
        { "bottom_planar_face_present", false },
        { "is_through_cut",             true },
        { "dia_mm",                     in.dia_mm },
        { "sheet_thickness_mm",         t },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "punch_press";
    tooling.tool_dia_mm       = in.dia_mm;
    tooling.tool_length_mm    = t * 2.0;          // punch + die clearance
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = M_PI * (in.dia_mm / 2.0) * (in.dia_mm / 2.0) * t;
    tooling.est_cycle_time_s  = 0.5;              // single stroke
    tooling.extra["machining_constraint"] =
        "Sheet-metal punch press — die clearance ≈ 10% of thickness; "
        "burr on exit side";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::sheet_punch applied: dia={} thickness={} faces {}→{}",
                  in.dia_mm, t, wp.faceCount(), wpNew->faceCount());

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
    const double t = sheetThickness(wp);
    // Only recognize on sheet stock (smallest extent ≤ 5 mm).
    if (t <= 0.0 || t > 5.0) return out;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const auto edgeFaces = buildEdgeFaceMap(wp.shape());

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;

        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double radius   = cyl.Radius();
        const gp_Ax1  axis    = cyl.Axis();
        const gp_Dir  adir    = axis.Direction();

        // Collect circular edges on this cylinder.
        std::vector<gp_Pnt> centers;
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            if (std::abs(std::abs(c.Axis().Direction().Dot(adir)) - 1.0) > 1e-3)
                continue;
            if (std::abs(c.Radius() - radius) > 1e-3) continue;
            centers.push_back(c.Location());
        }
        if (centers.size() < 2) continue;

        // Sort by Z so cTop has the higher Z (entry side for a top-down punch).
        auto cmpZ = [&](const gp_Pnt& a, const gp_Pnt& b) { return a.Z() < b.Z(); };
        const auto botIt = std::min_element(centers.begin(), centers.end(), cmpZ);
        const auto topIt = std::max_element(centers.begin(), centers.end(), cmpZ);
        const gp_Pnt cBot = *botIt;
        const gp_Pnt cTop = *topIt;
        const double axialExtent = cTop.Distance(cBot);

        // Sheet-punch test: cylinder axial extent must equal the sheet
        // thickness (within tolerance).
        if (std::abs(axialExtent - t) > std::max(0.05, t * 0.05)) continue;

        // Both circular-edge sides must be adjacent to a LARGE planar face
        // (= sheet top + sheet bottom), not a small disc (= drill blind end).
        // i.e. the smallest adjacent planar face is still much larger than
        // π r² of the cylinder.
        const double drillBottomArea = M_PI * radius * radius;
        double minAdjPlanarArea = std::numeric_limits<double>::max();
        int planarAdjCount = 0;
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            if (!edgeFaces.Contains(e)) continue;
            const auto& adj = edgeFaces.FindFromKey(e);
            for (NCollection_List<TopoDS_Shape>::Iterator it(adj); it.More(); it.Next()) {
                const TopoDS_Face& af = TopoDS::Face(it.Value());
                if (af.IsSame(cylFace)) continue;
                if (BRepAdaptor_Surface(af).GetType() != GeomAbs_Plane) continue;
                GProp_GProps gp;
                BRepGProp::SurfaceProperties(af, gp);
                minAdjPlanarArea = std::min(minAdjPlanarArea, gp.Mass());
                ++planarAdjCount;
            }
        }
        if (planarAdjCount < 2) continue;
        // Through-cut requires BOTH adjacent planar faces to be large.
        if (minAdjPlanarArea < drillBottomArea * 1.5) continue;

        // Punch direction = from top (entry) to bottom (exit).
        gp_Vec dirVec(cTop, cBot);
        if (dirVec.Magnitude() < 1e-6) continue;
        dirVec.Normalize();

        json recovered = {
            { "position_x_mm", cTop.X() },
            { "position_y_mm", cTop.Y() },
            { "axis_dir",      { dirVec.X(), dirVec.Y(), dirVec.Z() } },
            { "dia_mm",        2.0 * radius },
        };
        json matched = {
            { "cylindrical_face_id", fIdx },
            { "sheet_thickness_mm",  t },
            { "top_center", { cTop.X(), cTop.Y(), cTop.Z() } },
            { "bot_center", { cBot.X(), cBot.Y(), cBot.Z() } },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.90, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::sheet_punch
