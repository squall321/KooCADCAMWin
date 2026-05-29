// @lat: [[engine/skills#captive_nut_pocket]]

#include "captive_nut_pocket.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <Standard_Failure.hxx>
#include <gp.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <array>
#include <cmath>

namespace koocadcam::skill::captive_nut_pocket {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

struct NutEntry {
    const char* size;
    double      af_mm;      // across-flats (wrench size)
    double      height_mm;
};

constexpr std::array<NutEntry, 7> kTable {{
    { "M2",   4.0,  1.6 },
    { "M2.5", 5.0,  2.0 },
    { "M3",   5.5,  2.4 },
    { "M4",   7.0,  3.2 },
    { "M5",   8.0,  4.0 },
    { "M6",   10.0, 5.0 },
    { "M8",   13.0, 6.5 },
}};

const NutEntry* lookupEntry(const std::string& s) {
    for (const auto& e : kTable) if (s == e.size) return &e;
    return nullptr;
}

// Build a hexagonal prism cutter with flat-to-flat width = af, height = h,
// centered on (cx, cy) at z = topZ, extruded DOWNWARD by h.
// Hexagon orientation: two flats normal to +X (flats on left/right sides),
// so the AF is measured horizontally — vertices are at top/bottom.
//
// The hexagon's circumradius (corner) = af / sqrt(3).
TopoDS_Shape buildHexPrism(double cx, double cy, double topZ,
                           double af, double height)
{
    const double rCorner = af / std::sqrt(3.0);
    const double halfAF  = af / 2.0;

    // 6 vertices: starting at top, going CCW.
    //   v0 = top apex                        (0, +rCorner)
    //   v1 = upper-left flat top corner       (-halfAF, +halfHexHalf)
    //   etc.
    // Easier: use 30° offset so two vertices are at top & bottom; flats are
    // on left & right.
    std::array<gp_Pnt, 6> pts;
    for (int i = 0; i < 6; ++i) {
        const double a = (90.0 + 60.0 * i) * M_PI / 180.0;
        pts[i] = gp_Pnt(cx + rCorner * std::cos(a),
                        cy + rCorner * std::sin(a),
                        topZ + 0.05);
    }
    BRepBuilderAPI_MakeWire wire;
    for (int i = 0; i < 6; ++i) {
        const int j = (i + 1) % 6;
        BRepBuilderAPI_MakeEdge edge(pts[i], pts[j]);
        if (!edge.IsDone()) throw Standard_Failure("captive_nut_pocket: hex edge build failed");
        wire.Add(edge.Edge());
    }
    if (!wire.IsDone()) throw Standard_Failure("captive_nut_pocket: hex wire build failed");
    BRepBuilderAPI_MakeFace face(wire.Wire(), /*OnlyPlane=*/true);
    if (!face.IsDone()) throw Standard_Failure("captive_nut_pocket: hex face build failed");
    const gp_Vec extrude(0.0, 0.0, -(height + 0.1));
    BRepPrimAPI_MakePrism prism(face.Face(), extrude);
    prism.Build();
    if (!prism.IsDone()) throw Standard_Failure("captive_nut_pocket: hex prism build failed");
    // Note: AF along X with this orientation.  Vertical extent = 2 * rCorner.
    (void)halfAF;
    return prism.Shape();
}

// Build an axis-aligned rectangular slot extruded downward from topZ.
TopoDS_Shape buildSlotBox(double cx, double cy, double topZ,
                          double slotLength, double slotWidth, double depth,
                          const gp_Dir& slotDir)
{
    // gp_Ax2(P, V, Vx): V = local Z; Vx = local X.
    // We want a box of (DX=slotLength along slotDir, DY=slotWidth perp,
    // DZ=depth along -Z).  Use V = -Z so DZ goes down; Vx = slotDir.
    const gp_Dir mainZ(0, 0, -1);
    // Origin = lower-left corner at the upper face: shift by -length/2 along
    // slotDir and -width/2 along perpendicular.
    const gp_Dir perp(-slotDir.Y(), slotDir.X(), 0.0);
    const gp_Pnt origin(
        cx - slotLength / 2.0 * slotDir.X() - slotWidth / 2.0 * perp.X(),
        cy - slotLength / 2.0 * slotDir.Y() - slotWidth / 2.0 * perp.Y(),
        topZ + 0.05);
    const gp_Ax2 ax(origin, mainZ, slotDir);
    return pr::box(ax, slotLength, slotWidth, depth + 0.1);
}

}  // namespace

double nutAcrossFlatsFor(const std::string& s)
{ const auto* e = lookupEntry(s); return e ? e->af_mm : 0.0; }

double nutHeightFor(const std::string& s)
{ const auto* e = lookupEntry(s); return e ? e->height_mm : 0.0; }

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.nut_size.empty()) {
        r.add("DFM-NUT-SIZE", "error",
              "captive_nut_pocket: nut_size is empty");
        return r;
    }
    const auto* e = lookupEntry(in.nut_size);
    if (!e) {
        r.add("DFM-NUT-SIZE", "error",
              "captive_nut_pocket: unknown nut_size '" + in.nut_size +
              "' (supported: M2/M2.5/M3/M4/M5/M6/M8)");
        return r;
    }

    if (in.entry_edge != "+x" && in.entry_edge != "-x" &&
        in.entry_edge != "+y" && in.entry_edge != "-y") {
        r.add("DFM-NUT-EDGE", "error",
              "captive_nut_pocket: entry_edge '" + in.entry_edge +
              "' not in {+x, -x, +y, -y}");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness = zMax - zMin;
    const double pocketDepth = e->height_mm + 0.2;
    if (thickness > 0.0 && thickness < pocketDepth + 0.5) {
        r.add("DFM-NUT-DEPTH", "error",
              "captive_nut_pocket: stock thickness " +
              std::to_string(thickness) + " mm < pocket depth + 0.5 (" +
              std::to_string(pocketDepth + 0.5) + " mm)");
    }

    // The slot width = AF - 0.5 mm (intentionally narrower than hex flat-to-
    // flat so the spring-detent acts).  Sanity guard:
    const double slotWidth = e->af_mm - 0.5;
    if (slotWidth < 0.8) {
        r.add("DFM-002", "error",
              "captive_nut_pocket: derived access slot width " +
              std::to_string(slotWidth) + " < 0.8 mm");
    }

    // Ensure entry edge is reachable: pocket center to chosen edge within stock.
    if (in.entry_edge == "+x" && in.position_x_mm >= xMax)
        r.add("DFM-NUT-EDGE", "error",
              "captive_nut_pocket: pocket center already at/beyond +x edge");
    if (in.entry_edge == "-x" && in.position_x_mm <= xMin)
        r.add("DFM-NUT-EDGE", "error",
              "captive_nut_pocket: pocket center already at/beyond -x edge");
    if (in.entry_edge == "+y" && in.position_y_mm >= yMax)
        r.add("DFM-NUT-EDGE", "error",
              "captive_nut_pocket: pocket center already at/beyond +y edge");
    if (in.entry_edge == "-y" && in.position_y_mm <= yMin)
        r.add("DFM-NUT-EDGE", "error",
              "captive_nut_pocket: pocket center already at/beyond -y edge");

    if (in.slip_fit_mm < 0.0 || in.slip_fit_mm > 1.0) {
        r.add("DFM-INPUT", "warning",
              "captive_nut_pocket: slip_fit_mm " +
              std::to_string(in.slip_fit_mm) + " outside typical 0..1 mm");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "captive_nut_pocket DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto* e = lookupEntry(in.nut_size);
    const double afNominal = e->af_mm;
    const double hexAF     = afNominal + in.slip_fit_mm;   // pocket flat-to-flat
    const double pocketDep = e->height_mm + 0.2;

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("captive_nut_pocket: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;

    // ── Sub-feature 1: hex pocket ────────────────────────────────────────
    const TopoDS_Shape hexTool =
        buildHexPrism(in.position_x_mm, in.position_y_mm, topZ, hexAF, pocketDep);

    // ── Sub-feature 2: access slot from chosen edge ──────────────────────
    gp_Dir slotDir(1, 0, 0);
    double slotEndX = xMax;
    double slotEndY = in.position_y_mm;
    if (in.entry_edge == "+x") { slotDir = gp_Dir( 1, 0, 0); slotEndX = xMax; slotEndY = in.position_y_mm; }
    if (in.entry_edge == "-x") { slotDir = gp_Dir(-1, 0, 0); slotEndX = xMin; slotEndY = in.position_y_mm; }
    if (in.entry_edge == "+y") { slotDir = gp_Dir(0,  1, 0); slotEndX = in.position_x_mm; slotEndY = yMax; }
    if (in.entry_edge == "-y") { slotDir = gp_Dir(0, -1, 0); slotEndX = in.position_x_mm; slotEndY = yMin; }
    const double slotLength = std::hypot(slotEndX - in.position_x_mm,
                                         slotEndY - in.position_y_mm) + 0.1;
    const double slotWidth = afNominal - 0.5;   // narrower than hex AF
    const double slotCx = (in.position_x_mm + slotEndX) / 2.0;
    const double slotCy = (in.position_y_mm + slotEndY) / 2.0;
    const TopoDS_Shape slotTool =
        buildSlotBox(slotCx, slotCy, topZ, slotLength, slotWidth, pocketDep, slotDir);

    // ── Sub-feature 3: spring detent relief ──────────────────────────────
    // Small cylindrical relief at the back of the hex pocket (opposite to
    // entry_edge) so the nut clicks into place under spring pressure from
    // tabs that would be machined in via a separate pass.
    const double reliefDia = 0.8;  // smallest standard endmill
    const double reliefOffset = hexAF * 0.6;
    gp_Pnt reliefCenter(
        in.position_x_mm - slotDir.X() * reliefOffset,
        in.position_y_mm - slotDir.Y() * reliefOffset,
        topZ + 0.05);
    const gp_Ax2 reliefAx(reliefCenter, gp_Dir(0, 0, -1));
    const TopoDS_Shape reliefTool =
        pr::cylinder(reliefAx, reliefDia / 2.0, pocketDep + 0.1);

    // Fuse all three tools then single cut.
    TopoDS_Shape fused = pr::fuse(hexTool, slotTool);
    fused = pr::fuse(fused, reliefTool);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "entry_face_id",  *entryId },
        { "position_x_mm",  in.position_x_mm },
        { "position_y_mm",  in.position_y_mm },
        { "axis_dir",       { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "nut_size",       in.nut_size },
        { "slip_fit_mm",    in.slip_fit_mm },
        { "entry_edge",     in.entry_edge },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    3 },
        { "nut_size",            in.nut_size },
        { "nut_af_mm",           afNominal },
        { "pocket_af_mm",        hexAF },
        { "pocket_depth_mm",     pocketDep },
        { "access_slot_mm",      slotWidth },
        { "access_slot_length_mm", slotLength },
        { "relief_dia_mm",       reliefDia },
        { "entry_edge",          in.entry_edge },
        { "axis_dir",            { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type   = "endmill";
    tooling.tool_dia_mm = std::min(reliefDia, slotWidth);
    tooling.tool_length_mm = pocketDep + 5.0;
    tooling.tool_material  = "carbide";
    tooling.flute_count    = 2;
    tooling.cutting_speed_sfm = 280.0;
    tooling.feed_per_tooth_mm = 0.04;
    // Hex area = 3*sqrt(3)/2 * (AF/2)^2 * 2 ... use 2 * sqrt(3) * (AF/2)^2 form
    const double hexArea = 2.0 * std::sqrt(3.0) * (hexAF / 2.0) * (hexAF / 2.0);
    const double slotArea = slotLength * slotWidth;
    const double reliefArea = M_PI * (reliefDia / 2.0) * (reliefDia / 2.0);
    tooling.stock_removed_mm3 = (hexArea + slotArea + reliefArea) * pocketDep;
    tooling.est_cycle_time_s  = std::max(3.0, (hexArea + slotArea) / 30.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "endmill" }, { "tool_dia_mm", reliefDia },
              { "operation",  "hex_pocket_trochoidal" } },
            { { "tool_type", "endmill" }, { "tool_dia_mm", reliefDia },
              { "operation",  "access_slot" } },
            { { "tool_type", "endmill" }, { "tool_dia_mm", reliefDia },
              { "operation",  "spring_detent_relief" } },
        } },
        { "nut_standard", "ISO_4032" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::captive_nut_pocket applied: {} AF={} depth={}",
                  in.nut_size, hexAF, pocketDep);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition (metadata replay) ────────────────────────────────────────

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
                                { "is_compound", true } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::captive_nut_pocket
