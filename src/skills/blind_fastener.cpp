// @lat: [[engine/skills#blind_fastener]]

#include "blind_fastener.hpp"

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
#include <gp_Cylinder.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace koocadcam::skill::blind_fastener {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Standard blind-rivet diameter table ──────────────────────────────────
//
// The four most common blind-rivet body sizes, as documented by Stanley,
// POP®, Avdel®.  Grip-length ranges are simplified generic ranges per
// diameter (in mm); real-world catalogs split into multiple length grades.

namespace {

struct BlindSpec {
    double nominal_dia_mm;
    double grip_min_mm;
    double grip_max_mm;
};

constexpr std::array<BlindSpec, 4> kBlindTable {{
    { 3.2, 0.5,  9.5 },
    { 4.0, 0.5, 12.5 },
    { 4.8, 0.5, 14.5 },
    { 6.4, 0.5, 16.0 },
}};

constexpr double kHoleAllowance_mm = 0.05;

const BlindSpec* findSpec(double nominal_dia_mm)
{
    for (const auto& e : kBlindTable) {
        if (std::abs(e.nominal_dia_mm - nominal_dia_mm) < 1e-3) return &e;
    }
    return nullptr;
}

}  // namespace

const std::vector<double>& standardBlindDiameters()
{
    static const std::vector<double> kStd = { 3.2, 4.0, 4.8, 6.4 };
    return kStd;
}

double holeDiameterFor(double nominal_dia_mm)
{
    const BlindSpec* s = findSpec(nominal_dia_mm);
    return s ? (s->nominal_dia_mm + kHoleAllowance_mm) : 0.0;
}

double matchStandardDia(double observed_hole_dia_mm, double tol_mm)
{
    double best = 0.0;
    double bestErr = tol_mm;
    for (double d : standardBlindDiameters()) {
        const double err = std::abs((d + kHoleAllowance_mm) - observed_hole_dia_mm);
        if (err < bestErr) { bestErr = err; best = d; }
    }
    return best;
}

GripRange gripRangeFor(double nominal_dia_mm)
{
    const BlindSpec* s = findSpec(nominal_dia_mm);
    if (!s) return GripRange{ 0.0, 0.0 };
    return GripRange{ s->grip_min_mm, s->grip_max_mm };
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    const BlindSpec* spec = findSpec(in.nominal_dia_mm);
    if (!spec) {
        r.add("DFM-BF-DIA", "error",
              "blind_fastener nominal_dia " + std::to_string(in.nominal_dia_mm) +
              " mm not in standard blind-rivet set {3.2, 4.0, 4.8, 6.4}");
        return r;
    }
    if (in.grip_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "blind_fastener grip_length_mm must be > 0");
    } else if (in.grip_length_mm < spec->grip_min_mm ||
               in.grip_length_mm > spec->grip_max_mm)
    {
        r.add("DFM-BF-GRIP", "error",
              "blind_fastener grip_length " + std::to_string(in.grip_length_mm) +
              " mm outside spec range [" + std::to_string(spec->grip_min_mm) +
              ", " + std::to_string(spec->grip_max_mm) + "] mm for dia " +
              std::to_string(in.nominal_dia_mm) + " mm");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "blind_fastener DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("blind_fastener: entry_face datum unresolved");

    const double holeDia = holeDiameterFor(in.nominal_dia_mm);
    if (holeDia <= 0.0) throw SkillError("blind_fastener: hole dia unresolved");

    // Through-cut geometry — same approach as rivet_hole / sheet_punch.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    const double kOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    gp_Pnt toolStart;
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        if (adir.Z() < 0) {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMax + kOverhang);
        } else {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMin - kOverhang);
        }
    } else {
        const double margin = bboxDiag + 1.0;
        toolStart = gp_Pnt(
            in.position_x_mm - adir.X() * margin,
            in.position_y_mm - adir.Y() * margin,
            (zMin + zMax) / 2.0 - adir.Z() * margin);
    }
    const double toolHeight = bboxDiag + 2.0 * kOverhang;
    const gp_Ax2 toolAx(toolStart, adir);
    const TopoDS_Shape cutter = pr::cylinder(toolAx, holeDia / 2.0, toolHeight);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json params = {
        { "entry_face_id",   *entryId },
        { "position_x_mm",   in.position_x_mm },
        { "position_y_mm",   in.position_y_mm },
        { "axis_dir",        { adir.X(), adir.Y(), adir.Z() } },
        { "nominal_dia_mm",  in.nominal_dia_mm },
        { "grip_length_mm",  in.grip_length_mm },
        { "hole_dia_mm",     holeDia },
        { "through_hole",    true },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     1 },
        { "circular_edge_count",        2 },
        { "bottom_planar_face_present", false },
        { "nominal_dia_mm",             in.nominal_dia_mm },
        { "grip_length_mm",             in.grip_length_mm },
        { "hole_dia_mm",                holeDia },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
        { "through_hole",               true },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "drill;blind_rivet_gun";
    tooling.tool_dia_mm       = holeDia;
    tooling.tool_length_mm    = bboxDiag * 1.5 + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = M_PI * (holeDia / 2.0) * (holeDia / 2.0) * bboxDiag;
    tooling.est_cycle_time_s  = std::max(1.5, bboxDiag / 60.0 + 1.0);  // drill + pull
    tooling.extra = {
        { "fastener_class",  "blind_rivet" },
        { "nominal_dia_mm",  in.nominal_dia_mm },
        { "grip_length_mm",  in.grip_length_mm },
        { "tolerance_class", "H11" },
        { "machining_constraint",
          "One-sided install via blind-rivet mandrel pull; mandrel break is "
          "not modelled in slice-1." }
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::blind_fastener applied: dia={} grip={} hole={}",
                  in.nominal_dia_mm, in.grip_length_mm, holeDia);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// History replay first.  Topology fallback requires a through cylindrical
// hole whose diameter ≈ standard_nominal + 0.05 mm.

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

    // History replay.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "position_x_mm",  f.params.value("position_x_mm",  0.0) },
            { "position_y_mm",  f.params.value("position_y_mm",  0.0) },
            { "axis_dir",       f.params.value("axis_dir",       json::array({0.0,0.0,-1.0})) },
            { "nominal_dia_mm", f.params.value("nominal_dia_mm", 0.0) },
            { "grip_length_mm", f.params.value("grip_length_mm", 0.0) },
            { "hole_dia_mm",    f.params.value("hole_dia_mm",    0.0) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 0.95, json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // Topology fallback.
    const auto edgeFaces = buildEdgeFaceMap(wp.shape());

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;

        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double radius = cyl.Radius();
        const double dia    = 2.0 * radius;

        const double nominal = matchStandardDia(dia, 0.10);
        if (nominal <= 0.0) continue;

        // Verify through-hole topology.
        const gp_Ax1 axis = cyl.Axis();
        const gp_Dir adir = axis.Direction();
        std::vector<gp_Pnt> centers;
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            if (std::abs(std::abs(c.Axis().Direction().Dot(adir)) - 1.0) > 1e-3) continue;
            if (std::abs(c.Radius() - radius) > 1e-3) continue;
            centers.push_back(c.Location());
        }
        if (centers.size() < 2) continue;

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
        const gp_Pnt cBot = *minIt;
        const gp_Pnt cTop = *maxIt;
        const double axialExtent = cTop.Distance(cBot);

        // Both circle sides must be adjacent to large planar faces (= through).
        const double drillBottomArea = M_PI * radius * radius;
        double minAdjPlanarArea = std::numeric_limits<double>::max();
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            if (!edgeFaces.Contains(e)) continue;
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const auto& adj = edgeFaces.FindFromKey(e);
            for (NCollection_List<TopoDS_Shape>::Iterator it(adj); it.More(); it.Next()) {
                const TopoDS_Face& af = TopoDS::Face(it.Value());
                if (af.IsSame(cylFace)) continue;
                if (BRepAdaptor_Surface(af).GetType() != GeomAbs_Plane) continue;
                GProp_GProps gp;
                BRepGProp::SurfaceProperties(af, gp);
                minAdjPlanarArea = std::min(minAdjPlanarArea, gp.Mass());
            }
        }
        if (minAdjPlanarArea < drillBottomArea * 1.5) continue;   // blind, not through

        gp_Vec dirVec(cTop, cBot);
        if (dirVec.Magnitude() < 1e-9) continue;
        dirVec.Normalize();

        // grip_length ≈ axialExtent (joint stack equals the hole length).
        const GripRange gr = gripRangeFor(nominal);
        const double gripGuess = std::clamp(axialExtent, gr.min_mm, gr.max_mm);

        // Confidence higher when the diameter snaps tightly to spec.
        const double err = std::abs((nominal + kHoleAllowance_mm) - dia);
        double conf = 0.85;
        if (err > 0.03) conf -= 0.15;
        if (err > 0.07) conf -= 0.20;
        conf = std::clamp(conf, 0.30, 0.90);

        json recovered = {
            { "position_x_mm",  cTop.X() },
            { "position_y_mm",  cTop.Y() },
            { "axis_dir",       { dirVec.X(), dirVec.Y(), dirVec.Z() } },
            { "nominal_dia_mm", nominal },
            { "grip_length_mm", gripGuess },
            { "hole_dia_mm",    dia },
        };
        json matched = {
            { "cylindrical_face_id", fIdx },
            { "top_center", { cTop.X(), cTop.Y(), cTop.Z() } },
            { "bot_center", { cBot.X(), cBot.Y(), cBot.Z() } },
            { "standard_match_err_mm", err },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, conf, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::blind_fastener
