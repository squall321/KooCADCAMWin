// @lat: [[engine/skills#ultrasonic_weld]]

#include "ultrasonic_weld.hpp"

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
#include <cmath>
#include <limits>

namespace koocadcam::skill::ultrasonic_weld {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.spot_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "ultrasonic_weld spot_dia_mm must be > 0");
    }
    if (in.bead_height_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "ultrasonic_weld bead_height_mm must be > 0");
    }
    if (in.spot_dia_mm > 0.0 && in.spot_dia_mm < 1.0) {
        r.add("DFM-USW-DIA", "error",
              "ultrasonic_weld spot_dia_mm " + std::to_string(in.spot_dia_mm) +
              " < 1.0 mm (horn tip too small for stable knurl)");
    }
    if (in.spot_dia_mm > 12.0) {
        r.add("DFM-USW-DIA", "error",
              "ultrasonic_weld spot_dia_mm " + std::to_string(in.spot_dia_mm) +
              " > 12 mm (single-spot USW max — use seam pass)");
    }
    if (in.bead_height_mm > 0.5) {
        r.add("DFM-USW-HEIGHT", "error",
              "ultrasonic_weld bead_height_mm " +
              std::to_string(in.bead_height_mm) +
              " > 0.5 mm — USW is solid-state, very shallow");
    }
    if (in.sheet_thickness_mm > 3.0) {
        r.add("DFM-USW-THK", "warning",
              "ultrasonic_weld sheet_thickness_mm " +
              std::to_string(in.sheet_thickness_mm) +
              " > 3 mm — energy will not couple through thick stock; "
              "consider resistance spot weld");
    }
    if (in.vibration_amplitude_um <= 0.0) {
        r.add("DFM-USW-AMP", "warning",
              "ultrasonic_weld vibration_amplitude_um must be > 0");
    }
    if (in.vibration_amplitude_um > 80.0) {
        r.add("DFM-USW-AMP", "warning",
              "ultrasonic_weld vibration_amplitude_um " +
              std::to_string(in.vibration_amplitude_um) +
              " > 80 µm (off-spec for commercial converters)");
    }
    if (in.frequency_khz < 10.0 || in.frequency_khz > 60.0) {
        r.add("DFM-USW-FREQ", "warning",
              "ultrasonic_weld frequency_khz " +
              std::to_string(in.frequency_khz) +
              " outside 10–60 kHz commercial band");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "ultrasonic_weld DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("ultrasonic_weld: entry_face datum unresolved");
    if (!wp.isFacePlanar(*entryId))
        throw SkillError("ultrasonic_weld: entry_face must be planar");

    const gp_Dir outNorm = wp.faceNormal(*entryId);
    const gp_Pnt faceCtr = wp.faceCenter(*entryId);

    // Build a SHORT disc.  Sink 1 % into the parent for clean Boolean.
    const double kSink = in.bead_height_mm * 0.01;
    const double cylH  = in.bead_height_mm + kSink;

    const gp_Pnt bumpBase(in.position_x_mm, in.position_y_mm, faceCtr.Z());
    const gp_Pnt cylOrigin(
        bumpBase.X() - outNorm.X() * kSink,
        bumpBase.Y() - outNorm.Y() * kSink,
        bumpBase.Z() - outNorm.Z() * kSink);
    const gp_Ax2 cylAx(cylOrigin, outNorm);

    const TopoDS_Shape bead = pr::cylinder(cylAx, in.spot_dia_mm / 2.0, cylH);
    const TopoDS_Shape newShape = pr::fuse(wp.shape(), bead);

    json params = {
        { "entry_face_id",          *entryId },
        { "position_x_mm",          in.position_x_mm },
        { "position_y_mm",          in.position_y_mm },
        { "spot_dia_mm",            in.spot_dia_mm },
        { "bead_height_mm",         in.bead_height_mm },
        { "material",               in.material },
        { "vibration_amplitude_um", in.vibration_amplitude_um },
        { "frequency_khz",          in.frequency_khz },
        { "sheet_thickness_mm",     in.sheet_thickness_mm },
        { "entry_face_normal", { outNorm.X(), outNorm.Y(), outNorm.Z() } },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     1 },
        { "circular_edge_count",        2 },
        { "top_planar_disc_present",    true },
        { "spot_dia_mm",                in.spot_dia_mm },
        { "bead_height_mm",             in.bead_height_mm },
        { "additive",                   true },
        { "solid_state",                true },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "ultrasonic_horn";
    tooling.tool_dia_mm       = in.spot_dia_mm;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "titanium";   // typ. converter horn
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    // Volume nominally added (negative — same convention as spot_weld);
    // physically it is solid-state plastic flow, not deposition.
    tooling.stock_removed_mm3 =
        -(M_PI * (in.spot_dia_mm / 2.0) * (in.spot_dia_mm / 2.0) *
          in.bead_height_mm);
    tooling.est_cycle_time_s  = 0.5;   // USW cycles are milliseconds
    tooling.extra = {
        { "process",                "ultrasonic_weld" },
        { "material",               in.material },
        { "vibration_amplitude_um", in.vibration_amplitude_um },
        { "frequency_khz",          in.frequency_khz },
        { "sheet_thickness_mm",     in.sheet_thickness_mm },
        { "solid_state",            true },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::ultrasonic_weld applied: dia={} h={} f={}kHz faces {}→{}",
                  in.spot_dia_mm, in.bead_height_mm, in.frequency_khz,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// USW spots are identical in topology to spot_weld bumps; we differentiate
// by the much smaller height (≤ 0.5 mm).  Confidence is modest because
// metadata-less identification is inherently ambiguous.

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

    // Metadata replay.
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

    // Geometric fallback: small additive low-aspect disc with very shallow
    // height (< 0.5 mm).
    const auto edgeFaces = buildEdgeFaceMap(wp.shape());

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double radius = cyl.Radius();
        const double dia    = 2.0 * radius;
        if (dia > 12.0) continue;
        if (dia < 0.8)  continue;

        const gp_Ax1 axis = cyl.Axis();
        const gp_Dir adir = axis.Direction();

        std::vector<gp_Pnt> circleCenters;
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            if (std::abs(std::abs(c.Axis().Direction().Dot(adir)) - 1.0) > 1e-3)
                continue;
            if (std::abs(c.Radius() - radius) > 1e-3) continue;
            circleCenters.push_back(c.Location());
        }
        if (circleCenters.size() < 2) continue;

        auto projOnAxis = [&](const gp_Pnt& p) {
            return (p.X() - axis.Location().X()) * adir.X() +
                   (p.Y() - axis.Location().Y()) * adir.Y() +
                   (p.Z() - axis.Location().Z()) * adir.Z();
        };
        auto cmp = [&](const gp_Pnt& a, const gp_Pnt& b) {
            return projOnAxis(a) < projOnAxis(b);
        };
        const auto minIt = std::min_element(circleCenters.begin(), circleCenters.end(), cmp);
        const auto maxIt = std::max_element(circleCenters.begin(), circleCenters.end(), cmp);
        const gp_Pnt centerLow  = *minIt;
        const gp_Pnt centerHigh = *maxIt;
        const double height = centerHigh.Distance(centerLow);
        if (height <= 1e-6) continue;
        // USW signature: VERY shallow.
        if (height > 0.5) continue;

        // Confirm additive (LOW circle adjacent to LARGE workpiece face).
        const double discArea = M_PI * radius * radius;
        double lowCircleAdjArea  = 0.0;
        double highCircleAdjArea = 0.0;
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            if (!edgeFaces.Contains(e)) continue;
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Pnt cLoc = crv.Circle().Location();
            const bool isLow = (projOnAxis(cLoc) - projOnAxis(centerLow)) <
                               (projOnAxis(centerHigh) - projOnAxis(cLoc));
            double bestArea = 0.0;
            const auto& adj = edgeFaces.FindFromKey(e);
            for (NCollection_List<TopoDS_Shape>::Iterator it(adj); it.More(); it.Next()) {
                const TopoDS_Face& af = TopoDS::Face(it.Value());
                if (af.IsSame(cylFace)) continue;
                if (BRepAdaptor_Surface(af).GetType() != GeomAbs_Plane) continue;
                GProp_GProps gp;
                BRepGProp::SurfaceProperties(af, gp);
                bestArea = std::max(bestArea, gp.Mass());
            }
            if (isLow) lowCircleAdjArea  = std::max(lowCircleAdjArea, bestArea);
            else       highCircleAdjArea = std::max(highCircleAdjArea, bestArea);
        }
        const bool isBump = (lowCircleAdjArea  > discArea * 1.5) &&
                            (highCircleAdjArea > 0.0 &&
                             highCircleAdjArea < discArea * 1.5);
        if (!isBump) continue;

        json recovered = {
            { "position_x_mm",          centerLow.X() },
            { "position_y_mm",          centerLow.Y() },
            { "spot_dia_mm",            dia },
            { "bead_height_mm",         height },
            { "material",               "aluminum" },
            { "vibration_amplitude_um", 20.0 },
            { "frequency_khz",          20.0 },
            { "sheet_thickness_mm",     1.0 },
            { "entry_face_normal",
              { adir.X(), adir.Y(), adir.Z() } },
        };
        json matched = {
            { "cylindrical_face_id", fIdx },
            { "base_center", { centerLow.X(),  centerLow.Y(),  centerLow.Z()  } },
            { "top_center",  { centerHigh.X(), centerHigh.Y(), centerHigh.Z() } },
            { "aspect_ratio", height / dia },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.55, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::ultrasonic_weld
