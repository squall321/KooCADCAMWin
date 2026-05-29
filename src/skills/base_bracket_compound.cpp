// @lat: [[engine/skills#base_bracket_compound]]

#include "base_bracket_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>
#include <vector>

namespace koocadcam::skill::base_bracket_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

// Triangular gusset rib in the XZ plane, extruded along +Y by bracket_width.
TopoDS_Shape makeGussetRib(double legX, double legZ, double width)
{
    BRepBuilderAPI_MakePolygon poly;
    poly.Add(gp_Pnt(0.0,   0.0, 0.0));
    poly.Add(gp_Pnt(legX,  0.0, 0.0));
    poly.Add(gp_Pnt(0.0,   0.0, legZ));
    poly.Close();
    if (!poly.IsDone())
        throw SkillError("base_bracket_compound: gusset polygon build failed");

    BRepBuilderAPI_MakeFace face(poly.Wire(), true);
    if (!face.IsDone())
        throw SkillError("base_bracket_compound: gusset face build failed");

    gp_Vec ext(0.0, width, 0.0);
    BRepPrimAPI_MakePrism prism(face.Shape(), ext);
    prism.Build();
    if (!prism.IsDone())
        throw SkillError("base_bracket_compound: gusset prism build failed");
    return prism.Shape();
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.leg1_mm <= 0.0 || in.leg2_mm <= 0.0 ||
        in.plate_t_mm <= 0.0 || in.bracket_width_mm <= 0.0 ||
        in.bolt_dia_mm <= 0.0 || in.dowel_dia_mm <= 0.0 ||
        in.gusset_size_mm < 0.0)
    {
        r.add("DFM-BRK-INPUT", "error",
              "base_bracket_compound: all dimensions must be > 0 (gusset may be 0)");
        return r;
    }
    if (in.bolt_pattern.first <= 0 || in.bolt_pattern.second <= 0) {
        r.add("DFM-BRK-INPUT", "error",
              "base_bracket_compound: bolt_pattern components must be > 0");
        return r;
    }

    if (in.plate_t_mm < 4.0) {
        r.add("DFM-BRK-PLATE-T", "error",
              "base_bracket_compound: plate_t " + std::to_string(in.plate_t_mm) +
              " mm < 4 mm — sub-structural per DIN 18800 minimum bracket plate");
    }

    // Bolt edge distance check (AISC J3.4 conservative: 1.5 × dia).
    const int nx = in.bolt_pattern.first;
    const int ny = in.bolt_pattern.second;
    const double edgeMin = 1.5 * in.bolt_dia_mm;
    const double avX = in.leg1_mm - 2.0 * edgeMin;
    const double avY = in.bracket_width_mm - 2.0 * edgeMin;
    if (avX < 0.0 || avY < 0.0) {
        r.add("DFM-BRK-BOLT-EDGE", "error",
              "base_bracket_compound: insufficient room for bolts with 1.5×dia "
              "edge distance");
    }
    if (nx > 1 && avX < in.bolt_dia_mm) {
        r.add("DFM-BRK-BOLT-EDGE", "error",
              "base_bracket_compound: bolt grid wider than base plate footprint");
    }
    if (ny > 1 && avY < in.bolt_dia_mm) {
        r.add("DFM-BRK-BOLT-EDGE", "error",
              "base_bracket_compound: bolt grid deeper than base plate footprint");
    }

    // Dowel hole on standing plate ~ (leg2/2, width/2).  Edge ≥ 2 × dia.
    if (in.leg2_mm / 2.0 < 2.0 * in.dowel_dia_mm) {
        r.add("DFM-BRK-DOWEL", "error",
              "base_bracket_compound: dowel edge distance " +
              std::to_string(in.leg2_mm / 2.0) +
              " < 2 × dowel_dia (" + std::to_string(2.0 * in.dowel_dia_mm) +
              " mm)");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "base_bracket_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double L1 = in.leg1_mm;
    const double L2 = in.leg2_mm;
    const double t  = in.plate_t_mm;
    const double W  = in.bracket_width_mm;

    // Sub-feature 1: base plate (XY footprint, t thickness in Z).
    const gp_Pnt baseOrigin(0.0, 0.0, 0.0);
    const TopoDS_Shape basePlate = pr::box(
        gp_Ax2(baseOrigin, gp::DZ()), L1, W, t);

    // Sub-feature 2: standing plate (vertical, thickness t in X, L2 tall in Z).
    const gp_Pnt standOrigin(0.0, 0.0, 0.0);
    const TopoDS_Shape standPlate = pr::box(
        gp_Ax2(standOrigin, gp::DZ()), t, W, L2);

    TopoDS_Shape bracket = pr::fuse(basePlate, standPlate);

    // Sub-feature 8 (built early so we fuse before drilling): gusset rib.
    int hasGusset = 0;
    if (in.gusset_size_mm > 0.0) {
        // Position the rib so its right-angle apex sits at the inside corner
        // (x = t, z = t).  Triangle in XZ plane extruded along +Y for W.
        const double rib = std::min({ in.gusset_size_mm,
                                      L1 - t,
                                      L2 - t });
        if (rib > 0.0) {
            try {
                TopoDS_Shape gusset = makeGussetRib(rib, rib, W);
                // Shift it to start at (t, 0, t) so it touches the inside corner.
                // makeGussetRib produced one with apex at (0,0,0). We need
                // it translated by (+t, 0, +t).
                // Use a direct rebuild instead of a transform:
                BRepBuilderAPI_MakePolygon poly;
                poly.Add(gp_Pnt(t,         0.0, t));
                poly.Add(gp_Pnt(t + rib,   0.0, t));
                poly.Add(gp_Pnt(t,         0.0, t + rib));
                poly.Close();
                if (poly.IsDone()) {
                    BRepBuilderAPI_MakeFace face(poly.Wire(), true);
                    if (face.IsDone()) {
                        BRepPrimAPI_MakePrism prism(face.Shape(),
                                                    gp_Vec(0.0, W, 0.0));
                        prism.Build();
                        if (prism.IsDone()) {
                            bracket = pr::fuse(bracket, prism.Shape());
                            hasGusset = 1;
                        }
                    }
                }
                (void)gusset;
            } catch (...) {
                // Gusset is structural-bonus; skip if OCCT fails.
            }
        }
    }

    // Sub-features 3..6: mounting holes on base plate.
    const int nx = in.bolt_pattern.first;
    const int ny = in.bolt_pattern.second;
    const double r = in.bolt_dia_mm / 2.0;
    const double edgeMin = 1.5 * in.bolt_dia_mm;
    // Start grid slightly inboard of the leg1-tip end so they don't sit under
    // the standing plate.
    const double xStart = std::max(edgeMin, t + edgeMin);
    const double xEnd   = L1 - edgeMin;
    const double yStart = edgeMin;
    const double yEnd   = W - edgeMin;
    const double xStep = (nx > 1) ? ((xEnd - xStart) / (nx - 1)) : 0.0;
    const double yStep = (ny > 1) ? ((yEnd - yStart) / (ny - 1)) : 0.0;

    std::vector<TopoDS_Shape> drills;
    json boltPos = json::array();
    for (int i = 0; i < nx; ++i) {
        for (int j = 0; j < ny; ++j) {
            const double x = xStart + i * xStep;
            const double y = yStart + j * yStep;
            const gp_Ax2 ax(gp_Pnt(x, y, -0.1), gp::DZ());
            drills.push_back(pr::cylinder(ax, r, t + 0.2));
            boltPos.push_back({ { "x", x }, { "y", y } });
        }
    }
    if (!drills.empty()) {
        bracket = pr::cutMany(bracket, drills);
    }

    // Sub-feature 7: alignment dowel hole on standing plate (centered).
    const double dowelY = W / 2.0;
    const double dowelZ = L2 / 2.0;
    const gp_Ax2 dowelAx(gp_Pnt(-0.1, dowelY, dowelZ), gp::DX());
    const TopoDS_Shape dowelHole = pr::cylinder(dowelAx, in.dowel_dia_mm / 2.0, t + 0.2);
    bracket = pr::cut(bracket, dowelHole);

    const int nBolts = nx * ny;
    const int subfeatureCount = 2 + nBolts + 1 + hasGusset;

    json params = {
        { "leg1_mm",          L1 },
        { "leg2_mm",          L2 },
        { "plate_t_mm",       t },
        { "bracket_width_mm", W },
        { "bolt_pattern",     { nx, ny } },
        { "bolt_dia_mm",      in.bolt_dia_mm },
        { "dowel_dia_mm",     in.dowel_dia_mm },
        { "gusset_size_mm",   in.gusset_size_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    subfeatureCount },
        { "leg1_mm",             L1 },
        { "leg2_mm",             L2 },
        { "plate_t_mm",          t },
        { "bracket_width_mm",    W },
        { "n_bolts",             nBolts },
        { "bolt_dia_mm",         in.bolt_dia_mm },
        { "dowel_dia_mm",        in.dowel_dia_mm },
        { "bolt_positions",      boltPos },
        { "dowel_position",      { { "y", dowelY }, { "z", dowelZ } } },
        { "has_gusset",          hasGusset > 0 },
        { "fastener_total",      nBolts + 1 },
    };

    const double baseVol  = L1 * W * t;
    const double standVol = t  * W * L2 - t * W * t;  // subtract shared corner
    const double weightKg = (baseVol + standVol) * 7.85e-6;

    ToolingMeta tooling;
    tooling.tool_type         = "saw_cut;weld_assemble;drill";
    tooling.tool_dia_mm       = std::max(in.bolt_dia_mm, in.dowel_dia_mm);
    tooling.tool_length_mm    = std::max(t, W) + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.08;
    tooling.stock_removed_mm3 = M_PI * (in.bolt_dia_mm / 2.0) *
                                (in.bolt_dia_mm / 2.0) * t * nBolts +
                                M_PI * (in.dowel_dia_mm / 2.0) *
                                (in.dowel_dia_mm / 2.0) * t;
    tooling.est_cycle_time_s  = 90.0 + 4.0 * nBolts;
    tooling.extra = {
        { "subfeature_sequence", {
            { { "name", "base_plate" }     },
            { { "name", "standing_plate" } },
            { { "name", "mounting_holes" }, { "count", nBolts },
              { "dia_mm", in.bolt_dia_mm } },
            { { "name", "dowel_hole" }, { "dia_mm", in.dowel_dia_mm } },
            { { "name", "gusset_rib" }, { "present", hasGusset > 0 } },
        } },
        { "approx_weight_kg",    weightKg },
        { "standard",            "DIN_18800_AISC_J6" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(bracket, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::base_bracket_compound applied: L1={} L2={} t={} bolts={}",
                  L1, L2, t, nBolts);

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

}  // namespace koocadcam::skill::base_bracket_compound
