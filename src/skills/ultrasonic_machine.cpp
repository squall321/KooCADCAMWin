// @lat: [[engine/skills#ultrasonic_machine]]

#include "ultrasonic_machine.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>

namespace koocadcam::skill::ultrasonic_machine {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

constexpr const char* kShapeSquare = "square";
constexpr const char* kShapeRound  = "round";

// Hard-coded brittle-material catalog for USM applicability.  Heat-treatment
// catalog has its own; here we list materials where USM is the textbook
// process choice.  Out-of-list materials produce an info finding (NOT an
// error) — USM on a borderline material is the user's call.
bool isBrittleMaterial(const std::string& m)
{
    static const std::array<const char*, 11> kBrittle = {
        "glass_soda_lime",
        "glass_borosilicate",
        "fused_silica",
        "alumina",
        "zirconia",
        "silicon_nitride",
        "silicon_wafer",
        "tungsten_carbide",
        "ruby",
        "sapphire",
        "ferrite",
    };
    for (const auto* k : kBrittle)
        if (m == k) return true;
    return false;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.shape != kShapeSquare && in.shape != kShapeRound) {
        r.add("DFM-INPUT", "error",
              "ultrasonic_machine shape '" + in.shape +
              "' not recognised — use 'square' or 'round'");
        return r;
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "ultrasonic_machine depth_mm must be > 0");
    }
    if (in.shape == kShapeSquare) {
        if (in.length_mm <= 0.0 || in.width_mm <= 0.0) {
            r.add("DFM-INPUT", "error",
                  "ultrasonic_machine square pocket needs length_mm > 0 and width_mm > 0");
        }
    } else if (in.shape == kShapeRound) {
        if (in.diameter_mm <= 0.0) {
            r.add("DFM-INPUT", "error",
                  "ultrasonic_machine round pocket needs diameter_mm > 0");
        }
    }

    if (in.depth_mm > 10.0) {
        r.add("DFM-USM-DEPTH", "warning",
              "ultrasonic_machine depth_mm " + std::to_string(in.depth_mm) +
              " > 10 mm — abrasive slurry circulation degrades sharply with depth");
    }

    const std::string& mat = in.material.empty() ? wp.material() : in.material;
    if (!isBrittleMaterial(mat)) {
        r.add("DFM-USM-MATERIAL", "info",
              "ultrasonic_machine: material '" + mat +
              "' is not in the brittle-material catalog — USM is typically "
              "applied to hard/brittle materials (glass, ceramic, carbide); "
              "consider milling or EDM for ductile metals");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "ultrasonic_machine DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("ultrasonic_machine: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const double kOverhang = 0.05;
    TopoDS_Shape cutter;
    double stockRemoved = 0.0;

    if (in.shape == kShapeSquare) {
        const gp_Pnt bottomCenter(in.position_x_mm, in.position_y_mm,
                                  zMax - in.depth_mm);
        // Sharp corners — USM tool reproduces its own geometry exactly.
        cutter = pr::roundedRectPocketTool(
            bottomCenter, in.length_mm, in.width_mm,
            in.depth_mm + kOverhang, 0.0);
        stockRemoved = in.length_mm * in.width_mm * in.depth_mm;
    } else {
        // Round: cylinder cutter from above the entry face down.
        const gp_Pnt toolStart(in.position_x_mm, in.position_y_mm, zMax + kOverhang);
        const gp_Ax2 toolAx(toolStart, gp_Dir(0.0, 0.0, -1.0));
        cutter = pr::cylinder(toolAx, in.diameter_mm / 2.0, in.depth_mm + kOverhang);
        stockRemoved = M_PI * (in.diameter_mm / 2.0) * (in.diameter_mm / 2.0)
                     * in.depth_mm;
    }

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    const std::string mat = in.material.empty() ? wp.material() : in.material;

    json params = {
        { "entry_face_id",  *entryId },
        { "position_x_mm",  in.position_x_mm },
        { "position_y_mm",  in.position_y_mm },
        { "shape",          in.shape },
        { "length_mm",      in.length_mm },
        { "width_mm",       in.width_mm },
        { "diameter_mm",    in.diameter_mm },
        { "depth_mm",       in.depth_mm },
        { "material",       mat },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "shape",                      in.shape },
        { "bottom_planar_face_present", true },
        { "depth_mm",                   in.depth_mm },
        { "material",                   mat },
        { "material_brittle",           isBrittleMaterial(mat) },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "usm_horn";
    tooling.tool_dia_mm       = (in.shape == kShapeRound)
                                ? in.diameter_mm
                                : std::min(in.length_mm, in.width_mm);
    tooling.tool_length_mm    = in.depth_mm + 5.0;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = stockRemoved;
    // USM MRR: ~ 1 mm³/s on glass, 0.3 on alumina; conservative 0.5.
    tooling.est_cycle_time_s  = std::max(15.0, stockRemoved / 0.5);
    tooling.extra["dfm_findings"]        = findings;
    tooling.extra["abrasive_grit"]       = "B4C-220";
    tooling.extra["vibration_freq_khz"]  = 24.0;
    tooling.extra["machining_constraint"] =
        "USM — abrasive slurry process; brittle/hard materials only; "
        "no thermal/electrical constraints";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::ultrasonic_machine applied: shape={} depth={} mat={} faces {}→{}",
                  in.shape, in.depth_mm, mat,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// USM cavities are geometrically identical to milled / EDM pockets — the
// difference is process physics + material class.  Use metadata-replay
// only (confidence 1.0).

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::ultrasonic_machine
