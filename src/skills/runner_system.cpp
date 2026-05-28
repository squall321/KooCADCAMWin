// @lat: [[engine/skills#runner_system]]

#include "runner_system.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::runner_system {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

double segLen(const Segment& s)
{
    return std::hypot(std::hypot(s.end_x_mm - s.start_x_mm,
                                 s.end_y_mm - s.start_y_mm),
                      s.end_z_mm - s.start_z_mm);
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.segments.empty()) {
        r.add("DFM-RUNNER-N", "error",
              "runner_system requires at least one segment");
    }

    if (in.dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "runner_system dia_mm must be > 0");
    } else if (in.dia_mm < 2.0 || in.dia_mm > 8.0) {
        r.add("DFM-RUNNER-DIA", "error",
              "runner_system dia " + std::to_string(in.dia_mm) +
              " mm outside [2.0, 8.0] mm typical range");
    }

    for (size_t i = 0; i < in.segments.size(); ++i) {
        const double L = segLen(in.segments[i]);
        if (L < 1e-6) {
            r.add("DFM-INPUT", "error",
                  "runner_system segment " + std::to_string(i) +
                  " has zero length");
        } else if (in.dia_mm > 0.0 && L < in.dia_mm * 3.0) {
            r.add("DFM-RUNNER-LEN", "warning",
                  "runner_system segment " + std::to_string(i) +
                  " length " + std::to_string(L) +
                  " < 3 × dia — too short to be effective");
        }
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "runner_system DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double radius = in.dia_mm / 2.0;

    // Build one cylindrical cutter per segment, then fuse them into a single
    // compound BEFORE the cut (per Constraints: "For multi-cylinder cuts:
    // fuse first then cut").  cutMany packs all tools into one compound and
    // performs a single BRepAlgoAPI_Cut — equivalent and faster than N cuts.
    std::vector<TopoDS_Shape> cutters;
    cutters.reserve(in.segments.size());
    for (const auto& s : in.segments) {
        const gp_Pnt p0(s.start_x_mm, s.start_y_mm, s.start_z_mm);
        const gp_Pnt p1(s.end_x_mm,   s.end_y_mm,   s.end_z_mm);
        gp_Vec v(p0, p1);
        const double L = v.Magnitude();
        if (L < 1e-9) continue;
        v.Normalize();
        const gp_Dir dir(v.X(), v.Y(), v.Z());
        // gp_Ax2 V = local Z (pitfall).  Cylinder grows along `dir`.
        const gp_Ax2 ax(p0, dir);
        cutters.push_back(pr::cylinder(ax, radius, L));
    }
    if (cutters.empty()) {
        throw SkillError("runner_system: all segments degenerate");
    }

    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), cutters);

    json segArr = json::array();
    double totalVol = 0.0;
    for (const auto& s : in.segments) {
        const double L = segLen(s);
        totalVol += M_PI * radius * radius * L;
        segArr.push_back({
            { "start_x_mm", s.start_x_mm },
            { "start_y_mm", s.start_y_mm },
            { "start_z_mm", s.start_z_mm },
            { "end_x_mm",   s.end_x_mm },
            { "end_y_mm",   s.end_y_mm },
            { "end_z_mm",   s.end_z_mm },
            { "length_mm",  L },
        });
    }

    json params = {
        { "dia_mm",   in.dia_mm },
        { "segments", segArr },
    };
    json pattern = {
        { "kind",          kSkillId },
        { "dia_mm",        in.dia_mm },
        { "segment_count", static_cast<int>(in.segments.size()) },
    };
    ToolingMeta tooling;
    tooling.tool_type        = "mold_runner";
    tooling.tool_dia_mm      = in.dia_mm;
    tooling.stock_removed_mm3 = totalVol;
    tooling.est_cycle_time_s  = 0.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::runner_system applied: segments={} dia={} faces {}→{}",
                  in.segments.size(), in.dia_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "dia_mm",   f.params.value("dia_mm", 0.0) },
            { "segments", f.params.value("segments", json::array()) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 0.95, json{ { "source", "feature_history" } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::runner_system
