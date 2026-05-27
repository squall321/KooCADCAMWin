// @lat: [[engine/skills#Layer 3 Executor]]

#include "Executor.hpp"

#include "skills/Workpiece.hpp"
#include "skills/Datum.hpp"

// Slice-1 registered skills
#include "skills/drill_hole.hpp"
#include "skills/counterbore.hpp"
#include "skills/countersink.hpp"
#include "skills/mill_circular_pocket.hpp"
#include "skills/mill_rect_pocket.hpp"
#include "skills/mill_slot.hpp"
#include "skills/fillet_edge.hpp"
#include "skills/chamfer_edge.hpp"

#include <gp_Dir.hxx>

#include <spdlog/spdlog.h>

#include <cmath>
#include <exception>

namespace koocadcam::process {

namespace sk = koocadcam::skill;
using nlohmann::json;

// ─────────────────────────────────────────────────────────────────────────
// JSON datum helpers
// ─────────────────────────────────────────────────────────────────────────
//
// JSON entry-face encoding (shared by all skills that take an entry_face):
//
//   "entry_face":     "top"       → FaceByNormal{ gp_Dir(0,0,1) }    (+Z)
//   "entry_face":     "bottom"    → FaceByNormal{ gp_Dir(0,0,-1) }   (-Z)
//   "entry_face_id":  <int>       → FaceIdRef{<int>}
//   (none of the above)           → FaceLargestPlanar{}
//
// Future datum kinds (FaceByRay, FaceTopAtXY, FaceCylinderByAxis, …) are
// TODO; their JSON shorthand is reserved but not yet parsed.

namespace {

sk::FaceDatum parseFaceDatum(const json& params)
{
    if (params.contains("entry_face_id") && params["entry_face_id"].is_number_integer()) {
        return sk::FaceIdRef{ params["entry_face_id"].get<int>() };
    }
    if (params.contains("entry_face") && params["entry_face"].is_string()) {
        const std::string s = params["entry_face"].get<std::string>();
        if (s == "top")    return sk::FaceByNormal{ gp_Dir(0.0, 0.0, 1.0),  5.0, "largest" };
        if (s == "bottom") return sk::FaceByNormal{ gp_Dir(0.0, 0.0, -1.0), 5.0, "largest" };
    }
    return sk::FaceLargestPlanar{};
}

// Convenience: pull a double-typed field with a default.
double jdouble(const json& j, const char* key, double dflt)
{
    if (j.contains(key) && j[key].is_number()) {
        return j[key].get<double>();
    }
    return dflt;
}

// Convenience: pull a bool-typed field with a default.
bool jbool(const json& j, const char* key, bool dflt)
{
    if (j.contains(key) && j[key].is_boolean()) {
        return j[key].get<bool>();
    }
    return dflt;
}

// Convenience: pull a string-typed field with a default.
std::string jstring(const json& j, const char* key, const char* dflt)
{
    if (j.contains(key) && j[key].is_string()) {
        return j[key].get<std::string>();
    }
    return dflt;
}

// Convenience: parse a 3-element axis_dir array; default (0, 0, -1).
gp_Dir parseAxisDir(const json& j, const char* key = "axis_dir")
{
    if (j.contains(key) && j[key].is_array() && j[key].size() == 3 &&
        j[key][0].is_number() && j[key][1].is_number() && j[key][2].is_number())
    {
        double x = j[key][0].get<double>();
        double y = j[key][1].get<double>();
        double z = j[key][2].get<double>();
        const double m = std::sqrt(x*x + y*y + z*z);
        if (m > 1e-9) {
            return gp_Dir(x, y, z);
        }
    }
    return gp_Dir(0.0, 0.0, -1.0);
}

// ─────────────────────────────────────────────────────────────────────────
// Per-skill JSON → Input parsers
// ─────────────────────────────────────────────────────────────────────────
//
// Each parser keys match the FeatureSignature.params shape that the
// corresponding skill emits, so a plan can round-trip through a synthesised
// workpiece's feature history with minimal renaming.

sk::drill_hole::Input parseDrillHole(const json& p)
{
    sk::drill_hole::Input in;
    in.entry_face    = parseFaceDatum(p);
    in.position_x_mm = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm = jdouble(p, "position_y_mm", 0.0);
    in.axis_dir      = parseAxisDir(p);
    in.diameter_mm   = jdouble(p, "diameter_mm", 0.0);
    in.depth_mm      = jdouble(p, "depth_mm",    0.0);
    in.through_hole  = jbool  (p, "through_hole", false);
    return in;
}

sk::counterbore::Input parseCounterbore(const json& p)
{
    sk::counterbore::Input in;
    in.entry_face     = parseFaceDatum(p);
    in.position_x_mm  = jdouble(p, "position_x_mm",  0.0);
    in.position_y_mm  = jdouble(p, "position_y_mm",  0.0);
    in.axis_dir       = parseAxisDir(p);
    in.pilot_dia_mm   = jdouble(p, "pilot_dia_mm",   0.0);
    in.pilot_depth_mm = jdouble(p, "pilot_depth_mm", 0.0);
    in.seat_dia_mm    = jdouble(p, "seat_dia_mm",    0.0);
    in.seat_depth_mm  = jdouble(p, "seat_depth_mm",  0.0);
    return in;
}

sk::countersink::Input parseCountersink(const json& p)
{
    sk::countersink::Input in;
    in.entry_face      = parseFaceDatum(p);
    in.position_x_mm   = jdouble(p, "position_x_mm",   0.0);
    in.position_y_mm   = jdouble(p, "position_y_mm",   0.0);
    in.axis_dir        = parseAxisDir(p);
    in.pilot_dia_mm    = jdouble(p, "pilot_dia_mm",    0.0);
    in.pilot_depth_mm  = jdouble(p, "pilot_depth_mm",  0.0);
    in.cone_top_dia_mm = jdouble(p, "cone_top_dia_mm", 0.0);
    in.cone_angle_deg  = jdouble(p, "cone_angle_deg",  90.0);
    return in;
}

sk::mill_circular_pocket::Input parseMillCircularPocket(const json& p)
{
    sk::mill_circular_pocket::Input in;
    in.entry_face         = parseFaceDatum(p);
    in.position_x_mm      = jdouble(p, "position_x_mm",      0.0);
    in.position_y_mm      = jdouble(p, "position_y_mm",      0.0);
    in.axis_dir           = parseAxisDir(p);
    in.diameter_mm        = jdouble(p, "diameter_mm",        0.0);
    in.depth_mm           = jdouble(p, "depth_mm",           0.0);
    in.bottom_corner_r_mm = jdouble(p, "bottom_corner_r_mm", 0.0);
    return in;
}

sk::mill_rect_pocket::Input parseMillRectPocket(const json& p)
{
    sk::mill_rect_pocket::Input in;
    in.entry_face   = parseFaceDatum(p);
    in.center_x_mm  = jdouble(p, "center_x_mm", 0.0);
    in.center_y_mm  = jdouble(p, "center_y_mm", 0.0);
    in.axis_dir     = parseAxisDir(p);
    in.length_mm    = jdouble(p, "length_mm",   0.0);
    in.width_mm     = jdouble(p, "width_mm",    0.0);
    in.depth_mm     = jdouble(p, "depth_mm",    0.0);
    in.corner_r_mm  = jdouble(p, "corner_r_mm", 1.0);
    return in;
}

sk::mill_slot::Input parseMillSlot(const json& p)
{
    sk::mill_slot::Input in;
    in.entry_face = parseFaceDatum(p);
    in.start_x_mm = jdouble(p, "start_x_mm", 0.0);
    in.start_y_mm = jdouble(p, "start_y_mm", 0.0);
    in.end_x_mm   = jdouble(p, "end_x_mm",   0.0);
    in.end_y_mm   = jdouble(p, "end_y_mm",   0.0);
    in.axis_dir   = parseAxisDir(p);
    in.width_mm   = jdouble(p, "width_mm",   0.0);
    in.depth_mm   = jdouble(p, "depth_mm",   0.0);
    return in;
}

// Edge selector parser for fillet_edge / chamfer_edge.
//
// JSON form:
//   "edges_at_z_mm":  <double>, "tolerance_mm": <double>   → Z-band selector
//   (default if missing)                                   → Z-band at z=0
sk::fillet_edge::EdgeSelector parseEdgeSelector(const json& p)
{
    sk::fillet_edge::EdgesAtZBand band;
    band.z_mm         = jdouble(p, "edges_at_z_mm", 0.0);
    band.tolerance_mm = jdouble(p, "tolerance_mm",  1e-3);
    return band;
}

sk::fillet_edge::Input parseFilletEdge(const json& p)
{
    sk::fillet_edge::Input in;
    in.edge_selector = parseEdgeSelector(p);
    in.radius_mm     = jdouble(p, "radius_mm", 0.0);
    return in;
}

sk::chamfer_edge::Input parseChamferEdge(const json& p)
{
    sk::chamfer_edge::Input in;
    in.edge_selector   = parseEdgeSelector(p);
    in.chamfer_size_mm = jdouble(p, "chamfer_size_mm", 0.0);
    in.angle_deg       = jdouble(p, "angle_deg",       45.0);
    return in;
}

// ─────────────────────────────────────────────────────────────────────────
// Dispatch table
// ─────────────────────────────────────────────────────────────────────────

std::unordered_map<std::string, Executor::SkillFn> buildDispatchTable()
{
    std::unordered_map<std::string, Executor::SkillFn> t;

    t[sk::drill_hole::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::drill_hole::apply(wp, parseDrillHole(p));
    };
    t[sk::counterbore::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::counterbore::apply(wp, parseCounterbore(p));
    };
    t[sk::countersink::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::countersink::apply(wp, parseCountersink(p));
    };
    t[sk::mill_circular_pocket::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::mill_circular_pocket::apply(wp, parseMillCircularPocket(p));
    };
    t[sk::mill_rect_pocket::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::mill_rect_pocket::apply(wp, parseMillRectPocket(p));
    };
    t[sk::mill_slot::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::mill_slot::apply(wp, parseMillSlot(p));
    };
    t[sk::fillet_edge::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::fillet_edge::apply(wp, parseFilletEdge(p));
    };
    t[sk::chamfer_edge::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::chamfer_edge::apply(wp, parseChamferEdge(p));
    };

    // TODO (slice-2 expansion): face_milling, bore_cylindrical,
    // bore_with_shelf, hollow_cavity, spot_drill, ream, mill_open_pocket,
    // profile_milling, drill_through_hole, drill_and_tap, bore_and_finish,
    // pocket_with_corner_relief, mill_keyway, dovetail_slot, T_slot,
    // undercut_milling, tap_thread, thread_mill, engrave_text, engrave_path.
    return t;
}

}  // namespace

const std::unordered_map<std::string, Executor::SkillFn>& Executor::dispatchTable()
{
    static const std::unordered_map<std::string, SkillFn> table = buildDispatchTable();
    return table;
}

// ─────────────────────────────────────────────────────────────────────────
// Executor::execute
// ─────────────────────────────────────────────────────────────────────────

ExecutionResult Executor::execute(const ProcessPlan& plan,
                                  std::shared_ptr<sk::Workpiece> initial_workpiece)
{
    ExecutionResult result;
    result.workpiece = initial_workpiece;

    if (!initial_workpiece) {
        result.errors.push_back("Executor::execute: initial_workpiece is null");
        result.failedAtStep = 0;
        return result;
    }

    const auto& table = dispatchTable();
    auto current = initial_workpiece;

    const auto& steps = plan.steps();
    for (int i = 0; i < static_cast<int>(steps.size()); ++i) {
        const auto& step = steps[i];

        const auto it = table.find(step.skill_id);
        if (it == table.end()) {
            std::string msg = "Executor: unknown skill_id '" + step.skill_id +
                              "' at step " + std::to_string(i);
            spdlog::error("{}", msg);
            result.errors.push_back(msg);
            result.failedAtStep = i;
            result.workpiece = current;   // last good
            return result;
        }

        try {
            sk::SkillOutput out = it->second(*current, step.params);
            if (!out.workpiece) {
                std::string msg = "Executor: skill '" + step.skill_id +
                                  "' returned null workpiece at step " +
                                  std::to_string(i);
                result.errors.push_back(msg);
                result.failedAtStep = i;
                result.workpiece = current;
                return result;
            }
            current = out.workpiece;
            result.signatures.push_back(out.signature);
        } catch (const sk::SkillError& e) {
            std::string msg = "Executor: SkillError at step " + std::to_string(i) +
                              " (" + step.skill_id + "): " + e.what();
            spdlog::error("{}", msg);
            result.errors.push_back(msg);
            result.failedAtStep = i;
            result.workpiece = current;
            return result;
        } catch (const std::exception& e) {
            std::string msg = "Executor: exception at step " + std::to_string(i) +
                              " (" + step.skill_id + "): " + e.what();
            spdlog::error("{}", msg);
            result.errors.push_back(msg);
            result.failedAtStep = i;
            result.workpiece = current;
            return result;
        }
    }

    result.workpiece = current;
    result.failedAtStep = -1;
    return result;
}

}  // namespace koocadcam::process
