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

// Slice-2 registered skills (extended catalog)
#include "skills/spot_drill.hpp"
#include "skills/ream.hpp"
#include "skills/mill_open_pocket.hpp"
#include "skills/profile_milling.hpp"
#include "skills/drill_through_hole.hpp"
#include "skills/drill_and_tap.hpp"
#include "skills/bore_and_finish.hpp"
#include "skills/pocket_with_corner_relief.hpp"
#include "skills/mill_keyway.hpp"
#include "skills/dovetail_slot.hpp"
#include "skills/T_slot.hpp"
#include "skills/undercut_milling.hpp"
#include "skills/tap_thread.hpp"
#include "skills/thread_mill.hpp"
#include "skills/engrave_text.hpp"
#include "skills/engrave_path.hpp"
#include "skills/face_milling.hpp"
#include "skills/bore_cylindrical.hpp"
#include "skills/bore_with_shelf.hpp"
#include "skills/hollow_cavity.hpp"

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <array>

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

// Cylinder-axis face datum parser, used by skills whose datum addresses an
// existing cylindrical bore (ream, tap_thread, thread_mill).
//
// JSON form:
//   "existing_hole_datum": { "axis": [x, y, z], "tolerance_deg": <double> }
//   "existing_hole_datum_face_id": <int>
//
// TODO: full cylinder-axis search also needs a position_x/y/z anchor; for
// slice-2 we parse only the direction and place the axis through the origin.
// FaceCylinderByAxis uses gp_Ax1 (origin + direction); the resolver matches
// against direction only, so the origin field is unused in the current
// resolution path.  Callers that need precise positioning should fall back
// to FaceIdRef.
sk::FaceDatum parseCylinderAxisDatum(const json& params)
{
    if (params.contains("existing_hole_datum_face_id") &&
        params["existing_hole_datum_face_id"].is_number_integer()) {
        return sk::FaceIdRef{ params["existing_hole_datum_face_id"].get<int>() };
    }
    if (params.contains("existing_hole_datum") && params["existing_hole_datum"].is_object()) {
        const json& d = params["existing_hole_datum"];
        if (d.contains("axis") && d["axis"].is_array() && d["axis"].size() == 3 &&
            d["axis"][0].is_number() && d["axis"][1].is_number() && d["axis"][2].is_number())
        {
            double x = d["axis"][0].get<double>();
            double y = d["axis"][1].get<double>();
            double z = d["axis"][2].get<double>();
            const double m = std::sqrt(x*x + y*y + z*z);
            if (m > 1e-9) {
                const double tol = jdouble(d, "tolerance_deg", 5.0);
                return sk::FaceCylinderByAxis{ gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(x, y, z)), tol };
            }
        }
    }
    // TODO: when datum JSON is incomplete, fall back to the largest planar
    // face; the skill's resolver will fail cleanly when no cylinder matches.
    return sk::FaceLargestPlanar{};
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

// ── Slice-2 parsers ──────────────────────────────────────────────────────

sk::spot_drill::Input parseSpotDrill(const json& p)
{
    sk::spot_drill::Input in;
    in.entry_face     = parseFaceDatum(p);
    in.position_x_mm  = jdouble(p, "position_x_mm",  0.0);
    in.position_y_mm  = jdouble(p, "position_y_mm",  0.0);
    in.axis_dir       = parseAxisDir(p);
    in.diameter_mm    = jdouble(p, "diameter_mm",    0.0);
    in.cone_angle_deg = jdouble(p, "cone_angle_deg", 90.0);
    return in;
}

sk::ream::Input parseReam(const json& p)
{
    sk::ream::Input in;
    in.existing_hole_datum = parseCylinderAxisDatum(p);
    in.enlarge_by_mm       = jdouble(p, "enlarge_by_mm", 0.0);
    return in;
}

sk::mill_open_pocket::Input parseMillOpenPocket(const json& p)
{
    sk::mill_open_pocket::Input in;
    in.entry_face     = parseFaceDatum(p);
    in.axis_dir       = parseAxisDir(p);
    in.position_x_mm  = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm  = jdouble(p, "position_y_mm", 0.0);
    // open_direction defaults to (1, 0, 0); only override if JSON supplies a
    // valid 3-vector under "open_direction".
    if (p.contains("open_direction") && p["open_direction"].is_array() &&
        p["open_direction"].size() == 3 &&
        p["open_direction"][0].is_number() &&
        p["open_direction"][1].is_number() &&
        p["open_direction"][2].is_number())
    {
        const double x = p["open_direction"][0].get<double>();
        const double y = p["open_direction"][1].get<double>();
        const double z = p["open_direction"][2].get<double>();
        if (std::sqrt(x*x + y*y + z*z) > 1e-9) {
            in.open_direction = gp_Dir(x, y, z);
        }
    }
    in.length_mm      = jdouble(p, "length_mm",   0.0);
    in.width_mm       = jdouble(p, "width_mm",    0.0);
    in.depth_mm       = jdouble(p, "depth_mm",    0.0);
    in.corner_r_mm    = jdouble(p, "corner_r_mm", 1.0);
    return in;
}

sk::profile_milling::Input parseProfileMilling(const json& p)
{
    sk::profile_milling::Input in;
    in.entry_face    = parseFaceDatum(p);
    in.axis_dir      = parseAxisDir(p);
    in.offset_in_mm  = jdouble(p, "offset_in_mm", 1.0);
    in.depth_mm      = jdouble(p, "depth_mm",     0.0);
    return in;
}

sk::drill_through_hole::Input parseDrillThroughHole(const json& p)
{
    sk::drill_through_hole::Input in;
    in.entry_face    = parseFaceDatum(p);
    in.position_x_mm = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm = jdouble(p, "position_y_mm", 0.0);
    in.axis_dir      = parseAxisDir(p);
    in.diameter_mm   = jdouble(p, "diameter_mm",   0.0);
    return in;
}

sk::drill_and_tap::Input parseDrillAndTap(const json& p)
{
    sk::drill_and_tap::Input in;
    in.entry_face           = parseFaceDatum(p);
    in.position_x_mm        = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm        = jdouble(p, "position_y_mm", 0.0);
    in.axis_dir             = parseAxisDir(p);
    in.thread_size          = jstring(p, "thread_size", "M3");
    in.tap_depth_mm         = jdouble(p, "tap_depth_mm", 0.0);
    in.pilot_extra_depth_mm = jdouble(p, "pilot_extra_depth_mm", 1.0);
    in.through_hole         = jbool  (p, "through_hole", false);
    return in;
}

sk::bore_and_finish::Input parseBoreAndFinish(const json& p)
{
    sk::bore_and_finish::Input in;
    in.entry_face        = parseFaceDatum(p);
    in.position_x_mm     = jdouble(p, "position_x_mm",     0.0);
    in.position_y_mm     = jdouble(p, "position_y_mm",     0.0);
    in.axis_dir          = parseAxisDir(p);
    in.final_diameter_mm = jdouble(p, "final_diameter_mm", 0.0);
    in.depth_mm          = jdouble(p, "depth_mm",          0.0);
    in.tolerance_class   = jstring(p, "tolerance_class",   "H7");
    return in;
}

sk::pocket_with_corner_relief::Input parsePocketWithCornerRelief(const json& p)
{
    sk::pocket_with_corner_relief::Input in;
    in.entry_face            = parseFaceDatum(p);
    in.center_x_mm           = jdouble(p, "center_x_mm",           0.0);
    in.center_y_mm           = jdouble(p, "center_y_mm",           0.0);
    in.axis_dir              = parseAxisDir(p);
    in.length_mm             = jdouble(p, "length_mm",             0.0);
    in.width_mm              = jdouble(p, "width_mm",              0.0);
    in.depth_mm              = jdouble(p, "depth_mm",              0.0);
    in.corner_relief_dia_mm  = jdouble(p, "corner_relief_dia_mm",  0.0);
    return in;
}

sk::mill_keyway::Input parseMillKeyway(const json& p)
{
    sk::mill_keyway::Input in;
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

sk::dovetail_slot::Input parseDovetailSlot(const json& p)
{
    sk::dovetail_slot::Input in;
    in.entry_face       = parseFaceDatum(p);
    in.start_x_mm       = jdouble(p, "start_x_mm",      0.0);
    in.start_y_mm       = jdouble(p, "start_y_mm",      0.0);
    in.end_x_mm         = jdouble(p, "end_x_mm",        0.0);
    in.end_y_mm         = jdouble(p, "end_y_mm",        0.0);
    in.axis_dir         = parseAxisDir(p);
    in.top_width_mm     = jdouble(p, "top_width_mm",    0.0);
    in.bottom_width_mm  = jdouble(p, "bottom_width_mm", 0.0);
    in.depth_mm         = jdouble(p, "depth_mm",        0.0);
    in.wall_angle_deg   = jdouble(p, "wall_angle_deg",  30.0);
    return in;
}

sk::T_slot::Input parseTSlot(const json& p)
{
    sk::T_slot::Input in;
    in.entry_face      = parseFaceDatum(p);
    in.start_x_mm      = jdouble(p, "start_x_mm",      0.0);
    in.start_y_mm      = jdouble(p, "start_y_mm",      0.0);
    in.end_x_mm        = jdouble(p, "end_x_mm",        0.0);
    in.end_y_mm        = jdouble(p, "end_y_mm",        0.0);
    in.axis_dir        = parseAxisDir(p);
    in.neck_width_mm   = jdouble(p, "neck_width_mm",   0.0);
    in.flange_width_mm = jdouble(p, "flange_width_mm", 0.0);
    in.neck_depth_mm   = jdouble(p, "neck_depth_mm",   0.0);
    in.flange_depth_mm = jdouble(p, "flange_depth_mm", 0.0);
    return in;
}

sk::undercut_milling::Input parseUndercutMilling(const json& p)
{
    sk::undercut_milling::Input in;
    in.entry_face       = parseFaceDatum(p);
    in.position_y_mm    = jdouble(p, "position_y_mm",    0.0);
    in.position_z_mm    = jdouble(p, "position_z_mm",    0.0);
    in.pocket_width_mm  = jdouble(p, "pocket_width_mm",  0.0);
    in.pocket_depth_mm  = jdouble(p, "pocket_depth_mm",  0.0);
    in.pocket_height_mm = jdouble(p, "pocket_height_mm", 0.0);
    return in;
}

sk::tap_thread::Input parseTapThread(const json& p)
{
    sk::tap_thread::Input in;
    in.existing_hole_datum = parseCylinderAxisDatum(p);
    in.thread_size         = jstring(p, "thread_size",     "M3");
    in.pitch_mm            = jdouble(p, "pitch_mm",        0.5);
    in.thread_depth_mm     = jdouble(p, "thread_depth_mm", 0.0);
    return in;
}

sk::thread_mill::Input parseThreadMill(const json& p)
{
    sk::thread_mill::Input in;
    in.existing_hole_datum = parseCylinderAxisDatum(p);
    in.thread_size         = jstring(p, "thread_size",     "M3");
    in.pitch_mm            = jdouble(p, "pitch_mm",        0.5);
    in.thread_depth_mm     = jdouble(p, "thread_depth_mm", 0.0);
    in.is_external         = jbool  (p, "is_external",     false);
    in.tool_dia_mm         = jdouble(p, "tool_dia_mm",     1.0);
    return in;
}

sk::engrave_text::Input parseEngraveText(const json& p)
{
    sk::engrave_text::Input in;
    in.entry_face      = parseFaceDatum(p);
    in.position_x_mm   = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm   = jdouble(p, "position_y_mm", 0.0);
    // direction defaults to (1, 0, 0) (text baseline +X); only override on
    // a valid JSON 3-vector under "direction".
    if (p.contains("direction") && p["direction"].is_array() &&
        p["direction"].size() == 3 &&
        p["direction"][0].is_number() &&
        p["direction"][1].is_number() &&
        p["direction"][2].is_number())
    {
        const double x = p["direction"][0].get<double>();
        const double y = p["direction"][1].get<double>();
        const double z = p["direction"][2].get<double>();
        if (std::sqrt(x*x + y*y + z*z) > 1e-9) {
            in.direction = gp_Dir(x, y, z);
        }
    }
    in.text            = jstring(p, "text", "");
    in.font_size_mm    = jdouble(p, "font_size_mm",    2.0);
    in.stroke_width_mm = jdouble(p, "stroke_width_mm", 0.2);
    in.depth_mm        = jdouble(p, "depth_mm",        0.15);
    return in;
}

sk::engrave_path::Input parseEngravePath(const json& p)
{
    sk::engrave_path::Input in;
    in.entry_face = parseFaceDatum(p);
    in.width_mm   = jdouble(p, "width_mm", 0.2);
    in.depth_mm   = jdouble(p, "depth_mm", 0.15);
    // waypoints: array of [x, y] pairs.
    if (p.contains("waypoints") && p["waypoints"].is_array()) {
        for (const auto& wp : p["waypoints"]) {
            if (wp.is_array() && wp.size() == 2 &&
                wp[0].is_number() && wp[1].is_number()) {
                in.waypoints.push_back({ wp[0].get<double>(), wp[1].get<double>() });
            }
        }
    }
    return in;
}

sk::face_milling::Input parseFaceMilling(const json& p)
{
    sk::face_milling::Input in;
    in.entry_face = parseFaceDatum(p);
    in.depth_mm   = jdouble(p, "depth_mm", 0.0);
    return in;
}

sk::bore_cylindrical::Input parseBoreCylindrical(const json& p)
{
    sk::bore_cylindrical::Input in;
    in.entry_face      = parseFaceDatum(p);
    in.position_x_mm   = jdouble(p, "position_x_mm",   0.0);
    in.position_y_mm   = jdouble(p, "position_y_mm",   0.0);
    in.axis_dir        = parseAxisDir(p);
    in.diameter_mm     = jdouble(p, "diameter_mm",     0.0);
    in.depth_mm        = jdouble(p, "depth_mm",        0.0);
    in.tolerance_class = jstring(p, "tolerance_class", "H7");
    return in;
}

sk::bore_with_shelf::Input parseBoreWithShelf(const json& p)
{
    sk::bore_with_shelf::Input in;
    in.entry_face     = parseFaceDatum(p);
    in.position_x_mm  = jdouble(p, "position_x_mm",  0.0);
    in.position_y_mm  = jdouble(p, "position_y_mm",  0.0);
    in.axis_dir       = parseAxisDir(p);
    in.upper_dia_mm   = jdouble(p, "upper_dia_mm",   0.0);
    in.upper_depth_mm = jdouble(p, "upper_depth_mm", 0.0);
    in.lower_dia_mm   = jdouble(p, "lower_dia_mm",   0.0);
    in.lower_depth_mm = jdouble(p, "lower_depth_mm", 0.0);
    return in;
}

sk::hollow_cavity::Input parseHollowCavity(const json& p)
{
    sk::hollow_cavity::Input in;
    in.entry_face        = parseFaceDatum(p);
    in.wall_thickness_mm = jdouble(p, "wall_thickness_mm", 0.0);
    in.depth_mm          = jdouble(p, "depth_mm",          0.0);
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

    // ── Slice-2 expansion: full catalog ──────────────────────────────────
    t[sk::spot_drill::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::spot_drill::apply(wp, parseSpotDrill(p));
    };
    t[sk::ream::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::ream::apply(wp, parseReam(p));
    };
    t[sk::mill_open_pocket::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::mill_open_pocket::apply(wp, parseMillOpenPocket(p));
    };
    t[sk::profile_milling::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::profile_milling::apply(wp, parseProfileMilling(p));
    };
    t[sk::drill_through_hole::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::drill_through_hole::apply(wp, parseDrillThroughHole(p));
    };
    t[sk::drill_and_tap::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::drill_and_tap::apply(wp, parseDrillAndTap(p));
    };
    t[sk::bore_and_finish::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::bore_and_finish::apply(wp, parseBoreAndFinish(p));
    };
    t[sk::pocket_with_corner_relief::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::pocket_with_corner_relief::apply(wp, parsePocketWithCornerRelief(p));
    };
    t[sk::mill_keyway::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::mill_keyway::apply(wp, parseMillKeyway(p));
    };
    t[sk::dovetail_slot::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::dovetail_slot::apply(wp, parseDovetailSlot(p));
    };
    t[sk::T_slot::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::T_slot::apply(wp, parseTSlot(p));
    };
    t[sk::undercut_milling::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::undercut_milling::apply(wp, parseUndercutMilling(p));
    };
    t[sk::tap_thread::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::tap_thread::apply(wp, parseTapThread(p));
    };
    t[sk::thread_mill::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::thread_mill::apply(wp, parseThreadMill(p));
    };
    t[sk::engrave_text::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::engrave_text::apply(wp, parseEngraveText(p));
    };
    t[sk::engrave_path::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::engrave_path::apply(wp, parseEngravePath(p));
    };
    t[sk::face_milling::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::face_milling::apply(wp, parseFaceMilling(p));
    };
    t[sk::bore_cylindrical::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::bore_cylindrical::apply(wp, parseBoreCylindrical(p));
    };
    t[sk::bore_with_shelf::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::bore_with_shelf::apply(wp, parseBoreWithShelf(p));
    };
    t[sk::hollow_cavity::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::hollow_cavity::apply(wp, parseHollowCavity(p));
    };

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
            // Mirror the cumulative history into the step's output workpiece
            // so wp.features() reflects the full chain (each skill::apply()
            // only adds its OWN signature; prior steps would otherwise be
            // lost).  Push the new signature first, then sync the wp.
            result.signatures.push_back(out.signature);
            out.workpiece->setFeatures(result.signatures);
            current = out.workpiece;
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
