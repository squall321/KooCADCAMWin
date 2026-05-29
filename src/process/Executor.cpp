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

// ── Slice-6 / 7 / 8 expansion (geometric-effect skills only) ──────────────
// Lathe / turning
#include "skills/turn_external.hpp"
#include "skills/turn_internal.hpp"
#include "skills/face_turn.hpp"
#include "skills/groove_turn.hpp"
#include "skills/taper_turn.hpp"
#include "skills/drill_lathe.hpp"
#include "skills/contour_turn.hpp"
#include "skills/form_turn.hpp"
#include "skills/keyway_external.hpp"
// Milling / drilling variants
#include "skills/spot_face.hpp"
#include "skills/deep_hole_peck.hpp"
#include "skills/gun_drill.hpp"
#include "skills/micro_drill.hpp"
#include "skills/rigid_tap.hpp"
#include "skills/plunge_milling.hpp"
#include "skills/plunge_roughing.hpp"
#include "skills/trochoidal_mill.hpp"
#include "skills/adaptive_clearing.hpp"
#include "skills/rest_milling.hpp"
#include "skills/multi_step_bore.hpp"
#include "skills/edm_drill.hpp"
#include "skills/wire_edm.hpp"
// Sheet / forming
#include "skills/piercing.hpp"
#include "skills/sheet_punch.hpp"
#include "skills/bend.hpp"
#include "skills/flange.hpp"
#include "skills/hem.hpp"
#include "skills/notch.hpp"
#include "skills/emboss.hpp"
#include "skills/coining.hpp"
#include "skills/draw_bead.hpp"
// Separation cutting
#include "skills/_separation_common.hpp"
#include "skills/saw_cut.hpp"
#include "skills/laser_cut.hpp"
#include "skills/plasma_cut.hpp"
#include "skills/waterjet_cut.hpp"
#include "skills/oxyfuel_cut.hpp"
// Welding / fastening (compound geometric)
#include "skills/plug_weld_hole.hpp"
#include "skills/slot_weld_slot.hpp"
#include "skills/full_penetration_butt_prep.hpp"
#include "skills/partial_pen_fillet_prep.hpp"
#include "skills/rivet_hole.hpp"
#include "skills/blind_fastener.hpp"
#include "skills/clinch_nut.hpp"
#include "skills/threaded_insert.hpp"
#include "skills/heli_coil.hpp"
// Surfacing / mold
#include "skills/offset_surface.hpp"
#include "skills/mold_boss.hpp"
#include "skills/mold_rib.hpp"
#include "skills/mold_gate.hpp"
#include "skills/ejector_pin_hole.hpp"
#include "skills/cooling_channel.hpp"
#include "skills/runner_system.hpp"
// Forging (geometric)
#include "skills/open_die_forge.hpp"
#include "skills/upsetting.hpp"
#include "skills/flow_form.hpp"
// Compound features — bearings & seats
#include "skills/bearing_seat.hpp"
#include "skills/radial_bearing_seat_with_snapring.hpp"
#include "skills/sealed_bearing_seat_with_shield_relief.hpp"
#include "skills/needle_bearing_seat_press_fit.hpp"
#include "skills/thrust_bearing_seat_compound.hpp"
// Compound features — fastener seats
#include "skills/socket_head_bolt_seat.hpp"
#include "skills/countersunk_bolt_seat.hpp"
#include "skills/captive_nut_pocket.hpp"
#include "skills/set_screw_anti_rotation_pocket.hpp"
#include "skills/helicoil_pilot_compound.hpp"
#include "skills/threaded_npt_port.hpp"
#include "skills/jic_flare_port_seat.hpp"
#include "skills/banjo_fitting_seat.hpp"
#include "skills/manifold_cross_drill_compound.hpp"
// Compound features — drive components
#include "skills/shaft_with_keyway_step.hpp"
#include "skills/pulley_with_keyway_compound.hpp"
#include "skills/gear_blank_with_hub_step.hpp"
#include "skills/sprocket_blank_with_bore.hpp"
#include "skills/flywheel_blank_with_balance_drills.hpp"
#include "skills/cam_lobe.hpp"
#include "skills/gear_tooth_cut.hpp"
#include "skills/rack_tooth_cut.hpp"
// Compound features — seals / grooves
#include "skills/o_ring_groove_face.hpp"
#include "skills/o_ring_groove_radial.hpp"
#include "skills/lip_seal_seat.hpp"
#include "skills/caseback_o_ring_groove.hpp"
#include "skills/sapphire_glass_seat.hpp"
// Compound features — bushings / mounting
#include "skills/eccentric_bushing_seat.hpp"
#include "skills/vibration_isolator_seat.hpp"
#include "skills/rubber_grommet_seat.hpp"
#include "skills/magnetic_latch_pocket.hpp"
#include "skills/concealed_hinge_cup.hpp"
#include "skills/butt_hinge_pocket.hpp"
#include "skills/pcb_standoff_threaded.hpp"
#include "skills/usb_c_port_cutout.hpp"
#include "skills/shim_pocket.hpp"
#include "skills/undercut_relief.hpp"
#include "skills/datum_face_establish.hpp"

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

// ── Slice-6 / 7 / 8 parsers ──────────────────────────────────────────────
//
// All of the parsers below accept the same JSON shorthand for FaceDatum
// (parseFaceDatum) and axis directions (parseAxisDir).  Skills that don't
// take an entry face (open_die_forge, upsetting, flow_form, the lathe
// rebuild skills, etc.) read scalar fields directly.

// Helper: parse a generic 3D direction with a custom default.
gp_Dir parseDir(const json& p, const char* key, const gp_Dir& dflt)
{
    if (p.contains(key) && p[key].is_array() && p[key].size() == 3 &&
        p[key][0].is_number() && p[key][1].is_number() && p[key][2].is_number()) {
        const double x = p[key][0].get<double>();
        const double y = p[key][1].get<double>();
        const double z = p[key][2].get<double>();
        if (std::sqrt(x*x + y*y + z*z) > 1e-9) {
            return gp_Dir(x, y, z);
        }
    }
    return dflt;
}

// ── Lathe / turning ──────────────────────────────────────────────────────

sk::turn_external::Input parseTurnExternal(const json& p)
{
    sk::turn_external::Input in;
    in.start_z_mm      = jdouble(p, "start_z_mm",   0.0);
    in.end_z_mm        = jdouble(p, "end_z_mm",     0.0);
    in.final_dia_mm    = jdouble(p, "final_dia_mm", 0.0);
    if (p.contains("roughing_passes") && p["roughing_passes"].is_number_integer()) {
        in.roughing_passes = p["roughing_passes"].get<int>();
    }
    return in;
}

sk::turn_internal::Input parseTurnInternal(const json& p)
{
    sk::turn_internal::Input in;
    in.start_z_mm            = jdouble(p, "start_z_mm",           0.0);
    in.end_z_mm              = jdouble(p, "end_z_mm",             0.0);
    in.final_dia_mm          = jdouble(p, "final_dia_mm",         0.0);
    in.existing_bore_dia_mm  = jdouble(p, "existing_bore_dia_mm", 0.0);
    return in;
}

sk::face_turn::Input parseFaceTurn(const json& p)
{
    sk::face_turn::Input in;
    in.face_z_mm      = jdouble(p, "face_z_mm",      0.0);
    in.surface_finish = jstring(p, "surface_finish", "ra_1.6");
    return in;
}

sk::groove_turn::Input parseGrooveTurn(const json& p)
{
    sk::groove_turn::Input in;
    in.center_z_mm = jdouble(p, "center_z_mm", 0.0);
    in.width_mm    = jdouble(p, "width_mm",    0.0);
    in.depth_mm    = jdouble(p, "depth_mm",    0.0);
    return in;
}

sk::taper_turn::Input parseTaperTurn(const json& p)
{
    sk::taper_turn::Input in;
    in.z0_mm = jdouble(p, "z0_mm", 0.0);
    in.r0_mm = jdouble(p, "r0_mm", 0.0);
    in.z1_mm = jdouble(p, "z1_mm", 0.0);
    in.r1_mm = jdouble(p, "r1_mm", 0.0);
    return in;
}

sk::drill_lathe::Input parseDrillLathe(const json& p)
{
    sk::drill_lathe::Input in;
    in.diameter_mm = jdouble(p, "diameter_mm", 0.0);
    in.depth_mm    = jdouble(p, "depth_mm",    0.0);
    return in;
}

sk::contour_turn::Input parseContourTurn(const json& p)
{
    sk::contour_turn::Input in;
    if (p.contains("profile") && p["profile"].is_array()) {
        for (const auto& pt : p["profile"]) {
            if (pt.is_object()) {
                sk::contour_turn::ProfilePoint pp;
                pp.z_mm = jdouble(pt, "z_mm", 0.0);
                pp.r_mm = jdouble(pt, "r_mm", 0.0);
                in.profile.push_back(pp);
            }
        }
    }
    if (p.contains("pass_count") && p["pass_count"].is_number_integer()) {
        in.pass_count = p["pass_count"].get<int>();
    }
    return in;
}

sk::form_turn::Input parseFormTurn(const json& p)
{
    sk::form_turn::Input in;
    if (p.contains("profile") && p["profile"].is_array()) {
        for (const auto& pt : p["profile"]) {
            if (pt.is_object()) {
                sk::form_turn::ProfilePoint pp;
                pp.z_mm = jdouble(pt, "z_mm", 0.0);
                pp.r_mm = jdouble(pt, "r_mm", 0.0);
                in.profile.push_back(pp);
            }
        }
    }
    return in;
}

sk::keyway_external::Input parseKeywayExternal(const json& p)
{
    sk::keyway_external::Input in;
    in.entry_face        = parseFaceDatum(p);
    in.shaft_axis        = parseDir(p, "shaft_axis", gp_Dir(0.0, 0.0, 1.0));
    in.shaft_center_x_mm = jdouble(p, "shaft_center_x_mm", 0.0);
    in.shaft_center_y_mm = jdouble(p, "shaft_center_y_mm", 0.0);
    in.shaft_radius_mm   = jdouble(p, "shaft_radius_mm",   5.0);
    in.angle_rad         = jdouble(p, "angle_rad",         0.0);
    in.axial_center_mm   = jdouble(p, "axial_center_mm",   0.0);
    in.length_mm         = jdouble(p, "length_mm",         0.0);
    in.width_mm          = jdouble(p, "width_mm",          0.0);
    in.depth_mm          = jdouble(p, "depth_mm",          0.0);
    return in;
}

// ── Milling / drilling variants ──────────────────────────────────────────

sk::spot_face::Input parseSpotFace(const json& p)
{
    sk::spot_face::Input in;
    in.entry_face    = parseFaceDatum(p);
    in.position_x_mm = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm = jdouble(p, "position_y_mm", 0.0);
    in.axis_dir      = parseAxisDir(p);
    in.dia_mm        = jdouble(p, "dia_mm",        0.0);
    in.depth_mm      = jdouble(p, "depth_mm",      0.0);
    return in;
}

sk::deep_hole_peck::Input parseDeepHolePeck(const json& p)
{
    sk::deep_hole_peck::Input in;
    in.entry_face    = parseFaceDatum(p);
    in.position_x_mm = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm = jdouble(p, "position_y_mm", 0.0);
    in.axis_dir      = parseAxisDir(p);
    in.diameter_mm   = jdouble(p, "diameter_mm",   0.0);
    in.depth_mm      = jdouble(p, "depth_mm",      0.0);
    in.through_hole  = jbool  (p, "through_hole",  false);
    in.peck_depth_mm = jdouble(p, "peck_depth_mm", 0.0);
    return in;
}

sk::gun_drill::Input parseGunDrill(const json& p)
{
    sk::gun_drill::Input in;
    in.entry_face         = parseFaceDatum(p);
    in.position_x_mm      = jdouble(p, "position_x_mm",      0.0);
    in.position_y_mm      = jdouble(p, "position_y_mm",      0.0);
    in.axis_dir           = parseAxisDir(p);
    in.diameter_mm        = jdouble(p, "diameter_mm",        0.0);
    in.depth_mm           = jdouble(p, "depth_mm",           0.0);
    in.through_hole       = jbool  (p, "through_hole",       false);
    in.straightness_class = jstring(p, "straightness_class", "0.05_per_100mm");
    return in;
}

sk::micro_drill::Input parseMicroDrill(const json& p)
{
    sk::micro_drill::Input in;
    in.entry_face    = parseFaceDatum(p);
    in.position_x_mm = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm = jdouble(p, "position_y_mm", 0.0);
    in.axis_dir      = parseAxisDir(p);
    in.diameter_mm   = jdouble(p, "diameter_mm",   0.0);
    in.depth_mm      = jdouble(p, "depth_mm",      0.0);
    in.through_hole  = jbool  (p, "through_hole",  false);
    return in;
}

sk::rigid_tap::Input parseRigidTap(const json& p)
{
    sk::rigid_tap::Input in;
    in.existing_hole_datum = parseCylinderAxisDatum(p);
    in.thread_size         = jstring(p, "thread_size",     "M3");
    in.pitch_mm            = jdouble(p, "pitch_mm",        0.5);
    in.thread_depth_mm     = jdouble(p, "thread_depth_mm", 0.0);
    return in;
}

sk::plunge_milling::Input parsePlungeMilling(const json& p)
{
    sk::plunge_milling::Input in;
    in.entry_face    = parseFaceDatum(p);
    in.position_x_mm = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm = jdouble(p, "position_y_mm", 0.0);
    in.axis_dir      = parseAxisDir(p);
    in.dia_mm        = jdouble(p, "dia_mm",        0.0);
    in.depth_mm      = jdouble(p, "depth_mm",      0.0);
    return in;
}

sk::plunge_roughing::Input parsePlungeRoughing(const json& p)
{
    sk::plunge_roughing::Input in;
    in.entry_face         = parseFaceDatum(p);
    in.center_x_mm        = jdouble(p, "center_x_mm",        0.0);
    in.center_y_mm        = jdouble(p, "center_y_mm",        0.0);
    in.axis_dir           = parseAxisDir(p);
    in.length_mm          = jdouble(p, "length_mm",          0.0);
    in.width_mm           = jdouble(p, "width_mm",           0.0);
    in.depth_mm           = jdouble(p, "depth_mm",           0.0);
    in.corner_r_mm        = jdouble(p, "corner_r_mm",        1.0);
    in.tool_dia_mm        = jdouble(p, "tool_dia_mm",        6.0);
    in.plunge_stepover_mm = jdouble(p, "plunge_stepover_mm", 0.0);
    return in;
}

sk::trochoidal_mill::Input parseTrochoidalMill(const json& p)
{
    sk::trochoidal_mill::Input in;
    in.entry_face  = parseFaceDatum(p);
    in.center_x_mm = jdouble(p, "center_x_mm", 0.0);
    in.center_y_mm = jdouble(p, "center_y_mm", 0.0);
    in.axis_dir    = parseAxisDir(p);
    in.length_mm   = jdouble(p, "length_mm",   0.0);
    in.width_mm    = jdouble(p, "width_mm",    0.0);
    in.depth_mm    = jdouble(p, "depth_mm",    0.0);
    in.corner_r_mm = jdouble(p, "corner_r_mm", 1.0);
    in.tool_dia_mm = jdouble(p, "tool_dia_mm", 6.0);
    in.stepover_mm = jdouble(p, "stepover_mm", 0.0);
    return in;
}

sk::adaptive_clearing::Input parseAdaptiveClearing(const json& p)
{
    sk::adaptive_clearing::Input in;
    in.entry_face             = parseFaceDatum(p);
    in.center_x_mm            = jdouble(p, "center_x_mm",            0.0);
    in.center_y_mm            = jdouble(p, "center_y_mm",            0.0);
    in.axis_dir               = parseAxisDir(p);
    in.length_mm              = jdouble(p, "length_mm",              0.0);
    in.width_mm               = jdouble(p, "width_mm",               0.0);
    in.depth_mm               = jdouble(p, "depth_mm",               0.0);
    in.corner_r_mm            = jdouble(p, "corner_r_mm",            1.0);
    in.tool_dia_mm            = jdouble(p, "tool_dia_mm",            6.0);
    in.stepover_mm            = jdouble(p, "stepover_mm",            0.0);
    in.optimal_engagement_pct = jdouble(p, "optimal_engagement_pct", 30.0);
    return in;
}

sk::rest_milling::Input parseRestMilling(const json& p)
{
    sk::rest_milling::Input in;
    in.entry_face            = parseFaceDatum(p);
    in.center_x_mm           = jdouble(p, "center_x_mm",           0.0);
    in.center_y_mm           = jdouble(p, "center_y_mm",           0.0);
    in.axis_dir              = parseAxisDir(p);
    in.length_mm             = jdouble(p, "length_mm",             0.0);
    in.width_mm              = jdouble(p, "width_mm",              0.0);
    in.depth_mm              = jdouble(p, "depth_mm",              0.0);
    in.previous_tool_dia_mm  = jdouble(p, "previous_tool_dia_mm",  0.0);
    in.cleanup_tool_dia_mm   = jdouble(p, "cleanup_tool_dia_mm",   0.0);
    return in;
}

sk::multi_step_bore::Input parseMultiStepBore(const json& p)
{
    sk::multi_step_bore::Input in;
    in.entry_face    = parseFaceDatum(p);
    in.position_x_mm = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm = jdouble(p, "position_y_mm", 0.0);
    in.axis_dir      = parseAxisDir(p);
    if (p.contains("steps") && p["steps"].is_array()) {
        for (const auto& s : p["steps"]) {
            if (s.is_object()) {
                sk::multi_step_bore::Step step;
                step.dia_mm   = jdouble(s, "dia_mm",   0.0);
                step.depth_mm = jdouble(s, "depth_mm", 0.0);
                in.steps.push_back(step);
            }
        }
    }
    return in;
}

sk::edm_drill::Input parseEdmDrill(const json& p)
{
    sk::edm_drill::Input in;
    in.entry_face    = parseFaceDatum(p);
    in.position_x_mm = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm = jdouble(p, "position_y_mm", 0.0);
    in.axis_dir      = parseAxisDir(p);
    in.diameter_mm   = jdouble(p, "diameter_mm",   0.0);
    in.depth_mm      = jdouble(p, "depth_mm",      0.0);
    in.through_hole  = jbool  (p, "through_hole",  false);
    return in;
}

sk::wire_edm::Input parseWireEdm(const json& p)
{
    sk::wire_edm::Input in;
    in.entry_face = parseFaceDatum(p);
    in.axis_dir   = parseAxisDir(p);
    if (p.contains("contour_waypoints") && p["contour_waypoints"].is_array()) {
        for (const auto& wp : p["contour_waypoints"]) {
            if (wp.is_array() && wp.size() == 2 &&
                wp[0].is_number() && wp[1].is_number()) {
                in.contour_waypoints.push_back({ wp[0].get<double>(), wp[1].get<double>() });
            }
        }
    }
    return in;
}

// ── Sheet / forming ──────────────────────────────────────────────────────

sk::piercing::Input parsePiercing(const json& p)
{
    sk::piercing::Input in;
    in.entry_face    = parseFaceDatum(p);
    in.axis_dir      = parseAxisDir(p);
    in.position_x_mm = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm = jdouble(p, "position_y_mm", 0.0);
    in.diameter_mm   = jdouble(p, "diameter_mm",   0.0);
    in.length_mm     = jdouble(p, "length_mm",     0.0);
    in.width_mm      = jdouble(p, "width_mm",      0.0);
    in.clearance_mm  = jdouble(p, "clearance_mm",  0.0);
    const std::string s = jstring(p, "shape", "circle");
    in.shape = (s == "rectangle") ? sk::piercing::Shape::Rectangle : sk::piercing::Shape::Circle;
    return in;
}

sk::sheet_punch::Input parseSheetPunch(const json& p)
{
    sk::sheet_punch::Input in;
    in.entry_face    = parseFaceDatum(p);
    in.position_x_mm = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm = jdouble(p, "position_y_mm", 0.0);
    in.dia_mm        = jdouble(p, "dia_mm",        0.0);
    in.axis_dir      = parseAxisDir(p);
    return in;
}

sk::bend::Input parseBend(const json& p)
{
    sk::bend::Input in;
    if (p.contains("bend_line_start_xy") && p["bend_line_start_xy"].is_array() &&
        p["bend_line_start_xy"].size() == 2) {
        in.bend_line_start_xy = {
            p["bend_line_start_xy"][0].get<double>(),
            p["bend_line_start_xy"][1].get<double>()
        };
    }
    if (p.contains("bend_line_end_xy") && p["bend_line_end_xy"].is_array() &&
        p["bend_line_end_xy"].size() == 2) {
        in.bend_line_end_xy = {
            p["bend_line_end_xy"][0].get<double>(),
            p["bend_line_end_xy"][1].get<double>()
        };
    }
    in.angle_deg      = jdouble(p, "angle_deg",      90.0);
    in.bend_radius_mm = jdouble(p, "bend_radius_mm", 1.0);
    return in;
}

sk::flange::Input parseFlange(const json& p)
{
    sk::flange::Input in;
    in.edge_selector   = sk::flange::edgeSideFromString(jstring(p, "edge_selector", "x_max"));
    in.flange_width_mm = jdouble(p, "flange_width_mm", 5.0);
    in.bend_radius_mm  = jdouble(p, "bend_radius_mm",  1.0);
    return in;
}

sk::hem::Input parseHem(const json& p)
{
    sk::hem::Input in;
    in.edge_selector = sk::flange::edgeSideFromString(jstring(p, "edge_selector", "x_max"));
    in.hem_width_mm  = jdouble(p, "hem_width_mm", 4.0);
    in.flat_hem      = jbool  (p, "flat_hem",     true);
    return in;
}

sk::notch::Input parseNotch(const json& p)
{
    sk::notch::Input in;
    in.edge_selector = sk::flange::edgeSideFromString(jstring(p, "edge_selector", "x_max"));
    in.shape         = sk::notch::shapeFromString    (jstring(p, "shape",         "rectangle"));
    in.position_mm   = jdouble(p, "position_mm", 0.0);
    in.width_mm      = jdouble(p, "width_mm",    0.0);
    in.depth_mm      = jdouble(p, "depth_mm",    0.0);
    return in;
}

sk::emboss::Input parseEmboss(const json& p)
{
    sk::emboss::Input in;
    in.position_x_mm = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm = jdouble(p, "position_y_mm", 0.0);
    in.shape         = sk::emboss::shapeFromString(jstring(p, "shape", "circle"));
    in.diameter_mm   = jdouble(p, "diameter_mm",   0.0);
    in.length_mm     = jdouble(p, "length_mm",     0.0);
    in.width_mm      = jdouble(p, "width_mm",      0.0);
    in.height_mm     = jdouble(p, "height_mm",     0.0);
    in.corner_r_mm   = jdouble(p, "corner_r_mm",   0.0);
    return in;
}

sk::coining::Input parseCoining(const json& p)
{
    sk::coining::Input in;
    in.position_x_mm = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm = jdouble(p, "position_y_mm", 0.0);
    in.shape         = sk::coining::shapeFromString(jstring(p, "shape", "circle"));
    in.diameter_mm   = jdouble(p, "diameter_mm",   0.0);
    in.length_mm     = jdouble(p, "length_mm",     0.0);
    in.width_mm      = jdouble(p, "width_mm",      0.0);
    in.corner_r_mm   = jdouble(p, "corner_r_mm",   0.0);
    in.depth_mm      = jdouble(p, "depth_mm",      0.0);
    return in;
}

sk::draw_bead::Input parseDrawBead(const json& p)
{
    sk::draw_bead::Input in;
    if (p.contains("waypoints") && p["waypoints"].is_array()) {
        for (const auto& wp : p["waypoints"]) {
            if (wp.is_array() && wp.size() == 2 &&
                wp[0].is_number() && wp[1].is_number()) {
                in.waypoints.push_back({ wp[0].get<double>(), wp[1].get<double>() });
            }
        }
    }
    in.width_mm = jdouble(p, "width_mm", 0.0);
    in.depth_mm = jdouble(p, "depth_mm", 0.0);
    return in;
}

// ── Separation cutting (shared LinearCutInput) ───────────────────────────

sk::separation_common::LinearCutInput parseLinearCut(const json& p)
{
    sk::separation_common::LinearCutInput in;
    in.entry_face = parseFaceDatum(p);
    using K = sk::separation_common::LinearCutInput::Kind;
    const std::string k = jstring(p, "cut_kind", "Linear");
    if (k == "Circular") in.kind = K::Circular;
    else if (k == "Polyline") in.kind = K::Polyline;
    else in.kind = K::Linear;

    in.start_x_mm      = jdouble(p, "start_x_mm",      0.0);
    in.start_y_mm      = jdouble(p, "start_y_mm",      0.0);
    in.end_x_mm        = jdouble(p, "end_x_mm",        0.0);
    in.end_y_mm        = jdouble(p, "end_y_mm",        0.0);
    in.cx_mm           = jdouble(p, "cx_mm",           0.0);
    in.cy_mm           = jdouble(p, "cy_mm",           0.0);
    in.radius_mm       = jdouble(p, "radius_mm",       0.0);
    in.start_angle_deg = jdouble(p, "start_angle_deg", 0.0);
    in.end_angle_deg   = jdouble(p, "end_angle_deg",   0.0);
    in.cut_through_depth_mm = jdouble(p, "cut_through_depth_mm", 0.0);

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

// ── Welding / fastening ──────────────────────────────────────────────────

sk::plug_weld_hole::Input parsePlugWeldHole(const json& p)
{
    sk::plug_weld_hole::Input in;
    in.entry_face    = parseFaceDatum(p);
    in.position_x_mm = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm = jdouble(p, "position_y_mm", 0.0);
    in.axis_dir      = parseAxisDir(p);
    in.hole_dia      = jdouble(p, "hole_dia",     6.0);
    in.cs_top_dia    = jdouble(p, "cs_top_dia",   0.0);
    in.cs_angle_deg  = jdouble(p, "cs_angle_deg", 90.0);
    return in;
}

sk::slot_weld_slot::Input parseSlotWeldSlot(const json& p)
{
    sk::slot_weld_slot::Input in;
    in.entry_face    = parseFaceDatum(p);
    in.position_x_mm = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm = jdouble(p, "position_y_mm", 0.0);
    in.axis_dir      = parseAxisDir(p);
    in.slot_l        = jdouble(p, "slot_l", 0.0);
    in.slot_w        = jdouble(p, "slot_w", 0.0);
    in.slot_dir      = parseDir(p, "slot_dir", gp_Dir(1.0, 0.0, 0.0));
    return in;
}

sk::full_penetration_butt_prep::Input parseFullPenetrationButtPrep(const json& p)
{
    sk::full_penetration_butt_prep::Input in;
    in.entry_face       = parseFaceDatum(p);
    in.edge_dir         = parseDir(p, "edge_dir", gp_Dir(1.0, 0.0, 0.0));
    in.plate_t          = jdouble(p, "plate_t",          0.0);
    in.groove_angle     = jdouble(p, "groove_angle",     60.0);
    in.root_face_mm     = jdouble(p, "root_face_mm",     1.5);
    in.edge_center_x_mm = jdouble(p, "edge_center_x_mm", 0.0);
    in.edge_center_y_mm = jdouble(p, "edge_center_y_mm", 0.0);
    in.edge_length_mm   = jdouble(p, "edge_length_mm",   50.0);
    return in;
}

sk::partial_pen_fillet_prep::Input parsePartialPenFilletPrep(const json& p)
{
    sk::partial_pen_fillet_prep::Input in;
    in.entry_face       = parseFaceDatum(p);
    in.edge_dir         = parseDir(p, "edge_dir", gp_Dir(1.0, 0.0, 0.0));
    in.plate_t          = jdouble(p, "plate_t",          0.0);
    in.bevel_angle      = jdouble(p, "bevel_angle",      45.0);
    in.land_mm          = jdouble(p, "land_mm",          1.5);
    in.edge_center_x_mm = jdouble(p, "edge_center_x_mm", 0.0);
    in.edge_center_y_mm = jdouble(p, "edge_center_y_mm", 0.0);
    in.edge_length_mm   = jdouble(p, "edge_length_mm",   50.0);
    return in;
}

sk::rivet_hole::Input parseRivetHole(const json& p)
{
    sk::rivet_hole::Input in;
    in.entry_face           = parseFaceDatum(p);
    in.position_x_mm        = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm        = jdouble(p, "position_y_mm", 0.0);
    in.axis_dir             = parseAxisDir(p);
    in.diameter_mm          = jdouble(p, "diameter_mm", 0.0);
    in.rivet_diameter_class = jstring(p, "rivet_diameter_class", "");
    return in;
}

sk::blind_fastener::Input parseBlindFastener(const json& p)
{
    sk::blind_fastener::Input in;
    in.entry_face      = parseFaceDatum(p);
    in.position_x_mm   = jdouble(p, "position_x_mm",   0.0);
    in.position_y_mm   = jdouble(p, "position_y_mm",   0.0);
    in.axis_dir        = parseAxisDir(p);
    in.nominal_dia_mm  = jdouble(p, "nominal_dia_mm",  0.0);
    in.grip_length_mm  = jdouble(p, "grip_length_mm",  0.0);
    return in;
}

sk::clinch_nut::Input parseClinchNut(const json& p)
{
    sk::clinch_nut::Input in;
    in.entry_face         = parseFaceDatum(p);
    in.position_x_mm      = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm      = jdouble(p, "position_y_mm", 0.0);
    in.axis_dir           = parseAxisDir(p);
    in.thread_size        = jstring(p, "thread_size", "M3");
    in.sheet_thickness_mm = jdouble(p, "sheet_thickness_mm", 0.0);
    return in;
}

sk::threaded_insert::Input parseThreadedInsert(const json& p)
{
    sk::threaded_insert::Input in;
    in.entry_face        = parseFaceDatum(p);
    in.position_x_mm     = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm     = jdouble(p, "position_y_mm", 0.0);
    in.axis_dir          = parseAxisDir(p);
    in.thread_size       = jstring(p, "thread_size", "M3");
    in.insert_length_mm  = jdouble(p, "insert_length_mm", 0.0);
    in.insert_material   = jstring(p, "insert_material", "stainless_316");
    return in;
}

sk::heli_coil::Input parseHeliCoil(const json& p)
{
    sk::heli_coil::Input in;
    in.entry_face        = parseFaceDatum(p);
    in.position_x_mm     = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm     = jdouble(p, "position_y_mm", 0.0);
    in.axis_dir          = parseAxisDir(p);
    in.thread_size       = jstring(p, "thread_size", "M3");
    in.insert_length_mm  = jdouble(p, "insert_length_mm", 0.0);
    in.insert_material   = jstring(p, "insert_material", "stainless_316");
    in.coil_pitch_mm     = jdouble(p, "coil_pitch_mm", 0.0);
    if (p.contains("coil_count") && p["coil_count"].is_number_integer()) {
        in.coil_count = p["coil_count"].get<int>();
    }
    return in;
}

// ── Surfacing / mold ─────────────────────────────────────────────────────

sk::offset_surface::Input parseOffsetSurface(const json& p)
{
    sk::offset_surface::Input in;
    // target_face uses the same face-datum shorthand as entry_face.
    in.target_face = parseFaceDatum(p);
    in.offset_mm   = jdouble(p, "offset_mm", 1.0);
    return in;
}

sk::mold_boss::Input parseMoldBoss(const json& p)
{
    sk::mold_boss::Input in;
    in.entry_face      = parseFaceDatum(p);
    in.position_x_mm   = jdouble(p, "position_x_mm",   0.0);
    in.position_y_mm   = jdouble(p, "position_y_mm",   0.0);
    in.height_mm       = jdouble(p, "height_mm",       0.0);
    in.outer_dia_mm    = jdouble(p, "outer_dia_mm",    0.0);
    in.inner_dia_mm    = jdouble(p, "inner_dia_mm",    0.0);
    in.draft_angle_deg = jdouble(p, "draft_angle_deg", 1.0);
    return in;
}

sk::mold_rib::Input parseMoldRib(const json& p)
{
    sk::mold_rib::Input in;
    in.entry_face      = parseFaceDatum(p);
    in.start_x_mm      = jdouble(p, "start_x_mm",      0.0);
    in.start_y_mm      = jdouble(p, "start_y_mm",      0.0);
    in.end_x_mm        = jdouble(p, "end_x_mm",        0.0);
    in.end_y_mm        = jdouble(p, "end_y_mm",        0.0);
    in.height_mm       = jdouble(p, "height_mm",       0.0);
    in.thickness_mm    = jdouble(p, "thickness_mm",    0.0);
    in.draft_angle_deg = jdouble(p, "draft_angle_deg", 1.0);
    return in;
}

sk::mold_gate::Input parseMoldGate(const json& p)
{
    sk::mold_gate::Input in;
    in.entry_face      = parseFaceDatum(p);
    in.position_x_mm   = jdouble(p, "position_x_mm",   0.0);
    in.position_y_mm   = jdouble(p, "position_y_mm",   0.0);
    in.gate_type       = jstring(p, "gate_type",       "pin_gate");
    in.dia_or_width_mm = jdouble(p, "dia_or_width_mm", 0.0);
    in.length_mm       = jdouble(p, "length_mm",       0.0);
    return in;
}

sk::ejector_pin_hole::Input parseEjectorPinHole(const json& p)
{
    sk::ejector_pin_hole::Input in;
    in.entry_face    = parseFaceDatum(p);
    in.position_x_mm = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm = jdouble(p, "position_y_mm", 0.0);
    in.dia_mm        = jdouble(p, "dia_mm",        0.0);
    in.depth_mm      = jdouble(p, "depth_mm",      0.0);
    return in;
}

sk::cooling_channel::Input parseCoolingChannel(const json& p)
{
    sk::cooling_channel::Input in;
    if (p.contains("waypoints") && p["waypoints"].is_array()) {
        for (const auto& wp : p["waypoints"]) {
            if (wp.is_object()) {
                sk::cooling_channel::Waypoint w;
                w.x_mm = jdouble(wp, "x_mm", 0.0);
                w.y_mm = jdouble(wp, "y_mm", 0.0);
                w.z_mm = jdouble(wp, "z_mm", 0.0);
                in.waypoints.push_back(w);
            }
        }
    }
    in.dia_mm = jdouble(p, "dia_mm", 0.0);
    return in;
}

sk::runner_system::Input parseRunnerSystem(const json& p)
{
    sk::runner_system::Input in;
    if (p.contains("segments") && p["segments"].is_array()) {
        for (const auto& s : p["segments"]) {
            if (s.is_object()) {
                sk::runner_system::Segment seg;
                seg.start_x_mm = jdouble(s, "start_x_mm", 0.0);
                seg.start_y_mm = jdouble(s, "start_y_mm", 0.0);
                seg.start_z_mm = jdouble(s, "start_z_mm", 0.0);
                seg.end_x_mm   = jdouble(s, "end_x_mm",   0.0);
                seg.end_y_mm   = jdouble(s, "end_y_mm",   0.0);
                seg.end_z_mm   = jdouble(s, "end_z_mm",   0.0);
                in.segments.push_back(seg);
            }
        }
    }
    in.dia_mm = jdouble(p, "dia_mm", 0.0);
    return in;
}

// ── Forging (geometric) ──────────────────────────────────────────────────

sk::open_die_forge::Input parseOpenDieForge(const json& p)
{
    sk::open_die_forge::Input in;
    in.reduction_pct = jdouble(p, "reduction_pct", 25.0);
    in.temp_c        = jdouble(p, "temp_c",        1100.0);
    return in;
}

sk::upsetting::Input parseUpsetting(const json& p)
{
    sk::upsetting::Input in;
    in.compression_ratio = jdouble(p, "compression_ratio", 2.0);
    return in;
}

sk::flow_form::Input parseFlowForm(const json& p)
{
    sk::flow_form::Input in;
    in.outer_dia_mm          = jdouble(p, "outer_dia_mm",          0.0);
    in.original_thickness_mm = jdouble(p, "original_thickness_mm", 0.0);
    in.final_thickness_mm    = jdouble(p, "final_thickness_mm",    0.0);
    in.original_length_mm    = jdouble(p, "original_length_mm",    0.0);
    if (p.contains("pass_count") && p["pass_count"].is_number_integer()) {
        in.pass_count = p["pass_count"].get<int>();
    }
    return in;
}

// ── Compound features: bearings / seats ──────────────────────────────────

sk::bearing_seat::Input parseBearingSeat(const json& p)
{
    sk::bearing_seat::Input in;
    in.entry_face    = parseFaceDatum(p);
    in.position_x_mm = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm = jdouble(p, "position_y_mm", 0.0);
    in.axis_dir      = parseAxisDir(p);
    in.outer_dia_mm  = jdouble(p, "outer_dia_mm",  0.0);
    in.bore_depth_mm = jdouble(p, "bore_depth_mm", 0.0);
    in.bearing_id    = jstring(p, "bearing_id",    "6204");
    in.fit_class     = jstring(p, "fit_class",     "k6");
    in.preload_N     = jdouble(p, "preload_N",     0.0);
    return in;
}

sk::radial_bearing_seat_with_snapring::Input parseRadialBearingSeatWithSnapring(const json& p)
{
    sk::radial_bearing_seat_with_snapring::Input in;
    in.entry_face    = parseFaceDatum(p);
    in.position_x_mm = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm = jdouble(p, "position_y_mm", 0.0);
    in.axis_dir      = parseAxisDir(p);
    in.outer_dia_mm  = jdouble(p, "outer_dia_mm",  0.0);
    in.inner_dia_mm  = jdouble(p, "inner_dia_mm",  0.0);
    in.depth_mm      = jdouble(p, "depth_mm",      0.0);
    in.snap_ring_std = jstring(p, "snap_ring_std", "DIN471");
    return in;
}

sk::sealed_bearing_seat_with_shield_relief::Input parseSealedBearingSeatWithShieldRelief(const json& p)
{
    sk::sealed_bearing_seat_with_shield_relief::Input in;
    in.entry_face           = parseFaceDatum(p);
    in.position_x_mm        = jdouble(p, "position_x_mm",        0.0);
    in.position_y_mm        = jdouble(p, "position_y_mm",        0.0);
    in.axis_dir             = parseAxisDir(p);
    in.outer_dia_mm         = jdouble(p, "outer_dia_mm",         0.0);
    in.shield_relief_dia_mm = jdouble(p, "shield_relief_dia_mm", 0.0);
    in.seat_depth_mm        = jdouble(p, "seat_depth_mm",        0.0);
    return in;
}

sk::needle_bearing_seat_press_fit::Input parseNeedleBearingSeatPressFit(const json& p)
{
    sk::needle_bearing_seat_press_fit::Input in;
    in.entry_face       = parseFaceDatum(p);
    in.position_x_mm    = jdouble(p, "position_x_mm",    0.0);
    in.position_y_mm    = jdouble(p, "position_y_mm",    0.0);
    in.axis_dir         = parseAxisDir(p);
    in.outer_dia_mm     = jdouble(p, "outer_dia_mm",     0.0);
    in.depth_mm         = jdouble(p, "depth_mm",         0.0);
    in.retention_dia_mm = jdouble(p, "retention_dia_mm", 0.0);
    return in;
}

sk::thrust_bearing_seat_compound::Input parseThrustBearingSeatCompound(const json& p)
{
    sk::thrust_bearing_seat_compound::Input in;
    in.entry_face    = parseFaceDatum(p);
    in.position_x_mm = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm = jdouble(p, "position_y_mm", 0.0);
    in.axis_dir      = parseAxisDir(p);
    in.outer_dia_mm  = jdouble(p, "outer_dia_mm",  0.0);
    in.inner_dia_mm  = jdouble(p, "inner_dia_mm",  0.0);
    in.face_depth_mm = jdouble(p, "face_depth_mm", 1.0);
    in.seat_depth_mm = jdouble(p, "seat_depth_mm", 4.0);
    return in;
}

// ── Compound features: fastener seats ────────────────────────────────────

sk::socket_head_bolt_seat::Input parseSocketHeadBoltSeat(const json& p)
{
    sk::socket_head_bolt_seat::Input in;
    in.entry_face    = parseFaceDatum(p);
    in.position_x_mm = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm = jdouble(p, "position_y_mm", 0.0);
    in.axis_dir      = parseAxisDir(p);
    in.fastener_size = jstring(p, "fastener_size", "M3");
    in.head_slip_mm  = jdouble(p, "head_slip_mm",  0.4);
    return in;
}

sk::countersunk_bolt_seat::Input parseCountersunkBoltSeat(const json& p)
{
    sk::countersunk_bolt_seat::Input in;
    in.entry_face    = parseFaceDatum(p);
    in.position_x_mm = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm = jdouble(p, "position_y_mm", 0.0);
    in.axis_dir      = parseAxisDir(p);
    in.fastener_size = jstring(p, "fastener_size", "M3");
    in.csk_angle_deg = jdouble(p, "csk_angle_deg", 90.0);
    in.with_relief   = jbool  (p, "with_relief",   false);
    return in;
}

sk::captive_nut_pocket::Input parseCaptiveNutPocket(const json& p)
{
    sk::captive_nut_pocket::Input in;
    in.entry_face    = parseFaceDatum(p);
    in.position_x_mm = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm = jdouble(p, "position_y_mm", 0.0);
    in.axis_dir      = parseAxisDir(p);
    in.nut_size      = jstring(p, "nut_size",    "M3");
    in.slip_fit_mm   = jdouble(p, "slip_fit_mm", 0.2);
    in.entry_edge    = jstring(p, "entry_edge",  "+x");
    return in;
}

sk::set_screw_anti_rotation_pocket::Input parseSetScrewAntiRotationPocket(const json& p)
{
    sk::set_screw_anti_rotation_pocket::Input in;
    in.entry_face          = parseFaceDatum(p);
    in.position_x_mm       = jdouble(p, "position_x_mm",       0.0);
    in.position_y_mm       = jdouble(p, "position_y_mm",       0.0);
    in.axis_dir            = parseAxisDir(p);
    in.shaft_axis_dir      = parseDir(p, "shaft_axis_dir", gp_Dir(1.0, 0.0, 0.0));
    in.set_screw_M         = jstring(p, "set_screw_M",         "M4");
    in.shaft_intersect_dia = jdouble(p, "shaft_intersect_dia", 0.0);
    in.chamfer_size_mm     = jdouble(p, "chamfer_size_mm",     0.5);
    in.boss_thickness_mm   = jdouble(p, "boss_thickness_mm",   0.0);
    return in;
}

sk::helicoil_pilot_compound::Input parseHelicoilPilotCompound(const json& p)
{
    sk::helicoil_pilot_compound::Input in;
    in.entry_face         = parseFaceDatum(p);
    in.position_x_mm      = jdouble(p, "position_x_mm",     0.0);
    in.position_y_mm      = jdouble(p, "position_y_mm",     0.0);
    in.axis_dir           = parseAxisDir(p);
    in.target_thread_size = jstring(p, "target_thread_size", "M3");
    in.insert_length_mm   = jdouble(p, "insert_length_mm",   0.0);
    return in;
}

sk::threaded_npt_port::Input parseThreadedNptPort(const json& p)
{
    sk::threaded_npt_port::Input in;
    in.face_id           = parseFaceDatum(p);
    in.position_x_mm     = jdouble(p, "position_x_mm",     0.0);
    in.position_y_mm     = jdouble(p, "position_y_mm",     0.0);
    in.axis_dir          = parseAxisDir(p);
    in.npt_size          = jstring(p, "npt_size",          "1/4");
    in.recess_depth_mm   = jdouble(p, "recess_depth_mm",   1.5);
    in.chamfer_leg_mm    = jdouble(p, "chamfer_leg_mm",    0.5);
    in.part_thickness_mm = jdouble(p, "part_thickness_mm", 20.0);
    return in;
}

sk::jic_flare_port_seat::Input parseJicFlarePortSeat(const json& p)
{
    sk::jic_flare_port_seat::Input in;
    in.face_id              = parseFaceDatum(p);
    in.position_x_mm        = jdouble(p, "position_x_mm",        0.0);
    in.position_y_mm        = jdouble(p, "position_y_mm",        0.0);
    in.axis_dir             = parseAxisDir(p);
    in.jic_size             = jstring(p, "jic_size",             "-04");
    in.thread_engagement_mm = jdouble(p, "thread_engagement_mm", 0.0);
    in.chamfer_leg_mm       = jdouble(p, "chamfer_leg_mm",       0.5);
    in.part_thickness_mm    = jdouble(p, "part_thickness_mm",    25.0);
    return in;
}

sk::banjo_fitting_seat::Input parseBanjoFittingSeat(const json& p)
{
    sk::banjo_fitting_seat::Input in;
    in.face_id               = parseFaceDatum(p);
    in.position_x_mm         = jdouble(p, "position_x_mm",         0.0);
    in.position_y_mm         = jdouble(p, "position_y_mm",         0.0);
    in.bolt_axis             = parseDir(p, "bolt_axis", gp_Dir(0.0, 0.0, -1.0));
    in.bolt_M                = jstring(p, "bolt_M",                "M6");
    in.banjo_dia_mm          = jdouble(p, "banjo_dia_mm",          4.0);
    in.banjo_axial_z_mm      = jdouble(p, "banjo_axial_z_mm",      0.0);
    in.washer_seat_depth_mm  = jdouble(p, "washer_seat_depth_mm",  1.5);
    in.part_thickness_mm     = jdouble(p, "part_thickness_mm",     12.0);
    return in;
}

sk::manifold_cross_drill_compound::Input parseManifoldCrossDrillCompound(const json& p)
{
    sk::manifold_cross_drill_compound::Input in;
    in.block_face           = parseFaceDatum(p);
    in.drill1_axis          = parseDir(p, "drill1_axis", gp_Dir(1.0, 0.0, 0.0));
    in.drill2_axis          = parseDir(p, "drill2_axis", gp_Dir(0.0, 1.0, 0.0));
    in.drill1_origin_x_mm   = jdouble(p, "drill1_origin_x_mm", 0.0);
    in.drill1_origin_y_mm   = jdouble(p, "drill1_origin_y_mm", 0.0);
    in.drill1_origin_z_mm   = jdouble(p, "drill1_origin_z_mm", 0.0);
    in.drill_dia_mm         = jdouble(p, "drill_dia_mm",       6.0);
    in.plug_M               = jstring(p, "plug_M",             "M6");
    in.plug_seat_depth_mm   = jdouble(p, "plug_seat_depth_mm", 4.0);
    return in;
}

// ── Compound features: drive components ──────────────────────────────────

sk::shaft_with_keyway_step::Input parseShaftWithKeywayStep(const json& p)
{
    sk::shaft_with_keyway_step::Input in;
    in.full_dia_mm        = jdouble(p, "full_dia_mm",        0.0);
    in.shaft_length_mm    = jdouble(p, "shaft_length_mm",    0.0);
    in.step_z_mm          = jdouble(p, "step_z_mm",          0.0);
    in.step_dia_mm        = jdouble(p, "step_dia_mm",        0.0);
    in.key_w_mm           = jdouble(p, "key_w_mm",           0.0);
    in.key_h_mm           = jdouble(p, "key_h_mm",           0.0);
    in.key_len_mm         = jdouble(p, "key_len_mm",         0.0);
    in.key_center_z_mm    = jdouble(p, "key_center_z_mm",    0.0);
    in.snap_ring_z_mm     = jdouble(p, "snap_ring_z_mm",     0.0);
    in.snap_ring_w_mm     = jdouble(p, "snap_ring_w_mm",     1.2);
    in.snap_ring_depth_mm = jdouble(p, "snap_ring_depth_mm", 0.5);
    return in;
}

sk::pulley_with_keyway_compound::Input parsePulleyWithKeywayCompound(const json& p)
{
    sk::pulley_with_keyway_compound::Input in;
    in.outer_dia_mm         = jdouble(p, "outer_dia_mm",         0.0);
    in.groove_angle_deg     = jdouble(p, "groove_angle_deg",     38.0);
    in.groove_depth_mm      = jdouble(p, "groove_depth_mm",      0.0);
    in.thickness_mm         = jdouble(p, "thickness_mm",         0.0);
    in.key_w_mm             = jdouble(p, "key_w_mm",             0.0);
    in.key_h_mm             = jdouble(p, "key_h_mm",             0.0);
    in.bore_dia_mm          = jdouble(p, "bore_dia_mm",          0.0);
    in.balance_drill_dia_mm = jdouble(p, "balance_drill_dia_mm", 5.0);
    return in;
}

sk::gear_blank_with_hub_step::Input parseGearBlankWithHubStep(const json& p)
{
    sk::gear_blank_with_hub_step::Input in;
    in.blank_dia_mm   = jdouble(p, "blank_dia_mm",   0.0);
    in.blank_thick_mm = jdouble(p, "blank_thick_mm", 0.0);
    in.hub_dia_mm     = jdouble(p, "hub_dia_mm",     0.0);
    in.hub_height_mm  = jdouble(p, "hub_height_mm",  0.0);
    in.bore_dia_mm    = jdouble(p, "bore_dia_mm",    0.0);
    in.keyway_w_mm    = jdouble(p, "keyway_w_mm",    0.0);
    in.keyway_h_mm    = jdouble(p, "keyway_h_mm",    0.0);
    return in;
}

sk::sprocket_blank_with_bore::Input parseSprocketBlankWithBore(const json& p)
{
    sk::sprocket_blank_with_bore::Input in;
    in.outer_dia_mm     = jdouble(p, "outer_dia_mm",     0.0);
    in.bore_dia_mm      = jdouble(p, "bore_dia_mm",      0.0);
    in.thickness_mm     = jdouble(p, "thickness_mm",     0.0);
    in.lightening_count = jdouble(p, "lightening_count", 4.0);
    in.chamfer_size_mm  = jdouble(p, "chamfer_size_mm",  0.5);
    return in;
}

sk::flywheel_blank_with_balance_drills::Input parseFlywheelBlankWithBalanceDrills(const json& p)
{
    sk::flywheel_blank_with_balance_drills::Input in;
    in.outer_dia_mm           = jdouble(p, "outer_dia_mm",           0.0);
    in.thickness_mm           = jdouble(p, "thickness_mm",           0.0);
    in.bore_dia_mm            = jdouble(p, "bore_dia_mm",            0.0);
    if (p.contains("balance_drill_count") && p["balance_drill_count"].is_number_integer()) {
        in.balance_drill_count = p["balance_drill_count"].get<int>();
    }
    in.balance_drill_dia_mm   = jdouble(p, "balance_drill_dia_mm",   6.0);
    in.balance_drill_depth_mm = jdouble(p, "balance_drill_depth_mm", 5.0);
    in.hole_radius_mm         = jdouble(p, "hole_radius_mm",         0.0);
    in.angle_offset_deg       = jdouble(p, "angle_offset_deg",       0.0);
    return in;
}

sk::cam_lobe::Input parseCamLobe(const json& p)
{
    sk::cam_lobe::Input in;
    in.entry_face     = parseFaceDatum(p);
    in.center_x_mm    = jdouble(p, "center_x_mm",    0.0);
    in.center_y_mm    = jdouble(p, "center_y_mm",    0.0);
    in.angle_deg      = jdouble(p, "angle_deg",      0.0);
    in.lobe_lift_mm   = jdouble(p, "lobe_lift_mm",   1.0);
    in.lobe_angle_deg = jdouble(p, "lobe_angle_deg", 30.0);
    in.base_radius_mm = jdouble(p, "base_radius_mm", 10.0);
    in.thickness_mm   = jdouble(p, "thickness_mm",   5.0);
    in.axis_dir       = parseDir(p, "axis_dir", gp_Dir(0.0, 0.0, 1.0));
    return in;
}

sk::gear_tooth_cut::Input parseGearToothCut(const json& p)
{
    sk::gear_tooth_cut::Input in;
    in.entry_face          = parseFaceDatum(p);
    in.angle_rad           = jdouble(p, "angle_rad",           0.0);
    in.module_mm           = jdouble(p, "module_mm",           1.0);
    in.pressure_angle_deg  = jdouble(p, "pressure_angle_deg",  20.0);
    in.helix_angle_deg     = jdouble(p, "helix_angle_deg",     0.0);
    if (p.contains("num_teeth") && p["num_teeth"].is_number_integer()) {
        in.num_teeth = p["num_teeth"].get<int>();
    }
    in.root_radius_mm      = jdouble(p, "root_radius_mm",      9.0);
    in.tip_radius_mm       = jdouble(p, "tip_radius_mm",       11.0);
    in.face_width_mm       = jdouble(p, "face_width_mm",       5.0);
    in.axis_dir            = parseDir(p, "axis_dir", gp_Dir(0.0, 0.0, 1.0));
    return in;
}

sk::rack_tooth_cut::Input parseRackToothCut(const json& p)
{
    sk::rack_tooth_cut::Input in;
    in.entry_face         = parseFaceDatum(p);
    in.position_x_mm      = jdouble(p, "position_x_mm",      0.0);
    in.position_y_mm      = jdouble(p, "position_y_mm",      0.0);
    in.module_mm          = jdouble(p, "module_mm",          1.0);
    in.pressure_angle_deg = jdouble(p, "pressure_angle_deg", 20.0);
    in.depth_mm           = jdouble(p, "depth_mm",           0.0);
    in.width_mm           = jdouble(p, "width_mm",           0.0);
    in.length_mm          = jdouble(p, "length_mm",          0.0);
    in.length_dir         = parseDir(p, "length_dir", gp_Dir(1.0, 0.0, 0.0));
    in.axis_dir           = parseAxisDir(p);
    return in;
}

// ── Compound features: seals / grooves ───────────────────────────────────

sk::o_ring_groove_face::Input parseORingGrooveFace(const json& p)
{
    sk::o_ring_groove_face::Input in;
    in.face_id     = parseFaceDatum(p);
    in.center_x_mm = jdouble(p, "center_x_mm", 0.0);
    in.center_y_mm = jdouble(p, "center_y_mm", 0.0);
    in.axis_dir    = parseAxisDir(p);
    in.mean_dia_mm = jdouble(p, "mean_dia_mm", 0.0);
    in.o_ring_size = jstring(p, "o_ring_size", "");
    return in;
}

sk::o_ring_groove_radial::Input parseORingGrooveRadial(const json& p)
{
    sk::o_ring_groove_radial::Input in;
    in.bore_or_shaft_dia_mm = jdouble(p, "bore_or_shaft_dia_mm", 0.0);
    in.position_z_mm        = jdouble(p, "position_z_mm",        0.0);
    in.o_ring_size          = jstring(p, "o_ring_size",          "");
    return in;
}

sk::lip_seal_seat::Input parseLipSealSeat(const json& p)
{
    sk::lip_seal_seat::Input in;
    in.body_face         = parseFaceDatum(p);
    in.center_x_mm       = jdouble(p, "center_x_mm",       0.0);
    in.center_y_mm       = jdouble(p, "center_y_mm",       0.0);
    in.axis_dir          = parseAxisDir(p);
    in.shaft_id_mm       = jdouble(p, "shaft_id_mm",       0.0);
    in.seal_od_mm        = jdouble(p, "seal_od_mm",        0.0);
    in.seal_thickness_mm = jdouble(p, "seal_thickness_mm", 0.0);
    in.bore_depth_mm     = jdouble(p, "bore_depth_mm",     0.0);
    return in;
}

sk::caseback_o_ring_groove::Input parseCasebackORingGroove(const json& p)
{
    sk::caseback_o_ring_groove::Input in;
    in.caseback_face         = parseFaceDatum(p);
    in.center_x_mm           = jdouble(p, "center_x_mm",           0.0);
    in.center_y_mm           = jdouble(p, "center_y_mm",           0.0);
    in.axis_dir              = parseDir(p, "axis_dir", gp_Dir(0.0, 0.0, 1.0));
    in.mean_dia_mm           = jdouble(p, "mean_dia_mm",           0.0);
    in.o_ring_size           = jstring(p, "o_ring_size",           "-008");
    in.thread_pilot_od_mm    = jdouble(p, "thread_pilot_od_mm",    0.0);
    in.thread_pilot_depth_mm = jdouble(p, "thread_pilot_depth_mm", 0.4);
    return in;
}

sk::sapphire_glass_seat::Input parseSapphireGlassSeat(const json& p)
{
    sk::sapphire_glass_seat::Input in;
    in.case_face           = parseFaceDatum(p);
    in.center_x_mm         = jdouble(p, "center_x_mm",         0.0);
    in.center_y_mm         = jdouble(p, "center_y_mm",         0.0);
    in.axis_dir            = parseAxisDir(p);
    in.glass_dia_mm        = jdouble(p, "glass_dia_mm",        0.0);
    in.ledge_depth_mm      = jdouble(p, "ledge_depth_mm",      1.5);
    in.inner_bore_dia_mm   = jdouble(p, "inner_bore_dia_mm",   0.0);
    in.inner_bore_depth_mm = jdouble(p, "inner_bore_depth_mm", 0.0);
    in.o_ring_cs_mm        = jdouble(p, "o_ring_cs_mm",        1.0);
    in.o_ring_depth_mm     = jdouble(p, "o_ring_depth_mm",     0.4);
    in.o_ring_z_offset_mm  = jdouble(p, "o_ring_z_offset_mm",  0.5);
    return in;
}

// ── Compound features: bushings / mounting / enclosure ──────────────────

sk::eccentric_bushing_seat::Input parseEccentricBushingSeat(const json& p)
{
    sk::eccentric_bushing_seat::Input in;
    in.entry_face      = parseFaceDatum(p);
    in.position_x_mm   = jdouble(p, "position_x_mm",   0.0);
    in.position_y_mm   = jdouble(p, "position_y_mm",   0.0);
    in.axis_dir        = parseAxisDir(p);
    in.bore_od_mm      = jdouble(p, "bore_od_mm",      0.0);
    in.bore_depth_mm   = jdouble(p, "bore_depth_mm",   0.0);
    in.slot_arc_deg    = jdouble(p, "slot_arc_deg",    60.0);
    in.slot_width_mm   = jdouble(p, "slot_width_mm",   1.6);
    in.slot_depth_mm   = jdouble(p, "slot_depth_mm",   0.0);
    in.chamfer_size_mm = jdouble(p, "chamfer_size_mm", 0.5);
    return in;
}

sk::vibration_isolator_seat::Input parseVibrationIsolatorSeat(const json& p)
{
    sk::vibration_isolator_seat::Input in;
    in.entry_face     = parseFaceDatum(p);
    in.position_x_mm  = jdouble(p, "position_x_mm",  0.0);
    in.position_y_mm  = jdouble(p, "position_y_mm",  0.0);
    in.axis_dir       = parseAxisDir(p);
    in.stud_M         = jstring(p, "stud_M",         "M6");
    in.bushing_od_mm  = jdouble(p, "bushing_od_mm",  0.0);
    in.plate_thick_mm = jdouble(p, "plate_thick_mm", 0.0);
    return in;
}

sk::rubber_grommet_seat::Input parseRubberGrommetSeat(const json& p)
{
    sk::rubber_grommet_seat::Input in;
    in.entry_face    = parseFaceDatum(p);
    in.position_x_mm = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm = jdouble(p, "position_y_mm", 0.0);
    in.axis_dir      = parseAxisDir(p);
    in.hole_dia_mm   = jdouble(p, "hole_dia_mm",   0.0);
    in.plate_t_mm    = jdouble(p, "plate_t_mm",    0.0);
    return in;
}

sk::magnetic_latch_pocket::Input parseMagneticLatchPocket(const json& p)
{
    sk::magnetic_latch_pocket::Input in;
    in.entry_face    = parseFaceDatum(p);
    in.position_x_mm = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm = jdouble(p, "position_y_mm", 0.0);
    in.axis_dir      = parseAxisDir(p);
    return in;
}

sk::concealed_hinge_cup::Input parseConcealedHingeCup(const json& p)
{
    sk::concealed_hinge_cup::Input in;
    in.entry_face     = parseFaceDatum(p);
    in.position_x_mm  = jdouble(p, "position_x_mm",  0.0);
    in.position_y_mm  = jdouble(p, "position_y_mm",  0.0);
    in.axis_dir       = parseAxisDir(p);
    in.edge_offset_mm = jdouble(p, "edge_offset_mm", 6.0);
    return in;
}

sk::butt_hinge_pocket::Input parseButtHingePocket(const json& p)
{
    sk::butt_hinge_pocket::Input in;
    in.entry_face        = parseFaceDatum(p);
    in.center_x_mm       = jdouble(p, "center_x_mm",       0.0);
    in.center_y_mm       = jdouble(p, "center_y_mm",       0.0);
    in.axis_dir          = parseAxisDir(p);
    in.hinge_length_mm   = jdouble(p, "hinge_length_mm",   0.0);
    in.hinge_width_mm    = jdouble(p, "hinge_width_mm",    0.0);
    in.leaf_thickness_mm = jdouble(p, "leaf_thickness_mm", 1.5);
    return in;
}

sk::pcb_standoff_threaded::Input parsePcbStandoffThreaded(const json& p)
{
    sk::pcb_standoff_threaded::Input in;
    in.face            = parseFaceDatum(p);
    in.position_x_mm   = jdouble(p, "position_x_mm",   0.0);
    in.position_y_mm   = jdouble(p, "position_y_mm",   0.0);
    in.axis_dir        = parseDir(p, "axis_dir", gp_Dir(0.0, 0.0, 1.0));
    in.thread_M        = jstring(p, "thread_M",        "M3");
    in.boss_height_mm  = jdouble(p, "boss_height_mm",  6.0);
    in.boss_dia_mm     = jdouble(p, "boss_dia_mm",     0.0);
    in.thread_depth_mm = jdouble(p, "thread_depth_mm", 0.0);
    return in;
}

sk::usb_c_port_cutout::Input parseUsbCPortCutout(const json& p)
{
    sk::usb_c_port_cutout::Input in;
    in.face            = parseFaceDatum(p);
    in.position_x_mm   = jdouble(p, "position_x_mm",   0.0);
    in.position_y_mm   = jdouble(p, "position_y_mm",   0.0);
    in.axis_dir        = parseAxisDir(p);
    in.orientation_deg = jdouble(p, "orientation_deg", 0.0);
    return in;
}

sk::shim_pocket::Input parseShimPocket(const json& p)
{
    sk::shim_pocket::Input in;
    in.entry_face               = parseFaceDatum(p);
    in.position_x_mm            = jdouble(p, "position_x_mm",            0.0);
    in.position_y_mm            = jdouble(p, "position_y_mm",            0.0);
    in.axis_dir                 = parseAxisDir(p);
    in.shim_thk_mm              = jdouble(p, "shim_thk_mm",              0.0);
    in.shim_w_mm                = jdouble(p, "shim_w_mm",                0.0);
    in.shim_l_mm                = jdouble(p, "shim_l_mm",                0.0);
    in.retention_step_depth_mm  = jdouble(p, "retention_step_depth_mm",  0.1);
    in.retention_step_inset_mm  = jdouble(p, "retention_step_inset_mm",  0.5);
    in.tab_notch_w_mm           = jdouble(p, "tab_notch_w_mm",           1.0);
    in.tab_notch_d_mm           = jdouble(p, "tab_notch_d_mm",           0.5);
    return in;
}

sk::undercut_relief::Input parseUndercutRelief(const json& p)
{
    sk::undercut_relief::Input in;
    in.corner_x_mm = jdouble(p, "corner_x_mm", 0.0);
    in.corner_y_mm = jdouble(p, "corner_y_mm", 0.0);
    in.corner_z_mm = jdouble(p, "corner_z_mm", 0.0);
    in.axis_xy     = parseDir(p, "axis_xy", gp_Dir(1.0, 0.0, 0.0));
    in.dia_mm      = jdouble(p, "dia_mm",   0.0);
    in.depth_mm    = jdouble(p, "depth_mm", 0.0);
    return in;
}

sk::datum_face_establish::Input parseDatumFaceEstablish(const json& p)
{
    sk::datum_face_establish::Input in;
    in.datum_face            = parseFaceDatum(p);
    in.datum_class           = jstring(p, "datum_class",          "A");
    in.required_flatness_um  = jdouble(p, "required_flatness_um", 10.0);
    in.depth_mm              = jdouble(p, "depth_mm",             0.5);
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

    // ── Slice-6 / 7 / 8 expansion ────────────────────────────────────────
    // Lathe / turning
    t[sk::turn_external::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::turn_external::apply(wp, parseTurnExternal(p));
    };
    t[sk::turn_internal::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::turn_internal::apply(wp, parseTurnInternal(p));
    };
    t[sk::face_turn::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::face_turn::apply(wp, parseFaceTurn(p));
    };
    t[sk::groove_turn::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::groove_turn::apply(wp, parseGrooveTurn(p));
    };
    t[sk::taper_turn::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::taper_turn::apply(wp, parseTaperTurn(p));
    };
    t[sk::drill_lathe::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::drill_lathe::apply(wp, parseDrillLathe(p));
    };
    t[sk::contour_turn::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::contour_turn::apply(wp, parseContourTurn(p));
    };
    t[sk::form_turn::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::form_turn::apply(wp, parseFormTurn(p));
    };
    t[sk::keyway_external::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::keyway_external::apply(wp, parseKeywayExternal(p));
    };

    // Milling / drilling variants
    t[sk::spot_face::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::spot_face::apply(wp, parseSpotFace(p));
    };
    t[sk::deep_hole_peck::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::deep_hole_peck::apply(wp, parseDeepHolePeck(p));
    };
    t[sk::gun_drill::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::gun_drill::apply(wp, parseGunDrill(p));
    };
    t[sk::micro_drill::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::micro_drill::apply(wp, parseMicroDrill(p));
    };
    t[sk::rigid_tap::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::rigid_tap::apply(wp, parseRigidTap(p));
    };
    t[sk::plunge_milling::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::plunge_milling::apply(wp, parsePlungeMilling(p));
    };
    t[sk::plunge_roughing::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::plunge_roughing::apply(wp, parsePlungeRoughing(p));
    };
    t[sk::trochoidal_mill::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::trochoidal_mill::apply(wp, parseTrochoidalMill(p));
    };
    t[sk::adaptive_clearing::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::adaptive_clearing::apply(wp, parseAdaptiveClearing(p));
    };
    t[sk::rest_milling::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::rest_milling::apply(wp, parseRestMilling(p));
    };
    t[sk::multi_step_bore::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::multi_step_bore::apply(wp, parseMultiStepBore(p));
    };
    t[sk::edm_drill::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::edm_drill::apply(wp, parseEdmDrill(p));
    };
    t[sk::wire_edm::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::wire_edm::apply(wp, parseWireEdm(p));
    };

    // Sheet / forming
    t[sk::piercing::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::piercing::apply(wp, parsePiercing(p));
    };
    t[sk::sheet_punch::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::sheet_punch::apply(wp, parseSheetPunch(p));
    };
    t[sk::bend::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::bend::apply(wp, parseBend(p));
    };
    t[sk::flange::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::flange::apply(wp, parseFlange(p));
    };
    t[sk::hem::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::hem::apply(wp, parseHem(p));
    };
    t[sk::notch::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::notch::apply(wp, parseNotch(p));
    };
    t[sk::emboss::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::emboss::apply(wp, parseEmboss(p));
    };
    t[sk::coining::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::coining::apply(wp, parseCoining(p));
    };
    t[sk::draw_bead::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::draw_bead::apply(wp, parseDrawBead(p));
    };

    // Separation cutting (shared LinearCutInput)
    t[sk::saw_cut::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::saw_cut::apply(wp, parseLinearCut(p));
    };
    t[sk::laser_cut::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::laser_cut::apply(wp, parseLinearCut(p));
    };
    t[sk::plasma_cut::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::plasma_cut::apply(wp, parseLinearCut(p));
    };
    t[sk::waterjet_cut::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::waterjet_cut::apply(wp, parseLinearCut(p));
    };
    t[sk::oxyfuel_cut::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::oxyfuel_cut::apply(wp, parseLinearCut(p));
    };

    // Welding / fastening (compound)
    t[sk::plug_weld_hole::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::plug_weld_hole::apply(wp, parsePlugWeldHole(p));
    };
    t[sk::slot_weld_slot::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::slot_weld_slot::apply(wp, parseSlotWeldSlot(p));
    };
    t[sk::full_penetration_butt_prep::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::full_penetration_butt_prep::apply(wp, parseFullPenetrationButtPrep(p));
    };
    t[sk::partial_pen_fillet_prep::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::partial_pen_fillet_prep::apply(wp, parsePartialPenFilletPrep(p));
    };
    t[sk::rivet_hole::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::rivet_hole::apply(wp, parseRivetHole(p));
    };
    t[sk::blind_fastener::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::blind_fastener::apply(wp, parseBlindFastener(p));
    };
    t[sk::clinch_nut::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::clinch_nut::apply(wp, parseClinchNut(p));
    };
    t[sk::threaded_insert::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::threaded_insert::apply(wp, parseThreadedInsert(p));
    };
    t[sk::heli_coil::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::heli_coil::apply(wp, parseHeliCoil(p));
    };

    // Surfacing / mold
    t[sk::offset_surface::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::offset_surface::apply(wp, parseOffsetSurface(p));
    };
    t[sk::mold_boss::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::mold_boss::apply(wp, parseMoldBoss(p));
    };
    t[sk::mold_rib::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::mold_rib::apply(wp, parseMoldRib(p));
    };
    t[sk::mold_gate::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::mold_gate::apply(wp, parseMoldGate(p));
    };
    t[sk::ejector_pin_hole::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::ejector_pin_hole::apply(wp, parseEjectorPinHole(p));
    };
    t[sk::cooling_channel::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::cooling_channel::apply(wp, parseCoolingChannel(p));
    };
    t[sk::runner_system::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::runner_system::apply(wp, parseRunnerSystem(p));
    };

    // Forging (geometric)
    t[sk::open_die_forge::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::open_die_forge::apply(wp, parseOpenDieForge(p));
    };
    t[sk::upsetting::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::upsetting::apply(wp, parseUpsetting(p));
    };
    t[sk::flow_form::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::flow_form::apply(wp, parseFlowForm(p));
    };

    // Compound features — bearings & seats
    t[sk::bearing_seat::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::bearing_seat::apply(wp, parseBearingSeat(p));
    };
    t[sk::radial_bearing_seat_with_snapring::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::radial_bearing_seat_with_snapring::apply(wp, parseRadialBearingSeatWithSnapring(p));
    };
    t[sk::sealed_bearing_seat_with_shield_relief::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::sealed_bearing_seat_with_shield_relief::apply(wp, parseSealedBearingSeatWithShieldRelief(p));
    };
    t[sk::needle_bearing_seat_press_fit::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::needle_bearing_seat_press_fit::apply(wp, parseNeedleBearingSeatPressFit(p));
    };
    t[sk::thrust_bearing_seat_compound::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::thrust_bearing_seat_compound::apply(wp, parseThrustBearingSeatCompound(p));
    };

    // Compound features — fastener seats
    t[sk::socket_head_bolt_seat::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::socket_head_bolt_seat::apply(wp, parseSocketHeadBoltSeat(p));
    };
    t[sk::countersunk_bolt_seat::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::countersunk_bolt_seat::apply(wp, parseCountersunkBoltSeat(p));
    };
    t[sk::captive_nut_pocket::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::captive_nut_pocket::apply(wp, parseCaptiveNutPocket(p));
    };
    t[sk::set_screw_anti_rotation_pocket::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::set_screw_anti_rotation_pocket::apply(wp, parseSetScrewAntiRotationPocket(p));
    };
    t[sk::helicoil_pilot_compound::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::helicoil_pilot_compound::apply(wp, parseHelicoilPilotCompound(p));
    };
    t[sk::threaded_npt_port::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::threaded_npt_port::apply(wp, parseThreadedNptPort(p));
    };
    t[sk::jic_flare_port_seat::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::jic_flare_port_seat::apply(wp, parseJicFlarePortSeat(p));
    };
    t[sk::banjo_fitting_seat::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::banjo_fitting_seat::apply(wp, parseBanjoFittingSeat(p));
    };
    t[sk::manifold_cross_drill_compound::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::manifold_cross_drill_compound::apply(wp, parseManifoldCrossDrillCompound(p));
    };

    // Compound features — drive components
    t[sk::shaft_with_keyway_step::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::shaft_with_keyway_step::apply(wp, parseShaftWithKeywayStep(p));
    };
    t[sk::pulley_with_keyway_compound::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::pulley_with_keyway_compound::apply(wp, parsePulleyWithKeywayCompound(p));
    };
    t[sk::gear_blank_with_hub_step::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::gear_blank_with_hub_step::apply(wp, parseGearBlankWithHubStep(p));
    };
    t[sk::sprocket_blank_with_bore::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::sprocket_blank_with_bore::apply(wp, parseSprocketBlankWithBore(p));
    };
    t[sk::flywheel_blank_with_balance_drills::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::flywheel_blank_with_balance_drills::apply(wp, parseFlywheelBlankWithBalanceDrills(p));
    };
    t[sk::cam_lobe::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::cam_lobe::apply(wp, parseCamLobe(p));
    };
    t[sk::gear_tooth_cut::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::gear_tooth_cut::apply(wp, parseGearToothCut(p));
    };
    t[sk::rack_tooth_cut::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::rack_tooth_cut::apply(wp, parseRackToothCut(p));
    };

    // Compound features — seals / grooves
    t[sk::o_ring_groove_face::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::o_ring_groove_face::apply(wp, parseORingGrooveFace(p));
    };
    t[sk::o_ring_groove_radial::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::o_ring_groove_radial::apply(wp, parseORingGrooveRadial(p));
    };
    t[sk::lip_seal_seat::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::lip_seal_seat::apply(wp, parseLipSealSeat(p));
    };
    t[sk::caseback_o_ring_groove::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::caseback_o_ring_groove::apply(wp, parseCasebackORingGroove(p));
    };
    t[sk::sapphire_glass_seat::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::sapphire_glass_seat::apply(wp, parseSapphireGlassSeat(p));
    };

    // Compound features — bushings / mounting / enclosure
    t[sk::eccentric_bushing_seat::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::eccentric_bushing_seat::apply(wp, parseEccentricBushingSeat(p));
    };
    t[sk::vibration_isolator_seat::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::vibration_isolator_seat::apply(wp, parseVibrationIsolatorSeat(p));
    };
    t[sk::rubber_grommet_seat::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::rubber_grommet_seat::apply(wp, parseRubberGrommetSeat(p));
    };
    t[sk::magnetic_latch_pocket::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::magnetic_latch_pocket::apply(wp, parseMagneticLatchPocket(p));
    };
    t[sk::concealed_hinge_cup::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::concealed_hinge_cup::apply(wp, parseConcealedHingeCup(p));
    };
    t[sk::butt_hinge_pocket::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::butt_hinge_pocket::apply(wp, parseButtHingePocket(p));
    };
    t[sk::pcb_standoff_threaded::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::pcb_standoff_threaded::apply(wp, parsePcbStandoffThreaded(p));
    };
    t[sk::usb_c_port_cutout::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::usb_c_port_cutout::apply(wp, parseUsbCPortCutout(p));
    };
    t[sk::shim_pocket::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::shim_pocket::apply(wp, parseShimPocket(p));
    };
    t[sk::undercut_relief::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::undercut_relief::apply(wp, parseUndercutRelief(p));
    };
    t[sk::datum_face_establish::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::datum_face_establish::apply(wp, parseDatumFaceEstablish(p));
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
