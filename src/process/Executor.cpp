// @lat: [[engine/skills#Layer 3 Executor]]

#include "Executor.hpp"

#include "ParamClamp.hpp"
#include "skills/Workpiece.hpp"
#include "skills/Datum.hpp"

// Slice-1 registered skills
#include "skills/drill_hole.hpp"
#include "skills/extrude_boss_from_sketch.hpp"
#include "skills/revolve_boss.hpp"
#include "skills/dome_boss.hpp"
#include "skills/bolt_circle_pattern.hpp"
#include "skills/linear_hole_array.hpp"
#include "skills/rectangular_hole_grid.hpp"
#include "skills/coaxial_step_bore.hpp"
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
#include "skills/plain_bushing_bore_with_lube_groove.hpp"
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
#include "skills/blind_threaded_insert_seat.hpp"
#include "skills/bolt_hole_metric_spec.hpp"
#include "skills/tapped_hole_metric_spec.hpp"
#include "skills/unc_unf_hole_spec.hpp"
#include "skills/threaded_through_with_chamfers.hpp"
#include "skills/captive_screw_pocket_spec.hpp"
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
#include "skills/o_ring_groove_as568_spec.hpp"
#include "skills/x_ring_groove.hpp"
#include "skills/x_ring_groove_spec.hpp"
#include "skills/spiral_back_up_ring_groove.hpp"
#include "skills/dust_lip_seal_seat_compound.hpp"
#include "skills/face_seal_compound_compression.hpp"
#include "skills/gasket_face_with_drain_groove.hpp"
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

// Compound features — mechanical structures / drive / fluid / spring / linear-motion / adjusters
#include "skills/i_beam_compound_section.hpp"
#include "skills/box_section_with_endplate.hpp"
#include "skills/gusset_plate_compound.hpp"
#include "skills/lifting_lug_pad_eye.hpp"
#include "skills/base_bracket_compound.hpp"
#include "skills/din_rail_mount_slot.hpp"
#include "skills/dovetail_mount_compound.hpp"
#include "skills/t_slot_table_groove.hpp"
#include "skills/linear_rail_seat_compound.hpp"
#include "skills/ball_screw_nut_pocket.hpp"
#include "skills/lead_screw_anti_backlash_pocket.hpp"
#include "skills/linear_bushing_seat.hpp"
#include "skills/cam_follower_threaded_seat.hpp"
#include "skills/internal_water_jacket.hpp"
#include "skills/adjuster_screw_pocket_compound.hpp"
#include "skills/tab_lock_anti_rotation.hpp"
#include "skills/coil_spring_seat_compound.hpp"
#include "skills/wave_spring_groove.hpp"
#include "skills/gas_spring_clevis_pocket.hpp"
#include "skills/leaf_spring_anchor_compound.hpp"
#include "skills/torsion_spring_anchor_compound.hpp"
#include "skills/leaf_spring_anchor.hpp"

// ── Slice 9 expansion: valve / electrical / tooling / machine / hinge ────
// Valve seats
#include "skills/gate_valve_seat_compound.hpp"
#include "skills/ball_valve_seat_compound.hpp"
#include "skills/butterfly_valve_disc_seat.hpp"
#include "skills/check_valve_seat_with_stop.hpp"
#include "skills/needle_valve_seat.hpp"
// Electrical contacts
#include "skills/banana_socket_compound.hpp"
#include "skills/spring_contact_clip.hpp"
#include "skills/terminal_block_post.hpp"
#include "skills/busbar_lap_joint.hpp"
#include "skills/pcb_card_edge_socket.hpp"
// Tooling / jig
#include "skills/jig_plate_with_drill_bushings.hpp"
#include "skills/locator_pin_set.hpp"
#include "skills/gauge_block_step.hpp"
#include "skills/vise_jaw_with_v_groove.hpp"
#include "skills/pin_and_diamond_locating_set.hpp"
// Machine elements
#include "skills/cam_with_profile.hpp"
#include "skills/cam_follower_roller_seat.hpp"
#include "skills/eccentric_shaft_collar.hpp"
#include "skills/flywheel_with_balance.hpp"
#include "skills/governor_arm_with_pivot.hpp"
#include "skills/spur_gear_with_real_teeth.hpp"
#include "skills/helical_gear_teeth.hpp"
#include "skills/sprocket_with_chain_teeth.hpp"
#include "skills/spline_shaft_compound.hpp"
#include "skills/ratchet_pawl_set.hpp"
// Hinge / latch
#include "skills/piano_hinge_strip.hpp"
#include "skills/overcenter_latch.hpp"
#include "skills/snap_action_lock_pocket.hpp"
#include "skills/gas_strut_hinge_compound.hpp"
#include "skills/spring_loaded_door_latch.hpp"
#include "skills/cam_lock_cavity.hpp"

// ── Slice 9 expansion: context-aware + morphing + misc compound features ──
// Context-aware auto-synthesis
#include "skills/auto_boss_under_hole.hpp"
#include "skills/auto_standoff_floating_point.hpp"
#include "skills/auto_rib_between_two_walls.hpp"
#include "skills/auto_gusset_corner_brace.hpp"
#include "skills/auto_chamfer_all_outer_edges.hpp"
// Morphing (single-workpiece only — multi-workpiece morphs deferred)
#include "skills/blend_morph_two_shapes.hpp"
#include "skills/parametric_sweep_morph.hpp"
#include "skills/deformation_warp.hpp"
// Sheet variants (geometric — emboss/notch/coining/draw_bead already wired)
#include "skills/lance.hpp"
#include "skills/beading.hpp"
// Heat exchanger geometric
#include "skills/shell_roll.hpp"
#include "skills/hemispherical_head_form.hpp"
#include "skills/expand_tube.hpp"
#include "skills/tube_to_tubesheet_weld.hpp"
#include "skills/tube_swage.hpp"
// Misc structural connections
#include "skills/bolted_flange_compound.hpp"
#include "skills/pinned_clevis_joint.hpp"
#include "skills/expansion_joint_bellows_stub.hpp"
#include "skills/anchor_pad_compound.hpp"
#include "skills/flange_face_with_gasket_groove.hpp"

// ── Slice 16/smart-spec/PCB-electronics compound features ──────────────────
// ISO bore fit family
#include "skills/iso_h7_bore_spec.hpp"
#include "skills/press_fit_p7_bore_spec.hpp"
#include "skills/slip_fit_h11_bore_spec.hpp"
#include "skills/dowel_pin_h6_bore.hpp"
#include "skills/locating_g6_bore.hpp"
// Parametric rib / wall
#include "skills/parametric_rib_array.hpp"
#include "skills/draft_wall_with_radius.hpp"
#include "skills/top_face_recess_with_walls.hpp"
#include "skills/partition_wall_with_passthrough.hpp"
#include "skills/curved_lip_around_face.hpp"
// Heat / vent (electronics)
#include "skills/heat_sink_fin_array.hpp"
#include "skills/vent_slot_array.hpp"
#include "skills/louvered_vent.hpp"
#include "skills/perforated_grille_pattern.hpp"
#include "skills/breather_vent_compound.hpp"
// PCB / electronics mounting
#include "skills/pcb_standoff_array_under_board.hpp"
#include "skills/connector_cutout_with_keepout.hpp"
#include "skills/cable_grommet_pass_through.hpp"
#include "skills/isolator_grommet_seat.hpp"
#include "skills/tilt_post_for_lcd_panel.hpp"
// Connector cutouts (slice 16)
#include "skills/din_rail_clip_slot.hpp"
#include "skills/banana_jack_receptacle.hpp"
#include "skills/rj45_socket_cutout.hpp"
// Sliding / pivot mechanisms (smart-spec)
#include "skills/linear_slider_track.hpp"
#include "skills/pivot_pin_clevis_compound.hpp"
#include "skills/cam_actuated_slider.hpp"
#include "skills/over_center_toggle_pocket.hpp"
#include "skills/detented_position_slider.hpp"
// Watch + hinge (slice 16)
#include "skills/bezel_groove_assembly.hpp"
#include "skills/lug_with_spring_bar_holes.hpp"
#include "skills/crown_stem_cavity_compound.hpp"

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
//   "entry_face":     "largest"   → FaceLargestPlanar{}
//   "entry_face":     [x,y,z]     → FaceByNormal{ gp_Dir(x,y,z) }   (any direction)
//   "entry_face_id":  <int>       → FaceIdRef{<int>}
//   (none of the above)           → FaceLargestPlanar{}
//
// Full object form covers every FaceDatum variant so arbitrary / tilted entries
// round-trip (paired with recognizers that recover axis_dir + 3-D entry):
//   { "type": "by_normal",     "normal": [x,y,z], "tolerance_deg": t, "variant": v }
//   { "type": "by_ray",        "origin": [x,y,z], "direction": [x,y,z] }
//   { "type": "top_at_xy",     "x_mm": .., "y_mm": .. }
//   { "type": "cylinder_axis", "axis_origin": [x,y,z], "axis_dir": [x,y,z], "tolerance_deg": t }
//   { "type": "largest" }

namespace {

// Read a 3-number JSON array into out[3]; false if not a valid vec3.
bool readVec3(const json& j, double out[3])
{
    if (!j.is_array() || j.size() != 3) return false;
    for (int i = 0; i < 3; ++i) {
        if (!j[i].is_number()) return false;
        out[i] = j[i].get<double>();
    }
    return true;
}

bool nonZero3(const double v[3])
{
    return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]) > 1e-9;
}

sk::FaceDatum parseFaceDatum(const json& params)
{
    // Explicit face id wins.
    if (params.contains("entry_face_id") && params["entry_face_id"].is_number_integer())
        return sk::FaceIdRef{ params["entry_face_id"].get<int>() };

    if (!params.contains("entry_face")) return sk::FaceLargestPlanar{};
    const json& ef = params["entry_face"];

    // String shorthands.
    if (ef.is_string()) {
        const std::string s = ef.get<std::string>();
        if (s == "top")    return sk::FaceByNormal{ gp_Dir(0.0, 0.0, 1.0),  5.0, "largest" };
        if (s == "bottom") return sk::FaceByNormal{ gp_Dir(0.0, 0.0, -1.0), 5.0, "largest" };
        return sk::FaceLargestPlanar{};   // "largest"/"largest_planar"/unknown
    }

    // Bare [x,y,z] → the face whose normal points that way.
    double v[3];
    if (ef.is_array() && readVec3(ef, v) && nonZero3(v))
        return sk::FaceByNormal{ gp_Dir(v[0], v[1], v[2]), 5.0, "largest" };

    // Full object form: { "type": ..., ... }.
    if (ef.is_object()) {
        const std::string type = ef.value("type", std::string("by_normal"));
        if (type == "largest" || type == "largest_planar")
            return sk::FaceLargestPlanar{};
        if (type == "top_at_xy")
            return sk::FaceTopAtXY{ ef.value("x_mm", 0.0), ef.value("y_mm", 0.0) };
        if (type == "by_ray") {
            double o[3] = {0,0,0}, d[3] = {0,0,1};
            if (ef.contains("origin"))    readVec3(ef["origin"], o);
            if (ef.contains("direction")) readVec3(ef["direction"], d);
            if (nonZero3(d))
                return sk::FaceByRay{ gp_Pnt(o[0],o[1],o[2]), gp_Dir(d[0],d[1],d[2]) };
        }
        if (type == "cylinder_axis") {
            double o[3] = {0,0,0}, d[3] = {0,0,1};
            if (ef.contains("axis_origin")) readVec3(ef["axis_origin"], o);
            if (ef.contains("axis_dir"))    readVec3(ef["axis_dir"], d);
            if (nonZero3(d))
                return sk::FaceCylinderByAxis{
                    gp_Ax1(gp_Pnt(o[0],o[1],o[2]), gp_Dir(d[0],d[1],d[2])),
                    ef.value("tolerance_deg", 5.0) };
        }
        // default: by_normal
        double n[3] = {0,0,1};
        if (ef.contains("normal")) readVec3(ef["normal"], n);
        if (nonZero3(n))
            return sk::FaceByNormal{ gp_Dir(n[0],n[1],n[2]),
                                     ef.value("tolerance_deg", 5.0),
                                     ef.value("variant", std::string("largest")) };
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

sk::bolt_circle_pattern::Input parseBoltCirclePattern(const json& p)
{
    sk::bolt_circle_pattern::Input in;
    in.axis_dir           = parseAxisDir(p);
    // Default the entry face to the one facing OPPOSITE the drilling direction
    // (the grammar's recovered_params carry axis_dir but no entry face); an
    // explicit entry_face / entry_face_id in the params still wins.
    if (p.contains("entry_face") || p.contains("entry_face_id"))
        in.entry_face = parseFaceDatum(p);
    else
        in.entry_face = sk::FaceByNormal{
            gp_Dir(-in.axis_dir.X(), -in.axis_dir.Y(), -in.axis_dir.Z()) };
    in.hole_count         = static_cast<int>(jdouble(p, "hole_count", 0.0));
    in.bolt_circle_dia_mm = jdouble(p, "bolt_circle_dia_mm", 0.0);
    in.hole_dia_mm        = jdouble(p, "hole_dia_mm", 0.0);
    in.center_x_mm        = jdouble(p, "center_x_mm", 0.0);
    in.center_y_mm        = jdouble(p, "center_y_mm", 0.0);
    in.depth_mm           = jdouble(p, "depth_mm", 0.0);
    in.through_hole       = jbool  (p, "through_hole", true);
    in.start_angle_deg    = jdouble(p, "start_angle_deg", 0.0);
    return in;
}

sk::linear_hole_array::Input parseLinearHoleArray(const json& p)
{
    sk::linear_hole_array::Input in;
    in.axis_dir     = parseAxisDir(p);
    if (p.contains("entry_face") || p.contains("entry_face_id"))
        in.entry_face = parseFaceDatum(p);
    else
        in.entry_face = sk::FaceByNormal{
            gp_Dir(-in.axis_dir.X(), -in.axis_dir.Y(), -in.axis_dir.Z()) };
    in.hole_count   = static_cast<int>(jdouble(p, "hole_count", 0.0));
    in.hole_dia_mm  = jdouble(p, "hole_dia_mm", 0.0);
    in.start_x_mm   = jdouble(p, "start_x_mm", 0.0);
    in.start_y_mm   = jdouble(p, "start_y_mm", 0.0);
    in.pitch_mm     = jdouble(p, "pitch_mm", 0.0);
    if (p.contains("direction") && p["direction"].is_array() &&
        p["direction"].size() >= 2) {
        in.dir_x = p["direction"][0].get<double>();
        in.dir_y = p["direction"][1].get<double>();
    }
    in.depth_mm     = jdouble(p, "depth_mm", 0.0);
    in.through_hole = jbool  (p, "through_hole", true);
    return in;
}

sk::rectangular_hole_grid::Input parseRectangularHoleGrid(const json& p)
{
    sk::rectangular_hole_grid::Input in;
    in.axis_dir    = parseAxisDir(p);
    if (p.contains("entry_face") || p.contains("entry_face_id"))
        in.entry_face = parseFaceDatum(p);
    else
        in.entry_face = sk::FaceByNormal{
            gp_Dir(-in.axis_dir.X(), -in.axis_dir.Y(), -in.axis_dir.Z()) };
    in.cols        = static_cast<int>(jdouble(p, "cols", 0.0));
    in.rows        = static_cast<int>(jdouble(p, "rows", 0.0));
    in.hole_dia_mm = jdouble(p, "hole_dia_mm", 0.0);
    in.origin_x_mm = jdouble(p, "origin_x_mm", 0.0);
    in.origin_y_mm = jdouble(p, "origin_y_mm", 0.0);
    in.pitch_u_mm  = jdouble(p, "pitch_u_mm", 0.0);
    in.pitch_v_mm  = jdouble(p, "pitch_v_mm", 0.0);
    auto vec2 = [&](const char* key, double& dx, double& dy) {
        if (p.contains(key) && p[key].is_array() && p[key].size() >= 2) {
            dx = p[key][0].get<double>();
            dy = p[key][1].get<double>();
        }
    };
    vec2("u_dir", in.u_dx, in.u_dy);
    vec2("v_dir", in.v_dx, in.v_dy);
    in.depth_mm     = jdouble(p, "depth_mm", 0.0);
    in.through_hole = jbool  (p, "through_hole", true);
    return in;
}

sk::coaxial_step_bore::Input parseCoaxialStepBore(const json& p)
{
    sk::coaxial_step_bore::Input in;
    in.axis_dir    = parseAxisDir(p);
    if (p.contains("entry_face") || p.contains("entry_face_id"))
        in.entry_face = parseFaceDatum(p);
    else
        in.entry_face = sk::FaceByNormal{
            gp_Dir(-in.axis_dir.X(), -in.axis_dir.Y(), -in.axis_dir.Z()) };
    in.center_x_mm = jdouble(p, "position_x_mm", 0.0);
    in.center_y_mm = jdouble(p, "position_y_mm", 0.0);
    if (p.contains("steps") && p["steps"].is_array()) {
        for (const auto& s : p["steps"]) {
            if (!s.is_object()) continue;
            sk::coaxial_step_bore::Step st;
            st.diameter_mm = s.value("diameter_mm", 0.0);
            st.depth_mm    = s.value("depth_mm", 0.0);
            st.through     = s.value("through", false);
            in.steps.push_back(st);
        }
    }
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
    in.start_z_mm = jdouble(p, "start_z_mm", 0.0);
    in.end_x_mm   = jdouble(p, "end_x_mm",   0.0);
    in.end_y_mm   = jdouble(p, "end_y_mm",   0.0);
    in.end_z_mm   = jdouble(p, "end_z_mm",   0.0);
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

// ── Slice-9 compound bearing/seal/fastener parsers ──────────────────────

sk::plain_bushing_bore_with_lube_groove::Input parsePlainBushingBoreWithLubeGroove(const json& p)
{
    sk::plain_bushing_bore_with_lube_groove::Input in;
    in.entry_face      = parseFaceDatum(p);
    in.position_x_mm   = jdouble(p, "position_x_mm",   0.0);
    in.position_y_mm   = jdouble(p, "position_y_mm",   0.0);
    in.axis_dir        = parseAxisDir(p);
    in.bore_dia_mm     = jdouble(p, "bore_dia_mm",     0.0);
    in.length_mm       = jdouble(p, "length_mm",       0.0);
    in.groove_pitch_mm = jdouble(p, "groove_pitch_mm", 0.0);
    return in;
}

sk::o_ring_groove_as568_spec::Input parseORingGrooveAs568Spec(const json& p)
{
    sk::o_ring_groove_as568_spec::Input in;
    in.face_id     = parseFaceDatum(p);
    in.center_x_mm = jdouble(p, "center_x_mm", 0.0);
    in.center_y_mm = jdouble(p, "center_y_mm", 0.0);
    in.axis_dir    = parseAxisDir(p);
    in.mean_dia_mm = jdouble(p, "mean_dia_mm", 0.0);
    in.dash_size   = jstring(p, "dash_size",   "");
    return in;
}

sk::x_ring_groove::Input parseXRingGroove(const json& p)
{
    sk::x_ring_groove::Input in;
    in.bore_or_shaft_dia_mm = jdouble(p, "bore_or_shaft_dia_mm", 0.0);
    in.mean_dia_mm          = jdouble(p, "mean_dia_mm",          0.0);
    in.position_z_mm        = jdouble(p, "position_z_mm",        0.0);
    in.x_ring_size          = jstring(p, "x_ring_size",          "");
    return in;
}

sk::x_ring_groove_spec::Input parseXRingGrooveSpec(const json& p)
{
    sk::x_ring_groove_spec::Input in;
    in.face_id     = parseFaceDatum(p);
    in.center_x_mm = jdouble(p, "center_x_mm", 0.0);
    in.center_y_mm = jdouble(p, "center_y_mm", 0.0);
    in.axis_dir    = parseAxisDir(p);
    in.mean_dia_mm = jdouble(p, "mean_dia_mm", 0.0);
    in.dash_size   = jstring(p, "dash_size",   "");
    return in;
}

sk::spiral_back_up_ring_groove::Input parseSpiralBackUpRingGroove(const json& p)
{
    sk::spiral_back_up_ring_groove::Input in;
    in.bore_or_shaft_dia_mm = jdouble(p, "bore_or_shaft_dia_mm", 0.0);
    in.position_z_mm        = jdouble(p, "position_z_mm",        0.0);
    in.ring_size            = jstring(p, "ring_size",            "");
    in.position             = jstring(p, "position",             "downstream");
    return in;
}

sk::dust_lip_seal_seat_compound::Input parseDustLipSealSeatCompound(const json& p)
{
    sk::dust_lip_seal_seat_compound::Input in;
    in.entry_face            = parseFaceDatum(p);
    in.center_x_mm           = jdouble(p, "center_x_mm",           0.0);
    in.center_y_mm           = jdouble(p, "center_y_mm",           0.0);
    in.axis_dir              = parseAxisDir(p);
    in.bore_dia_mm           = jdouble(p, "bore_dia_mm",           0.0);
    in.bore_depth_mm         = jdouble(p, "bore_depth_mm",         0.0);
    in.seal_thickness_mm     = jdouble(p, "seal_thickness_mm",     7.0);
    in.seal_can_OD_offset_mm = jdouble(p, "seal_can_OD_offset_mm", 0.5);
    in.lip_relief_depth_mm   = jdouble(p, "lip_relief_depth_mm",   0.5);
    in.nose_chamfer_mm       = jdouble(p, "nose_chamfer_mm",       0.5);
    in.seal_series           = jstring(p, "seal_series",           "");
    return in;
}

sk::face_seal_compound_compression::Input parseFaceSealCompoundCompression(const json& p)
{
    sk::face_seal_compound_compression::Input in;
    in.face_id               = parseFaceDatum(p);
    in.center_x_mm           = jdouble(p, "center_x_mm",           0.0);
    in.center_y_mm           = jdouble(p, "center_y_mm",           0.0);
    in.axis_dir              = parseAxisDir(p);
    in.primary_radius_mm     = jdouble(p, "primary_radius_mm",     0.0);
    in.spring_radius_mm      = jdouble(p, "spring_radius_mm",      0.0);
    in.spring_extra_depth_mm = jdouble(p, "spring_extra_depth_mm", 0.5);
    in.dash_size             = jstring(p, "dash_size",             "");
    return in;
}

sk::gasket_face_with_drain_groove::Input parseGasketFaceWithDrainGroove(const json& p)
{
    sk::gasket_face_with_drain_groove::Input in;
    in.face_id          = parseFaceDatum(p);
    in.center_x_mm      = jdouble(p, "center_x_mm",      0.0);
    in.center_y_mm      = jdouble(p, "center_y_mm",      0.0);
    in.axis_dir         = parseAxisDir(p);
    in.face_dia_mm      = jdouble(p, "face_dia_mm",      0.0);
    in.relief_depth_mm  = jdouble(p, "relief_depth_mm",  0.05);
    in.drain_radius_mm  = jdouble(p, "drain_radius_mm",  0.0);
    in.drain_width_mm   = jdouble(p, "drain_width_mm",   1.0);
    in.drain_depth_mm   = jdouble(p, "drain_depth_mm",   0.5);
    in.id_chamfer_mm    = jdouble(p, "id_chamfer_mm",    0.5);
    in.gasket_class     = jstring(p, "gasket_class",     "");
    in.roughness_class  = jstring(p, "roughness_class",  "stock");
    return in;
}

sk::blind_threaded_insert_seat::Input parseBlindThreadedInsertSeat(const json& p)
{
    sk::blind_threaded_insert_seat::Input in;
    in.entry_face      = parseFaceDatum(p);
    in.position_x_mm   = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm   = jdouble(p, "position_y_mm", 0.0);
    in.axis_dir        = parseAxisDir(p);
    in.insert_size     = jstring(p, "insert_size",     "M3");
    in.insert_material = jstring(p, "insert_material", "metal");
    return in;
}

sk::bolt_hole_metric_spec::Input parseBoltHoleMetricSpec(const json& p)
{
    sk::bolt_hole_metric_spec::Input in;
    in.entry_face        = parseFaceDatum(p);
    in.position_x_mm     = jdouble(p, "position_x_mm",     0.0);
    in.position_y_mm     = jdouble(p, "position_y_mm",     0.0);
    in.axis_dir          = parseAxisDir(p);
    in.thread_size       = jstring(p, "thread_size",       "M6");
    in.fit_class         = jstring(p, "fit_class",         "medium");
    in.chamfer_size_mm   = jdouble(p, "chamfer_size_mm",   0.5);
    in.chamfer_angle_deg = jdouble(p, "chamfer_angle_deg", 30.0);
    return in;
}

sk::tapped_hole_metric_spec::Input parseTappedHoleMetricSpec(const json& p)
{
    sk::tapped_hole_metric_spec::Input in;
    in.entry_face            = parseFaceDatum(p);
    in.position_x_mm         = jdouble(p, "position_x_mm",         0.0);
    in.position_y_mm         = jdouble(p, "position_y_mm",         0.0);
    in.axis_dir              = parseAxisDir(p);
    in.thread_size           = jstring(p, "thread_size",           "M6");
    in.tap_depth_mm          = jdouble(p, "tap_depth_mm",          8.0);
    in.through               = jbool  (p, "through",               false);
    in.chamfer_size_mm       = jdouble(p, "chamfer_size_mm",       0.4);
    in.pilot_extra_depth_mm  = jdouble(p, "pilot_extra_depth_mm",  1.0);
    return in;
}

sk::unc_unf_hole_spec::Input parseUncUnfHoleSpec(const json& p)
{
    sk::unc_unf_hole_spec::Input in;
    in.entry_face        = parseFaceDatum(p);
    in.position_x_mm     = jdouble(p, "position_x_mm",     0.0);
    in.position_y_mm     = jdouble(p, "position_y_mm",     0.0);
    in.axis_dir          = parseAxisDir(p);
    in.fastener_size     = jstring(p, "fastener_size",     "1/4-20");
    in.fit_class         = jstring(p, "fit_class",         "normal");
    in.chamfer_size_mm   = jdouble(p, "chamfer_size_mm",   0.5);
    in.chamfer_angle_deg = jdouble(p, "chamfer_angle_deg", 45.0);
    return in;
}

sk::threaded_through_with_chamfers::Input parseThreadedThroughWithChamfers(const json& p)
{
    sk::threaded_through_with_chamfers::Input in;
    in.entry_face       = parseFaceDatum(p);
    in.position_x_mm    = jdouble(p, "position_x_mm",    0.0);
    in.position_y_mm    = jdouble(p, "position_y_mm",    0.0);
    in.axis_dir         = parseAxisDir(p);
    in.thread_size      = jstring(p, "thread_size",      "M6");
    in.chamfer_size_mm  = jdouble(p, "chamfer_size_mm",  0.5);
    in.add_relief       = jbool  (p, "add_relief",       true);
    in.relief_offset_mm = jdouble(p, "relief_offset_mm", 2.0);
    return in;
}

sk::captive_screw_pocket_spec::Input parseCaptiveScrewPocketSpec(const json& p)
{
    sk::captive_screw_pocket_spec::Input in;
    in.entry_face       = parseFaceDatum(p);
    in.position_x_mm    = jdouble(p, "position_x_mm",    0.0);
    in.position_y_mm    = jdouble(p, "position_y_mm",    0.0);
    in.axis_dir         = parseAxisDir(p);
    in.thread_size      = jstring(p, "thread_size",      "M6");
    in.chamfer_size_mm  = jdouble(p, "chamfer_size_mm",  0.5);
    in.groove_offset_mm = jdouble(p, "groove_offset_mm", 2.0);
    return in;
}

// ── Mechanical structures / drive / fluid / spring / linear-motion / adjusters ──

// Helper: parse a gp_Ax1 from JSON of the form
//   { "origin": [x,y,z], "axis": [x,y,z] }
// or via separate keys "origin_<axis>_mm" / "axis_dir".  Falls back to
// (origin=(0,0,0), direction=(0,0,-1)) when JSON is incomplete.
gp_Ax1 parseAx1(const json& p, const char* origin_key = "origin",
                const char* axis_key = "axis",
                const gp_Dir& dflt_dir = gp_Dir(0.0, 0.0, -1.0))
{
    gp_Pnt origin(0.0, 0.0, 0.0);
    gp_Dir dir = dflt_dir;
    if (p.contains(origin_key) && p[origin_key].is_array() && p[origin_key].size() == 3 &&
        p[origin_key][0].is_number() && p[origin_key][1].is_number() && p[origin_key][2].is_number()) {
        origin = gp_Pnt(p[origin_key][0].get<double>(),
                        p[origin_key][1].get<double>(),
                        p[origin_key][2].get<double>());
    }
    if (p.contains(axis_key) && p[axis_key].is_array() && p[axis_key].size() == 3 &&
        p[axis_key][0].is_number() && p[axis_key][1].is_number() && p[axis_key][2].is_number()) {
        const double x = p[axis_key][0].get<double>();
        const double y = p[axis_key][1].get<double>();
        const double z = p[axis_key][2].get<double>();
        if (std::sqrt(x*x + y*y + z*z) > 1e-9) {
            dir = gp_Dir(x, y, z);
        }
    }
    return gp_Ax1(origin, dir);
}

// Frame/chassis
sk::i_beam_compound_section::Input parseIBeamCompoundSection(const json& p)
{
    sk::i_beam_compound_section::Input in;
    in.length_mm   = jdouble(p, "length_mm",   in.length_mm);
    in.height_mm   = jdouble(p, "height_mm",   in.height_mm);
    in.flange_w_mm = jdouble(p, "flange_w_mm", in.flange_w_mm);
    in.flange_t_mm = jdouble(p, "flange_t_mm", in.flange_t_mm);
    in.web_t_mm    = jdouble(p, "web_t_mm",    in.web_t_mm);
    return in;
}

sk::box_section_with_endplate::Input parseBoxSectionWithEndplate(const json& p)
{
    sk::box_section_with_endplate::Input in;
    in.length_mm     = jdouble(p, "length_mm",     in.length_mm);
    in.width_mm      = jdouble(p, "width_mm",      in.width_mm);
    in.height_mm     = jdouble(p, "height_mm",     in.height_mm);
    in.wall_t_mm     = jdouble(p, "wall_t_mm",     in.wall_t_mm);
    in.endplate_t_mm = jdouble(p, "endplate_t_mm", in.endplate_t_mm);
    if (p.contains("bolt_grid") && p["bolt_grid"].is_array() && p["bolt_grid"].size() == 2 &&
        p["bolt_grid"][0].is_number_integer() && p["bolt_grid"][1].is_number_integer()) {
        in.bolt_grid = { p["bolt_grid"][0].get<int>(), p["bolt_grid"][1].get<int>() };
    }
    in.bolt_dia_mm   = jdouble(p, "bolt_dia_mm",   in.bolt_dia_mm);
    in.edge_dist_mm  = jdouble(p, "edge_dist_mm",  in.edge_dist_mm);
    return in;
}

sk::gusset_plate_compound::Input parseGussetPlateCompound(const json& p)
{
    sk::gusset_plate_compound::Input in;
    in.leg_a_mm         = jdouble(p, "leg_a_mm",         in.leg_a_mm);
    in.leg_b_mm         = jdouble(p, "leg_b_mm",         in.leg_b_mm);
    in.plate_t_mm       = jdouble(p, "plate_t_mm",       in.plate_t_mm);
    in.lift_hole_dia_mm = jdouble(p, "lift_hole_dia_mm", in.lift_hole_dia_mm);
    in.bevel_mm         = jdouble(p, "bevel_mm",         in.bevel_mm);
    in.notch_r_mm       = jdouble(p, "notch_r_mm",       in.notch_r_mm);
    return in;
}

sk::lifting_lug_pad_eye::Input parseLiftingLugPadEye(const json& p)
{
    sk::lifting_lug_pad_eye::Input in;
    in.plate_t_mm      = jdouble(p, "plate_t_mm",      in.plate_t_mm);
    in.pin_hole_dia_mm = jdouble(p, "pin_hole_dia_mm", in.pin_hole_dia_mm);
    in.lug_height_mm   = jdouble(p, "lug_height_mm",   in.lug_height_mm);
    in.lug_width_mm    = jdouble(p, "lug_width_mm",    in.lug_width_mm);
    in.chamfer_mm      = jdouble(p, "chamfer_mm",      in.chamfer_mm);
    in.base_bevel_mm   = jdouble(p, "base_bevel_mm",   in.base_bevel_mm);
    return in;
}

sk::base_bracket_compound::Input parseBaseBracketCompound(const json& p)
{
    sk::base_bracket_compound::Input in;
    in.leg1_mm          = jdouble(p, "leg1_mm",          in.leg1_mm);
    in.leg2_mm          = jdouble(p, "leg2_mm",          in.leg2_mm);
    in.plate_t_mm       = jdouble(p, "plate_t_mm",       in.plate_t_mm);
    in.bracket_width_mm = jdouble(p, "bracket_width_mm", in.bracket_width_mm);
    if (p.contains("bolt_pattern") && p["bolt_pattern"].is_array() && p["bolt_pattern"].size() == 2 &&
        p["bolt_pattern"][0].is_number_integer() && p["bolt_pattern"][1].is_number_integer()) {
        in.bolt_pattern = { p["bolt_pattern"][0].get<int>(), p["bolt_pattern"][1].get<int>() };
    }
    in.bolt_dia_mm    = jdouble(p, "bolt_dia_mm",    in.bolt_dia_mm);
    in.dowel_dia_mm   = jdouble(p, "dowel_dia_mm",   in.dowel_dia_mm);
    in.gusset_size_mm = jdouble(p, "gusset_size_mm", in.gusset_size_mm);
    return in;
}

// Mounting
sk::din_rail_mount_slot::Input parseDinRailMountSlot(const json& p)
{
    sk::din_rail_mount_slot::Input in;
    in.entry_face      = parseFaceDatum(p);
    in.start_x_mm      = jdouble(p, "start_x_mm",      0.0);
    in.start_y_mm      = jdouble(p, "start_y_mm",      0.0);
    in.rail_dir        = parseDir(p, "rail_dir", gp_Dir(1.0, 0.0, 0.0));
    in.axis_dir        = parseAxisDir(p);
    in.rail_length_mm  = jdouble(p, "rail_length_mm",  0.0);
    in.plate_t_mm      = jdouble(p, "plate_t_mm",      in.plate_t_mm);
    return in;
}

sk::dovetail_mount_compound::Input parseDovetailMountCompound(const json& p)
{
    sk::dovetail_mount_compound::Input in;
    in.entry_face          = parseFaceDatum(p);
    in.start_x_mm          = jdouble(p, "start_x_mm",          0.0);
    in.start_y_mm          = jdouble(p, "start_y_mm",          0.0);
    in.slot_dir            = parseDir(p, "slot_dir", gp_Dir(1.0, 0.0, 0.0));
    in.axis_dir            = parseAxisDir(p);
    in.slot_length_mm      = jdouble(p, "slot_length_mm",      0.0);
    in.slot_width_mm       = jdouble(p, "slot_width_mm",       0.0);
    in.slot_depth_mm       = jdouble(p, "slot_depth_mm",       0.0);
    in.dovetail_angle_deg  = jdouble(p, "dovetail_angle_deg",  in.dovetail_angle_deg);
    return in;
}

sk::t_slot_table_groove::Input parseTSlotTableGroove(const json& p)
{
    sk::t_slot_table_groove::Input in;
    in.entry_face = parseFaceDatum(p);
    in.start_x_mm = jdouble(p, "start_x_mm", 0.0);
    in.start_y_mm = jdouble(p, "start_y_mm", 0.0);
    in.end_x_mm   = jdouble(p, "end_x_mm",   0.0);
    in.end_y_mm   = jdouble(p, "end_y_mm",   0.0);
    in.axis_dir   = parseAxisDir(p);
    in.top_w_mm   = jdouble(p, "top_w_mm",   0.0);
    in.top_d_mm   = jdouble(p, "top_d_mm",   0.0);
    in.bot_w_mm   = jdouble(p, "bot_w_mm",   0.0);
    in.bot_d_mm   = jdouble(p, "bot_d_mm",   0.0);
    return in;
}

// Linear motion
sk::linear_rail_seat_compound::Input parseLinearRailSeatCompound(const json& p)
{
    sk::linear_rail_seat_compound::Input in;
    in.entry_face          = parseFaceDatum(p);
    in.length_mm           = jdouble(p, "length_mm",           0.0);
    in.rail_centerline_mm  = jdouble(p, "rail_centerline_mm",  0.0);
    in.pitch_mm            = jdouble(p, "pitch_mm",            0.0);
    in.start_x_mm          = jdouble(p, "start_x_mm",          0.0);
    return in;
}

sk::ball_screw_nut_pocket::Input parseBallScrewNutPocket(const json& p)
{
    sk::ball_screw_nut_pocket::Input in;
    in.entry_face            = parseFaceDatum(p);
    in.position_x_mm         = jdouble(p, "position_x_mm",         0.0);
    in.position_y_mm         = jdouble(p, "position_y_mm",         0.0);
    in.axis_dir              = parseAxisDir(p);
    in.nut_od_mm             = jdouble(p, "nut_od_mm",             0.0);
    in.nut_l_mm              = jdouble(p, "nut_l_mm",              0.0);
    in.mounting_pattern      = jstring(p, "mounting_pattern",      "square4");
    in.mounting_pcd_mm       = jdouble(p, "mounting_pcd_mm",       0.0);
    in.mounting_hole_dia_mm  = jdouble(p, "mounting_hole_dia_mm",  in.mounting_hole_dia_mm);
    in.shaft_thru_dia_mm     = jdouble(p, "shaft_thru_dia_mm",     0.0);
    return in;
}

sk::lead_screw_anti_backlash_pocket::Input parseLeadScrewAntiBacklashPocket(const json& p)
{
    sk::lead_screw_anti_backlash_pocket::Input in;
    in.entry_face       = parseFaceDatum(p);
    in.position_x_mm    = jdouble(p, "position_x_mm",    0.0);
    in.position_y_mm    = jdouble(p, "position_y_mm",    0.0);
    in.axis_dir         = parseAxisDir(p);
    in.half_nut_od_mm   = jdouble(p, "half_nut_od_mm",   0.0);
    in.half_nut_l_mm    = jdouble(p, "half_nut_l_mm",    0.0);
    in.spring_slot_w_mm = jdouble(p, "spring_slot_w_mm", 0.0);
    return in;
}

sk::linear_bushing_seat::Input parseLinearBushingSeat(const json& p)
{
    sk::linear_bushing_seat::Input in;
    in.entry_face    = parseFaceDatum(p);
    in.axis          = parseAx1(p, "axis_origin", "axis_dir");
    in.bushing_od_mm = jdouble(p, "bushing_od_mm", 0.0);
    in.bushing_l_mm  = jdouble(p, "bushing_l_mm",  0.0);
    return in;
}

sk::cam_follower_threaded_seat::Input parseCamFollowerThreadedSeat(const json& p)
{
    sk::cam_follower_threaded_seat::Input in;
    in.entry_face      = parseFaceDatum(p);
    in.position_x_mm   = jdouble(p, "position_x_mm",   0.0);
    in.position_y_mm   = jdouble(p, "position_y_mm",   0.0);
    in.axis_dir        = parseAxisDir(p);
    in.cam_follower_M  = jstring(p, "cam_follower_M",  "M8");
    in.tap_depth_mm    = jdouble(p, "tap_depth_mm",    0.0);
    return in;
}

// Fluid ports
sk::internal_water_jacket::Input parseInternalWaterJacket(const json& p)
{
    sk::internal_water_jacket::Input in;
    in.reference_face       = parseFaceDatum(p);
    in.bore_axis            = parseDir(p, "bore_axis", gp_Dir(0.0, 0.0, 1.0));
    in.bore_origin_x_mm     = jdouble(p, "bore_origin_x_mm",     in.bore_origin_x_mm);
    in.bore_origin_y_mm     = jdouble(p, "bore_origin_y_mm",     in.bore_origin_y_mm);
    in.bore_axial_start_mm  = jdouble(p, "bore_axial_start_mm",  in.bore_axial_start_mm);
    in.bore_inner_r_mm      = jdouble(p, "bore_inner_r_mm",      in.bore_inner_r_mm);
    in.outer_r_mm           = jdouble(p, "outer_r_mm",           in.outer_r_mm);
    in.jacket_length_mm     = jdouble(p, "jacket_length_mm",     in.jacket_length_mm);
    in.jacket_pitch_mm      = jdouble(p, "jacket_pitch_mm",      in.jacket_pitch_mm);
    in.groove_w_mm          = jdouble(p, "groove_w_mm",          in.groove_w_mm);
    in.groove_depth_mm      = jdouble(p, "groove_depth_mm",      in.groove_depth_mm);
    in.port_dia_mm          = jdouble(p, "port_dia_mm",          in.port_dia_mm);
    in.inlet_axial_pos_mm   = jdouble(p, "inlet_axial_pos_mm",   in.inlet_axial_pos_mm);
    in.outlet_axial_pos_mm  = jdouble(p, "outlet_axial_pos_mm",  in.outlet_axial_pos_mm);
    in.inlet_clock_deg      = jdouble(p, "inlet_clock_deg",      in.inlet_clock_deg);
    in.outlet_clock_deg     = jdouble(p, "outlet_clock_deg",     in.outlet_clock_deg);
    return in;
}

// Adjusters
sk::adjuster_screw_pocket_compound::Input parseAdjusterScrewPocketCompound(const json& p)
{
    sk::adjuster_screw_pocket_compound::Input in;
    in.entry_face              = parseFaceDatum(p);
    in.position_x_mm           = jdouble(p, "position_x_mm",           0.0);
    in.position_y_mm           = jdouble(p, "position_y_mm",           0.0);
    in.axis_dir                = parseAxisDir(p);
    in.fine_thread             = jstring(p, "fine_thread",             "M4x0.5");
    in.lock_nut_M              = jstring(p, "lock_nut_M",              "M4");
    in.tap_depth_mm            = jdouble(p, "tap_depth_mm",            0.0);
    in.nut_thickness_mm        = jdouble(p, "nut_thickness_mm",        0.0);
    in.scribe_groove_width_mm  = jdouble(p, "scribe_groove_width_mm",  in.scribe_groove_width_mm);
    in.scribe_groove_depth_mm  = jdouble(p, "scribe_groove_depth_mm",  in.scribe_groove_depth_mm);
    return in;
}

sk::tab_lock_anti_rotation::Input parseTabLockAntiRotation(const json& p)
{
    sk::tab_lock_anti_rotation::Input in;
    in.bore_axis       = parseAx1(p, "bore_origin", "bore_axis_dir", gp_Dir(0.0, 0.0, 1.0));
    in.bore_dia_mm     = jdouble(p, "bore_dia_mm",     0.0);
    in.tab_w_mm        = jdouble(p, "tab_w_mm",        0.0);
    in.tab_d_mm        = jdouble(p, "tab_d_mm",        0.0);
    in.tab_axial_z_mm  = jdouble(p, "tab_axial_z_mm",  0.0);
    in.chamfer_size_mm = jdouble(p, "chamfer_size_mm", in.chamfer_size_mm);
    return in;
}

// Spring seats
sk::coil_spring_seat_compound::Input parseCoilSpringSeatCompound(const json& p)
{
    sk::coil_spring_seat_compound::Input in;
    in.face_id        = parseFaceDatum(p);
    in.position_x_mm  = jdouble(p, "position_x_mm",  0.0);
    in.position_y_mm  = jdouble(p, "position_y_mm",  0.0);
    in.axis_dir       = parseAxisDir(p);
    in.spring_od      = jdouble(p, "spring_od",      0.0);
    in.free_height    = jdouble(p, "free_height",    0.0);
    in.pilot_dia      = jdouble(p, "pilot_dia",      0.0);
    in.slip_fit_mm    = jdouble(p, "slip_fit_mm",    in.slip_fit_mm);
    return in;
}

sk::wave_spring_groove::Input parseWaveSpringGroove(const json& p)
{
    sk::wave_spring_groove::Input in;
    in.bore_axis          = parseAx1(p, "bore_origin", "bore_axis_dir", gp_Dir(0.0, 0.0, 1.0));
    in.mean_dia           = jdouble(p, "mean_dia",           0.0);
    in.wave_spring_id     = jdouble(p, "wave_spring_id",     0.0);
    in.wave_spring_height = jdouble(p, "wave_spring_height", 0.0);
    in.axial_position_mm  = jdouble(p, "axial_position_mm",  0.0);
    return in;
}

sk::gas_spring_clevis_pocket::Input parseGasSpringClevisPocket(const json& p)
{
    sk::gas_spring_clevis_pocket::Input in;
    in.face_id             = parseFaceDatum(p);
    in.position_x_mm       = jdouble(p, "position_x_mm",       0.0);
    in.position_y_mm       = jdouble(p, "position_y_mm",       0.0);
    in.axis_dir            = parseAxisDir(p);
    in.clevis_w            = jdouble(p, "clevis_w",            0.0);
    in.clevis_d            = jdouble(p, "clevis_d",            0.0);
    in.clevis_length       = jdouble(p, "clevis_length",       0.0);
    in.pin_dia             = jdouble(p, "pin_dia",             in.pin_dia);
    in.retention_groove_w  = jdouble(p, "retention_groove_w",  in.retention_groove_w);
    return in;
}

sk::leaf_spring_anchor_compound::Input parseLeafSpringAnchorCompound(const json& p)
{
    sk::leaf_spring_anchor_compound::Input in;
    in.face_id        = parseFaceDatum(p);
    in.position_x_mm  = jdouble(p, "position_x_mm",  0.0);
    in.position_y_mm  = jdouble(p, "position_y_mm",  0.0);
    in.axis_dir       = parseAxisDir(p);
    in.slot_length_mm = jdouble(p, "slot_length_mm", 0.0);
    in.slot_width_mm  = jdouble(p, "slot_width_mm",  0.0);
    in.slot_depth_mm  = jdouble(p, "slot_depth_mm",  0.0);
    in.screw_M        = jstring(p, "screw_M",        "M4");
    return in;
}

sk::torsion_spring_anchor_compound::Input parseTorsionSpringAnchorCompound(const json& p)
{
    sk::torsion_spring_anchor_compound::Input in;
    in.face_id             = parseFaceDatum(p);
    in.position_x_mm       = jdouble(p, "position_x_mm",       0.0);
    in.position_y_mm       = jdouble(p, "position_y_mm",       0.0);
    in.axis_dir            = parseAxisDir(p);
    in.pivot_bore          = jdouble(p, "pivot_bore",          0.0);
    in.pivot_depth_mm      = jdouble(p, "pivot_depth_mm",      0.0);
    in.arm_slot_width      = jdouble(p, "arm_slot_width",      0.0);
    in.arm_slot_length     = jdouble(p, "arm_slot_length",     0.0);
    in.anchor_angle_1_deg  = jdouble(p, "anchor_angle_1_deg",  in.anchor_angle_1_deg);
    in.anchor_angle_2_deg  = jdouble(p, "anchor_angle_2_deg",  in.anchor_angle_2_deg);
    in.anchor_radius_mm    = jdouble(p, "anchor_radius_mm",    0.0);
    return in;
}

sk::leaf_spring_anchor::Input parseLeafSpringAnchor(const json& p)
{
    sk::leaf_spring_anchor::Input in;
    in.entry_face     = parseFaceDatum(p);
    in.position_x_mm  = jdouble(p, "position_x_mm",  0.0);
    in.position_y_mm  = jdouble(p, "position_y_mm",  0.0);
    in.axis_dir       = parseAxisDir(p);
    in.slot_length_mm = jdouble(p, "slot_length_mm", 0.0);
    in.slot_width_mm  = jdouble(p, "slot_width_mm",  0.0);
    in.depth_mm       = jdouble(p, "depth_mm",       in.depth_mm);
    return in;
}

// ── Slice 16/smart-spec/PCB-electronics parsers ──────────────────────────

sk::iso_h7_bore_spec::Input parseIsoH7BoreSpec(const json& p)
{
    sk::iso_h7_bore_spec::Input in;
    in.entry_face     = parseFaceDatum(p);
    in.position_x_mm  = jdouble(p, "position_x_mm",  0.0);
    in.position_y_mm  = jdouble(p, "position_y_mm",  0.0);
    in.axis_dir       = parseAxisDir(p);
    in.nominal_dia_mm = jdouble(p, "nominal_dia_mm", 0.0);
    in.depth_mm       = jdouble(p, "depth_mm",       0.0);
    in.chamfer_mm     = jdouble(p, "chamfer_mm",     in.chamfer_mm);
    in.spec_key       = jstring(p, "spec_key",       in.spec_key.c_str());
    return in;
}

sk::press_fit_p7_bore_spec::Input parsePressFitP7BoreSpec(const json& p)
{
    sk::press_fit_p7_bore_spec::Input in;
    in.entry_face     = parseFaceDatum(p);
    in.position_x_mm  = jdouble(p, "position_x_mm",  0.0);
    in.position_y_mm  = jdouble(p, "position_y_mm",  0.0);
    in.axis_dir       = parseAxisDir(p);
    in.nominal_dia_mm = jdouble(p, "nominal_dia_mm", 0.0);
    in.depth_mm       = jdouble(p, "depth_mm",       0.0);
    in.chamfer_mm     = jdouble(p, "chamfer_mm",     in.chamfer_mm);
    in.spec_key       = jstring(p, "spec_key",       in.spec_key.c_str());
    return in;
}

sk::slip_fit_h11_bore_spec::Input parseSlipFitH11BoreSpec(const json& p)
{
    sk::slip_fit_h11_bore_spec::Input in;
    in.entry_face     = parseFaceDatum(p);
    in.position_x_mm  = jdouble(p, "position_x_mm",  0.0);
    in.position_y_mm  = jdouble(p, "position_y_mm",  0.0);
    in.axis_dir       = parseAxisDir(p);
    in.nominal_dia_mm = jdouble(p, "nominal_dia_mm", 0.0);
    in.depth_mm       = jdouble(p, "depth_mm",       0.0);
    in.chamfer_mm     = jdouble(p, "chamfer_mm",     in.chamfer_mm);
    in.spec_key       = jstring(p, "spec_key",       in.spec_key.c_str());
    return in;
}

sk::dowel_pin_h6_bore::Input parseDowelPinH6Bore(const json& p)
{
    sk::dowel_pin_h6_bore::Input in;
    in.entry_face     = parseFaceDatum(p);
    in.position_x_mm  = jdouble(p, "position_x_mm",  0.0);
    in.position_y_mm  = jdouble(p, "position_y_mm",  0.0);
    in.axis_dir       = parseAxisDir(p);
    in.nominal_dia_mm = jdouble(p, "nominal_dia_mm", 0.0);
    in.depth_mm       = jdouble(p, "depth_mm",       0.0);
    in.chamfer_mm     = jdouble(p, "chamfer_mm",     in.chamfer_mm);
    in.spec_key       = jstring(p, "spec_key",       in.spec_key.c_str());
    return in;
}

sk::locating_g6_bore::Input parseLocatingG6Bore(const json& p)
{
    sk::locating_g6_bore::Input in;
    in.entry_face     = parseFaceDatum(p);
    in.position_x_mm  = jdouble(p, "position_x_mm",  0.0);
    in.position_y_mm  = jdouble(p, "position_y_mm",  0.0);
    in.axis_dir       = parseAxisDir(p);
    in.nominal_dia_mm = jdouble(p, "nominal_dia_mm", 0.0);
    in.depth_mm       = jdouble(p, "depth_mm",       0.0);
    in.chamfer_mm     = jdouble(p, "chamfer_mm",     in.chamfer_mm);
    in.spec_key       = jstring(p, "spec_key",       in.spec_key.c_str());
    return in;
}

sk::parametric_rib_array::Input parseParametricRibArray(const json& p)
{
    sk::parametric_rib_array::Input in;
    in.entry_face   = parseFaceDatum(p);
    in.spec_class   = jstring(p, "spec_class",   "");
    if (p.contains("rib_count") && p["rib_count"].is_number_integer()) {
        in.rib_count = p["rib_count"].get<int>();
    }
    in.pitch_mm     = jdouble(p, "pitch_mm",     0.0);
    in.length_mm    = jdouble(p, "length_mm",    0.0);
    in.height_mm    = jdouble(p, "height_mm",    0.0);
    in.thickness_mm = jdouble(p, "thickness_mm", 0.0);
    in.anchor_x_mm  = jdouble(p, "anchor_x_mm",  0.0);
    in.anchor_y_mm  = jdouble(p, "anchor_y_mm",  0.0);
    in.array_axis   = parseDir(p, "array_axis",  gp_Dir(1.0, 0.0, 0.0));
    in.extrude_dir  = parseDir(p, "extrude_dir", gp_Dir(0.0, 0.0, 1.0));
    return in;
}

sk::draft_wall_with_radius::Input parseDraftWallWithRadius(const json& p)
{
    sk::draft_wall_with_radius::Input in;
    in.entry_face        = parseFaceDatum(p);
    in.spec_class        = jstring(p, "spec_class",        "");
    in.center_x_mm       = jdouble(p, "center_x_mm",       0.0);
    in.center_y_mm       = jdouble(p, "center_y_mm",       0.0);
    in.length_mm         = jdouble(p, "length_mm",         0.0);
    in.base_thickness_mm = jdouble(p, "base_thickness_mm", 0.0);
    in.height_mm         = jdouble(p, "height_mm",         0.0);
    in.length_dir        = parseDir(p, "length_dir", gp_Dir(1.0, 0.0, 0.0));
    return in;
}

sk::top_face_recess_with_walls::Input parseTopFaceRecessWithWalls(const json& p)
{
    sk::top_face_recess_with_walls::Input in;
    in.entry_face      = parseFaceDatum(p);
    in.spec_class      = jstring(p, "spec_class",      "");
    in.center_x_mm     = jdouble(p, "center_x_mm",     0.0);
    in.center_y_mm     = jdouble(p, "center_y_mm",     0.0);
    in.inner_length_mm = jdouble(p, "inner_length_mm", 0.0);
    in.inner_width_mm  = jdouble(p, "inner_width_mm",  0.0);
    in.recess_depth_mm = jdouble(p, "recess_depth_mm", 0.0);
    return in;
}

sk::partition_wall_with_passthrough::Input parsePartitionWallWithPassthrough(const json& p)
{
    sk::partition_wall_with_passthrough::Input in;
    in.entry_face         = parseFaceDatum(p);
    in.spec_class         = jstring(p, "spec_class",         "");
    in.center_x_mm        = jdouble(p, "center_x_mm",        0.0);
    in.center_y_mm        = jdouble(p, "center_y_mm",        0.0);
    in.wall_length_mm     = jdouble(p, "wall_length_mm",     0.0);
    in.wall_thickness_mm  = jdouble(p, "wall_thickness_mm",  0.0);
    in.wall_height_mm     = jdouble(p, "wall_height_mm",     0.0);
    in.cut_offset_x_mm    = jdouble(p, "cut_offset_x_mm",    0.0);
    in.cut_offset_z_mm    = jdouble(p, "cut_offset_z_mm",    0.0);
    in.length_dir         = parseDir(p, "length_dir", gp_Dir(1.0, 0.0, 0.0));
    return in;
}

sk::curved_lip_around_face::Input parseCurvedLipAroundFace(const json& p)
{
    sk::curved_lip_around_face::Input in;
    in.entry_face = parseFaceDatum(p);
    in.spec_class = jstring(p, "spec_class", "");
    return in;
}

sk::heat_sink_fin_array::Input parseHeatSinkFinArray(const json& p)
{
    sk::heat_sink_fin_array::Input in;
    in.base_face         = parseFaceDatum(p);
    if (p.contains("fin_count") && p["fin_count"].is_number_integer()) {
        in.fin_count = p["fin_count"].get<int>();
    }
    in.spec_key          = jstring(p, "spec_key",          "");
    in.fin_thickness_mm  = jdouble(p, "fin_thickness_mm",  0.0);
    in.fin_height_mm     = jdouble(p, "fin_height_mm",     0.0);
    in.fin_pitch_mm      = jdouble(p, "fin_pitch_mm",      0.0);
    in.airflow_dir       = parseDir(p, "airflow_dir", gp_Dir(1.0, 0.0, 0.0));
    in.origin_x_mm       = jdouble(p, "origin_x_mm",       0.0);
    in.origin_y_mm       = jdouble(p, "origin_y_mm",       0.0);
    in.fin_length_mm     = jdouble(p, "fin_length_mm",     0.0);
    return in;
}

sk::vent_slot_array::Input parseVentSlotArray(const json& p)
{
    sk::vent_slot_array::Input in;
    in.face              = parseFaceDatum(p);
    if (p.contains("slot_count") && p["slot_count"].is_number_integer()) {
        in.slot_count = p["slot_count"].get<int>();
    }
    in.spec_key          = jstring(p, "spec_key",          "");
    in.slot_length_mm    = jdouble(p, "slot_length_mm",    0.0);
    in.slot_width_mm     = jdouble(p, "slot_width_mm",     0.0);
    in.pitch_mm          = jdouble(p, "pitch_mm",          0.0);
    in.slot_axis         = parseDir(p, "slot_axis", gp_Dir(1.0, 0.0, 0.0));
    in.center_x_mm       = jdouble(p, "center_x_mm",       0.0);
    in.center_y_mm       = jdouble(p, "center_y_mm",       0.0);
    in.wall_thickness_mm = jdouble(p, "wall_thickness_mm", 0.0);
    return in;
}

sk::louvered_vent::Input parseLouveredVent(const json& p)
{
    sk::louvered_vent::Input in;
    in.face              = parseFaceDatum(p);
    if (p.contains("louver_count") && p["louver_count"].is_number_integer()) {
        in.louver_count = p["louver_count"].get<int>();
    }
    in.spec_key          = jstring(p, "spec_key",          "");
    in.tilt_angle_deg    = jdouble(p, "tilt_angle_deg",    0.0);
    in.slot_length_mm    = jdouble(p, "slot_length_mm",    0.0);
    in.slot_width_mm     = jdouble(p, "slot_width_mm",     0.0);
    in.pitch_mm          = jdouble(p, "pitch_mm",          0.0);
    in.slot_axis         = parseDir(p, "slot_axis", gp_Dir(1.0, 0.0, 0.0));
    in.center_x_mm       = jdouble(p, "center_x_mm",       0.0);
    in.center_y_mm       = jdouble(p, "center_y_mm",       0.0);
    in.wall_thickness_mm = jdouble(p, "wall_thickness_mm", 0.0);
    return in;
}

sk::perforated_grille_pattern::Input parsePerforatedGrillePattern(const json& p)
{
    sk::perforated_grille_pattern::Input in;
    in.face                = parseFaceDatum(p);
    in.spec_key            = jstring(p, "spec_key",            "");
    in.hole_dia_mm         = jdouble(p, "hole_dia_mm",         0.0);
    in.hex_pitch_mm        = jdouble(p, "hex_pitch_mm",        0.0);
    in.footprint_length_mm = jdouble(p, "footprint_length_mm", 0.0);
    in.footprint_width_mm  = jdouble(p, "footprint_width_mm",  0.0);
    in.center_x_mm         = jdouble(p, "center_x_mm",         0.0);
    in.center_y_mm         = jdouble(p, "center_y_mm",         0.0);
    in.grid_axis           = parseDir(p, "grid_axis", gp_Dir(1.0, 0.0, 0.0));
    in.wall_thickness_mm   = jdouble(p, "wall_thickness_mm",   0.0);
    return in;
}

sk::breather_vent_compound::Input parseBreatherVentCompound(const json& p)
{
    sk::breather_vent_compound::Input in;
    in.entry_face            = parseFaceDatum(p);
    in.spec_key              = jstring(p, "spec_key",              "");
    in.position_x_mm         = jdouble(p, "position_x_mm",         0.0);
    in.position_y_mm         = jdouble(p, "position_y_mm",         0.0);
    in.axis_dir              = parseAxisDir(p);
    in.through_dia_mm        = jdouble(p, "through_dia_mm",        0.0);
    in.counterbore_dia_mm    = jdouble(p, "counterbore_dia_mm",    0.0);
    in.counterbore_depth_mm  = jdouble(p, "counterbore_depth_mm",  0.0);
    in.groove_dia_mm         = jdouble(p, "groove_dia_mm",         0.0);
    in.groove_width_mm       = jdouble(p, "groove_width_mm",       0.0);
    in.groove_depth_mm       = jdouble(p, "groove_depth_mm",       0.0);
    return in;
}

sk::pcb_standoff_array_under_board::Input parsePcbStandoffArrayUnderBoard(const json& p)
{
    sk::pcb_standoff_array_under_board::Input in;
    in.base_face            = parseFaceDatum(p);
    in.axis_dir             = parseDir(p, "axis_dir", gp_Dir(0.0, 0.0, 1.0));
    in.screw_size           = jstring(p, "screw_size",           in.screw_size.c_str());
    in.standoff_height_mm   = jdouble(p, "standoff_height_mm",   in.standoff_height_mm);
    in.boss_od_mm           = jdouble(p, "boss_od_mm",           in.boss_od_mm);
    in.pilot_extra_depth_mm = jdouble(p, "pilot_extra_depth_mm", in.pilot_extra_depth_mm);
    in.PCB_thickness_mm     = jdouble(p, "PCB_thickness_mm",     in.PCB_thickness_mm);
    in.plate_margin_mm      = jdouble(p, "plate_margin_mm",      in.plate_margin_mm);
    in.plate_thickness_mm   = jdouble(p, "plate_thickness_mm",   in.plate_thickness_mm);
    if (p.contains("sites") && p["sites"].is_array()) {
        for (const auto& s : p["sites"]) {
            if (s.is_object()) {
                sk::pcb_standoff_array_under_board::StandoffSite site;
                site.x_mm = jdouble(s, "x_mm", 0.0);
                site.y_mm = jdouble(s, "y_mm", 0.0);
                in.sites.push_back(site);
            }
        }
    }
    return in;
}

sk::connector_cutout_with_keepout::Input parseConnectorCutoutWithKeepout(const json& p)
{
    sk::connector_cutout_with_keepout::Input in;
    in.face                = parseFaceDatum(p);
    in.axis_dir            = parseAxisDir(p);
    in.connector_type      = jstring(p, "connector_type",      in.connector_type.c_str());
    in.position_x_mm       = jdouble(p, "position_x_mm",       0.0);
    in.position_y_mm       = jdouble(p, "position_y_mm",       0.0);
    in.chamfer_override_mm = jdouble(p, "chamfer_override_mm", in.chamfer_override_mm);
    return in;
}

sk::cable_grommet_pass_through::Input parseCableGrommetPassThrough(const json& p)
{
    sk::cable_grommet_pass_through::Input in;
    in.entry_face    = parseFaceDatum(p);
    in.axis_dir      = parseAxisDir(p);
    in.position_x_mm = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm = jdouble(p, "position_y_mm", 0.0);
    in.grommet_size  = jstring(p, "grommet_size",  in.grommet_size.c_str());
    return in;
}

sk::isolator_grommet_seat::Input parseIsolatorGrommetSeat(const json& p)
{
    sk::isolator_grommet_seat::Input in;
    in.entry_face     = parseFaceDatum(p);
    in.axis_dir       = parseAxisDir(p);
    in.position_x_mm  = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm  = jdouble(p, "position_y_mm", 0.0);
    in.isolator_size  = jstring(p, "isolator_size", in.isolator_size.c_str());
    return in;
}

sk::tilt_post_for_lcd_panel::Input parseTiltPostForLcdPanel(const json& p)
{
    sk::tilt_post_for_lcd_panel::Input in;
    in.base_face     = parseFaceDatum(p);
    in.axis_dir      = parseAxisDir(p);
    in.position_x_mm = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm = jdouble(p, "position_y_mm", 0.0);
    in.panel_size    = jstring(p, "panel_size",    in.panel_size.c_str());
    return in;
}

sk::din_rail_clip_slot::Input parseDinRailClipSlot(const json& p)
{
    sk::din_rail_clip_slot::Input in;
    in.enclosure_face = parseFaceDatum(p);
    in.center_x_mm    = jdouble(p, "center_x_mm",    0.0);
    in.center_y_mm    = jdouble(p, "center_y_mm",    0.0);
    in.axis_dir       = parseAxisDir(p);
    in.rail_dir_deg   = jdouble(p, "rail_dir_deg",   0.0);
    in.rail_length_mm = jdouble(p, "rail_length_mm", in.rail_length_mm);
    return in;
}

sk::banana_jack_receptacle::Input parseBananaJackReceptacle(const json& p)
{
    sk::banana_jack_receptacle::Input in;
    in.face          = parseFaceDatum(p);
    in.position_x_mm = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm = jdouble(p, "position_y_mm", 0.0);
    in.axis_dir      = parseAxisDir(p);
    if (p.contains("count") && p["count"].is_number_integer()) {
        in.count = p["count"].get<int>();
    }
    in.spacing_mm    = jdouble(p, "spacing_mm",    in.spacing_mm);
    in.direction_deg = jdouble(p, "direction_deg", 0.0);
    return in;
}

sk::rj45_socket_cutout::Input parseRj45SocketCutout(const json& p)
{
    sk::rj45_socket_cutout::Input in;
    in.face          = parseFaceDatum(p);
    in.position_x_mm = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm = jdouble(p, "position_y_mm", 0.0);
    in.axis_dir      = parseAxisDir(p);
    if (p.contains("port_count") && p["port_count"].is_number_integer()) {
        in.port_count = p["port_count"].get<int>();
    }
    in.spacing_mm    = jdouble(p, "spacing_mm",    in.spacing_mm);
    in.direction_deg = jdouble(p, "direction_deg", 0.0);
    return in;
}

sk::linear_slider_track::Input parseLinearSliderTrack(const json& p)
{
    sk::linear_slider_track::Input in;
    in.entry_face      = parseFaceDatum(p);
    in.start_x_mm      = jdouble(p, "start_x_mm",      0.0);
    in.start_y_mm      = jdouble(p, "start_y_mm",      0.0);
    in.end_x_mm        = jdouble(p, "end_x_mm",        0.0);
    in.end_y_mm        = jdouble(p, "end_y_mm",        0.0);
    in.axis_dir        = parseAxisDir(p);
    in.slot_width_mm   = jdouble(p, "slot_width_mm",   0.0);
    in.slot_depth_mm   = jdouble(p, "slot_depth_mm",   0.0);
    in.end_stop_thk_mm = jdouble(p, "end_stop_thk_mm", 0.0);
    in.track_class     = jstring(p, "track_class",     in.track_class.c_str());
    return in;
}

sk::pivot_pin_clevis_compound::Input parsePivotPinClevisCompound(const json& p)
{
    sk::pivot_pin_clevis_compound::Input in;
    in.entry_face   = parseFaceDatum(p);
    in.center_x_mm  = jdouble(p, "center_x_mm",  0.0);
    in.center_y_mm  = jdouble(p, "center_y_mm",  0.0);
    in.u_axis       = parseDir(p, "u_axis",   gp_Dir(0.0, 0.0, -1.0));
    in.pin_axis     = parseDir(p, "pin_axis", gp_Dir(0.0, 1.0,  0.0));
    in.u_depth_mm   = jdouble(p, "u_depth_mm",   0.0);
    in.clevis_size  = jstring(p, "clevis_size",  in.clevis_size.c_str());
    return in;
}

sk::cam_actuated_slider::Input parseCamActuatedSlider(const json& p)
{
    sk::cam_actuated_slider::Input in;
    in.entry_face      = parseFaceDatum(p);
    in.slot_start_x_mm = jdouble(p, "slot_start_x_mm", 0.0);
    in.slot_start_y_mm = jdouble(p, "slot_start_y_mm", 0.0);
    in.slot_end_x_mm   = jdouble(p, "slot_end_x_mm",   0.0);
    in.slot_end_y_mm   = jdouble(p, "slot_end_y_mm",   0.0);
    in.axis_dir        = parseAxisDir(p);
    in.slot_depth_mm   = jdouble(p, "slot_depth_mm",   0.0);
    in.pivot_offset_mm = jdouble(p, "pivot_offset_mm", 0.0);
    in.cam_class       = jstring(p, "cam_class",       in.cam_class.c_str());
    return in;
}

sk::over_center_toggle_pocket::Input parseOverCenterTogglePocket(const json& p)
{
    sk::over_center_toggle_pocket::Input in;
    in.entry_face      = parseFaceDatum(p);
    in.pivot_x_mm      = jdouble(p, "pivot_x_mm",      0.0);
    in.pivot_y_mm      = jdouble(p, "pivot_y_mm",      0.0);
    in.axis_dir        = parseAxisDir(p);
    in.pocket_depth_mm = jdouble(p, "pocket_depth_mm", 0.0);
    in.slot_angle_deg  = jdouble(p, "slot_angle_deg",  0.0);
    in.toggle_class    = jstring(p, "toggle_class",    in.toggle_class.c_str());
    return in;
}

sk::detented_position_slider::Input parseDetentedPositionSlider(const json& p)
{
    sk::detented_position_slider::Input in;
    in.entry_face      = parseFaceDatum(p);
    in.slot_start_x_mm = jdouble(p, "slot_start_x_mm", 0.0);
    in.slot_start_y_mm = jdouble(p, "slot_start_y_mm", 0.0);
    in.slot_end_x_mm   = jdouble(p, "slot_end_x_mm",   0.0);
    in.slot_end_y_mm   = jdouble(p, "slot_end_y_mm",   0.0);
    in.axis_dir        = parseAxisDir(p);
    in.slot_width_mm   = jdouble(p, "slot_width_mm",   0.0);
    in.slot_depth_mm   = jdouble(p, "slot_depth_mm",   0.0);
    in.detent_class    = jstring(p, "detent_class",    in.detent_class.c_str());
    return in;
}

sk::bezel_groove_assembly::Input parseBezelGrooveAssembly(const json& p)
{
    sk::bezel_groove_assembly::Input in;
    in.case_face         = parseFaceDatum(p);
    in.center_x_mm       = jdouble(p, "center_x_mm",       0.0);
    in.center_y_mm       = jdouble(p, "center_y_mm",       0.0);
    in.axis_dir          = parseAxisDir(p);
    in.outer_dia_mm      = jdouble(p, "outer_dia_mm",      0.0);
    in.inner_dia_mm      = jdouble(p, "inner_dia_mm",      0.0);
    in.groove_depth_mm   = jdouble(p, "groove_depth_mm",   0.0);
    in.taper_deg         = jdouble(p, "taper_deg",         in.taper_deg);
    in.bottom_fillet_mm  = jdouble(p, "bottom_fillet_mm",  in.bottom_fillet_mm);
    return in;
}

sk::lug_with_spring_bar_holes::Input parseLugWithSpringBarHoles(const json& p)
{
    sk::lug_with_spring_bar_holes::Input in;
    in.case_axis            = parseAx1(p, "case_axis_origin", "case_axis_dir",
                                       gp_Dir(0.0, 0.0, 1.0));
    in.case_outer_radius_mm = jdouble(p, "case_outer_radius_mm", 0.0);
    in.lug_angle_deg        = jdouble(p, "lug_angle_deg",        0.0);
    in.lug_attach_z_mm      = jdouble(p, "lug_attach_z_mm",      0.0);
    in.lug_length_mm        = jdouble(p, "lug_length_mm",        in.lug_length_mm);
    in.lug_width_mm         = jdouble(p, "lug_width_mm",         in.lug_width_mm);
    in.lug_thickness_mm     = jdouble(p, "lug_thickness_mm",     in.lug_thickness_mm);
    in.corner_chamfer_mm    = jdouble(p, "corner_chamfer_mm",    in.corner_chamfer_mm);
    in.spring_bar_dia_mm    = jdouble(p, "spring_bar_dia_mm",    in.spring_bar_dia_mm);
    in.spring_bar_inset_mm  = jdouble(p, "spring_bar_inset_mm",  in.spring_bar_inset_mm);
    return in;
}

sk::crown_stem_cavity_compound::Input parseCrownStemCavityCompound(const json& p)
{
    sk::crown_stem_cavity_compound::Input in;
    in.case_axis            = parseAx1(p, "case_axis_origin", "case_axis_dir",
                                       gp_Dir(0.0, 0.0, 1.0));
    in.case_outer_radius_mm = jdouble(p, "case_outer_radius_mm", 0.0);
    in.angle_deg            = jdouble(p, "angle_deg",            0.0);
    in.port_center_z_mm     = jdouble(p, "port_center_z_mm",     0.0);
    in.cavity_dia_mm        = jdouble(p, "cavity_dia_mm",        in.cavity_dia_mm);
    in.cavity_depth_mm      = jdouble(p, "cavity_depth_mm",      in.cavity_depth_mm);
    in.stem_dia_mm          = jdouble(p, "stem_dia_mm",          in.stem_dia_mm);
    in.stem_total_length_mm = jdouble(p, "stem_total_length_mm", in.stem_total_length_mm);
    in.o_ring_cs_mm         = jdouble(p, "o_ring_cs_mm",         in.o_ring_cs_mm);
    in.o_ring_depth_mm      = jdouble(p, "o_ring_depth_mm",      in.o_ring_depth_mm);
    in.o_ring_offset_mm     = jdouble(p, "o_ring_offset_mm",     in.o_ring_offset_mm);
    return in;
}

// ── Slice 9 parsers — context-aware / morph / sheet / heat-exchanger / misc

sk::auto_boss_under_hole::Input parseAutoBossUnderHole(const json& p)
{
    sk::auto_boss_under_hole::Input in;
    in.entry_face     = parseFaceDatum(p);
    in.position_x_mm  = jdouble(p, "position_x_mm",  in.position_x_mm);
    in.position_y_mm  = jdouble(p, "position_y_mm",  in.position_y_mm);
    in.start_z_mm     = jdouble(p, "start_z_mm",     in.start_z_mm);
    in.axis_dir       = parseAxisDir(p);
    in.boss_dia_mm    = jdouble(p, "boss_dia_mm",    in.boss_dia_mm);
    in.screw_spec     = jstring(p, "screw_spec",     in.screw_spec.c_str());
    in.total_depth_mm = jdouble(p, "total_depth_mm", in.total_depth_mm);
    return in;
}

sk::auto_standoff_floating_point::Input parseAutoStandoffFloatingPoint(const json& p)
{
    sk::auto_standoff_floating_point::Input in;
    in.entry_face              = parseFaceDatum(p);
    in.position_x_mm           = jdouble(p, "position_x_mm",           in.position_x_mm);
    in.position_y_mm           = jdouble(p, "position_y_mm",           in.position_y_mm);
    in.height_mm               = jdouble(p, "height_mm",               in.height_mm);
    // axis_dir defaults to +Z (standoff grows up); only override if valid
    // non-zero 3-vector is supplied.
    if (p.contains("axis_dir") && p["axis_dir"].is_array() && p["axis_dir"].size() == 3 &&
        p["axis_dir"][0].is_number() && p["axis_dir"][1].is_number() && p["axis_dir"][2].is_number()) {
        const double x = p["axis_dir"][0].get<double>();
        const double y = p["axis_dir"][1].get<double>();
        const double z = p["axis_dir"][2].get<double>();
        if (std::sqrt(x*x + y*y + z*z) > 1e-9) {
            in.axis_dir = gp_Dir(x, y, z);
        }
    }
    in.thread_spec             = jstring(p, "thread_spec",             in.thread_spec.c_str());
    in.standoff_od_mm          = jdouble(p, "standoff_od_mm",          in.standoff_od_mm);
    in.base_plate_thickness_mm = jdouble(p, "base_plate_thickness_mm", in.base_plate_thickness_mm);
    return in;
}

sk::auto_rib_between_two_walls::Input parseAutoRibBetweenTwoWalls(const json& p)
{
    sk::auto_rib_between_two_walls::Input in;
    if (p.contains("wall_A_face_id") && p["wall_A_face_id"].is_number_integer()) {
        in.wall_A_face_id = p["wall_A_face_id"].get<int>();
    }
    if (p.contains("wall_B_face_id") && p["wall_B_face_id"].is_number_integer()) {
        in.wall_B_face_id = p["wall_B_face_id"].get<int>();
    }
    in.rib_thick_mm  = jdouble(p, "rib_thick_mm",  in.rib_thick_mm);
    in.rib_height_mm = jdouble(p, "rib_height_mm", in.rib_height_mm);
    in.rib_style     = jstring(p, "rib_style",     in.rib_style.c_str());
    in.wall_thick_mm = jdouble(p, "wall_thick_mm", in.wall_thick_mm);
    return in;
}

sk::auto_gusset_corner_brace::Input parseAutoGussetCornerBrace(const json& p)
{
    sk::auto_gusset_corner_brace::Input in;
    if (p.contains("corner_edge_id") && p["corner_edge_id"].is_number_integer()) {
        in.corner_edge_id = p["corner_edge_id"].get<int>();
    }
    in.leg_length_mm   = jdouble(p, "leg_length_mm",   in.leg_length_mm);
    in.gusset_thick_mm = jdouble(p, "gusset_thick_mm", in.gusset_thick_mm);
    in.service_class   = jstring(p, "service_class",   in.service_class.c_str());
    return in;
}

sk::auto_chamfer_all_outer_edges::Input parseAutoChamferAllOuterEdges(const json& p)
{
    sk::auto_chamfer_all_outer_edges::Input in;
    in.chamfer_size_mm      = jdouble(p, "chamfer_size_mm",      in.chamfer_size_mm);
    in.chamfer_size_key     = jstring(p, "chamfer_size_key",     in.chamfer_size_key.c_str());
    in.include_top_edges    = jbool  (p, "include_top_edges",    in.include_top_edges);
    in.include_bottom_edges = jbool  (p, "include_bottom_edges", in.include_bottom_edges);
    in.min_wall_thick_mm    = jdouble(p, "min_wall_thick_mm",    in.min_wall_thick_mm);
    return in;
}

// blend_morph_two_shapes — single-workpiece morph using two cross-sections
// embedded directly in params (no second-workpiece dependency).
sk::blend_morph_two_shapes::Input parseBlendMorphTwoShapes(const json& p)
{
    sk::blend_morph_two_shapes::Input in;
    if (p.contains("centre_a") && p["centre_a"].is_array() && p["centre_a"].size() == 3 &&
        p["centre_a"][0].is_number() && p["centre_a"][1].is_number() && p["centre_a"][2].is_number()) {
        in.centre_a = gp_Pnt(p["centre_a"][0].get<double>(),
                             p["centre_a"][1].get<double>(),
                             p["centre_a"][2].get<double>());
    }
    in.radius_a = jdouble(p, "radius_a", in.radius_a);
    in.z_a      = jdouble(p, "z_a",      in.z_a);
    if (p.contains("centre_b") && p["centre_b"].is_array() && p["centre_b"].size() == 3 &&
        p["centre_b"][0].is_number() && p["centre_b"][1].is_number() && p["centre_b"][2].is_number()) {
        in.centre_b = gp_Pnt(p["centre_b"][0].get<double>(),
                             p["centre_b"][1].get<double>(),
                             p["centre_b"][2].get<double>());
    }
    in.radius_b = jdouble(p, "radius_b", in.radius_b);
    in.z_b      = jdouble(p, "z_b",      in.z_b);
    return in;
}

sk::parametric_sweep_morph::Input parseParametricSweepMorph(const json& p)
{
    sk::parametric_sweep_morph::Input in;
    if (p.contains("path_start") && p["path_start"].is_array() && p["path_start"].size() == 3 &&
        p["path_start"][0].is_number() && p["path_start"][1].is_number() && p["path_start"][2].is_number()) {
        in.path_start = gp_Pnt(p["path_start"][0].get<double>(),
                               p["path_start"][1].get<double>(),
                               p["path_start"][2].get<double>());
    }
    if (p.contains("path_end") && p["path_end"].is_array() && p["path_end"].size() == 3 &&
        p["path_end"][0].is_number() && p["path_end"][1].is_number() && p["path_end"][2].is_number()) {
        in.path_end = gp_Pnt(p["path_end"][0].get<double>(),
                             p["path_end"][1].get<double>(),
                             p["path_end"][2].get<double>());
    }
    in.start_radius = jdouble(p, "start_radius", in.start_radius);
    in.end_radius   = jdouble(p, "end_radius",   in.end_radius);
    if (p.contains("intermediate_count") && p["intermediate_count"].is_number_integer()) {
        in.intermediate_count = p["intermediate_count"].get<int>();
    }
    return in;
}

sk::deformation_warp::Input parseDeformationWarp(const json& p)
{
    sk::deformation_warp::Input in;
    in.scale_x = jdouble(p, "scale_x", in.scale_x);
    in.scale_y = jdouble(p, "scale_y", in.scale_y);
    in.scale_z = jdouble(p, "scale_z", in.scale_z);
    if (p.contains("axis_origin") && p["axis_origin"].is_array() &&
        p["axis_origin"].size() == 3 &&
        p["axis_origin"][0].is_number() && p["axis_origin"][1].is_number() &&
        p["axis_origin"][2].is_number()) {
        in.axis_origin = gp_Pnt(p["axis_origin"][0].get<double>(),
                                p["axis_origin"][1].get<double>(),
                                p["axis_origin"][2].get<double>());
    }
    // axis_dir defaults to +Z; only override on a valid non-zero 3-vector.
    if (p.contains("axis_dir") && p["axis_dir"].is_array() && p["axis_dir"].size() == 3 &&
        p["axis_dir"][0].is_number() && p["axis_dir"][1].is_number() && p["axis_dir"][2].is_number()) {
        const double x = p["axis_dir"][0].get<double>();
        const double y = p["axis_dir"][1].get<double>();
        const double z = p["axis_dir"][2].get<double>();
        if (std::sqrt(x*x + y*y + z*z) > 1e-9) {
            in.axis_dir = gp_Dir(x, y, z);
        }
    }
    in.angle_deg = jdouble(p, "angle_deg", in.angle_deg);
    return in;
}

sk::lance::Input parseLance(const json& p)
{
    sk::lance::Input in;
    if (p.contains("waypoints") && p["waypoints"].is_array()) {
        for (const auto& wp : p["waypoints"]) {
            if (wp.is_array() && wp.size() == 2 &&
                wp[0].is_number() && wp[1].is_number()) {
                in.waypoints.push_back({ wp[0].get<double>(), wp[1].get<double>() });
            }
        }
    }
    in.tab_width_mm = jdouble(p, "tab_width_mm", in.tab_width_mm);
    in.lift_mm      = jdouble(p, "lift_mm",      in.lift_mm);
    return in;
}

sk::beading::Input parseBeading(const json& p)
{
    sk::beading::Input in;
    if (p.contains("waypoints") && p["waypoints"].is_array()) {
        for (const auto& wp : p["waypoints"]) {
            if (wp.is_array() && wp.size() == 2 &&
                wp[0].is_number() && wp[1].is_number()) {
                in.waypoints.push_back({ wp[0].get<double>(), wp[1].get<double>() });
            }
        }
    }
    in.width_mm = jdouble(p, "width_mm", in.width_mm);
    in.depth_mm = jdouble(p, "depth_mm", in.depth_mm);
    in.mode     = sk::beading::modeFromString(jstring(p, "mode", "ridge"));
    return in;
}

sk::shell_roll::Input parseShellRoll(const json& p)
{
    sk::shell_roll::Input in;
    in.plate_thick_mm  = jdouble(p, "plate_thick_mm",  in.plate_thick_mm);
    in.shell_dia_mm    = jdouble(p, "shell_dia_mm",    in.shell_dia_mm);
    in.shell_length_mm = jdouble(p, "shell_length_mm", in.shell_length_mm);
    return in;
}

sk::hemispherical_head_form::Input parseHemisphericalHeadForm(const json& p)
{
    sk::hemispherical_head_form::Input in;
    in.dia_mm         = jdouble(p, "dia_mm",         in.dia_mm);
    in.plate_thick_mm = jdouble(p, "plate_thick_mm", in.plate_thick_mm);
    return in;
}

sk::expand_tube::Input parseExpandTube(const json& p)
{
    sk::expand_tube::Input in;
    in.target_expansion_pct = jdouble(p, "target_expansion_pct", in.target_expansion_pct);
    in.method               = jstring(p, "method",               in.method.c_str());
    return in;
}

sk::tube_to_tubesheet_weld::Input parseTubeToTubesheetWeld(const json& p)
{
    sk::tube_to_tubesheet_weld::Input in;
    in.tube_dia_mm    = jdouble(p, "tube_dia_mm",    in.tube_dia_mm);
    in.sheet_thick_mm = jdouble(p, "sheet_thick_mm", in.sheet_thick_mm);
    if (p.contains("joint_count") && p["joint_count"].is_number_integer()) {
        in.joint_count = p["joint_count"].get<int>();
    }
    return in;
}

sk::tube_swage::Input parseTubeSwage(const json& p)
{
    sk::tube_swage::Input in;
    in.start_z_mm   = jdouble(p, "start_z_mm",   in.start_z_mm);
    in.end_z_mm     = jdouble(p, "end_z_mm",     in.end_z_mm);
    in.target_od_mm = jdouble(p, "target_od_mm", in.target_od_mm);
    return in;
}

sk::bolted_flange_compound::Input parseBoltedFlangeCompound(const json& p)
{
    sk::bolted_flange_compound::Input in;
    in.entry_face             = parseFaceDatum(p);
    in.center_x_mm            = jdouble(p, "center_x_mm",            in.center_x_mm);
    in.center_y_mm            = jdouble(p, "center_y_mm",            in.center_y_mm);
    // axis_dir defaults to +Z for this skill.
    if (p.contains("axis_dir") && p["axis_dir"].is_array() && p["axis_dir"].size() == 3 &&
        p["axis_dir"][0].is_number() && p["axis_dir"][1].is_number() && p["axis_dir"][2].is_number()) {
        const double x = p["axis_dir"][0].get<double>();
        const double y = p["axis_dir"][1].get<double>();
        const double z = p["axis_dir"][2].get<double>();
        if (std::sqrt(x*x + y*y + z*z) > 1e-9) {
            in.axis_dir = gp_Dir(x, y, z);
        }
    }
    in.flange_outer_dia_mm    = jdouble(p, "flange_outer_dia_mm",    in.flange_outer_dia_mm);
    in.flange_thickness_mm    = jdouble(p, "flange_thickness_mm",    in.flange_thickness_mm);
    in.pipe_bore_dia_mm       = jdouble(p, "pipe_bore_dia_mm",       in.pipe_bore_dia_mm);
    in.raised_face_dia_mm     = jdouble(p, "raised_face_dia_mm",     in.raised_face_dia_mm);
    in.raised_face_height_mm  = jdouble(p, "raised_face_height_mm",  in.raised_face_height_mm);
    in.gasket_groove_id_mm    = jdouble(p, "gasket_groove_id_mm",    in.gasket_groove_id_mm);
    in.gasket_groove_od_mm    = jdouble(p, "gasket_groove_od_mm",    in.gasket_groove_od_mm);
    in.gasket_groove_depth_mm = jdouble(p, "gasket_groove_depth_mm", in.gasket_groove_depth_mm);
    if (p.contains("bolt_count") && p["bolt_count"].is_number_integer()) {
        in.bolt_count = p["bolt_count"].get<int>();
    }
    in.bolt_circle_dia_mm = jdouble(p, "bolt_circle_dia_mm", in.bolt_circle_dia_mm);
    in.bolt_hole_dia_mm   = jdouble(p, "bolt_hole_dia_mm",   in.bolt_hole_dia_mm);
    return in;
}

sk::pinned_clevis_joint::Input parsePinnedClevisJoint(const json& p)
{
    sk::pinned_clevis_joint::Input in;
    in.entry_face            = parseFaceDatum(p);
    in.center_x_mm           = jdouble(p, "center_x_mm",           in.center_x_mm);
    in.center_y_mm           = jdouble(p, "center_y_mm",           in.center_y_mm);
    in.axis_dir              = parseAxisDir(p);  // default (0,0,-1) matches struct default
    in.block_length_mm       = jdouble(p, "block_length_mm",       in.block_length_mm);
    in.block_width_mm        = jdouble(p, "block_width_mm",        in.block_width_mm);
    in.block_height_mm       = jdouble(p, "block_height_mm",       in.block_height_mm);
    in.tongue_slot_width_mm  = jdouble(p, "tongue_slot_width_mm",  in.tongue_slot_width_mm);
    in.tongue_slot_depth_mm  = jdouble(p, "tongue_slot_depth_mm",  in.tongue_slot_depth_mm);
    in.pin_hole_dia_mm       = jdouble(p, "pin_hole_dia_mm",       in.pin_hole_dia_mm);
    in.pin_hole_z_offset_mm  = jdouble(p, "pin_hole_z_offset_mm",  in.pin_hole_z_offset_mm);
    in.cotter_pin_dia_mm     = jdouble(p, "cotter_pin_dia_mm",     in.cotter_pin_dia_mm);
    in.cotter_pin_offset_mm  = jdouble(p, "cotter_pin_offset_mm",  in.cotter_pin_offset_mm);
    return in;
}

sk::expansion_joint_bellows_stub::Input parseExpansionJointBellowsStub(const json& p)
{
    sk::expansion_joint_bellows_stub::Input in;
    in.entry_face            = parseFaceDatum(p);
    in.center_x_mm           = jdouble(p, "center_x_mm",           in.center_x_mm);
    in.center_y_mm           = jdouble(p, "center_y_mm",           in.center_y_mm);
    // axis_dir defaults to +Z; only override on valid 3-vector.
    if (p.contains("axis_dir") && p["axis_dir"].is_array() && p["axis_dir"].size() == 3 &&
        p["axis_dir"][0].is_number() && p["axis_dir"][1].is_number() && p["axis_dir"][2].is_number()) {
        const double x = p["axis_dir"][0].get<double>();
        const double y = p["axis_dir"][1].get<double>();
        const double z = p["axis_dir"][2].get<double>();
        if (std::sqrt(x*x + y*y + z*z) > 1e-9) {
            in.axis_dir = gp_Dir(x, y, z);
        }
    }
    in.flange_outer_dia_mm       = jdouble(p, "flange_outer_dia_mm",       in.flange_outer_dia_mm);
    in.flange_thickness_mm       = jdouble(p, "flange_thickness_mm",       in.flange_thickness_mm);
    in.pipe_outer_dia_mm         = jdouble(p, "pipe_outer_dia_mm",         in.pipe_outer_dia_mm);
    in.pipe_inner_dia_mm         = jdouble(p, "pipe_inner_dia_mm",         in.pipe_inner_dia_mm);
    in.pipe_length_mm            = jdouble(p, "pipe_length_mm",            in.pipe_length_mm);
    if (p.contains("corrugation_count") && p["corrugation_count"].is_number_integer()) {
        in.corrugation_count = p["corrugation_count"].get<int>();
    }
    in.corrugation_pitch_mm      = jdouble(p, "corrugation_pitch_mm",      in.corrugation_pitch_mm);
    in.corrugation_outer_dia_mm  = jdouble(p, "corrugation_outer_dia_mm",  in.corrugation_outer_dia_mm);
    return in;
}

sk::anchor_pad_compound::Input parseAnchorPadCompound(const json& p)
{
    sk::anchor_pad_compound::Input in;
    in.entry_face            = parseFaceDatum(p);
    in.center_x_mm           = jdouble(p, "center_x_mm",           in.center_x_mm);
    in.center_y_mm           = jdouble(p, "center_y_mm",           in.center_y_mm);
    // axis_dir defaults to +Z.
    if (p.contains("axis_dir") && p["axis_dir"].is_array() && p["axis_dir"].size() == 3 &&
        p["axis_dir"][0].is_number() && p["axis_dir"][1].is_number() && p["axis_dir"][2].is_number()) {
        const double x = p["axis_dir"][0].get<double>();
        const double y = p["axis_dir"][1].get<double>();
        const double z = p["axis_dir"][2].get<double>();
        if (std::sqrt(x*x + y*y + z*z) > 1e-9) {
            in.axis_dir = gp_Dir(x, y, z);
        }
    }
    in.plate_length_mm        = jdouble(p, "plate_length_mm",        in.plate_length_mm);
    in.plate_width_mm         = jdouble(p, "plate_width_mm",         in.plate_width_mm);
    in.plate_thickness_mm     = jdouble(p, "plate_thickness_mm",     in.plate_thickness_mm);
    in.stud_dia_mm            = jdouble(p, "stud_dia_mm",            in.stud_dia_mm);
    in.stud_height_mm         = jdouble(p, "stud_height_mm",         in.stud_height_mm);
    in.stud_pitch_x_mm        = jdouble(p, "stud_pitch_x_mm",        in.stud_pitch_x_mm);
    in.stud_pitch_y_mm        = jdouble(p, "stud_pitch_y_mm",        in.stud_pitch_y_mm);
    in.conduit_dia_mm         = jdouble(p, "conduit_dia_mm",         in.conduit_dia_mm);
    in.leveling_nut_dia_mm    = jdouble(p, "leveling_nut_dia_mm",    in.leveling_nut_dia_mm);
    in.leveling_nut_depth_mm  = jdouble(p, "leveling_nut_depth_mm",  in.leveling_nut_depth_mm);
    return in;
}

sk::flange_face_with_gasket_groove::Input parseFlangeFaceWithGasketGroove(const json& p)
{
    sk::flange_face_with_gasket_groove::Input in;
    in.entry_face              = parseFaceDatum(p);
    in.center_x_mm             = jdouble(p, "center_x_mm",             in.center_x_mm);
    in.center_y_mm             = jdouble(p, "center_y_mm",             in.center_y_mm);
    in.axis_dir                = parseAxisDir(p);  // default (0,0,-1) matches struct
    in.flange_outer_dia_mm     = jdouble(p, "flange_outer_dia_mm",     in.flange_outer_dia_mm);
    in.facing_pass_depth_mm    = jdouble(p, "facing_pass_depth_mm",    in.facing_pass_depth_mm);
    in.gasket_groove_id_mm     = jdouble(p, "gasket_groove_id_mm",     in.gasket_groove_id_mm);
    in.gasket_groove_od_mm     = jdouble(p, "gasket_groove_od_mm",     in.gasket_groove_od_mm);
    in.gasket_groove_depth_mm  = jdouble(p, "gasket_groove_depth_mm",  in.gasket_groove_depth_mm);
    in.groove_fillet_r_mm      = jdouble(p, "groove_fillet_r_mm",      in.groove_fillet_r_mm);
    return in;
}

// ─────────────────────────────────────────────────────────────────────────
// Slice 9: valve / electrical / tooling / machine / hinge parsers
// ─────────────────────────────────────────────────────────────────────────

// ── Valve seats ──────────────────────────────────────────────────────────

sk::gate_valve_seat_compound::Input parseGateValveSeatCompound(const json& p)
{
    sk::gate_valve_seat_compound::Input in;
    in.entry_face        = parseFaceDatum(p);
    in.center_x_mm       = jdouble(p, "center_x_mm",       0.0);
    in.center_y_mm       = jdouble(p, "center_y_mm",       0.0);
    in.center_z_mm       = jdouble(p, "center_z_mm",       0.0);
    in.flow_axis         = parseDir(p, "flow_axis", gp_Dir(1.0, 0.0, 0.0));
    in.stem_axis         = parseDir(p, "stem_axis", gp_Dir(0.0, 0.0, 1.0));
    in.seat_bore_dia_mm  = jdouble(p, "seat_bore_dia_mm",  in.seat_bore_dia_mm);
    in.seat_od_mm        = jdouble(p, "seat_od_mm",        in.seat_od_mm);
    in.seat_taper_deg    = jdouble(p, "seat_taper_deg",    in.seat_taper_deg);
    in.seat_length_mm    = jdouble(p, "seat_length_mm",    in.seat_length_mm);
    in.slot_width_mm     = jdouble(p, "slot_width_mm",     in.slot_width_mm);
    in.slot_height_mm    = jdouble(p, "slot_height_mm",    in.slot_height_mm);
    in.body_y_span_mm    = jdouble(p, "body_y_span_mm",    in.body_y_span_mm);
    in.stem_bore_dia_mm  = jdouble(p, "stem_bore_dia_mm",  in.stem_bore_dia_mm);
    in.stem_bore_depth_mm = jdouble(p, "stem_bore_depth_mm", in.stem_bore_depth_mm);
    in.recess_dia_mm     = jdouble(p, "recess_dia_mm",     in.recess_dia_mm);
    in.recess_depth_mm   = jdouble(p, "recess_depth_mm",   in.recess_depth_mm);
    return in;
}

sk::ball_valve_seat_compound::Input parseBallValveSeatCompound(const json& p)
{
    sk::ball_valve_seat_compound::Input in;
    in.entry_face            = parseFaceDatum(p);
    in.center_x_mm           = jdouble(p, "center_x_mm",           0.0);
    in.center_y_mm           = jdouble(p, "center_y_mm",           0.0);
    in.center_z_mm           = jdouble(p, "center_z_mm",           0.0);
    in.flow_axis             = parseDir(p, "flow_axis", gp_Dir(1.0, 0.0, 0.0));
    in.stem_axis             = parseDir(p, "stem_axis", gp_Dir(0.0, 0.0, 1.0));
    in.size_spec             = jstring(p, "size_spec",             "DN25");
    in.ball_dia_mm           = jdouble(p, "ball_dia_mm",           0.0);
    in.port_dia_mm           = jdouble(p, "port_dia_mm",           0.0);
    in.seat_groove_id_mm     = jdouble(p, "seat_groove_id_mm",     0.0);
    in.seat_groove_od_mm     = jdouble(p, "seat_groove_od_mm",     0.0);
    in.seat_groove_depth_mm  = jdouble(p, "seat_groove_depth_mm",  0.0);
    in.stem_bore_dia_mm      = jdouble(p, "stem_bore_dia_mm",      0.0);
    in.stem_bore_depth_mm    = jdouble(p, "stem_bore_depth_mm",    0.0);
    in.flange_recess_dia_mm  = jdouble(p, "flange_recess_dia_mm",  0.0);
    in.flange_recess_depth_mm = jdouble(p, "flange_recess_depth_mm", 0.0);
    in.ball_clearance_mm     = jdouble(p, "ball_clearance_mm",     in.ball_clearance_mm);
    return in;
}

sk::butterfly_valve_disc_seat::Input parseButterflyValveDiscSeat(const json& p)
{
    sk::butterfly_valve_disc_seat::Input in;
    in.entry_face                   = parseFaceDatum(p);
    in.center_x_mm                  = jdouble(p, "center_x_mm",                  0.0);
    in.center_y_mm                  = jdouble(p, "center_y_mm",                  0.0);
    in.center_z_mm                  = jdouble(p, "center_z_mm",                  0.0);
    in.flow_axis                    = parseDir(p, "flow_axis", gp_Dir(1.0, 0.0, 0.0));
    in.stem_axis                    = parseDir(p, "stem_axis", gp_Dir(0.0, 0.0, 1.0));
    in.flow_bore_dia_mm             = jdouble(p, "flow_bore_dia_mm",             in.flow_bore_dia_mm);
    in.flow_bore_len_mm             = jdouble(p, "flow_bore_len_mm",             in.flow_bore_len_mm);
    in.seat_groove_id_mm            = jdouble(p, "seat_groove_id_mm",            in.seat_groove_id_mm);
    in.seat_groove_radial_depth_mm  = jdouble(p, "seat_groove_radial_depth_mm",  in.seat_groove_radial_depth_mm);
    in.seat_groove_width_mm         = jdouble(p, "seat_groove_width_mm",         in.seat_groove_width_mm);
    in.stub_bearing_dia_mm          = jdouble(p, "stub_bearing_dia_mm",          in.stub_bearing_dia_mm);
    in.stub_bearing_depth_mm        = jdouble(p, "stub_bearing_depth_mm",        in.stub_bearing_depth_mm);
    in.stem_port_dia_mm             = jdouble(p, "stem_port_dia_mm",             in.stem_port_dia_mm);
    in.body_y_span_mm               = jdouble(p, "body_y_span_mm",               in.body_y_span_mm);
    return in;
}

sk::check_valve_seat_with_stop::Input parseCheckValveSeatWithStop(const json& p)
{
    sk::check_valve_seat_with_stop::Input in;
    in.entry_face       = parseFaceDatum(p);
    in.center_x_mm      = jdouble(p, "center_x_mm",      0.0);
    in.center_y_mm      = jdouble(p, "center_y_mm",      0.0);
    in.center_z_mm      = jdouble(p, "center_z_mm",      0.0);
    in.flow_axis        = parseDir(p, "flow_axis", gp_Dir(1.0, 0.0, 0.0));
    in.inlet_dia_mm     = jdouble(p, "inlet_dia_mm",     in.inlet_dia_mm);
    in.inlet_length_mm  = jdouble(p, "inlet_length_mm",  in.inlet_length_mm);
    in.seat_angle_deg   = jdouble(p, "seat_angle_deg",   in.seat_angle_deg);
    in.seat_drop_mm     = jdouble(p, "seat_drop_mm",     in.seat_drop_mm);
    in.outlet_dia_mm    = jdouble(p, "outlet_dia_mm",    in.outlet_dia_mm);
    in.outlet_length_mm = jdouble(p, "outlet_length_mm", in.outlet_length_mm);
    in.stop_dia_mm      = jdouble(p, "stop_dia_mm",      in.stop_dia_mm);
    in.stop_length_mm   = jdouble(p, "stop_length_mm",   in.stop_length_mm);
    in.stop_offset_mm   = jdouble(p, "stop_offset_mm",   in.stop_offset_mm);
    return in;
}

sk::needle_valve_seat::Input parseNeedleValveSeat(const json& p)
{
    sk::needle_valve_seat::Input in;
    in.entry_face             = parseFaceDatum(p);
    in.position_x_mm          = jdouble(p, "position_x_mm",          0.0);
    in.position_y_mm          = jdouble(p, "position_y_mm",          0.0);
    in.axis_dir               = parseAxisDir(p);
    in.pocket_entry_dia_mm    = jdouble(p, "pocket_entry_dia_mm",    in.pocket_entry_dia_mm);
    in.pocket_taper_deg       = jdouble(p, "pocket_taper_deg",       in.pocket_taper_deg);
    in.pocket_length_mm       = jdouble(p, "pocket_length_mm",       in.pocket_length_mm);
    in.control_bore_dia_mm    = jdouble(p, "control_bore_dia_mm",    in.control_bore_dia_mm);
    in.control_bore_depth_mm  = jdouble(p, "control_bore_depth_mm",  in.control_bore_depth_mm);
    in.stem_boss_dia_mm       = jdouble(p, "stem_boss_dia_mm",       in.stem_boss_dia_mm);
    in.stem_boss_depth_mm     = jdouble(p, "stem_boss_depth_mm",     in.stem_boss_depth_mm);
    in.thread_relief_dia_mm   = jdouble(p, "thread_relief_dia_mm",   in.thread_relief_dia_mm);
    in.thread_relief_depth_mm = jdouble(p, "thread_relief_depth_mm", in.thread_relief_depth_mm);
    return in;
}

// ── Electrical contacts ──────────────────────────────────────────────────

sk::banana_socket_compound::Input parseBananaSocketCompound(const json& p)
{
    sk::banana_socket_compound::Input in;
    in.entry_face                 = parseFaceDatum(p);
    in.position_x_mm              = jdouble(p, "position_x_mm",              0.0);
    in.position_y_mm              = jdouble(p, "position_y_mm",              0.0);
    in.axis_dir                   = parseAxisDir(p);
    in.bore_dia_mm                = jdouble(p, "bore_dia_mm",                in.bore_dia_mm);
    in.bore_depth_mm              = jdouble(p, "bore_depth_mm",              in.bore_depth_mm);
    in.slit_width_mm              = jdouble(p, "slit_width_mm",              in.slit_width_mm);
    in.slit_depth_mm              = jdouble(p, "slit_depth_mm",              in.slit_depth_mm);
    in.retention_groove_dia_mm    = jdouble(p, "retention_groove_dia_mm",    in.retention_groove_dia_mm);
    in.retention_groove_depth_mm  = jdouble(p, "retention_groove_depth_mm",  in.retention_groove_depth_mm);
    in.retention_groove_pos_mm    = jdouble(p, "retention_groove_pos_mm",    in.retention_groove_pos_mm);
    in.insulation_step_dia_mm     = jdouble(p, "insulation_step_dia_mm",     in.insulation_step_dia_mm);
    in.insulation_step_depth_mm   = jdouble(p, "insulation_step_depth_mm",   in.insulation_step_depth_mm);
    return in;
}

sk::spring_contact_clip::Input parseSpringContactClip(const json& p)
{
    sk::spring_contact_clip::Input in;
    in.entry_face            = parseFaceDatum(p);
    in.position_x_mm         = jdouble(p, "position_x_mm",         0.0);
    in.position_y_mm         = jdouble(p, "position_y_mm",         0.0);
    in.axis_dir              = parseAxisDir(p);
    in.orientation_deg       = jdouble(p, "orientation_deg",       0.0);
    in.clip_length_mm        = jdouble(p, "clip_length_mm",        in.clip_length_mm);
    in.clip_width_mm         = jdouble(p, "clip_width_mm",         in.clip_width_mm);
    in.clip_depth_mm         = jdouble(p, "clip_depth_mm",         in.clip_depth_mm);
    in.back_stop_width_mm    = jdouble(p, "back_stop_width_mm",    in.back_stop_width_mm);
    in.back_stop_depth_mm    = jdouble(p, "back_stop_depth_mm",    in.back_stop_depth_mm);
    in.barb_slot_width_mm    = jdouble(p, "barb_slot_width_mm",    in.barb_slot_width_mm);
    in.barb_slot_height_mm   = jdouble(p, "barb_slot_height_mm",   in.barb_slot_height_mm);
    in.barb_slot_depth_mm    = jdouble(p, "barb_slot_depth_mm",    in.barb_slot_depth_mm);
    in.barb_slot_z_offset_mm = jdouble(p, "barb_slot_z_offset_mm", in.barb_slot_z_offset_mm);
    return in;
}

sk::terminal_block_post::Input parseTerminalBlockPost(const json& p)
{
    sk::terminal_block_post::Input in;
    in.entry_face         = parseFaceDatum(p);
    in.position_x_mm      = jdouble(p, "position_x_mm",      0.0);
    in.position_y_mm      = jdouble(p, "position_y_mm",      0.0);
    in.axis_dir           = parseDir(p, "axis_dir", gp_Dir(0.0, 0.0, 1.0));
    in.stud_dia_mm        = jdouble(p, "stud_dia_mm",        in.stud_dia_mm);
    in.stud_height_mm     = jdouble(p, "stud_height_mm",     in.stud_height_mm);
    in.shoulder_dia_mm    = jdouble(p, "shoulder_dia_mm",    in.shoulder_dia_mm);
    in.shoulder_height_mm = jdouble(p, "shoulder_height_mm", in.shoulder_height_mm);
    in.skirt_dia_mm       = jdouble(p, "skirt_dia_mm",       in.skirt_dia_mm);
    in.skirt_height_mm    = jdouble(p, "skirt_height_mm",    in.skirt_height_mm);
    in.wire_hole_dia_mm   = jdouble(p, "wire_hole_dia_mm",   in.wire_hole_dia_mm);
    in.wire_hole_z_mm     = jdouble(p, "wire_hole_z_mm",     in.wire_hole_z_mm);
    return in;
}

sk::busbar_lap_joint::Input parseBusbarLapJoint(const json& p)
{
    sk::busbar_lap_joint::Input in;
    in.entry_face       = parseFaceDatum(p);
    in.end_x_mm         = jdouble(p, "end_x_mm",         0.0);
    in.end_y_mm         = jdouble(p, "end_y_mm",         0.0);
    in.axis_dir         = parseAxisDir(p);
    in.lap_length_mm    = jdouble(p, "lap_length_mm",    in.lap_length_mm);
    in.bar_width_mm     = jdouble(p, "bar_width_mm",     in.bar_width_mm);
    in.bar_thickness_mm = jdouble(p, "bar_thickness_mm", in.bar_thickness_mm);
    in.bolt_dia_mm      = jdouble(p, "bolt_dia_mm",      in.bolt_dia_mm);
    in.bolt_pitch_mm    = jdouble(p, "bolt_pitch_mm",    in.bolt_pitch_mm);
    in.notch_depth_mm   = jdouble(p, "notch_depth_mm",   in.notch_depth_mm);
    in.notch_width_mm   = jdouble(p, "notch_width_mm",   in.notch_width_mm);
    if (p.contains("dimple_grid_nx") && p["dimple_grid_nx"].is_number_integer()) {
        in.dimple_grid_nx = p["dimple_grid_nx"].get<int>();
    }
    if (p.contains("dimple_grid_ny") && p["dimple_grid_ny"].is_number_integer()) {
        in.dimple_grid_ny = p["dimple_grid_ny"].get<int>();
    }
    in.dimple_dia_mm    = jdouble(p, "dimple_dia_mm",    in.dimple_dia_mm);
    in.dimple_depth_mm  = jdouble(p, "dimple_depth_mm",  in.dimple_depth_mm);
    return in;
}

sk::pcb_card_edge_socket::Input parsePcbCardEdgeSocket(const json& p)
{
    sk::pcb_card_edge_socket::Input in;
    in.entry_face        = parseFaceDatum(p);
    in.start_x_mm        = jdouble(p, "start_x_mm",        0.0);
    in.start_y_mm        = jdouble(p, "start_y_mm",        0.0);
    in.end_x_mm          = jdouble(p, "end_x_mm",          0.0);
    in.end_y_mm          = jdouble(p, "end_y_mm",          0.0);
    in.axis_dir          = parseAxisDir(p);
    in.slot_width_mm     = jdouble(p, "slot_width_mm",     in.slot_width_mm);
    in.slot_depth_mm     = jdouble(p, "slot_depth_mm",     in.slot_depth_mm);
    if (p.contains("finger_count") && p["finger_count"].is_number_integer()) {
        in.finger_count = p["finger_count"].get<int>();
    }
    in.pitch_mm          = jdouble(p, "pitch_mm",          in.pitch_mm);
    in.finger_dia_mm     = jdouble(p, "finger_dia_mm",     in.finger_dia_mm);
    in.finger_depth_mm   = jdouble(p, "finger_depth_mm",   in.finger_depth_mm);
    in.key_tab_width_mm  = jdouble(p, "key_tab_width_mm",  in.key_tab_width_mm);
    in.key_tab_depth_mm  = jdouble(p, "key_tab_depth_mm",  in.key_tab_depth_mm);
    in.key_tab_offset_mm = jdouble(p, "key_tab_offset_mm", in.key_tab_offset_mm);
    return in;
}

// ── Tooling / jig ────────────────────────────────────────────────────────

sk::jig_plate_with_drill_bushings::Input parseJigPlateWithDrillBushings(const json& p)
{
    sk::jig_plate_with_drill_bushings::Input in;
    in.entry_face       = parseFaceDatum(p);
    in.axis_dir         = parseAxisDir(p);
    in.bushing_id_mm    = jdouble(p, "bushing_id_mm",    in.bushing_id_mm);
    in.bushing_od_mm    = jdouble(p, "bushing_od_mm",    in.bushing_od_mm);
    in.flange_od_mm     = jdouble(p, "flange_od_mm",     in.flange_od_mm);
    in.flange_depth_mm  = jdouble(p, "flange_depth_mm",  in.flange_depth_mm);
    in.chamfer_mm       = jdouble(p, "chamfer_mm",       in.chamfer_mm);
    if (p.contains("sites") && p["sites"].is_array()) {
        for (const auto& s : p["sites"]) {
            if (s.is_object()) {
                sk::jig_plate_with_drill_bushings::BushingSite site;
                site.x_mm = jdouble(s, "x_mm", 0.0);
                site.y_mm = jdouble(s, "y_mm", 0.0);
                in.sites.push_back(site);
            }
        }
    }
    return in;
}

sk::locator_pin_set::Input parseLocatorPinSet(const json& p)
{
    sk::locator_pin_set::Input in;
    in.base_face          = parseFaceDatum(p);
    in.position_x_mm      = jdouble(p, "position_x_mm",      0.0);
    in.position_y_mm      = jdouble(p, "position_y_mm",      0.0);
    in.axis_dir           = parseDir(p, "axis_dir", gp_Dir(0.0, 0.0, 1.0));
    in.shoulder_dia_mm    = jdouble(p, "shoulder_dia_mm",    in.shoulder_dia_mm);
    in.shoulder_height_mm = jdouble(p, "shoulder_height_mm", in.shoulder_height_mm);
    in.post_dia_mm        = jdouble(p, "post_dia_mm",        in.post_dia_mm);
    in.post_height_mm     = jdouble(p, "post_height_mm",     in.post_height_mm);
    in.lead_in_chamfer_mm = jdouble(p, "lead_in_chamfer_mm", in.lead_in_chamfer_mm);
    in.groove_dia_mm      = jdouble(p, "groove_dia_mm",      in.groove_dia_mm);
    in.groove_width_mm    = jdouble(p, "groove_width_mm",    in.groove_width_mm);
    in.groove_z_offset_mm = jdouble(p, "groove_z_offset_mm", in.groove_z_offset_mm);
    return in;
}

sk::gauge_block_step::Input parseGaugeBlockStep(const json& p)
{
    sk::gauge_block_step::Input in;
    in.top_face       = parseFaceDatum(p);
    in.x_start_mm     = jdouble(p, "x_start_mm",     in.x_start_mm);
    in.step_length_mm = jdouble(p, "step_length_mm", in.step_length_mm);
    in.y_full_mm      = jdouble(p, "y_full_mm",      in.y_full_mm);
    if (p.contains("step_depths_mm") && p["step_depths_mm"].is_array()) {
        for (const auto& d : p["step_depths_mm"]) {
            if (d.is_number()) {
                in.step_depths_mm.push_back(d.get<double>());
            }
        }
    }
    return in;
}

sk::vise_jaw_with_v_groove::Input parseViseJawWithVGroove(const json& p)
{
    sk::vise_jaw_with_v_groove::Input in;
    in.working_face         = parseFaceDatum(p);
    in.v_groove_angle_deg   = jdouble(p, "v_groove_angle_deg",   in.v_groove_angle_deg);
    in.v_groove_depth_mm    = jdouble(p, "v_groove_depth_mm",    in.v_groove_depth_mm);
    in.v_groove_z_mm        = jdouble(p, "v_groove_z_mm",        in.v_groove_z_mm);
    in.pilot_dia_mm         = jdouble(p, "pilot_dia_mm",         in.pilot_dia_mm);
    in.counterbore_dia_mm   = jdouble(p, "counterbore_dia_mm",   in.counterbore_dia_mm);
    in.counterbore_depth_mm = jdouble(p, "counterbore_depth_mm", in.counterbore_depth_mm);
    if (p.contains("mount_holes") && p["mount_holes"].is_array()) {
        for (const auto& h : p["mount_holes"]) {
            if (h.is_object()) {
                sk::vise_jaw_with_v_groove::MountHole mh;
                mh.x_mm = jdouble(h, "x_mm", 0.0);
                mh.z_mm = jdouble(h, "z_mm", 0.0);
                in.mount_holes.push_back(mh);
            }
        }
    }
    return in;
}

sk::pin_and_diamond_locating_set::Input parsePinAndDiamondLocatingSet(const json& p)
{
    sk::pin_and_diamond_locating_set::Input in;
    in.entry_face                 = parseFaceDatum(p);
    in.axis_dir                   = parseAxisDir(p);
    in.round_x_mm                 = jdouble(p, "round_x_mm",                 0.0);
    in.round_y_mm                 = jdouble(p, "round_y_mm",                 0.0);
    in.diamond_x_mm               = jdouble(p, "diamond_x_mm",               in.diamond_x_mm);
    in.diamond_y_mm               = jdouble(p, "diamond_y_mm",               0.0);
    in.pin_dia_mm                 = jdouble(p, "pin_dia_mm",                 in.pin_dia_mm);
    in.hole_depth_mm              = jdouble(p, "hole_depth_mm",              in.hole_depth_mm);
    in.diamond_slot_extension_mm  = jdouble(p, "diamond_slot_extension_mm",  in.diamond_slot_extension_mm);
    in.chamfer_mm                 = jdouble(p, "chamfer_mm",                 in.chamfer_mm);
    return in;
}

// ── Machine elements ─────────────────────────────────────────────────────

sk::cam_with_profile::Input parseCamWithProfile(const json& p)
{
    sk::cam_with_profile::Input in;
    if (p.contains("profile") && p["profile"].is_array()) {
        for (const auto& wp : p["profile"]) {
            if (wp.is_object()) {
                sk::cam_with_profile::Waypoint w;
                w.theta_rad = jdouble(wp, "theta_rad", 0.0);
                w.r_mm      = jdouble(wp, "r_mm",      0.0);
                in.profile.push_back(w);
            }
        }
    }
    in.thickness_mm    = jdouble(p, "thickness_mm",    in.thickness_mm);
    in.shaft_dia_mm    = jdouble(p, "shaft_dia_mm",    in.shaft_dia_mm);
    in.keyway_width_mm = jdouble(p, "keyway_width_mm", in.keyway_width_mm);
    in.keyway_depth_mm = jdouble(p, "keyway_depth_mm", in.keyway_depth_mm);
    in.center_x_mm     = jdouble(p, "center_x_mm",     0.0);
    in.center_y_mm     = jdouble(p, "center_y_mm",     0.0);
    in.center_z_mm     = jdouble(p, "center_z_mm",     0.0);
    return in;
}

sk::cam_follower_roller_seat::Input parseCamFollowerRollerSeat(const json& p)
{
    sk::cam_follower_roller_seat::Input in;
    in.roller_dia_mm          = jdouble(p, "roller_dia_mm",          in.roller_dia_mm);
    in.roller_length_mm       = jdouble(p, "roller_length_mm",       in.roller_length_mm);
    in.radial_clearance_mm    = jdouble(p, "radial_clearance_mm",    in.radial_clearance_mm);
    in.arm_clearance_width_mm = jdouble(p, "arm_clearance_width_mm", in.arm_clearance_width_mm);
    in.arm_clearance_depth_mm = jdouble(p, "arm_clearance_depth_mm", in.arm_clearance_depth_mm);
    in.retention_dia_mm       = jdouble(p, "retention_dia_mm",       in.retention_dia_mm);
    in.retention_depth_mm     = jdouble(p, "retention_depth_mm",     in.retention_depth_mm);
    in.center_x_mm            = jdouble(p, "center_x_mm",            0.0);
    in.center_y_mm            = jdouble(p, "center_y_mm",            0.0);
    in.center_z_mm            = jdouble(p, "center_z_mm",            0.0);
    return in;
}

sk::eccentric_shaft_collar::Input parseEccentricShaftCollar(const json& p)
{
    sk::eccentric_shaft_collar::Input in;
    in.outer_dia_mm       = jdouble(p, "outer_dia_mm",       in.outer_dia_mm);
    in.thickness_mm       = jdouble(p, "thickness_mm",       in.thickness_mm);
    in.bore_dia_mm        = jdouble(p, "bore_dia_mm",        in.bore_dia_mm);
    in.eccentricity_mm    = jdouble(p, "eccentricity_mm",    in.eccentricity_mm);
    in.set_screw_dia_mm   = jdouble(p, "set_screw_dia_mm",   in.set_screw_dia_mm);
    in.set_screw_depth_mm = jdouble(p, "set_screw_depth_mm", in.set_screw_depth_mm);
    in.center_x_mm        = jdouble(p, "center_x_mm",        0.0);
    in.center_y_mm        = jdouble(p, "center_y_mm",        0.0);
    in.center_z_mm        = jdouble(p, "center_z_mm",        0.0);
    return in;
}

sk::flywheel_with_balance::Input parseFlywheelWithBalance(const json& p)
{
    sk::flywheel_with_balance::Input in;
    in.outer_dia_mm           = jdouble(p, "outer_dia_mm",           in.outer_dia_mm);
    in.thickness_mm           = jdouble(p, "thickness_mm",           in.thickness_mm);
    in.hub_bore_dia_mm        = jdouble(p, "hub_bore_dia_mm",        in.hub_bore_dia_mm);
    in.balance_pcd_mm         = jdouble(p, "balance_pcd_mm",         in.balance_pcd_mm);
    in.balance_drill_dia_mm   = jdouble(p, "balance_drill_dia_mm",   in.balance_drill_dia_mm);
    in.balance_drill_depth_mm = jdouble(p, "balance_drill_depth_mm", in.balance_drill_depth_mm);
    if (p.contains("balance_count") && p["balance_count"].is_number_integer()) {
        in.balance_count = p["balance_count"].get<int>();
    }
    in.chamfer_mm  = jdouble(p, "chamfer_mm",  in.chamfer_mm);
    in.center_x_mm = jdouble(p, "center_x_mm", 0.0);
    in.center_y_mm = jdouble(p, "center_y_mm", 0.0);
    in.center_z_mm = jdouble(p, "center_z_mm", 0.0);
    return in;
}

sk::governor_arm_with_pivot::Input parseGovernorArmWithPivot(const json& p)
{
    sk::governor_arm_with_pivot::Input in;
    in.arm_length_mm      = jdouble(p, "arm_length_mm",      in.arm_length_mm);
    in.arm_width_mm       = jdouble(p, "arm_width_mm",       in.arm_width_mm);
    in.arm_thickness_mm   = jdouble(p, "arm_thickness_mm",   in.arm_thickness_mm);
    in.pivot_bore_dia_mm  = jdouble(p, "pivot_bore_dia_mm",  in.pivot_bore_dia_mm);
    in.ball_seat_dia_mm   = jdouble(p, "ball_seat_dia_mm",   in.ball_seat_dia_mm);
    in.ball_seat_depth_mm = jdouble(p, "ball_seat_depth_mm", in.ball_seat_depth_mm);
    in.corner_x_mm        = jdouble(p, "corner_x_mm",        0.0);
    in.corner_y_mm        = jdouble(p, "corner_y_mm",        0.0);
    in.corner_z_mm        = jdouble(p, "corner_z_mm",        0.0);
    return in;
}

sk::spur_gear_with_real_teeth::Input parseSpurGearWithRealTeeth(const json& p)
{
    sk::spur_gear_with_real_teeth::Input in;
    in.blank_dia_mm       = jdouble(p, "blank_dia_mm",       0.0);
    in.blank_thick_mm     = jdouble(p, "blank_thick_mm",     0.0);
    in.module_mm          = jdouble(p, "module_mm",          0.0);
    in.pitch_dia_mm       = jdouble(p, "pitch_dia_mm",       0.0);
    in.pressure_angle_deg = jdouble(p, "pressure_angle_deg", in.pressure_angle_deg);
    in.bore_dia_mm        = jdouble(p, "bore_dia_mm",        0.0);
    return in;
}

sk::helical_gear_teeth::Input parseHelicalGearTeeth(const json& p)
{
    sk::helical_gear_teeth::Input in;
    in.blank_dia_mm       = jdouble(p, "blank_dia_mm",       0.0);
    in.blank_thick_mm     = jdouble(p, "blank_thick_mm",     0.0);
    in.normal_module_mm   = jdouble(p, "normal_module_mm",   0.0);
    in.pitch_dia_mm       = jdouble(p, "pitch_dia_mm",       0.0);
    in.helix_angle_deg    = jdouble(p, "helix_angle_deg",    in.helix_angle_deg);
    in.pressure_angle_deg = jdouble(p, "pressure_angle_deg", in.pressure_angle_deg);
    in.bore_dia_mm        = jdouble(p, "bore_dia_mm",        0.0);
    in.right_hand         = jbool  (p, "right_hand",         in.right_hand);
    return in;
}

sk::sprocket_with_chain_teeth::Input parseSprocketWithChainTeeth(const json& p)
{
    sk::sprocket_with_chain_teeth::Input in;
    in.blank_dia_mm    = jdouble(p, "blank_dia_mm",   0.0);
    in.blank_thick_mm  = jdouble(p, "blank_thick_mm", 0.0);
    in.ansi_chain_size = jstring(p, "ansi_chain_size", "");
    if (p.contains("num_teeth") && p["num_teeth"].is_number_integer()) {
        in.num_teeth = p["num_teeth"].get<int>();
    }
    in.bore_dia_mm = jdouble(p, "bore_dia_mm", 0.0);
    return in;
}

sk::spline_shaft_compound::Input parseSplineShaftCompound(const json& p)
{
    sk::spline_shaft_compound::Input in;
    in.shaft_dia_mm     = jdouble(p, "shaft_dia_mm",     0.0);
    in.shaft_length_mm  = jdouble(p, "shaft_length_mm",  0.0);
    in.module_mm        = jdouble(p, "module_mm",        0.0);
    if (p.contains("num_splines") && p["num_splines"].is_number_integer()) {
        in.num_splines = p["num_splines"].get<int>();
    }
    in.spline_length_mm = jdouble(p, "spline_length_mm", 0.0);
    in.end_chamfer_mm   = jdouble(p, "end_chamfer_mm",   in.end_chamfer_mm);
    return in;
}

sk::ratchet_pawl_set::Input parseRatchetPawlSet(const json& p)
{
    sk::ratchet_pawl_set::Input in;
    in.wheel_dia_mm      = jdouble(p, "wheel_dia_mm",      0.0);
    in.wheel_thick_mm    = jdouble(p, "wheel_thick_mm",    0.0);
    if (p.contains("num_teeth") && p["num_teeth"].is_number_integer()) {
        in.num_teeth = p["num_teeth"].get<int>();
    }
    in.tooth_depth_mm    = jdouble(p, "tooth_depth_mm",    0.0);
    in.steep_flank_deg   = jdouble(p, "steep_flank_deg",   in.steep_flank_deg);
    in.shallow_flank_deg = jdouble(p, "shallow_flank_deg", in.shallow_flank_deg);
    in.bore_dia_mm       = jdouble(p, "bore_dia_mm",       0.0);
    in.ccw_locking       = jbool  (p, "ccw_locking",       in.ccw_locking);
    return in;
}

// ── Hinge / latch ────────────────────────────────────────────────────────

sk::piano_hinge_strip::Input parsePianoHingeStrip(const json& p)
{
    sk::piano_hinge_strip::Input in;
    in.entry_face            = parseFaceDatum(p);
    in.origin_x_mm           = jdouble(p, "origin_x_mm",           0.0);
    in.origin_y_mm           = jdouble(p, "origin_y_mm",           0.0);
    in.pin_axis_dir          = parseDir(p, "pin_axis_dir", gp_Dir(1.0, 0.0, 0.0));
    in.strip_length_mm       = jdouble(p, "strip_length_mm",       0.0);
    in.strip_width_mm        = jdouble(p, "strip_width_mm",        0.0);
    in.strip_thickness_mm    = jdouble(p, "strip_thickness_mm",    0.0);
    if (p.contains("knuckle_count") && p["knuckle_count"].is_number_integer()) {
        in.knuckle_count = p["knuckle_count"].get<int>();
    }
    in.knuckle_pitch_mm      = jdouble(p, "knuckle_pitch_mm",      in.knuckle_pitch_mm);
    in.knuckle_slot_width_mm = jdouble(p, "knuckle_slot_width_mm", in.knuckle_slot_width_mm);
    in.knuckle_slot_depth_mm = jdouble(p, "knuckle_slot_depth_mm", 0.0);
    in.pin_dia_mm            = jdouble(p, "pin_dia_mm",            in.pin_dia_mm);
    in.mount_hole_dia_mm     = jdouble(p, "mount_hole_dia_mm",     in.mount_hole_dia_mm);
    in.mount_hole_inset_mm   = jdouble(p, "mount_hole_inset_mm",   in.mount_hole_inset_mm);
    return in;
}

sk::overcenter_latch::Input parseOvercenterLatch(const json& p)
{
    sk::overcenter_latch::Input in;
    in.entry_face                = parseFaceDatum(p);
    in.position_x_mm             = jdouble(p, "position_x_mm",             0.0);
    in.position_y_mm             = jdouble(p, "position_y_mm",             0.0);
    in.axis_dir                  = parseAxisDir(p);
    in.lever_arm_length_mm       = jdouble(p, "lever_arm_length_mm",       in.lever_arm_length_mm);
    in.lever_arm_width_mm        = jdouble(p, "lever_arm_width_mm",        in.lever_arm_width_mm);
    in.lever_pocket_depth_mm     = jdouble(p, "lever_pocket_depth_mm",     in.lever_pocket_depth_mm);
    in.pivot_bore_dia_mm         = jdouble(p, "pivot_bore_dia_mm",         in.pivot_bore_dia_mm);
    in.pivot_bore_depth_mm       = jdouble(p, "pivot_bore_depth_mm",       in.pivot_bore_depth_mm);
    in.cam_slot_offset_mm        = jdouble(p, "cam_slot_offset_mm",        in.cam_slot_offset_mm);
    in.cam_slot_length_mm        = jdouble(p, "cam_slot_length_mm",        in.cam_slot_length_mm);
    in.cam_slot_width_mm         = jdouble(p, "cam_slot_width_mm",         in.cam_slot_width_mm);
    in.cam_slot_depth_mm         = jdouble(p, "cam_slot_depth_mm",         in.cam_slot_depth_mm);
    in.retention_hook_length_mm  = jdouble(p, "retention_hook_length_mm",  in.retention_hook_length_mm);
    in.retention_hook_width_mm   = jdouble(p, "retention_hook_width_mm",   in.retention_hook_width_mm);
    in.retention_hook_depth_mm   = jdouble(p, "retention_hook_depth_mm",   in.retention_hook_depth_mm);
    in.grip_notch_width_mm       = jdouble(p, "grip_notch_width_mm",       in.grip_notch_width_mm);
    in.grip_notch_depth_mm       = jdouble(p, "grip_notch_depth_mm",       in.grip_notch_depth_mm);
    return in;
}

sk::snap_action_lock_pocket::Input parseSnapActionLockPocket(const json& p)
{
    sk::snap_action_lock_pocket::Input in;
    in.entry_face                 = parseFaceDatum(p);
    in.position_x_mm              = jdouble(p, "position_x_mm",              0.0);
    in.position_y_mm              = jdouble(p, "position_y_mm",              0.0);
    in.axis_dir                   = parseAxisDir(p);
    in.lock_body_dia_mm           = jdouble(p, "lock_body_dia_mm",           in.lock_body_dia_mm);
    in.lock_body_depth_mm         = jdouble(p, "lock_body_depth_mm",         in.lock_body_depth_mm);
    in.key_counterbore_dia_mm     = jdouble(p, "key_counterbore_dia_mm",     in.key_counterbore_dia_mm);
    in.key_counterbore_depth_mm   = jdouble(p, "key_counterbore_depth_mm",   in.key_counterbore_depth_mm);
    in.cam_slot_length_mm         = jdouble(p, "cam_slot_length_mm",         in.cam_slot_length_mm);
    in.cam_slot_width_mm          = jdouble(p, "cam_slot_width_mm",          in.cam_slot_width_mm);
    in.cam_slot_depth_offset_mm   = jdouble(p, "cam_slot_depth_offset_mm",   in.cam_slot_depth_offset_mm);
    in.spring_pocket_dia_mm       = jdouble(p, "spring_pocket_dia_mm",       in.spring_pocket_dia_mm);
    in.spring_pocket_offset_mm    = jdouble(p, "spring_pocket_offset_mm",    in.spring_pocket_offset_mm);
    in.spring_pocket_depth_mm     = jdouble(p, "spring_pocket_depth_mm",     in.spring_pocket_depth_mm);
    return in;
}

sk::gas_strut_hinge_compound::Input parseGasStrutHingeCompound(const json& p)
{
    sk::gas_strut_hinge_compound::Input in;
    in.entry_face                = parseFaceDatum(p);
    in.frame_pivot_x_mm          = jdouble(p, "frame_pivot_x_mm",          0.0);
    in.frame_pivot_y_mm          = jdouble(p, "frame_pivot_y_mm",          0.0);
    in.door_pivot_x_mm           = jdouble(p, "door_pivot_x_mm",           in.door_pivot_x_mm);
    in.door_pivot_y_mm           = jdouble(p, "door_pivot_y_mm",           0.0);
    in.axis_dir                  = parseAxisDir(p);
    in.bracket_thickness_mm      = jdouble(p, "bracket_thickness_mm",      in.bracket_thickness_mm);
    in.pivot_bore_dia_mm         = jdouble(p, "pivot_bore_dia_mm",         in.pivot_bore_dia_mm);
    in.clevis_pocket_length_mm   = jdouble(p, "clevis_pocket_length_mm",   in.clevis_pocket_length_mm);
    in.clevis_pocket_width_mm    = jdouble(p, "clevis_pocket_width_mm",    in.clevis_pocket_width_mm);
    in.clevis_pocket_depth_mm    = jdouble(p, "clevis_pocket_depth_mm",    in.clevis_pocket_depth_mm);
    in.ball_groove_inner_dia_mm  = jdouble(p, "ball_groove_inner_dia_mm",  in.ball_groove_inner_dia_mm);
    in.ball_groove_outer_dia_mm  = jdouble(p, "ball_groove_outer_dia_mm",  in.ball_groove_outer_dia_mm);
    in.ball_groove_depth_mm      = jdouble(p, "ball_groove_depth_mm",      in.ball_groove_depth_mm);
    in.lightening_hole_dia_mm    = jdouble(p, "lightening_hole_dia_mm",    in.lightening_hole_dia_mm);
    return in;
}

sk::spring_loaded_door_latch::Input parseSpringLoadedDoorLatch(const json& p)
{
    sk::spring_loaded_door_latch::Input in;
    in.entry_face              = parseFaceDatum(p);
    in.position_x_mm           = jdouble(p, "position_x_mm",           0.0);
    in.position_y_mm           = jdouble(p, "position_y_mm",           0.0);
    in.axis_dir                = parseAxisDir(p);
    in.latch_pocket_size_mm    = jdouble(p, "latch_pocket_size_mm",    in.latch_pocket_size_mm);
    in.latch_pocket_depth_mm   = jdouble(p, "latch_pocket_depth_mm",   in.latch_pocket_depth_mm);
    in.screw_hole_dia_mm       = jdouble(p, "screw_hole_dia_mm",       in.screw_hole_dia_mm);
    in.screw_hole_depth_mm     = jdouble(p, "screw_hole_depth_mm",     in.screw_hole_depth_mm);
    in.screw_spacing_mm        = jdouble(p, "screw_spacing_mm",        in.screw_spacing_mm);
    in.plunger_bore_dia_mm     = jdouble(p, "plunger_bore_dia_mm",     in.plunger_bore_dia_mm);
    in.plunger_bore_depth_mm   = jdouble(p, "plunger_bore_depth_mm",   in.plunger_bore_depth_mm);
    in.receiver_slot_width_mm  = jdouble(p, "receiver_slot_width_mm",  in.receiver_slot_width_mm);
    in.receiver_slot_height_mm = jdouble(p, "receiver_slot_height_mm", in.receiver_slot_height_mm);
    in.receiver_slot_depth_mm  = jdouble(p, "receiver_slot_depth_mm",  in.receiver_slot_depth_mm);
    return in;
}

sk::cam_lock_cavity::Input parseCamLockCavity(const json& p)
{
    sk::cam_lock_cavity::Input in;
    in.entry_face    = parseFaceDatum(p);
    in.position_x_mm = jdouble(p, "position_x_mm", 0.0);
    in.position_y_mm = jdouble(p, "position_y_mm", 0.0);
    in.axis_dir      = parseAxisDir(p);
    return in;
}

// Sketch features — replay a reverse-engineered extrude / revolve from its
// recovered params (so a recovered sketch boss survives inferProcessPlan ->
// Executor, the same contract holes already have).
sk::extrude_boss_from_sketch::Input parseExtrudeBossFromSketch(const json& p)
{
    sk::extrude_boss_from_sketch::Input in;
    // recovered params carry "face_normal" [x,y,z]; fall back to parseFaceDatum
    // ("entry_face") for hand-authored plans, else the top face.
    if (p.contains("face_normal") && p["face_normal"].is_array() &&
        p["face_normal"].size() == 3) {
        const auto& n = p["face_normal"];
        in.entry_face = sk::FaceByNormal{
            gp_Dir(n[0].get<double>(), n[1].get<double>(), n[2].get<double>()), 5.0, "largest" };
    } else {
        in.entry_face = parseFaceDatum(p);
    }
    in.height_mm = jdouble(p, "height_mm", 5.0);
    // Prefer the richer line|arc `profile`; fall back to the legacy `polygon`.
    if (p.contains("profile") && p["profile"].is_array()) {
        for (const auto& s : p["profile"]) {
            sk::extrude_boss_from_sketch::ProfileSeg seg;
            seg.kind = (s.value("kind", std::string("line")) == "arc")
                     ? sk::extrude_boss_from_sketch::ProfileSeg::Kind::Arc
                     : sk::extrude_boss_from_sketch::ProfileSeg::Kind::Line;
            seg.x   = s.value("x", 0.0);
            seg.y   = s.value("y", 0.0);
            seg.cx  = s.value("cx", 0.0);
            seg.cy  = s.value("cy", 0.0);
            seg.ccw = s.value("ccw", true);
            in.profile.push_back(seg);
        }
    } else if (p.contains("polygon") && p["polygon"].is_array()) {
        for (const auto& pt : p["polygon"])
            in.polygon.emplace_back(pt.value("x", 0.0), pt.value("y", 0.0));
    }
    return in;
}

sk::dome_boss::Input parseDomeBoss(const json& p)
{
    sk::dome_boss::Input in;
    // recovered params carry "face_normal"; fall back to entry_face / top face.
    if (p.contains("face_normal") && p["face_normal"].is_array() &&
        p["face_normal"].size() == 3) {
        const auto& n = p["face_normal"];
        in.entry_face = sk::FaceByNormal{
            gp_Dir(n[0].get<double>(), n[1].get<double>(), n[2].get<double>()), 5.0, "largest" };
    } else {
        in.entry_face = parseFaceDatum(p);
    }
    in.center_x_mm    = jdouble(p, "center_x_mm",    0.0);
    in.center_y_mm    = jdouble(p, "center_y_mm",    0.0);
    in.base_radius_mm = jdouble(p, "base_radius_mm", 0.0);
    in.height_mm      = jdouble(p, "height_mm",      0.0);
    in.sections       = static_cast<int>(jdouble(p, "sections", 6.0));
    return in;
}

sk::revolve_boss::Input parseRevolveBoss(const json& p)
{
    sk::revolve_boss::Input in;
    if (p.contains("profile_polyline") && p["profile_polyline"].is_array())
        for (const auto& pt : p["profile_polyline"])
            in.profile_polyline.emplace_back(pt.value("r", 0.0), pt.value("z", 0.0));
    if (p.contains("axis_origin") && p["axis_origin"].is_array() && p["axis_origin"].size() == 3) {
        const auto& a = p["axis_origin"];
        in.axis_origin = gp_Pnt(a[0].get<double>(), a[1].get<double>(), a[2].get<double>());
    }
    if (p.contains("axis_dir") && p["axis_dir"].is_array() && p["axis_dir"].size() == 3) {
        const auto& a = p["axis_dir"];
        in.axis_dir = gp_Dir(a[0].get<double>(), a[1].get<double>(), a[2].get<double>());
    }
    in.revolution_angle_deg = jdouble(p, "revolution_angle_deg", 360.0);
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
    t[sk::extrude_boss_from_sketch::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::extrude_boss_from_sketch::apply(wp, parseExtrudeBossFromSketch(p));
    };
    t[sk::revolve_boss::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::revolve_boss::apply(wp, parseRevolveBoss(p));
    };
    t[sk::dome_boss::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::dome_boss::apply(wp, parseDomeBoss(p));
    };
    t[sk::bolt_circle_pattern::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::bolt_circle_pattern::apply(wp, parseBoltCirclePattern(p));
    };
    t[sk::linear_hole_array::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::linear_hole_array::apply(wp, parseLinearHoleArray(p));
    };
    t[sk::rectangular_hole_grid::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::rectangular_hole_grid::apply(wp, parseRectangularHoleGrid(p));
    };
    t[sk::coaxial_step_bore::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::coaxial_step_bore::apply(wp, parseCoaxialStepBore(p));
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

    // ── Slice-9: bearings / O-ring & X-ring / fastener-seat spec skills ─
    t[sk::plain_bushing_bore_with_lube_groove::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::plain_bushing_bore_with_lube_groove::apply(wp, parsePlainBushingBoreWithLubeGroove(p));
    };
    t[sk::o_ring_groove_as568_spec::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::o_ring_groove_as568_spec::apply(wp, parseORingGrooveAs568Spec(p));
    };
    t[sk::x_ring_groove::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::x_ring_groove::apply(wp, parseXRingGroove(p));
    };
    t[sk::x_ring_groove_spec::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::x_ring_groove_spec::apply(wp, parseXRingGrooveSpec(p));
    };
    t[sk::spiral_back_up_ring_groove::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::spiral_back_up_ring_groove::apply(wp, parseSpiralBackUpRingGroove(p));
    };
    t[sk::dust_lip_seal_seat_compound::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::dust_lip_seal_seat_compound::apply(wp, parseDustLipSealSeatCompound(p));
    };
    t[sk::face_seal_compound_compression::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::face_seal_compound_compression::apply(wp, parseFaceSealCompoundCompression(p));
    };
    t[sk::gasket_face_with_drain_groove::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::gasket_face_with_drain_groove::apply(wp, parseGasketFaceWithDrainGroove(p));
    };
    t[sk::blind_threaded_insert_seat::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::blind_threaded_insert_seat::apply(wp, parseBlindThreadedInsertSeat(p));
    };
    t[sk::bolt_hole_metric_spec::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::bolt_hole_metric_spec::apply(wp, parseBoltHoleMetricSpec(p));
    };
    t[sk::tapped_hole_metric_spec::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::tapped_hole_metric_spec::apply(wp, parseTappedHoleMetricSpec(p));
    };
    t[sk::unc_unf_hole_spec::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::unc_unf_hole_spec::apply(wp, parseUncUnfHoleSpec(p));
    };
    t[sk::threaded_through_with_chamfers::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::threaded_through_with_chamfers::apply(wp, parseThreadedThroughWithChamfers(p));
    };
    t[sk::captive_screw_pocket_spec::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::captive_screw_pocket_spec::apply(wp, parseCaptiveScrewPocketSpec(p));
    };

    // Mechanical structures / drive / fluid / spring / linear-motion / adjusters
    t[sk::i_beam_compound_section::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::i_beam_compound_section::apply(wp, parseIBeamCompoundSection(p));
    };
    t[sk::box_section_with_endplate::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::box_section_with_endplate::apply(wp, parseBoxSectionWithEndplate(p));
    };
    t[sk::gusset_plate_compound::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::gusset_plate_compound::apply(wp, parseGussetPlateCompound(p));
    };
    t[sk::lifting_lug_pad_eye::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::lifting_lug_pad_eye::apply(wp, parseLiftingLugPadEye(p));
    };
    t[sk::base_bracket_compound::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::base_bracket_compound::apply(wp, parseBaseBracketCompound(p));
    };
    t[sk::din_rail_mount_slot::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::din_rail_mount_slot::apply(wp, parseDinRailMountSlot(p));
    };
    t[sk::dovetail_mount_compound::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::dovetail_mount_compound::apply(wp, parseDovetailMountCompound(p));
    };
    t[sk::t_slot_table_groove::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::t_slot_table_groove::apply(wp, parseTSlotTableGroove(p));
    };
    t[sk::linear_rail_seat_compound::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::linear_rail_seat_compound::apply(wp, parseLinearRailSeatCompound(p));
    };
    t[sk::ball_screw_nut_pocket::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::ball_screw_nut_pocket::apply(wp, parseBallScrewNutPocket(p));
    };
    t[sk::lead_screw_anti_backlash_pocket::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::lead_screw_anti_backlash_pocket::apply(wp, parseLeadScrewAntiBacklashPocket(p));
    };
    t[sk::linear_bushing_seat::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::linear_bushing_seat::apply(wp, parseLinearBushingSeat(p));
    };
    t[sk::cam_follower_threaded_seat::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::cam_follower_threaded_seat::apply(wp, parseCamFollowerThreadedSeat(p));
    };
    t[sk::internal_water_jacket::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::internal_water_jacket::apply(wp, parseInternalWaterJacket(p));
    };
    t[sk::adjuster_screw_pocket_compound::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::adjuster_screw_pocket_compound::apply(wp, parseAdjusterScrewPocketCompound(p));
    };
    t[sk::tab_lock_anti_rotation::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::tab_lock_anti_rotation::apply(wp, parseTabLockAntiRotation(p));
    };
    t[sk::coil_spring_seat_compound::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::coil_spring_seat_compound::apply(wp, parseCoilSpringSeatCompound(p));
    };
    t[sk::wave_spring_groove::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::wave_spring_groove::apply(wp, parseWaveSpringGroove(p));
    };
    t[sk::gas_spring_clevis_pocket::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::gas_spring_clevis_pocket::apply(wp, parseGasSpringClevisPocket(p));
    };
    t[sk::leaf_spring_anchor_compound::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::leaf_spring_anchor_compound::apply(wp, parseLeafSpringAnchorCompound(p));
    };
    t[sk::torsion_spring_anchor_compound::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::torsion_spring_anchor_compound::apply(wp, parseTorsionSpringAnchorCompound(p));
    };
    t[sk::leaf_spring_anchor::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::leaf_spring_anchor::apply(wp, parseLeafSpringAnchor(p));
    };

    // ── Slice 16/smart-spec/PCB-electronics compound features ────────────
    // ISO bore fit family
    t[sk::iso_h7_bore_spec::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::iso_h7_bore_spec::apply(wp, parseIsoH7BoreSpec(p));
    };
    t[sk::press_fit_p7_bore_spec::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::press_fit_p7_bore_spec::apply(wp, parsePressFitP7BoreSpec(p));
    };
    t[sk::slip_fit_h11_bore_spec::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::slip_fit_h11_bore_spec::apply(wp, parseSlipFitH11BoreSpec(p));
    };
    t[sk::dowel_pin_h6_bore::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::dowel_pin_h6_bore::apply(wp, parseDowelPinH6Bore(p));
    };
    t[sk::locating_g6_bore::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::locating_g6_bore::apply(wp, parseLocatingG6Bore(p));
    };
    // Parametric rib / wall
    t[sk::parametric_rib_array::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::parametric_rib_array::apply(wp, parseParametricRibArray(p));
    };
    t[sk::draft_wall_with_radius::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::draft_wall_with_radius::apply(wp, parseDraftWallWithRadius(p));
    };
    t[sk::top_face_recess_with_walls::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::top_face_recess_with_walls::apply(wp, parseTopFaceRecessWithWalls(p));
    };
    t[sk::partition_wall_with_passthrough::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::partition_wall_with_passthrough::apply(wp, parsePartitionWallWithPassthrough(p));
    };
    t[sk::curved_lip_around_face::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::curved_lip_around_face::apply(wp, parseCurvedLipAroundFace(p));
    };
    // Heat / vent (electronics)
    t[sk::heat_sink_fin_array::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::heat_sink_fin_array::apply(wp, parseHeatSinkFinArray(p));
    };
    t[sk::vent_slot_array::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::vent_slot_array::apply(wp, parseVentSlotArray(p));
    };
    t[sk::louvered_vent::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::louvered_vent::apply(wp, parseLouveredVent(p));
    };
    t[sk::perforated_grille_pattern::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::perforated_grille_pattern::apply(wp, parsePerforatedGrillePattern(p));
    };
    t[sk::breather_vent_compound::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::breather_vent_compound::apply(wp, parseBreatherVentCompound(p));
    };
    // PCB / electronics mounting
    t[sk::pcb_standoff_array_under_board::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::pcb_standoff_array_under_board::apply(wp, parsePcbStandoffArrayUnderBoard(p));
    };
    t[sk::connector_cutout_with_keepout::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::connector_cutout_with_keepout::apply(wp, parseConnectorCutoutWithKeepout(p));
    };
    t[sk::cable_grommet_pass_through::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::cable_grommet_pass_through::apply(wp, parseCableGrommetPassThrough(p));
    };
    t[sk::isolator_grommet_seat::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::isolator_grommet_seat::apply(wp, parseIsolatorGrommetSeat(p));
    };
    t[sk::tilt_post_for_lcd_panel::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::tilt_post_for_lcd_panel::apply(wp, parseTiltPostForLcdPanel(p));
    };
    // Connector cutouts (slice 16)
    t[sk::din_rail_clip_slot::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::din_rail_clip_slot::apply(wp, parseDinRailClipSlot(p));
    };
    t[sk::banana_jack_receptacle::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::banana_jack_receptacle::apply(wp, parseBananaJackReceptacle(p));
    };
    t[sk::rj45_socket_cutout::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::rj45_socket_cutout::apply(wp, parseRj45SocketCutout(p));
    };
    // Sliding / pivot mechanisms (smart-spec)
    t[sk::linear_slider_track::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::linear_slider_track::apply(wp, parseLinearSliderTrack(p));
    };
    t[sk::pivot_pin_clevis_compound::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::pivot_pin_clevis_compound::apply(wp, parsePivotPinClevisCompound(p));
    };
    t[sk::cam_actuated_slider::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::cam_actuated_slider::apply(wp, parseCamActuatedSlider(p));
    };
    t[sk::over_center_toggle_pocket::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::over_center_toggle_pocket::apply(wp, parseOverCenterTogglePocket(p));
    };
    t[sk::detented_position_slider::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::detented_position_slider::apply(wp, parseDetentedPositionSlider(p));
    };
    // Watch + hinge (slice 16)
    t[sk::bezel_groove_assembly::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::bezel_groove_assembly::apply(wp, parseBezelGrooveAssembly(p));
    };
    t[sk::lug_with_spring_bar_holes::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::lug_with_spring_bar_holes::apply(wp, parseLugWithSpringBarHoles(p));
    };
    t[sk::crown_stem_cavity_compound::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::crown_stem_cavity_compound::apply(wp, parseCrownStemCavityCompound(p));
    };

    // ── Slice 9: context-aware / morph / sheet / heat-exchanger / misc ────
    // Context-aware auto-synthesis
    t[sk::auto_boss_under_hole::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::auto_boss_under_hole::apply(wp, parseAutoBossUnderHole(p));
    };
    t[sk::auto_standoff_floating_point::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::auto_standoff_floating_point::apply(wp, parseAutoStandoffFloatingPoint(p));
    };
    t[sk::auto_rib_between_two_walls::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::auto_rib_between_two_walls::apply(wp, parseAutoRibBetweenTwoWalls(p));
    };
    t[sk::auto_gusset_corner_brace::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::auto_gusset_corner_brace::apply(wp, parseAutoGussetCornerBrace(p));
    };
    t[sk::auto_chamfer_all_outer_edges::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::auto_chamfer_all_outer_edges::apply(wp, parseAutoChamferAllOuterEdges(p));
    };
    // Morphing (single-workpiece)
    t[sk::blend_morph_two_shapes::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::blend_morph_two_shapes::apply(wp, parseBlendMorphTwoShapes(p));
    };
    t[sk::parametric_sweep_morph::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::parametric_sweep_morph::apply(wp, parseParametricSweepMorph(p));
    };
    t[sk::deformation_warp::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::deformation_warp::apply(wp, parseDeformationWarp(p));
    };
    // Sheet variants
    t[sk::lance::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::lance::apply(wp, parseLance(p));
    };
    t[sk::beading::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::beading::apply(wp, parseBeading(p));
    };
    // Heat exchanger geometric
    t[sk::shell_roll::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::shell_roll::apply(wp, parseShellRoll(p));
    };
    t[sk::hemispherical_head_form::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::hemispherical_head_form::apply(wp, parseHemisphericalHeadForm(p));
    };
    t[sk::expand_tube::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::expand_tube::apply(wp, parseExpandTube(p));
    };
    t[sk::tube_to_tubesheet_weld::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::tube_to_tubesheet_weld::apply(wp, parseTubeToTubesheetWeld(p));
    };
    t[sk::tube_swage::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::tube_swage::apply(wp, parseTubeSwage(p));
    };
    // Misc structural connections
    t[sk::bolted_flange_compound::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::bolted_flange_compound::apply(wp, parseBoltedFlangeCompound(p));
    };
    t[sk::pinned_clevis_joint::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::pinned_clevis_joint::apply(wp, parsePinnedClevisJoint(p));
    };
    t[sk::expansion_joint_bellows_stub::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::expansion_joint_bellows_stub::apply(wp, parseExpansionJointBellowsStub(p));
    };
    t[sk::anchor_pad_compound::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::anchor_pad_compound::apply(wp, parseAnchorPadCompound(p));
    };
    t[sk::flange_face_with_gasket_groove::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::flange_face_with_gasket_groove::apply(wp, parseFlangeFaceWithGasketGroove(p));
    };

    // ── Slice 9: valve / electrical / tooling / machine / hinge ──────────
    // Valve seats
    t[sk::gate_valve_seat_compound::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::gate_valve_seat_compound::apply(wp, parseGateValveSeatCompound(p));
    };
    t[sk::ball_valve_seat_compound::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::ball_valve_seat_compound::apply(wp, parseBallValveSeatCompound(p));
    };
    t[sk::butterfly_valve_disc_seat::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::butterfly_valve_disc_seat::apply(wp, parseButterflyValveDiscSeat(p));
    };
    t[sk::check_valve_seat_with_stop::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::check_valve_seat_with_stop::apply(wp, parseCheckValveSeatWithStop(p));
    };
    t[sk::needle_valve_seat::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::needle_valve_seat::apply(wp, parseNeedleValveSeat(p));
    };
    // Electrical contacts
    t[sk::banana_socket_compound::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::banana_socket_compound::apply(wp, parseBananaSocketCompound(p));
    };
    t[sk::spring_contact_clip::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::spring_contact_clip::apply(wp, parseSpringContactClip(p));
    };
    t[sk::terminal_block_post::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::terminal_block_post::apply(wp, parseTerminalBlockPost(p));
    };
    t[sk::busbar_lap_joint::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::busbar_lap_joint::apply(wp, parseBusbarLapJoint(p));
    };
    t[sk::pcb_card_edge_socket::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::pcb_card_edge_socket::apply(wp, parsePcbCardEdgeSocket(p));
    };
    // Tooling / jig
    t[sk::jig_plate_with_drill_bushings::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::jig_plate_with_drill_bushings::apply(wp, parseJigPlateWithDrillBushings(p));
    };
    t[sk::locator_pin_set::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::locator_pin_set::apply(wp, parseLocatorPinSet(p));
    };
    t[sk::gauge_block_step::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::gauge_block_step::apply(wp, parseGaugeBlockStep(p));
    };
    t[sk::vise_jaw_with_v_groove::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::vise_jaw_with_v_groove::apply(wp, parseViseJawWithVGroove(p));
    };
    t[sk::pin_and_diamond_locating_set::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::pin_and_diamond_locating_set::apply(wp, parsePinAndDiamondLocatingSet(p));
    };
    // Machine elements
    t[sk::cam_with_profile::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::cam_with_profile::apply(wp, parseCamWithProfile(p));
    };
    t[sk::cam_follower_roller_seat::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::cam_follower_roller_seat::apply(wp, parseCamFollowerRollerSeat(p));
    };
    t[sk::eccentric_shaft_collar::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::eccentric_shaft_collar::apply(wp, parseEccentricShaftCollar(p));
    };
    t[sk::flywheel_with_balance::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::flywheel_with_balance::apply(wp, parseFlywheelWithBalance(p));
    };
    t[sk::governor_arm_with_pivot::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::governor_arm_with_pivot::apply(wp, parseGovernorArmWithPivot(p));
    };
    t[sk::spur_gear_with_real_teeth::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::spur_gear_with_real_teeth::apply(wp, parseSpurGearWithRealTeeth(p));
    };
    t[sk::helical_gear_teeth::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::helical_gear_teeth::apply(wp, parseHelicalGearTeeth(p));
    };
    t[sk::sprocket_with_chain_teeth::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::sprocket_with_chain_teeth::apply(wp, parseSprocketWithChainTeeth(p));
    };
    t[sk::spline_shaft_compound::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::spline_shaft_compound::apply(wp, parseSplineShaftCompound(p));
    };
    t[sk::ratchet_pawl_set::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::ratchet_pawl_set::apply(wp, parseRatchetPawlSet(p));
    };
    // Hinge / latch
    t[sk::piano_hinge_strip::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::piano_hinge_strip::apply(wp, parsePianoHingeStrip(p));
    };
    t[sk::overcenter_latch::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::overcenter_latch::apply(wp, parseOvercenterLatch(p));
    };
    t[sk::snap_action_lock_pocket::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::snap_action_lock_pocket::apply(wp, parseSnapActionLockPocket(p));
    };
    t[sk::gas_strut_hinge_compound::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::gas_strut_hinge_compound::apply(wp, parseGasStrutHingeCompound(p));
    };
    t[sk::spring_loaded_door_latch::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::spring_loaded_door_latch::apply(wp, parseSpringLoadedDoorLatch(p));
    };
    t[sk::cam_lock_cavity::kSkillId] = [](const sk::Workpiece& wp, const json& p) {
        return sk::cam_lock_cavity::apply(wp, parseCamLockCavity(p));
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
    // Preserve the legacy abort-on-error semantics for existing callers.
    return execute(plan, std::move(initial_workpiece), ExecuteOptions{});
}

ExecutionResult Executor::execute(const ProcessPlan& plan,
                                  std::shared_ptr<sk::Workpiece> initial_workpiece,
                                  const ExecuteOptions& opts)
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

    // Helper: record a per-step error and decide whether to abort.
    //   Returns `true` if execution should stop (abort mode),
    //   `false` if execution should keep going (continue_on_error mode).
    // Mutates: result.errors, result.failedAtStep, result.skipped_step_count,
    // result.step_errors, result.workpiece (rewound to `current`).
    auto record_step_error = [&](int step_index,
                                 const std::string& skill_id,
                                 const std::string& msg) -> bool
    {
        result.errors.push_back(msg);
        result.failedAtStep = step_index;
        result.workpiece    = current;   // keep last-good workpiece

        if (opts.continue_on_error) {
            result.skipped_step_count++;
            result.step_errors.emplace_back(step_index, msg);
            spdlog::warn("Executor: step {} ({}) failed: {} — continuing",
                         step_index, skill_id, msg);
            return false;   // do not abort
        }
        spdlog::error("{}", msg);
        return true;        // abort
    };

    const auto& steps = plan.steps();
    for (int i = 0; i < static_cast<int>(steps.size()); ++i) {
        const auto& step = steps[i];

        const auto it = table.find(step.skill_id);
        if (it == table.end()) {
            std::string msg = "Executor: unknown skill_id '" + step.skill_id +
                              "' at step " + std::to_string(i);
            if (record_step_error(i, step.skill_id, msg)) {
                return result;
            }
            continue;   // continue_on_error: skip this step
        }

        try {
            // B3.4: clamp recovered params to the live stock so an over-specified
            // RE candidate (e.g. wall_thickness > stock) cannot crash apply().
            // No-op for skills without a clamp rule.
            const json clamped = clampParams(step.skill_id, step.params, *current);
            sk::SkillOutput out = it->second(*current, clamped);
            if (!out.workpiece) {
                std::string msg = "Executor: skill '" + step.skill_id +
                                  "' returned null workpiece at step " +
                                  std::to_string(i);
                if (record_step_error(i, step.skill_id, msg)) {
                    return result;
                }
                continue;
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
            if (record_step_error(i, step.skill_id, msg)) {
                return result;
            }
            continue;
        } catch (const std::exception& e) {
            std::string msg = "Executor: exception at step " + std::to_string(i) +
                              " (" + step.skill_id + "): " + e.what();
            if (record_step_error(i, step.skill_id, msg)) {
                return result;
            }
            continue;
        }
    }

    result.workpiece = current;
    // In continue_on_error mode, failedAtStep reflects the last failed step
    // index; ok() will be false if any step errored.  In abort mode this
    // line resets to -1 only if the loop completed without aborting.
    if (!opts.continue_on_error) {
        result.failedAtStep = -1;
    } else if (result.skipped_step_count == 0) {
        result.failedAtStep = -1;
    }
    return result;
}

}  // namespace koocadcam::process
