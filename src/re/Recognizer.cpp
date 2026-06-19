// @lat: [[engine/reverse-route#Recognizer]]

#include "Recognizer.hpp"

#include "CompoundGrammar.hpp"
#include "process/StepInvocation.hpp"

#include "skills/bore_cylindrical.hpp"
#include "skills/bore_with_shelf.hpp"
#include "skills/micro_drill.hpp"
#include "skills/gun_drill.hpp"
#include "skills/multi_step_bore.hpp"
#include "skills/drill_through_hole.hpp"
#include "skills/ream.hpp"
#include "skills/spot_drill.hpp"
#include "skills/spot_face.hpp"
#include "skills/chamfer_edge.hpp"
#include "skills/counterbore.hpp"
#include "skills/countersink.hpp"
#include "skills/drill_hole.hpp"
#include "skills/fillet_edge.hpp"
#include "skills/hollow_cavity.hpp"
#include "skills/mill_circular_pocket.hpp"
#include "skills/mill_rect_pocket.hpp"
#include "skills/mill_slot.hpp"

// ── Wave 16+ compound feature recognizers ────────────────────────────────
// Bearing / seal / fastener compounds.
#include "skills/radial_bearing_seat_with_snapring.hpp"
#include "skills/thrust_bearing_seat_compound.hpp"
#include "skills/plain_bushing_bore_with_lube_groove.hpp"
#include "skills/sealed_bearing_seat_with_shield_relief.hpp"
#include "skills/needle_bearing_seat_press_fit.hpp"
#include "skills/o_ring_groove_radial.hpp"
#include "skills/o_ring_groove_face.hpp"
#include "skills/o_ring_groove_as568_spec.hpp"
#include "skills/x_ring_groove.hpp"
#include "skills/x_ring_groove_spec.hpp"
#include "skills/lip_seal_seat.hpp"
#include "skills/spiral_back_up_ring_groove.hpp"
#include "skills/dust_lip_seal_seat_compound.hpp"
#include "skills/face_seal_compound_compression.hpp"
#include "skills/gasket_face_with_drain_groove.hpp"
#include "skills/countersunk_bolt_seat.hpp"
#include "skills/socket_head_bolt_seat.hpp"
#include "skills/helicoil_pilot_compound.hpp"
#include "skills/captive_nut_pocket.hpp"
#include "skills/blind_threaded_insert_seat.hpp"
#include "skills/bolt_hole_metric_spec.hpp"
#include "skills/tapped_hole_metric_spec.hpp"
#include "skills/unc_unf_hole_spec.hpp"
#include "skills/threaded_through_with_chamfers.hpp"
#include "skills/captive_screw_pocket_spec.hpp"

// Mechanical structures.
#include "skills/i_beam_compound_section.hpp"
#include "skills/box_section_with_endplate.hpp"
#include "skills/gusset_plate_compound.hpp"
#include "skills/lifting_lug_pad_eye.hpp"
#include "skills/base_bracket_compound.hpp"
#include "skills/vibration_isolator_seat.hpp"
#include "skills/rubber_grommet_seat.hpp"
#include "skills/din_rail_mount_slot.hpp"
#include "skills/dovetail_mount_compound.hpp"
#include "skills/t_slot_table_groove.hpp"
#include "skills/pulley_with_keyway_compound.hpp"
#include "skills/sprocket_blank_with_bore.hpp"
#include "skills/gear_blank_with_hub_step.hpp"
#include "skills/shaft_with_keyway_step.hpp"
#include "skills/flywheel_blank_with_balance_drills.hpp"
#include "skills/linear_rail_seat_compound.hpp"
#include "skills/ball_screw_nut_pocket.hpp"
#include "skills/lead_screw_anti_backlash_pocket.hpp"
#include "skills/linear_bushing_seat.hpp"
#include "skills/cam_follower_threaded_seat.hpp"
#include "skills/manifold_cross_drill_compound.hpp"
#include "skills/threaded_npt_port.hpp"
#include "skills/jic_flare_port_seat.hpp"
#include "skills/banjo_fitting_seat.hpp"

// Valves / electrical / tooling / machine / hinges.
#include "skills/gate_valve_seat_compound.hpp"
#include "skills/ball_valve_seat_compound.hpp"
#include "skills/butterfly_valve_disc_seat.hpp"
#include "skills/check_valve_seat_with_stop.hpp"
#include "skills/needle_valve_seat.hpp"
#include "skills/banana_socket_compound.hpp"
#include "skills/spring_contact_clip.hpp"
#include "skills/terminal_block_post.hpp"
#include "skills/busbar_lap_joint.hpp"
#include "skills/pcb_card_edge_socket.hpp"
#include "skills/jig_plate_with_drill_bushings.hpp"
#include "skills/locator_pin_set.hpp"
#include "skills/gauge_block_step.hpp"
#include "skills/vise_jaw_with_v_groove.hpp"
#include "skills/pin_and_diamond_locating_set.hpp"
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
#include "skills/piano_hinge_strip.hpp"
#include "skills/overcenter_latch.hpp"
#include "skills/snap_action_lock_pocket.hpp"
#include "skills/gas_strut_hinge_compound.hpp"
#include "skills/spring_loaded_door_latch.hpp"
#include "skills/butt_hinge_pocket.hpp"
#include "skills/magnetic_latch_pocket.hpp"
#include "skills/cam_lock_cavity.hpp"

// ISO fit / parametric walls / PCB / heat & vent / smart-spec.
#include "skills/iso_h7_bore_spec.hpp"
#include "skills/press_fit_p7_bore_spec.hpp"
#include "skills/slip_fit_h11_bore_spec.hpp"
#include "skills/dowel_pin_h6_bore.hpp"
#include "skills/locating_g6_bore.hpp"
#include "skills/parametric_rib_array.hpp"
#include "skills/draft_wall_with_radius.hpp"
#include "skills/top_face_recess_with_walls.hpp"
#include "skills/partition_wall_with_passthrough.hpp"
#include "skills/curved_lip_around_face.hpp"
#include "skills/heat_sink_fin_array.hpp"
#include "skills/vent_slot_array.hpp"
#include "skills/louvered_vent.hpp"
#include "skills/perforated_grille_pattern.hpp"
#include "skills/breather_vent_compound.hpp"
#include "skills/pcb_standoff_array_under_board.hpp"
#include "skills/connector_cutout_with_keepout.hpp"
#include "skills/cable_grommet_pass_through.hpp"
#include "skills/isolator_grommet_seat.hpp"
#include "skills/tilt_post_for_lcd_panel.hpp"
#include "skills/din_rail_clip_slot.hpp"
#include "skills/banana_jack_receptacle.hpp"
#include "skills/rj45_socket_cutout.hpp"
#include "skills/usb_c_port_cutout.hpp"
#include "skills/linear_slider_track.hpp"
#include "skills/pivot_pin_clevis_compound.hpp"
#include "skills/cam_actuated_slider.hpp"
#include "skills/over_center_toggle_pocket.hpp"
#include "skills/detented_position_slider.hpp"
#include "skills/bezel_groove_assembly.hpp"
#include "skills/lug_with_spring_bar_holes.hpp"
#include "skills/crown_stem_cavity_compound.hpp"
#include "skills/caseback_o_ring_groove.hpp"
#include "skills/sapphire_glass_seat.hpp"

// Context-aware auto / sheet-metal / heat-exchanger / structural geom.
#include "skills/auto_boss_under_hole.hpp"
#include "skills/auto_standoff_floating_point.hpp"
#include "skills/auto_rib_between_two_walls.hpp"
#include "skills/auto_gusset_corner_brace.hpp"
#include "skills/auto_chamfer_all_outer_edges.hpp"
#include "skills/notch.hpp"
#include "skills/lance.hpp"
#include "skills/beading.hpp"
#include "skills/coining.hpp"
#include "skills/draw_bead.hpp"
#include "skills/shell_roll.hpp"
#include "skills/hemispherical_head_form.hpp"
#include "skills/expand_tube.hpp"
#include "skills/tube_to_tubesheet_weld.hpp"
#include "skills/tube_swage.hpp"
#include "skills/bolted_flange_compound.hpp"
#include "skills/pinned_clevis_joint.hpp"
#include "skills/expansion_joint_bellows_stub.hpp"
#include "skills/anchor_pad_compound.hpp"
#include "skills/flange_face_with_gasket_groove.hpp"
#include "skills/leaf_spring_anchor.hpp"
#include "skills/leaf_spring_anchor_compound.hpp"
#include "skills/torsion_spring_anchor_compound.hpp"
#include "skills/adjuster_screw_pocket_compound.hpp"
#include "skills/tab_lock_anti_rotation.hpp"
#include "skills/set_screw_anti_rotation_pocket.hpp"
#include "skills/eccentric_bushing_seat.hpp"
#include "skills/extrude_boss_from_sketch.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>

namespace koocadcam::re {

using nlohmann::json;

// ── Recognizer registry ──────────────────────────────────────────────────
//
// Each entry wraps one skill's recognize() into the uniform RecognizeFn
// signature.  Slice-1 lists the eleven recognizers whose round-trip
// behaviour is documented.  Adding a new skill is a one-line entry once
// its recognize() has acceptable test coverage.

namespace {

std::vector<RecognizeFn> buildRegistry()
{
    std::vector<RecognizeFn> reg;
    // Stock removal first (informational only — registry order is independent
    // of plan order; plan order is set by inferProcessPlan() below).
    reg.emplace_back(&skill::hollow_cavity::recognize);
    reg.emplace_back(&skill::mill_circular_pocket::recognize);
    reg.emplace_back(&skill::mill_rect_pocket::recognize);
    reg.emplace_back(&skill::mill_slot::recognize);
    // Holes & drills.
    reg.emplace_back(&skill::drill_hole::recognize);
    reg.emplace_back(&skill::counterbore::recognize);
    reg.emplace_back(&skill::countersink::recognize);
    reg.emplace_back(&skill::bore_cylindrical::recognize);
    reg.emplace_back(&skill::bore_with_shelf::recognize);
    // Previously implemented but never registered (breakthrough-plan audit
    // finding): their recognize() existed yet could not run.  Non-precise
    // tier, so the foreign-CAD fallback cap (0.5) bounds their candidates.
    reg.emplace_back(&skill::micro_drill::recognize);
    reg.emplace_back(&skill::gun_drill::recognize);
    reg.emplace_back(&skill::multi_step_bore::recognize);
    // PRECISE-tier skills that were never registered (corpus audit, plan B7):
    // their recognize() existed and the cap exempts them, yet analyze()
    // never called them — structural recall 0 until now.
    reg.emplace_back(&skill::drill_through_hole::recognize);
    reg.emplace_back(&skill::ream::recognize);
    reg.emplace_back(&skill::spot_drill::recognize);
    reg.emplace_back(&skill::spot_face::recognize);
    // Edge operations.
    reg.emplace_back(&skill::fillet_edge::recognize);
    reg.emplace_back(&skill::chamfer_edge::recognize);

    // ── Wave 16+ compound recognizers ────────────────────────────────────
    // The registry order is informational only; inferProcessPlan() buckets
    // candidates by skill_id via classify().  Compound features (most of
    // these) fall through to Group::Unknown today and are appended after
    // edge ops — promoting them to dedicated buckets is future work.

    // Bearing / seal / fastener compounds (25).
    reg.emplace_back(&skill::radial_bearing_seat_with_snapring::recognize);
    reg.emplace_back(&skill::thrust_bearing_seat_compound::recognize);
    reg.emplace_back(&skill::plain_bushing_bore_with_lube_groove::recognize);
    reg.emplace_back(&skill::sealed_bearing_seat_with_shield_relief::recognize);
    reg.emplace_back(&skill::needle_bearing_seat_press_fit::recognize);
    reg.emplace_back(&skill::o_ring_groove_radial::recognize);
    reg.emplace_back(&skill::o_ring_groove_face::recognize);
    reg.emplace_back(&skill::o_ring_groove_as568_spec::recognize);
    reg.emplace_back(&skill::x_ring_groove::recognize);
    reg.emplace_back(&skill::x_ring_groove_spec::recognize);
    reg.emplace_back(&skill::lip_seal_seat::recognize);
    reg.emplace_back(&skill::spiral_back_up_ring_groove::recognize);
    reg.emplace_back(&skill::dust_lip_seal_seat_compound::recognize);
    reg.emplace_back(&skill::face_seal_compound_compression::recognize);
    reg.emplace_back(&skill::gasket_face_with_drain_groove::recognize);
    reg.emplace_back(&skill::countersunk_bolt_seat::recognize);
    reg.emplace_back(&skill::socket_head_bolt_seat::recognize);
    reg.emplace_back(&skill::helicoil_pilot_compound::recognize);
    reg.emplace_back(&skill::captive_nut_pocket::recognize);
    reg.emplace_back(&skill::blind_threaded_insert_seat::recognize);
    reg.emplace_back(&skill::bolt_hole_metric_spec::recognize);
    reg.emplace_back(&skill::tapped_hole_metric_spec::recognize);
    reg.emplace_back(&skill::unc_unf_hole_spec::recognize);
    reg.emplace_back(&skill::threaded_through_with_chamfers::recognize);
    reg.emplace_back(&skill::captive_screw_pocket_spec::recognize);

    // Mechanical structures (24).
    reg.emplace_back(&skill::i_beam_compound_section::recognize);
    reg.emplace_back(&skill::box_section_with_endplate::recognize);
    reg.emplace_back(&skill::gusset_plate_compound::recognize);
    reg.emplace_back(&skill::lifting_lug_pad_eye::recognize);
    reg.emplace_back(&skill::base_bracket_compound::recognize);
    reg.emplace_back(&skill::vibration_isolator_seat::recognize);
    reg.emplace_back(&skill::rubber_grommet_seat::recognize);
    reg.emplace_back(&skill::din_rail_mount_slot::recognize);
    reg.emplace_back(&skill::dovetail_mount_compound::recognize);
    reg.emplace_back(&skill::t_slot_table_groove::recognize);
    reg.emplace_back(&skill::pulley_with_keyway_compound::recognize);
    reg.emplace_back(&skill::sprocket_blank_with_bore::recognize);
    reg.emplace_back(&skill::gear_blank_with_hub_step::recognize);
    reg.emplace_back(&skill::shaft_with_keyway_step::recognize);
    reg.emplace_back(&skill::flywheel_blank_with_balance_drills::recognize);
    reg.emplace_back(&skill::linear_rail_seat_compound::recognize);
    reg.emplace_back(&skill::ball_screw_nut_pocket::recognize);
    reg.emplace_back(&skill::lead_screw_anti_backlash_pocket::recognize);
    reg.emplace_back(&skill::linear_bushing_seat::recognize);
    reg.emplace_back(&skill::cam_follower_threaded_seat::recognize);
    reg.emplace_back(&skill::manifold_cross_drill_compound::recognize);
    reg.emplace_back(&skill::threaded_npt_port::recognize);
    reg.emplace_back(&skill::jic_flare_port_seat::recognize);
    reg.emplace_back(&skill::banjo_fitting_seat::recognize);

    // Valves / electrical / tooling / machine / hinges (33).
    reg.emplace_back(&skill::gate_valve_seat_compound::recognize);
    reg.emplace_back(&skill::ball_valve_seat_compound::recognize);
    reg.emplace_back(&skill::butterfly_valve_disc_seat::recognize);
    reg.emplace_back(&skill::check_valve_seat_with_stop::recognize);
    reg.emplace_back(&skill::needle_valve_seat::recognize);
    reg.emplace_back(&skill::banana_socket_compound::recognize);
    reg.emplace_back(&skill::spring_contact_clip::recognize);
    reg.emplace_back(&skill::terminal_block_post::recognize);
    reg.emplace_back(&skill::busbar_lap_joint::recognize);
    reg.emplace_back(&skill::pcb_card_edge_socket::recognize);
    reg.emplace_back(&skill::jig_plate_with_drill_bushings::recognize);
    reg.emplace_back(&skill::locator_pin_set::recognize);
    reg.emplace_back(&skill::gauge_block_step::recognize);
    reg.emplace_back(&skill::vise_jaw_with_v_groove::recognize);
    reg.emplace_back(&skill::pin_and_diamond_locating_set::recognize);
    reg.emplace_back(&skill::cam_with_profile::recognize);
    reg.emplace_back(&skill::cam_follower_roller_seat::recognize);
    reg.emplace_back(&skill::eccentric_shaft_collar::recognize);
    reg.emplace_back(&skill::flywheel_with_balance::recognize);
    reg.emplace_back(&skill::governor_arm_with_pivot::recognize);
    reg.emplace_back(&skill::spur_gear_with_real_teeth::recognize);
    reg.emplace_back(&skill::helical_gear_teeth::recognize);
    reg.emplace_back(&skill::sprocket_with_chain_teeth::recognize);
    reg.emplace_back(&skill::spline_shaft_compound::recognize);
    reg.emplace_back(&skill::ratchet_pawl_set::recognize);
    reg.emplace_back(&skill::piano_hinge_strip::recognize);
    reg.emplace_back(&skill::overcenter_latch::recognize);
    reg.emplace_back(&skill::snap_action_lock_pocket::recognize);
    reg.emplace_back(&skill::gas_strut_hinge_compound::recognize);
    reg.emplace_back(&skill::spring_loaded_door_latch::recognize);
    reg.emplace_back(&skill::butt_hinge_pocket::recognize);
    reg.emplace_back(&skill::magnetic_latch_pocket::recognize);
    reg.emplace_back(&skill::cam_lock_cavity::recognize);

    // ISO fit / parametric walls / PCB / heat & vent / smart-spec (33).
    reg.emplace_back(&skill::iso_h7_bore_spec::recognize);
    reg.emplace_back(&skill::press_fit_p7_bore_spec::recognize);
    reg.emplace_back(&skill::slip_fit_h11_bore_spec::recognize);
    reg.emplace_back(&skill::dowel_pin_h6_bore::recognize);
    reg.emplace_back(&skill::locating_g6_bore::recognize);
    reg.emplace_back(&skill::parametric_rib_array::recognize);
    reg.emplace_back(&skill::draft_wall_with_radius::recognize);
    reg.emplace_back(&skill::top_face_recess_with_walls::recognize);
    reg.emplace_back(&skill::partition_wall_with_passthrough::recognize);
    reg.emplace_back(&skill::curved_lip_around_face::recognize);
    reg.emplace_back(&skill::heat_sink_fin_array::recognize);
    reg.emplace_back(&skill::vent_slot_array::recognize);
    reg.emplace_back(&skill::louvered_vent::recognize);
    reg.emplace_back(&skill::perforated_grille_pattern::recognize);
    reg.emplace_back(&skill::breather_vent_compound::recognize);
    reg.emplace_back(&skill::pcb_standoff_array_under_board::recognize);
    reg.emplace_back(&skill::connector_cutout_with_keepout::recognize);
    reg.emplace_back(&skill::cable_grommet_pass_through::recognize);
    reg.emplace_back(&skill::isolator_grommet_seat::recognize);
    reg.emplace_back(&skill::tilt_post_for_lcd_panel::recognize);
    reg.emplace_back(&skill::din_rail_clip_slot::recognize);
    reg.emplace_back(&skill::banana_jack_receptacle::recognize);
    reg.emplace_back(&skill::rj45_socket_cutout::recognize);
    reg.emplace_back(&skill::usb_c_port_cutout::recognize);
    reg.emplace_back(&skill::linear_slider_track::recognize);
    reg.emplace_back(&skill::pivot_pin_clevis_compound::recognize);
    reg.emplace_back(&skill::cam_actuated_slider::recognize);
    reg.emplace_back(&skill::over_center_toggle_pocket::recognize);
    reg.emplace_back(&skill::detented_position_slider::recognize);
    reg.emplace_back(&skill::bezel_groove_assembly::recognize);
    reg.emplace_back(&skill::lug_with_spring_bar_holes::recognize);
    reg.emplace_back(&skill::crown_stem_cavity_compound::recognize);
    reg.emplace_back(&skill::caseback_o_ring_groove::recognize);
    reg.emplace_back(&skill::sapphire_glass_seat::recognize);

    // Context-aware auto / sheet-metal / heat-exchanger / structural (27).
    reg.emplace_back(&skill::auto_boss_under_hole::recognize);
    reg.emplace_back(&skill::auto_standoff_floating_point::recognize);
    reg.emplace_back(&skill::auto_rib_between_two_walls::recognize);
    reg.emplace_back(&skill::auto_gusset_corner_brace::recognize);
    reg.emplace_back(&skill::auto_chamfer_all_outer_edges::recognize);
    reg.emplace_back(&skill::notch::recognize);
    reg.emplace_back(&skill::lance::recognize);
    reg.emplace_back(&skill::beading::recognize);
    reg.emplace_back(&skill::coining::recognize);
    reg.emplace_back(&skill::draw_bead::recognize);
    reg.emplace_back(&skill::shell_roll::recognize);
    reg.emplace_back(&skill::hemispherical_head_form::recognize);
    reg.emplace_back(&skill::expand_tube::recognize);
    reg.emplace_back(&skill::tube_to_tubesheet_weld::recognize);
    reg.emplace_back(&skill::tube_swage::recognize);
    reg.emplace_back(&skill::bolted_flange_compound::recognize);
    reg.emplace_back(&skill::pinned_clevis_joint::recognize);
    reg.emplace_back(&skill::expansion_joint_bellows_stub::recognize);
    reg.emplace_back(&skill::anchor_pad_compound::recognize);
    reg.emplace_back(&skill::flange_face_with_gasket_groove::recognize);
    reg.emplace_back(&skill::leaf_spring_anchor::recognize);
    reg.emplace_back(&skill::leaf_spring_anchor_compound::recognize);
    reg.emplace_back(&skill::torsion_spring_anchor_compound::recognize);
    reg.emplace_back(&skill::adjuster_screw_pocket_compound::recognize);
    reg.emplace_back(&skill::tab_lock_anti_rotation::recognize);
    reg.emplace_back(&skill::set_screw_anti_rotation_pocket::recognize);
    reg.emplace_back(&skill::eccentric_bushing_seat::recognize);

    // Sketch-based ADDITIVE feature (extrude): the first non-machining feature
    // the recognizer knows.  Its metadata path round-trips an extrusion
    // (recognize -> regenerate); the geometric profile recovery is a follow-up.
    reg.emplace_back(&skill::extrude_boss_from_sketch::recognize);

    return reg;
}

// ── Skill-id grouping ────────────────────────────────────────────────────
//
// Used by inferProcessPlan() to bucket candidates by machining stage.
// Substring/prefix matching keeps it stable as new skill IDs are added
// under existing families.

enum class Group { A_Stock = 0, B_Hole = 1, C_Edge = 2, Unknown = 3 };

Group classify(std::string_view skillId)
{
    static const std::string_view kStock[] = {
        "hollow_cavity",
        "mill_open_pocket",
        "profile_milling",
        "mill_rect_pocket",
        "mill_circular_pocket",
        "mill_slot",
        "mill_keyway",
        "dovetail_slot",
        "T_slot",
    };
    for (auto s : kStock) {
        if (skillId == s) return Group::A_Stock;
    }

    if (skillId.find("drill") == 0) return Group::B_Hole;
    if (skillId.find("bore")  == 0) return Group::B_Hole;
    static const std::string_view kHole[] = {
        "counterbore",
        "countersink",
        "spot_drill",
        "ream",
        "pocket_with_corner_relief",
    };
    for (auto s : kHole) {
        if (skillId == s) return Group::B_Hole;
    }

    static const std::string_view kEdge[] = {
        "chamfer_edge",
        "fillet_edge",
        "face_milling",
    };
    for (auto s : kEdge) {
        if (skillId == s) return Group::C_Edge;
    }

    return Group::Unknown;
}

}  // namespace

const std::vector<RecognizeFn>& recognizerRegistry()
{
    static const std::vector<RecognizeFn> kRegistry = buildRegistry();
    return kRegistry;
}

// ── analyze() ────────────────────────────────────────────────────────────

// Recognizers that recover a feature by RIGOROUS geometric measurement (exact
// cylinder radius, coaxial-pair step, bevel angle, ISO-273 diameter match).
// Their geometric candidates are trustworthy on foreign CAD.  Every other
// recognizer is a domain-compound heuristic: on a part WE synthesised it
// replays its own metadata at high confidence (fine), but on a FOREIGN STEP it
// only has a loose geometric fallback that pattern-matches generic planar +
// cylindrical faces — a false-positive source (jig_plate, gauge_block, …).  We
// cap such geometric-fallback candidates so they fall below the RE confidence
// threshold (0.7) while leaving metadata replay and the precise tier untouched.
const std::unordered_set<std::string>& preciseRecognizers()
{
    static const std::unordered_set<std::string> kPrecise = {
        "drill_hole", "drill_through_hole", "counterbore", "countersink",
        "chamfer_edge", "fillet_edge", "bore_cylindrical", "bore_with_shelf",
        "ream", "spot_drill", "spot_face", "mill_circular_pocket",
        "mill_rect_pocket", "mill_slot", "bolt_hole_metric_spec",
        // NOTE: extrude_boss_from_sketch is deliberately NOT cap-exempt.  Its
        // geometric path B recovers a correct profile+height (proven by
        // recognize() round-trip tests), but at full confidence it false-positives
        // on machining parts that happen to have a congruent planar cap pair
        // (fpscan CapDemotesAllDomainCompoundCandidates).  It therefore stays
        // capped to 0.5 in analyze(); surfacing it at >= 0.7 needs a more
        // specific extrusion gate (distinguish a real prism from a pocketed
        // block) — a tracked follow-up.  The metadata replay path is cap-exempt
        // via source="metadata_replay" and remains the live round-trip path.
    };
    return kPrecise;
}

std::vector<skill::RecognizedFeature> analyze(const skill::Workpiece& wp,
                                              bool applyCap)
{
    std::vector<skill::RecognizedFeature> all;
    for (const auto& fn : recognizerRegistry()) {
        try {
            auto cands = fn(wp);
            all.insert(all.end(), cands.begin(), cands.end());
        } catch (const std::exception& e) {
            spdlog::warn("re::analyze: a recognizer threw: {}", e.what());
        } catch (...) {
            spdlog::warn("re::analyze: a recognizer threw an unknown exception");
        }
    }

    // Demote loose geometric fallbacks from non-precise (domain-compound)
    // recognizers.  Metadata replay ("source":"metadata_replay") and the
    // precise tier are exempt, so synthetic round-trips are unaffected.
    // applyCap=false exposes the RAW confidence (corpus un-cap-safety scan).
    // NOTE (B1.x, measured 2026-06-14): a grounding-by-subsumption un-cap was
    // hypothesised here — keep a domain compound's raw confidence when its face
    // set STRICTLY CONTAINS a recognized precise atom's faces.  fpscan_test
    // REFUTED it: "wraps an atom" != "is the feature".  On a plain countersink
    // it un-capped tapped_hole_metric_spec (0.70) and countersunk_bolt_seat
    // (0.65); on a plain pocket, cam_actuated_slider / over_center_toggle_pocket
    // — and the >=0.7 ones would then BEAT the correct atom via the dedupe
    // specificity rule.  Safe un-capping needs B1.2 SPECIFIC atomic-prerequisite
    // matching (the compound's exact sub-features), not generic superset.  The
    // flat cap stays; fpscan proved it load-bearing.
    const auto& precise = preciseRecognizers();
    if (applyCap) {
        for (auto& c : all) {
            const bool replay =
                c.matched_geometry.is_object() &&
                c.matched_geometry.value("source", std::string()) == "metadata_replay";
            if (!replay && !precise.count(c.skill_id))
                c.confidence = std::min(c.confidence, 0.5);
        }
    }

    // B1.2 declarative compound grammar (orthodox un-cap path).  AFTER the cap:
    // grammar compounds are grounded in the PRECISE atoms above (drill_hole etc,
    // exempt from the cap, so still at full confidence) and emitted only when a
    // SPECIFIC declared pattern matches — they are not loose geometric fallbacks,
    // so they carry their grounded confidence instead of being demoted.  This is
    // the safe alternative to the refuted generic-superset un-cap noted above.
    {
        auto compounds = recognizeCompounds(all);
        all.insert(all.end(),
                   std::make_move_iterator(compounds.begin()),
                   std::make_move_iterator(compounds.end()));
    }

    std::stable_sort(all.begin(), all.end(),
        [](const skill::RecognizedFeature& a, const skill::RecognizedFeature& b) {
            return a.confidence > b.confidence;
        });
    return all;
}

std::vector<skill::RecognizedFeature>
analyzeFiltered(const skill::Workpiece& wp, double min_confidence)
{
    auto all = analyze(wp);
    all.erase(std::remove_if(all.begin(), all.end(),
        [min_confidence](const skill::RecognizedFeature& c) {
            return c.confidence < min_confidence;
        }), all.end());
    return all;
}

// ── dedupe() ─────────────────────────────────────────────────────────────
//
// Fingerprint extraction: walk matched_geometry and collect every integer
// face ID it references.  Two candidates "overlap" iff their face-ID sets
// intersect.  We greedily process candidates in confidence-descending
// order, keeping the first and dropping any subsequent candidate whose
// face-ID set intersects an already-kept set.
//
// Two skills may legitimately claim the same cylindrical face (e.g.
// drill_hole and bore_cylindrical, or drill_hole and mill_circular_pocket
// for borderline aspect ratios).  Slice-1 picks the higher-raw-confidence
// candidate; resolving "drill vs. pocket" with a smarter heuristic
// (depth/diameter ratio, surrounding context) is a future improvement.

namespace {

// Recursively extract integer face IDs from matched_geometry JSON.  Keys
// we care about: any field whose name contains "face_id" (singular or
// plural).  Values can be either a single integer or an array of integers.
void collectFaceIds(const json& mg, std::unordered_set<int>& out)
{
    if (mg.is_object()) {
        for (auto it = mg.begin(); it != mg.end(); ++it) {
            const std::string key = it.key();
            // Heuristic: any key mentioning "face_id" or "face_ids"
            // contributes integer IDs.
            const bool isFaceKey =
                key.find("face_id")  != std::string::npos ||
                key.find("face_ids") != std::string::npos ||
                key.find("cylinder_ids") != std::string::npos ||
                key.find("cyl_face")  != std::string::npos;
            if (isFaceKey) {
                if (it.value().is_number_integer()) {
                    out.insert(it.value().get<int>());
                } else if (it.value().is_array()) {
                    for (const auto& v : it.value()) {
                        if (v.is_number_integer()) out.insert(v.get<int>());
                    }
                }
            } else if (it.value().is_object() || it.value().is_array()) {
                collectFaceIds(it.value(), out);
            }
        }
    } else if (mg.is_array()) {
        for (const auto& v : mg) collectFaceIds(v, out);
    }
}

bool intersects(const std::unordered_set<int>& a, const std::unordered_set<int>& b)
{
    // Iterate over the smaller set for efficiency.
    const auto& small = (a.size() < b.size()) ? a : b;
    const auto& large = (a.size() < b.size()) ? b : a;
    for (int v : small) {
        if (large.count(v)) return true;
    }
    return false;
}

// True iff `sub` is a STRICT subset of `super` (everything in sub is in
// super, and super explains at least one additional face).
bool isStrictSubset(const std::unordered_set<int>& sub,
                    const std::unordered_set<int>& super)
{
    if (sub.size() >= super.size()) return false;
    for (int v : sub) {
        if (!super.count(v)) return false;
    }
    return true;
}

// STEP-round-trip-STABLE geometric fingerprint.  Face IDs renumber on every
// Workpiece(shape) build, so two recognizers describing the SAME physical hole
// (e.g. a through-hole seen from both ends, or a cylinder split into multiple
// STEP faces) get different face-ID sets and escape face-ID dedupe.  Key the
// feature by its measured GEOMETRY instead: skill + rounded diameters/depth +
// rounded entry position.  Axis is intentionally omitted so a through-hole
// reported from either end collapses to one.  Returns "" when the candidate
// exposes no positional/dimensional content (then we fall back to face-ID
// overlap and never over-merge dimensionless features like raw chamfers).
std::string geomFingerprint(const skill::RecognizedFeature& c)
{
    const json& p = c.recovered_params;
    if (!p.is_object()) return "";
    static const char* kDimKeys[] = {
        "diameter_mm", "seat_dia_mm", "pilot_dia_mm", "bore_dia_mm",
        "position_x_mm", "position_y_mm", "position_z_mm", "depth_mm"
    };
    std::string key = c.skill_id;
    bool any = false;
    for (const char* k : kDimKeys) {
        if (p.contains(k) && p[k].is_number()) {
            any = true;
            // 0.05 mm grid: collapses rebuild noise, keeps distinct features apart.
            const long q = std::lround(p[k].get<double>() * 20.0);
            key += '|'; key += k; key += ':'; key += std::to_string(q);
        }
    }
    return any ? key : "";
}

}  // namespace

std::vector<skill::RecognizedFeature>
dedupe(const std::vector<skill::RecognizedFeature>& candidates)
{
    // Work on a confidence-sorted copy (descending).
    std::vector<skill::RecognizedFeature> sorted = candidates;
    std::stable_sort(sorted.begin(), sorted.end(),
        [](const skill::RecognizedFeature& a, const skill::RecognizedFeature& b) {
            return a.confidence > b.confidence;
        });

    std::vector<skill::RecognizedFeature> kept;
    std::vector<std::unordered_set<int>> keptFaceSets;
    std::unordered_set<std::string>      seenGeom;   // geometric fingerprints

    for (const auto& c : sorted) {
        // 1) Geometric fingerprint — collapses the SAME physical feature even
        //    when its face IDs differ (the dominant duplicate source on real
        //    multi-face / through-hole STEP geometry).
        const std::string fp = geomFingerprint(c);
        if (!fp.empty()) {
            if (seenGeom.count(fp)) continue;       // exact same measured feature
            seenGeom.insert(fp);
        }

        // 2) Face-ID overlap — drop a competing claim (e.g. drill vs. bore) on
        //    geometry an already-kept candidate owns.
        //
        //    SPECIFICITY EXCEPTION (plan B1.5, measured 2026-06-11): raw
        //    confidence order lets drill_hole (0.95) claim a shared cylinder
        //    first and shadow the MORE COMPLETE explanation — the corpus
        //    measured post-dedupe recall 0 for counterbore / countersink /
        //    bore_with_shelf / mill_rect_pocket / mill_slot despite
        //    recall_pre 1.0 (same mechanism dropped AS1's counterbores).
        //    Rule: a challenger at or above the RE threshold whose face set
        //    STRICTLY CONTAINS every overlapping incumbent's set replaces
        //    those incumbents (the counterbore explains the drill's pilot
        //    AND the seat AND the entry).  Equal sets (drill vs bore vs
        //    circular pocket on the same two faces — genuine geometric
        //    aliases) still resolve by confidence, preserving the
        //    aspect-ratio ownership heuristics.
        std::unordered_set<int> fids;
        collectFaceIds(c.matched_geometry, fids);
        if (!fids.empty()) {
            std::vector<std::size_t> subsumed;
            bool blocked = false;
            for (std::size_t i = 0; i < keptFaceSets.size(); ++i) {
                if (!intersects(fids, keptFaceSets[i])) continue;
                if (c.confidence >= 0.7 &&
                    isStrictSubset(keptFaceSets[i], fids)) {
                    subsumed.push_back(i);
                } else {
                    blocked = true;
                    break;
                }
            }
            if (blocked) continue;
            for (auto it = subsumed.rbegin(); it != subsumed.rend(); ++it) {
                kept.erase(kept.begin() + static_cast<std::ptrdiff_t>(*it));
                keptFaceSets.erase(keptFaceSets.begin()
                                   + static_cast<std::ptrdiff_t>(*it));
            }
        }

        kept.push_back(c);
        keptFaceSets.push_back(std::move(fids));
    }
    return kept;
}

// ── liftRecoveredStep() ──────────────────────────────────────────────────
//
// Promoted from the integration tests (bearing_housing / watch_re_roundtrip /
// watch_complete each hand-rolled an identical copy).  Makes a recovered
// chamfer_edge / fillet_edge step directly replayable.

process::StepInvocation liftRecoveredStep(const process::StepInvocation& step)
{
    if (step.skill_id != "chamfer_edge" && step.skill_id != "fillet_edge")
        return step;

    process::StepInvocation s = step;
    if (s.params.is_object()) {
        // Lift the nested edge_selector up to the Executor's shorthand keys.
        if (s.params.contains("edge_selector") && s.params["edge_selector"].is_object()) {
            const auto& es = s.params["edge_selector"];
            if (es.contains("z_mm"))         s.params["edges_at_z_mm"] = es["z_mm"];
            if (es.contains("tolerance_mm")) s.params["tolerance_mm"]  = es["tolerance_mm"];
        }
        // Floor a sub-manufacturable chamfer to the DFM minimum.  The recognized
        // chamfer_size is MEASURED from the bevel strip (chamfer_edge's accept
        // band is [0.05, 50] mm), so it must NOT be over-written upward — an
        // earlier 2.0→0.5 "clamp" silently shrank legitimately-large measured
        // chamfers (3 mm → 0.5 mm) and is removed.
        if (s.skill_id == "chamfer_edge" &&
            s.params.contains("chamfer_size_mm") &&
            s.params["chamfer_size_mm"].is_number()) {
            const double sz = s.params["chamfer_size_mm"].get<double>();
            if (sz < 0.1) s.params["chamfer_size_mm"] = 0.1;   // DFM floor only
        }
    }
    return s;
}

// ── inferProcessPlan() ───────────────────────────────────────────────────
//
// Buckets the de-duplicated candidates by Group A/B/C using the classify()
// helper above and appends each bucket to the ProcessPlan in stock-removal →
// hole → edge order.

process::ProcessPlan
inferProcessPlan(const skill::Workpiece& wp, double min_confidence)
{
    auto candidates = analyzeFiltered(wp, min_confidence);
    candidates      = dedupe(candidates);

    // Bucket by group.
    std::vector<skill::RecognizedFeature> buckets[4];   // A, B, C, Unknown
    for (auto& c : candidates) {
        const Group g = classify(c.skill_id);
        buckets[static_cast<int>(g)].push_back(std::move(c));
    }

    // Within each bucket, confidence is already descending (we sorted in
    // analyze()/dedupe()).  Make it explicit for safety.
    for (auto& b : buckets) {
        std::stable_sort(b.begin(), b.end(),
            [](const skill::RecognizedFeature& a, const skill::RecognizedFeature& b2) {
                return a.confidence > b2.confidence;
            });
    }

    // SUBSUMPTION: a recognised, DISPATCHABLE hole-pattern compound REPLACES the
    // individual drill steps it consumed, so the recovered plan carries the
    // pattern as ONE editable step — edit it (e.g. 4→6 bolts) and it regenerates
    // cleanly instead of leaving the original drills behind.  Only the
    // generative hole-pattern compounds qualify (each has an apply()); other
    // compounds keep their atoms.
    {
        static const std::unordered_set<std::string> kHolePatternCompounds = {
            "bolt_circle_pattern", "linear_hole_array", "rectangular_hole_grid",
        };
        // A consumed centre carries its compound's hole diameter and whether
        // bores (not just drills) belong to it — so we DON'T over-subsume an
        // unrelated hole that merely sits near a centre (different diameter), or
        // a counterbore SEAT bore at a bolt-hole centre (a step bore alone owns
        // coaxial bores).
        struct Consumed { double x, y, dia; bool dropBores; };
        std::vector<Consumed> consumed;
        for (const auto& c : buckets[static_cast<int>(Group::Unknown)]) {
            const auto& rp = c.recovered_params;
            if (!rp.is_object()) continue;
            if (kHolePatternCompounds.count(c.skill_id)) {
                const double hd = rp.value("hole_dia_mm", 0.0);
                if (rp.contains("hole_centers") && rp["hole_centers"].is_array()) {
                    for (const auto& p : rp["hole_centers"]) {
                        if (p.is_array() && p.size() >= 2 &&
                            p[0].is_number() && p[1].is_number())
                            consumed.push_back({ p[0].get<double>(), p[1].get<double>(),
                                                 hd, /*dropBores=*/false });
                    }
                }
            } else if (c.skill_id == "coaxial_step_bore") {
                consumed.push_back({ rp.value("position_x_mm", 1e9),
                                     rp.value("position_y_mm", 1e9),
                                     0.0, /*dropBores=*/true });
            }
        }
        if (!consumed.empty()) {
            auto& holes = buckets[static_cast<int>(Group::B_Hole)];
            holes.erase(std::remove_if(holes.begin(), holes.end(),
                [&consumed](const skill::RecognizedFeature& c) {
                    const bool isDrill = c.skill_id == "drill_hole" ||
                                         c.skill_id == "drill_through_hole";
                    const bool isBore  = c.skill_id == "bore_cylindrical";
                    if (!isDrill && !isBore) return false;
                    const auto& p = c.recovered_params;
                    if (!p.is_object()) return false;
                    const double x = p.value("position_x_mm", 1e9);
                    const double y = p.value("position_y_mm", 1e9);
                    const double d = p.value("diameter_mm", -1.0);
                    for (const auto& cc : consumed) {
                        if (std::hypot(x - cc.x, y - cc.y) >= 0.5) continue;
                        if (cc.dropBores) return true;            // step-bore axis: any drill/bore
                        // Hole pattern: only the matching-diameter drills.
                        if (isDrill && (cc.dia <= 0.0 || std::abs(d - cc.dia) < 0.2))
                            return true;
                    }
                    return false;
                }), holes.end());
        }
    }

    process::ProcessPlan plan;

    auto appendBucket = [&plan](const std::vector<skill::RecognizedFeature>& bucket,
                                const char* groupName) {
        for (const auto& c : bucket) {
            process::StepInvocation step;
            step.skill_id   = c.skill_id;
            step.params     = c.recovered_params;
            step.depends_on = {};   // slice-1: linear; topological deps TODO.
            step.note       = std::string("re::inferProcessPlan group=") +
                              groupName + " conf=" + std::to_string(c.confidence);
            // Normalise edge-op params so the returned plan is replay-ready.
            plan.append(liftRecoveredStep(step));
        }
    };

    appendBucket(buckets[static_cast<int>(Group::A_Stock)], "A_stock_removal");
    appendBucket(buckets[static_cast<int>(Group::B_Hole)],  "B_hole");
    appendBucket(buckets[static_cast<int>(Group::C_Edge)],  "C_edge");
    // Unknown skill_ids are appended LAST (between edge ops and end) so
    // their inclusion isn't silently dropped; in practice the registry is
    // controlled so this bucket should stay empty.
    appendBucket(buckets[static_cast<int>(Group::Unknown)], "Unknown");

    return plan;
}

}  // namespace koocadcam::re
