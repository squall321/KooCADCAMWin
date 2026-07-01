// @lat: [[engine/reverse-route#fpscan]]
//
// Un-cap safety scan (breakthrough plan B1.x measurement, 2026-06-14).
//
// B1.1/B1.2 want to lift the ~750 domain-compound recognizers above the 0.7
// inference threshold on foreign CAD.  Doing that SAFELY requires knowing,
// per recognizer, whether it spuriously fires at high RAW confidence on parts
// that do NOT contain its feature — those are NOT safe to un-cap.
//
// This test builds a panel of foreign-equivalent parts (synthesize a precise
// feature, STEP-round-trip to strip metadata), runs re::analyze with the
// foreign-CAD cap DISABLED (raw confidence) and ENABLED (production), and:
//   1. proves the cap is doing its job — every non-precise candidate it
//      returns is demoted to ≤ 0.5 (load-bearing invariant; guards anyone who
//      tries to delete the cap);
//   2. measures + prints the per-skill spurious-fire table the B1.2 grounding
//      work consumes: a domain compound with 0 spurious raw≥0.7 fires across
//      the panel is a candidate for grounded un-capping.

#include <gtest/gtest.h>

#include "re/Recognizer.hpp"
#include "io/StepIO.hpp"
#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/bore_cylindrical.hpp"
#include "skills/bore_with_shelf.hpp"
#include "skills/chamfer_edge.hpp"
#include "skills/counterbore.hpp"
#include "skills/countersink.hpp"
#include "skills/drill_hole.hpp"
#include "skills/fillet_edge.hpp"
#include "skills/mill_circular_pocket.hpp"
#include "skills/mill_rect_pocket.hpp"
#include "skills/mill_slot.hpp"

#include <TopoDS_Shape.hxx>
#include <gp_Dir.hxx>

#include <cstdio>
#include <filesystem>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace koocadcam;

namespace {

// A grammar-derived compound (e.g. bolt_circle_pattern) is NOT one of the ~750
// loose domain recognizers the cap governs: re::analyze emits it AFTER the cap,
// grounded in measured precise atoms, so it is deliberately uncapped.  The
// cap-invariant and the un-cap table both ignore it (source begins "grammar:").
bool isGrammarCompound(const skill::RecognizedFeature& c)
{
    return c.matched_geometry.is_object() &&
           c.matched_geometry.value("source", std::string())
               .rfind("grammar:", 0) == 0;
}

// The precise tier — recognizers the cap already exempts.  A candidate from
// any OTHER skill_id is a domain-compound (capped) recognizer.
const std::set<std::string>& preciseTier()
{
    static const std::set<std::string> k = {
        "drill_hole", "drill_through_hole", "counterbore", "countersink",
        "chamfer_edge", "fillet_edge", "bore_cylindrical", "bore_with_shelf",
        "ream", "spot_drill", "spot_face", "mill_circular_pocket",
        "mill_rect_pocket", "mill_slot", "bolt_hole_metric_spec",
        // box_pocket is precise-tier (a sharp-corner rect recess): it legitimately
        // fires on a filleted rect_pocket panel part (a rect pocket IS a rect
        // pocket), so it is exempt from the domain-compound cap invariant here.
        "box_pocket",
        "crown_knurl",   // precise-tier radial notch ring (anchored on a cone knob)
    };
    return k;
}

skill::Workpiece stepRoundTrip(const TopoDS_Shape& s)
{
    namespace fs = std::filesystem;
    const fs::path p = fs::temp_directory_path() /
                       ("koo_fpscan_" + std::to_string(::rand()) + ".step");
    std::string err;
    io::StepIO::write(s, p, err);
    auto reim = io::StepIO::read(p, err);
    std::error_code ec; fs::remove(p, ec);
    return skill::Workpiece(*reim);
}

struct Part { std::string label; TopoDS_Shape shape; };

std::vector<Part> buildPanel()
{
    std::vector<Part> panel;
    auto block = [] { return skill::createCuboidStock(60.0, 60.0, 20.0); };

    panel.push_back({ "bare_stock", block()->shape() });

    {   skill::drill_hole::Input in;
        in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm = 30; in.position_y_mm = 30;
        in.axis_dir = gp_Dir(0, 0, -1); in.diameter_mm = 8; in.through_hole = true;
        panel.push_back({ "drill", skill::drill_hole::apply(*block(), in).workpiece->shape() }); }

    {   skill::counterbore::Input in;
        in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm = 30; in.position_y_mm = 30; in.axis_dir = gp_Dir(0, 0, -1);
        in.pilot_dia_mm = 6; in.pilot_depth_mm = 12; in.seat_dia_mm = 12; in.seat_depth_mm = 4;
        panel.push_back({ "counterbore", skill::counterbore::apply(*block(), in).workpiece->shape() }); }

    {   skill::mill_circular_pocket::Input in;
        in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm = 30; in.position_y_mm = 30; in.axis_dir = gp_Dir(0, 0, -1);
        in.diameter_mm = 18; in.depth_mm = 4;
        panel.push_back({ "circ_pocket", skill::mill_circular_pocket::apply(*block(), in).workpiece->shape() }); }

    {   skill::mill_rect_pocket::Input in;
        in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.center_x_mm = 30; in.center_y_mm = 30; in.axis_dir = gp_Dir(0, 0, -1);
        in.length_mm = 20; in.width_mm = 12; in.depth_mm = 4; in.corner_r_mm = 1;
        panel.push_back({ "rect_pocket", skill::mill_rect_pocket::apply(*block(), in).workpiece->shape() }); }

    {   skill::drill_hole::Input dh;
        dh.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        dh.position_x_mm = 30; dh.position_y_mm = 30;
        dh.axis_dir = gp_Dir(0, 0, -1); dh.diameter_mm = 8; dh.through_hole = true;
        auto drilled = skill::drill_hole::apply(*block(), dh).workpiece;
        skill::chamfer_edge::Input ch;
        ch.edge_selector = skill::fillet_edge::EdgesAtZBand{ 20.0, 1e-3 };
        ch.chamfer_size_mm = 1.0; ch.angle_deg = 45.0;
        panel.push_back({ "drill+chamfer", skill::chamfer_edge::apply(*drilled, ch).workpiece->shape() }); }

    {   skill::countersink::Input in;
        in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm = 30; in.position_y_mm = 30; in.axis_dir = gp_Dir(0, 0, -1);
        in.pilot_dia_mm = 5; in.pilot_depth_mm = 10; in.cone_top_dia_mm = 10; in.cone_angle_deg = 90;
        panel.push_back({ "countersink", skill::countersink::apply(*block(), in).workpiece->shape() }); }

    {   skill::bore_cylindrical::Input in;
        in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm = 30; in.position_y_mm = 30; in.axis_dir = gp_Dir(0, 0, -1);
        in.diameter_mm = 16; in.depth_mm = 10;
        panel.push_back({ "bore", skill::bore_cylindrical::apply(*block(), in).workpiece->shape() }); }

    {   skill::bore_with_shelf::Input in;
        in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm = 30; in.position_y_mm = 30; in.axis_dir = gp_Dir(0, 0, -1);
        in.upper_dia_mm = 8; in.upper_depth_mm = 5; in.lower_dia_mm = 16; in.lower_depth_mm = 8;
        panel.push_back({ "stepped_bore", skill::bore_with_shelf::apply(*block(), in).workpiece->shape() }); }

    {   skill::mill_slot::Input in;
        in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.start_x_mm = 20; in.start_y_mm = 30; in.end_x_mm = 40; in.end_y_mm = 30;
        in.axis_dir = gp_Dir(0, 0, -1); in.width_mm = 6; in.depth_mm = 4;
        panel.push_back({ "slot", skill::mill_slot::apply(*block(), in).workpiece->shape() }); }

    // Multi-feature: a real recovered part has several features at once.
    {   skill::counterbore::Input cb;
        cb.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        cb.position_x_mm = 15; cb.position_y_mm = 15; cb.axis_dir = gp_Dir(0, 0, -1);
        cb.pilot_dia_mm = 6; cb.pilot_depth_mm = 12; cb.seat_dia_mm = 12; cb.seat_depth_mm = 4;
        auto w = skill::counterbore::apply(*block(), cb).workpiece;
        skill::drill_hole::Input dh;
        dh.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        dh.position_x_mm = 45; dh.position_y_mm = 45;
        dh.axis_dir = gp_Dir(0, 0, -1); dh.diameter_mm = 5; dh.through_hole = true;
        w = skill::drill_hole::apply(*w, dh).workpiece;
        skill::mill_circular_pocket::Input mp;
        mp.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        mp.position_x_mm = 45; mp.position_y_mm = 15; mp.axis_dir = gp_Dir(0, 0, -1);
        mp.diameter_mm = 14; mp.depth_mm = 4;
        w = skill::mill_circular_pocket::apply(*w, mp).workpiece;
        panel.push_back({ "multi(cbore+drill+pocket)", w->shape() }); }

    // Different bare-stock proportions (a thin plate, a tall bar) trip
    // aspect-ratio / planar-step recognizers that the 60x60x20 block does not.
    panel.push_back({ "thin_plate", skill::createCuboidStock(80.0, 80.0, 4.0)->shape() });
    panel.push_back({ "tall_bar",  skill::createCuboidStock(20.0, 20.0, 80.0)->shape() });

    return panel;
}

}  // namespace

// ── 1. The cap is load-bearing: it demotes every domain-compound candidate ─
TEST(FpScan, CapDemotesAllDomainCompoundCandidates)
{
    bool anyRawSpurious = false;
    for (const auto& part : buildPanel()) {
        skill::Workpiece wp = stepRoundTrip(part.shape);
        const auto capped = re::analyze(wp, /*applyCap=*/true);
        for (const auto& c : capped) {
            if (preciseTier().count(c.skill_id)) continue;
            if (isGrammarCompound(c)) continue;         // grounded grammar compound, uncapped by design
            const bool replay =
                c.matched_geometry.is_object() &&
                c.matched_geometry.value("source", std::string()) == "metadata_replay";
            if (replay) continue;                       // metadata replay is exempt by design
            EXPECT_LE(c.confidence, 0.5)
                << "cap leaked a domain-compound candidate above 0.5: "
                << c.skill_id << " on " << part.label;
        }
        const auto raw = re::analyze(wp, /*applyCap=*/false);
        for (const auto& c : raw)
            if (!preciseTier().count(c.skill_id) && !isGrammarCompound(c) &&
                c.confidence >= 0.7)
                anyRawSpurious = true;
    }
    // The cap exists BECAUSE domain compounds fire at raw ≥ 0.7 on parts that
    // don't contain them; if this ever stops being true the cap is dead weight.
    EXPECT_TRUE(anyRawSpurious)
        << "no domain compound fired raw>=0.7 spuriously — the cap may be "
           "removable, or the panel is too thin to measure";
}

// ── 2. Measure + print the un-cap safety table for B1.2 grounding ─────────
//
// Two columns per domain compound:
//   raw      — fired at raw >= 0.7 on N parts (none contain its feature).
//   survives — of those, how many SURVIVE re::dedupe when run on the
//              un-capped candidate set: i.e. the compound is NOT subsumed by a
//              precise atom (drill/bore/pocket) claiming the same geometry.
// A compound with survives=0 is ALREADY safe to un-cap (dedupe's specificity
// rule, B1.5, drops it whenever a real atom explains the geometry).  survives>0
// is the genuine un-cap risk B1.2 must ground.
TEST(FpScan, ReportUncapSafetyTable)
{
    const auto panel = buildPanel();
    std::map<std::string, std::set<std::string>> rawFires;
    std::map<std::string, std::set<std::string>> survives;

    for (const auto& part : panel) {
        skill::Workpiece wp = stepRoundTrip(part.shape);
        const auto raw     = re::analyze(wp, /*applyCap=*/false);
        const auto deduped = re::dedupe(raw);   // un-capped → real specificity contest
        for (const auto& c : raw)
            if (!preciseTier().count(c.skill_id) && !isGrammarCompound(c) &&
                c.confidence >= 0.7)
                rawFires[c.skill_id].insert(part.label);
        for (const auto& c : deduped)
            if (!preciseTier().count(c.skill_id) && !isGrammarCompound(c) &&
                c.confidence >= 0.7)
                survives[c.skill_id].insert(part.label);
    }

    std::printf("\n[fpscan] domain-compound un-cap safety across a %zu-part "
                "foreign panel (none contain these features):\n", panel.size());
    std::printf("[fpscan]   %-40s  raw  survives-dedupe\n", "skill_id");
    std::size_t safeViaDedupe = 0, needGrounding = 0;
    for (const auto& [skill, parts] : rawFires) {
        const std::size_t surv = survives.count(skill) ? survives.at(skill).size() : 0;
        std::printf("[fpscan]   %-40s  %zu/%zu     %zu/%zu  %s\n",
                    skill.c_str(), parts.size(), panel.size(),
                    surv, panel.size(),
                    surv == 0 ? "<- subsumed (un-cap safe)" : "<- SURVIVES (needs B1.2)");
        if (surv == 0) ++safeViaDedupe; else ++needGrounding;
    }
    std::printf("[fpscan] %zu compounds are already subsumed by dedupe "
                "(un-cap safe); %zu survive and need B1.2 atomic grounding.\n",
                safeViaDedupe, needGrounding);
    SUCCEED();
}
