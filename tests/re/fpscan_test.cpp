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
#include "skills/chamfer_edge.hpp"
#include "skills/counterbore.hpp"
#include "skills/drill_hole.hpp"
#include "skills/fillet_edge.hpp"
#include "skills/mill_circular_pocket.hpp"
#include "skills/mill_rect_pocket.hpp"

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

// The precise tier — recognizers the cap already exempts.  A candidate from
// any OTHER skill_id is a domain-compound (capped) recognizer.
const std::set<std::string>& preciseTier()
{
    static const std::set<std::string> k = {
        "drill_hole", "drill_through_hole", "counterbore", "countersink",
        "chamfer_edge", "fillet_edge", "bore_cylindrical", "bore_with_shelf",
        "ream", "spot_drill", "spot_face", "mill_circular_pocket",
        "mill_rect_pocket", "mill_slot", "bolt_hole_metric_spec",
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
            if (!preciseTier().count(c.skill_id) && c.confidence >= 0.7)
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
            if (!preciseTier().count(c.skill_id) && c.confidence >= 0.7)
                rawFires[c.skill_id].insert(part.label);
        for (const auto& c : deduped)
            if (!preciseTier().count(c.skill_id) && c.confidence >= 0.7)
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
