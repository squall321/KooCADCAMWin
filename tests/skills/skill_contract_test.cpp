// @lat: [[process/test-strategy#skill-contract]]
//
// Skill feature-chain CONTRACT regression test (breakthrough plan B2.3).
//
// The Workpiece contract (Workpiece.hpp) requires every skill's apply() to
// return a workpiece whose features() is the FULL build history: all prior
// signatures, in order, plus exactly one new signature appended.  ~332 of
// 777 skills silently violated this until the breakthrough-plan audit; the
// Executor masks it for plan-executed parts (setFeatures), so only a direct
// apply-chain test like this one can catch the drift.
//
// Phase 1 covers the 10 highest-value skills (the precise recognizer tier).
// Each skill is applied ON TOP of a drill_hole so the prior chain is
// non-empty — the exact scenario the violators broke.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/bore_cylindrical.hpp"
#include "skills/chamfer_edge.hpp"
#include "skills/counterbore.hpp"
#include "skills/countersink.hpp"
#include "skills/drill_hole.hpp"
#include "skills/fillet_edge.hpp"
#include "skills/hollow_cavity.hpp"
#include "skills/mill_circular_pocket.hpp"
#include "skills/mill_rect_pocket.hpp"
#include "skills/mill_slot.hpp"

#include <gp_Dir.hxx>

#include <functional>
#include <memory>
#include <string>
#include <vector>

using namespace koocadcam;

namespace {

// 80x80x15 stock with one drill_hole at a corner region → prior chain of 1.
// Every contract case applies its skill far from (15,15) so geometry never
// interferes with the pre-existing hole.
std::shared_ptr<skill::Workpiece> stockWithOnePriorFeature()
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 15.0);
    skill::drill_hole::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 15.0;
    in.position_y_mm = 15.0;
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.diameter_mm   = 3.0;
    in.depth_mm      = 6.0;
    auto out = skill::drill_hole::apply(*stock, in);
    return out.workpiece;
}

struct ContractCase {
    const char* skill_id;
    std::function<skill::SkillOutput(const skill::Workpiece&)> apply;
};

const std::vector<ContractCase>& contractCases()
{
    static const std::vector<ContractCase> kCases = {
        { "drill_hole", [](const skill::Workpiece& wp) {
            skill::drill_hole::Input in;
            in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
            in.position_x_mm = 60.0; in.position_y_mm = 60.0;
            in.axis_dir      = gp_Dir(0, 0, -1);
            in.diameter_mm   = 4.0; in.depth_mm = 6.0;
            return skill::drill_hole::apply(wp, in);
        }},
        { "counterbore", [](const skill::Workpiece& wp) {
            skill::counterbore::Input in;
            in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
            in.position_x_mm = 60.0; in.position_y_mm = 20.0;
            in.axis_dir      = gp_Dir(0, 0, -1);
            in.pilot_dia_mm  = 4.0; in.pilot_depth_mm = 8.0;
            in.seat_dia_mm   = 8.0; in.seat_depth_mm  = 3.0;
            return skill::counterbore::apply(wp, in);
        }},
        { "countersink", [](const skill::Workpiece& wp) {
            skill::countersink::Input in;
            in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
            in.position_x_mm   = 20.0; in.position_y_mm = 60.0;
            in.axis_dir        = gp_Dir(0, 0, -1);
            in.pilot_dia_mm    = 4.0; in.pilot_depth_mm = 8.0;
            in.cone_top_dia_mm = 8.0; in.cone_angle_deg = 90.0;
            return skill::countersink::apply(wp, in);
        }},
        { "bore_cylindrical", [](const skill::Workpiece& wp) {
            skill::bore_cylindrical::Input in;
            in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
            in.position_x_mm = 40.0; in.position_y_mm = 60.0;
            in.axis_dir      = gp_Dir(0, 0, -1);
            in.diameter_mm   = 10.0; in.depth_mm = 5.0;
            return skill::bore_cylindrical::apply(wp, in);
        }},
        { "mill_circular_pocket", [](const skill::Workpiece& wp) {
            skill::mill_circular_pocket::Input in;
            in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
            in.position_x_mm = 60.0; in.position_y_mm = 40.0;
            in.axis_dir      = gp_Dir(0, 0, -1);
            in.diameter_mm   = 12.0; in.depth_mm = 3.0;
            return skill::mill_circular_pocket::apply(wp, in);
        }},
        { "mill_rect_pocket", [](const skill::Workpiece& wp) {
            skill::mill_rect_pocket::Input in;
            in.entry_face  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
            in.center_x_mm = 40.0; in.center_y_mm = 20.0;
            in.axis_dir    = gp_Dir(0, 0, -1);
            in.length_mm   = 12.0; in.width_mm = 8.0;
            in.depth_mm    = 3.0;  in.corner_r_mm = 1.0;
            return skill::mill_rect_pocket::apply(wp, in);
        }},
        { "mill_slot", [](const skill::Workpiece& wp) {
            skill::mill_slot::Input in;
            in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
            in.start_x_mm = 30.0; in.start_y_mm = 40.0;
            in.end_x_mm   = 50.0; in.end_y_mm   = 40.0;
            in.axis_dir   = gp_Dir(0, 0, -1);
            in.width_mm   = 4.0;  in.depth_mm = 3.0;
            return skill::mill_slot::apply(wp, in);
        }},
        { "chamfer_edge", [](const skill::Workpiece& wp) {
            skill::chamfer_edge::Input in;
            in.edge_selector   = skill::fillet_edge::EdgesAtZBand{ 0.0, 1e-3 };
            in.chamfer_size_mm = 0.8;
            in.angle_deg       = 45.0;
            return skill::chamfer_edge::apply(wp, in);
        }},
        { "fillet_edge", [](const skill::Workpiece& wp) {
            skill::fillet_edge::Input in;
            in.edge_selector = skill::fillet_edge::EdgesAtZBand{ 0.0, 1e-3 };
            in.radius_mm     = 0.8;
            return skill::fillet_edge::apply(wp, in);
        }},
        { "hollow_cavity", [](const skill::Workpiece& wp) {
            skill::hollow_cavity::Input in;
            in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
            in.wall_thickness_mm = 30.0;   // 20x20 cavity centered, clears the hole
            in.depth_mm          = 4.0;
            return skill::hollow_cavity::apply(wp, in);
        }},
    };
    return kCases;
}

}  // namespace

class SkillContract : public ::testing::TestWithParam<std::size_t> {};

// apply() on a workpiece with prior history must return prior + 1 features,
// prior entries preserved verbatim and in order, new signature appended last.
TEST_P(SkillContract, ApplyPreservesFeatureChain)
{
    const auto& c = contractCases()[GetParam()];

    auto prior = stockWithOnePriorFeature();
    ASSERT_EQ(prior->features().size(), 1u) << "fixture precondition";
    const auto priorSig = prior->features().front();

    auto out = c.apply(*prior);
    ASSERT_TRUE(out.workpiece) << c.skill_id << ": apply returned no workpiece";

    const auto& feats = out.workpiece->features();
    ASSERT_EQ(feats.size(), 2u)
        << c.skill_id << " dropped the prior feature chain (contract: "
        << "features() must be FULL history = prior + 1)";
    EXPECT_EQ(feats[0].skill_id, priorSig.skill_id)
        << c.skill_id << " did not preserve prior features in order";
    EXPECT_EQ(feats[1].skill_id, c.skill_id)
        << c.skill_id << " must append its own signature last";
}

INSTANTIATE_TEST_SUITE_P(
    PreciseTier, SkillContract,
    ::testing::Range<std::size_t>(0, contractCases().size()),
    [](const ::testing::TestParamInfo<std::size_t>& info) {
        return std::string(contractCases()[info.index].skill_id);
    });
