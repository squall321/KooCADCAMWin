// @lat: [[process/test-strategy#skill round-trip]]
//
// thread_chase skill tests (slice — metadata-only repair variant).
//   1. DFM-CHASE-PARENT rejects without prior thread.
//   2. With a parent thread, apply is a geometric NO-OP.
//   3. DFM-CHASE-PASSES rejects passes outside [1, 5].
//   4. Recognize replays signature at confidence 1.0.
//   5. Pattern carries the parent thread's parameters.
//   6. Multiple chases stack in feature history.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/thread_chase.hpp"
#include "skills/thread_mill_external.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

// Build a workpiece with an existing external thread.
std::shared_ptr<skill::Workpiece> makeThreadedStock()
{
    auto stock = skill::createCylindricalStock(8.0, 25.0);

    skill::thread_mill_external::Input mill;
    mill.cylinder_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    mill.thread_size      = "M6";
    mill.pitch_mm         = 1.0;
    mill.thread_length_mm = 8.0;
    mill.tool_dia_mm      = 1.5;
    auto threaded = skill::thread_mill_external::apply(*stock, mill);
    return threaded.workpiece;
}
}  // namespace

// ── 1. DFM-CHASE-PARENT rejects without prior thread ─────────────────────
TEST(SkillThreadChase, ValidateRejectsMissingParent)
{
    auto stock = skill::createCylindricalStock(8.0, 25.0);

    skill::thread_chase::Input in;
    in.existing_thread_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    in.chase_depth_passes = 1;

    auto r = skill::thread_chase::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CHASE-PARENT") found = true;
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::thread_chase::apply(*stock, in), skill::SkillError);
}

// ── 2. Apply is a geometric NO-OP ────────────────────────────────────────
TEST(SkillThreadChase, ApplyIsNoOpOnGeometry)
{
    auto threaded = makeThreadedStock();
    const double volBefore = volumeOf(threaded->shape());
    const int facesBefore  = threaded->faceCount();

    skill::thread_chase::Input in;
    in.existing_thread_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    in.chase_depth_passes = 2;

    auto out = skill::thread_chase::apply(*threaded, in);

    // Volume and face count are exactly unchanged — NO-OP.
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), facesBefore);

    // Feature history accumulates: 1 (parent) + 1 (chase) = 2.
    EXPECT_EQ(out.workpiece->features().size(), 2u);
    EXPECT_EQ(out.signature.skill_id, std::string("thread_chase"));
    EXPECT_EQ(out.signature.pattern.at("geometry").get<std::string>(),
              std::string("no_op_metadata_only"));
    EXPECT_EQ(out.signature.tooling.stock_removed_mm3, 0.0);
}

// ── 3. DFM-CHASE-PASSES rejects out-of-range passes ──────────────────────
TEST(SkillThreadChase, ValidateRejectsOutOfRangePasses)
{
    auto threaded = makeThreadedStock();

    skill::thread_chase::Input in;
    in.existing_thread_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };

    // 0 passes
    in.chase_depth_passes = 0;
    {
        auto r = skill::thread_chase::validate(*threaded, in);
        EXPECT_FALSE(r.passed);
    }
    // 6 passes
    in.chase_depth_passes = 6;
    {
        auto r = skill::thread_chase::validate(*threaded, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-CHASE-PASSES") found = true;
        EXPECT_TRUE(found);
    }
    // 3 passes — accepted
    in.chase_depth_passes = 3;
    {
        auto r = skill::thread_chase::validate(*threaded, in);
        EXPECT_TRUE(r.passed);
    }
}

// ── 4. Recognize replays signature at confidence 1.0 ─────────────────────
TEST(SkillThreadChase, RecognizeReplaysSignature)
{
    auto threaded = makeThreadedStock();

    skill::thread_chase::Input in;
    in.existing_thread_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    in.chase_depth_passes = 2;

    auto out   = skill::thread_chase::apply(*threaded, in);
    auto cands = skill::thread_chase::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("thread_chase"));
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].matched_geometry["source"], "metadata");
    EXPECT_EQ(cands[0].matched_geometry["geometric_change"].get<bool>(), false);
    EXPECT_EQ(cands[0].recovered_params["chase_depth_passes"].get<int>(), 2);
}

// ── 5. Pattern carries parent thread parameters ──────────────────────────
TEST(SkillThreadChase, PatternCarriesParentParams)
{
    auto threaded = makeThreadedStock();

    skill::thread_chase::Input in;
    in.existing_thread_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    in.chase_depth_passes = 1;

    auto out = skill::thread_chase::apply(*threaded, in);

    EXPECT_EQ(out.signature.pattern.at("parent_skill").get<std::string>(),
              std::string("thread_mill_external"));
    EXPECT_EQ(out.signature.pattern.at("parent_thread_size").get<std::string>(),
              std::string("M6"));
    EXPECT_NEAR(out.signature.pattern.at("parent_pitch_mm").get<double>(), 1.0, 1e-6);
    EXPECT_GE(out.signature.pattern.at("target_thread_id").get<int>(), 0);
}

// ── 6. Multiple chases stack in feature history ──────────────────────────
TEST(SkillThreadChase, MultipleChasesStack)
{
    auto threaded = makeThreadedStock();

    skill::thread_chase::Input in;
    in.existing_thread_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    in.chase_depth_passes = 1;

    auto out1 = skill::thread_chase::apply(*threaded, in);
    auto out2 = skill::thread_chase::apply(*out1.workpiece, in);

    // 1 parent + 2 chases = 3 features.
    EXPECT_EQ(out2.workpiece->features().size(), 3u);
    auto cands = skill::thread_chase::recognize(*out2.workpiece);
    EXPECT_EQ(cands.size(), 2u);
}
