// @lat: [[process/test-strategy#skill round-trip]]
//
// tear_down_inspect skill — 5-case sweep:
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKind+params
//   4. RecognizeMetadataReplay
//   5. Specific: long teardown time emits info-level finding but does not block

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/tear_down_inspect.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

bool hasFinding(const skill::DFMReport& r, const std::string& code)
{
    for (const auto& f : r.findings) if (f.code == code) return true;
    return false;
}

}  // namespace

// ─── 1. Apply: geometry COMPLETELY unchanged ─────────────────────────────
TEST(SkillTearDownInspect, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::tear_down_inspect::Input in;
    in.steps_taken  = 5;
    in.time_min     = 30.0;
    in.parts_marked = 1;

    auto out = skill::tear_down_inspect::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("tear_down_inspect"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillTearDownInspect, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::tear_down_inspect::Input badSteps;
    badSteps.steps_taken  = 0;
    badSteps.time_min     = 10.0;
    badSteps.parts_marked = 0;
    EXPECT_FALSE(skill::tear_down_inspect::validate(*stock, badSteps).passed);
    EXPECT_THROW(skill::tear_down_inspect::apply(*stock, badSteps),
                 skill::SkillError);

    skill::tear_down_inspect::Input badTime;
    badTime.steps_taken  = 3;
    badTime.time_min     = 0.0;
    badTime.parts_marked = 0;
    EXPECT_FALSE(skill::tear_down_inspect::validate(*stock, badTime).passed);
    EXPECT_THROW(skill::tear_down_inspect::apply(*stock, badTime),
                 skill::SkillError);

    skill::tear_down_inspect::Input badMarked;
    badMarked.steps_taken  = 3;
    badMarked.time_min     = 10.0;
    badMarked.parts_marked = -1;
    EXPECT_FALSE(skill::tear_down_inspect::validate(*stock, badMarked).passed);
    EXPECT_THROW(skill::tear_down_inspect::apply(*stock, badMarked),
                 skill::SkillError);
}

// ─── 3. Signature records kind + params ──────────────────────────────────
TEST(SkillTearDownInspect, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::tear_down_inspect::Input in;
    in.steps_taken  = 7;
    in.time_min     = 45.0;
    in.parts_marked = 2;

    auto out = skill::tear_down_inspect::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("tear_down_inspect"));
    EXPECT_EQ(out.signature.pattern["steps_taken"].get<int>(), 7);
    EXPECT_NEAR(out.signature.pattern["time_min"].get<double>(), 45.0, 1e-6);
    EXPECT_EQ(out.signature.pattern["parts_marked"].get<int>(), 2);
    EXPECT_FALSE(out.signature.pattern["inspection_pass"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.params["steps_taken"].get<int>(), 7);
    EXPECT_NEAR(out.signature.params["time_min"].get<double>(), 45.0, 1e-6);
    EXPECT_EQ(out.signature.params["parts_marked"].get<int>(), 2);
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillTearDownInspect, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::tear_down_inspect::Input in;
    in.steps_taken  = 4;
    in.time_min     = 20.0;
    in.parts_marked = 0;
    auto out = skill::tear_down_inspect::apply(*stock, in);

    auto cands = skill::tear_down_inspect::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["steps_taken"].get<int>(), 4);

    // Raw workpiece (no metadata) → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::tear_down_inspect::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: long teardown emits info-finding but proceeds ──────────
TEST(SkillTearDownInspect, LongTeardownIsInfoNotError)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::tear_down_inspect::Input in;
    in.steps_taken  = 12;
    in.time_min     = 600.0;       // 10 h — past 480-min advisory
    in.parts_marked = 0;

    auto r = skill::tear_down_inspect::validate(*stock, in);
    EXPECT_TRUE(r.passed) << "long teardown is info-only, should not block";
    EXPECT_TRUE(hasFinding(r, "DFM-TDI-LONG"));
    EXPECT_NO_THROW(skill::tear_down_inspect::apply(*stock, in));
}
