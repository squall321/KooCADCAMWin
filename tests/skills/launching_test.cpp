// @lat: [[process/test-strategy#skill round-trip]]
//
// launching — vessel launch event (side / end / floating).  Covers:
//   1. apply preserves geometry verbatim
//   2. validate rejects bad method (not in {side,end,floating})
//   3. signature records kind + launch_method + vessel_mass_t
//   4. recognize replays metadata
//   5. side launch of huge vessel (> 5000 t) emits info but does NOT block

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/launching.hpp"

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

// ─── 1. Apply clones geometry unchanged ──────────────────────────────────
TEST(SkillLaunching, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "marine_steel");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::launching::Input in;
    in.launch_method = "floating";
    in.vessel_mass_t = 80000.0;

    auto out = skill::launching::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. validate rejects bad input ───────────────────────────────────────
TEST(SkillLaunching, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "marine_steel");

    skill::launching::Input in;
    in.launch_method = "catapult";    // not in {side,end,floating}
    in.vessel_mass_t = 1000.0;

    auto r = skill::launching::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-LAUNCH-METHOD"));
    EXPECT_THROW(skill::launching::apply(*stock, in), skill::SkillError);

    // Non-positive vessel mass also blocks.
    in.launch_method = "end";
    in.vessel_mass_t = 0.0;
    auto r2 = skill::launching::validate(*stock, in);
    EXPECT_FALSE(r2.passed);
}

// ─── 3. signature records kind + key params ──────────────────────────────
TEST(SkillLaunching, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "marine_steel");

    skill::launching::Input in;
    in.launch_method = "end";
    in.vessel_mass_t = 2500.0;

    auto out = skill::launching::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("launching"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("launching"));
    EXPECT_EQ(out.signature.pattern["launch_method"].get<std::string>(),
              std::string("end"));
    EXPECT_NEAR(out.signature.pattern["vessel_mass_t"].get<double>(),
                2500.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("slipway"));
}

// ─── 4. recognize replays metadata ───────────────────────────────────────
TEST(SkillLaunching, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "marine_steel");

    skill::launching::Input in;
    in.launch_method = "floating";
    in.vessel_mass_t = 95000.0;
    auto out = skill::launching::apply(*stock, in);

    auto cands = skill::launching::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["launch_method"].get<std::string>(),
              std::string("floating"));

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::launching::recognize(raw).empty());
}

// ─── 5. Side launch + huge vessel emits info but does NOT block ──────────
TEST(SkillLaunching, HugeSideLaunchEmitsInfoNotError)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "marine_steel");

    skill::launching::Input in;
    in.launch_method = "side";
    in.vessel_mass_t = 6000.0;     // > 5000 t side launch impractical

    auto r = skill::launching::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "huge side launch should emit info, not block";
    EXPECT_TRUE(hasFinding(r, "DFM-LAUNCH-SIDE"));
    EXPECT_NO_THROW(skill::launching::apply(*stock, in));
}
