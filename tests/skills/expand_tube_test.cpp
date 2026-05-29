// @lat: [[process/test-strategy#skill round-trip]]
//
// expand_tube — 5-case sweep for hydraulic / mechanical tube expansion.
// Metadata-only.  Verifies DFM range clamps (2 – 15 %), method selection
// influences tooling metadata, and pattern records target_expansion_pct.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/expand_tube.hpp"

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

}  // namespace

// ─── 1. Apply clones geometry unchanged ───────────────────────────────────
TEST(SkillExpandTube, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCylindricalStock(19.05, 200.0, "carbon_steel_1045");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::expand_tube::Input in;
    in.target_expansion_pct = 8.0;
    in.method               = "hydraulic";

    auto out = skill::expand_tube::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillExpandTube, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(19.05, 200.0);

    skill::expand_tube::Input zero;
    zero.target_expansion_pct = 0.0;       // invalid (non-positive)
    zero.method               = "hydraulic";
    auto rz = skill::expand_tube::validate(*stock, zero);
    EXPECT_FALSE(rz.passed);
    EXPECT_THROW(skill::expand_tube::apply(*stock, zero), skill::SkillError);

    skill::expand_tube::Input over;
    over.target_expansion_pct = 25.0;      // > 15 %
    over.method               = "mechanical";
    auto ro = skill::expand_tube::validate(*stock, over);
    EXPECT_FALSE(ro.passed);
    bool found = false;
    for (const auto& f : ro.findings)
        if (f.code == "DFM-EXPAND-RANGE") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::expand_tube::apply(*stock, over), skill::SkillError);
}

// ─── 3. Signature records kind + params ───────────────────────────────────
TEST(SkillExpandTube, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCylindricalStock(19.05, 200.0);

    skill::expand_tube::Input in;
    in.target_expansion_pct = 10.0;
    in.method               = "mechanical";

    auto out = skill::expand_tube::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("expand_tube"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("expand_tube"));
    EXPECT_NEAR(out.signature.pattern["target_expansion_pct"].get<double>(),
                10.0, 1e-6);
    EXPECT_EQ(out.signature.pattern["method"].get<std::string>(),
              std::string("mechanical"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillExpandTube, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(19.05, 200.0);

    skill::expand_tube::Input in;
    in.target_expansion_pct = 7.5;
    in.method               = "hydraulic";

    auto out = skill::expand_tube::apply(*stock, in);
    auto cands = skill::expand_tube::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["target_expansion_pct"].get<double>(),
                7.5, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["method"].get<std::string>(),
              std::string("hydraulic"));

    skill::Workpiece raw(out.workpiece->shape());
    auto candsEmpty = skill::expand_tube::recognize(raw);
    EXPECT_TRUE(candsEmpty.empty());
}

// ─── 5. Specific: method selects distinct tooling metadata ────────────────
// Hydraulic and mechanical expansion use different physical tools and run
// at different cycle times.  Verify the apply() pipeline propagates the
// method choice into tool_type AND est_cycle_time_s.
TEST(SkillExpandTube, MethodSelectsDistinctTooling)
{
    auto stock = skill::createCylindricalStock(19.05, 200.0);

    skill::expand_tube::Input hyd;
    hyd.target_expansion_pct = 8.0;
    hyd.method               = "hydraulic";

    skill::expand_tube::Input mech = hyd;
    mech.method = "mechanical";

    auto outH = skill::expand_tube::apply(*stock, hyd);
    auto outM = skill::expand_tube::apply(*stock, mech);

    EXPECT_NE(outH.signature.tooling.tool_type, outM.signature.tooling.tool_type);
    EXPECT_LT(outH.signature.tooling.est_cycle_time_s,
              outM.signature.tooling.est_cycle_time_s)
        << "hydraulic should be faster than mechanical roller";
}
