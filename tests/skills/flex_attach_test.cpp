// @lat: [[process/test-strategy#skill round-trip]]
//
// flex_attach — flex-cable attach (ZIF / ACF) skill, 5-case sweep.
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput  (low retention force + unknown method)
//   3. SignatureRecordsKindAndKeyParams
//   4. RecognizeMetadataReplay
//   5. AcfToolingDiffersFromZif   (specific assertion)

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/flex_attach.hpp"

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

// ─── 1. Apply: geometry is COMPLETELY unchanged ──────────────────────────
TEST(SkillFlexAttach, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 1.6, "FR4");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::flex_attach::Input in;
    in.attach_method     = "ZIF";
    in.contact_count     = 30;
    in.retention_force_n = 15.0;

    auto out = skill::flex_attach::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);

    EXPECT_EQ(out.signature.skill_id, std::string("flex_attach"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. DFM: invalid inputs are rejected ─────────────────────────────────
TEST(SkillFlexAttach, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 1.6, "FR4");

    skill::flex_attach::Input weak;
    weak.attach_method     = "ZIF";
    weak.contact_count     = 30;
    weak.retention_force_n = 0.3;         // < 1.0 N
    auto r1 = skill::flex_attach::validate(*stock, weak);
    EXPECT_FALSE(r1.passed);
    EXPECT_TRUE(hasFinding(r1, "DFM-FLEX-FORCE-LOW"));
    EXPECT_THROW(skill::flex_attach::apply(*stock, weak), skill::SkillError);

    skill::flex_attach::Input unknown;
    unknown.attach_method     = "spring_clip";   // not in catalog
    unknown.contact_count     = 20;
    unknown.retention_force_n = 10.0;
    auto r2 = skill::flex_attach::validate(*stock, unknown);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-FLEX-METHOD"));

    skill::flex_attach::Input zero;
    zero.attach_method     = "ZIF";
    zero.contact_count     = 0;                  // invalid
    zero.retention_force_n = 10.0;
    auto r3 = skill::flex_attach::validate(*stock, zero);
    EXPECT_FALSE(r3.passed);
}

// ─── 3. Signature carries kind + key parameters ──────────────────────────
TEST(SkillFlexAttach, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 1.6, "FR4");

    skill::flex_attach::Input in;
    in.attach_method     = "ACF";
    in.contact_count     = 60;
    in.retention_force_n = 25.0;

    auto out = skill::flex_attach::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("flex_attach"));
    EXPECT_EQ(out.signature.pattern["attach_method"].get<std::string>(),
              std::string("ACF"));
    EXPECT_EQ(out.signature.pattern["contact_count"].get<int>(), 60);
    EXPECT_NEAR(out.signature.pattern["retention_force_n"].get<double>(),
                25.0, 1e-6);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillFlexAttach, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 1.6, "FR4");

    skill::flex_attach::Input in;
    in.attach_method     = "ZIF";
    in.contact_count     = 24;
    in.retention_force_n = 12.0;

    auto out = skill::flex_attach::apply(*stock, in);

    auto cands = skill::flex_attach::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["attach_method"].get<std::string>(),
              std::string("ZIF"));
    EXPECT_EQ(cands[0].recovered_params["contact_count"].get<int>(), 24);

    skill::Workpiece raw(out.workpiece->shape());
    auto candsEmpty = skill::flex_attach::recognize(raw);
    EXPECT_TRUE(candsEmpty.empty())
        << "flex_attach is not geometrically recognizable";
}

// ─── 5. Tooling differs between ACF (hot bar) and ZIF (specific) ─────────
TEST(SkillFlexAttach, AcfToolingDiffersFromZif)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 1.6, "FR4");

    skill::flex_attach::Input zifIn;
    zifIn.attach_method     = "ZIF";
    zifIn.contact_count     = 30;
    zifIn.retention_force_n = 15.0;
    auto outZif = skill::flex_attach::apply(*stock, zifIn);

    skill::flex_attach::Input acfIn;
    acfIn.attach_method     = "ACF";
    acfIn.contact_count     = 30;
    acfIn.retention_force_n = 15.0;
    auto outAcf = skill::flex_attach::apply(*stock, acfIn);

    EXPECT_EQ(outZif.signature.tooling.tool_type,
              std::string("manual_or_robot_inserter"));
    EXPECT_EQ(outAcf.signature.tooling.tool_type,
              std::string("hot_bar_bonder"));

    // ACF hot-bar cycle (15 s) should be longer than ZIF (5 s).
    EXPECT_GT(outAcf.signature.tooling.est_cycle_time_s,
              outZif.signature.tooling.est_cycle_time_s);
}
