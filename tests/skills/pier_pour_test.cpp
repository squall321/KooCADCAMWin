// @lat: [[process/test-strategy#skill round-trip]]
//
// pier_pour skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of empty pier_id, metadata-only recognition, and the
// specific assertion that the cycle-time scales with poured volume.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/pier_pour.hpp"

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

skill::pier_pour::Input goodInput()
{
    skill::pier_pour::Input in;
    in.pier_id   = "P3";
    in.volume_m3 = 25.0;
    in.mix_grade = "C40/50";
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillPierPour, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::pier_pour::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("pier_pour"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ────────────────────────────────────────
TEST(SkillPierPour, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.pier_id = "";
    auto r = skill::pier_pour::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::pier_pour::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.volume_m3 = 0.0;
    auto r2 = skill::pier_pour::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));

    auto in3 = goodInput();
    in3.mix_grade = "";
    auto r3 = skill::pier_pour::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillPierPour, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.pier_id   = "P7";
    in.volume_m3 = 42.5;
    in.mix_grade = "C50/60";
    auto out = skill::pier_pour::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("pier_pour"));
    EXPECT_EQ(out.signature.pattern["pier_id"].get<std::string>(),
              std::string("P7"));
    EXPECT_NEAR(out.signature.pattern["volume_m3"].get<double>(), 42.5, 1e-9);
    EXPECT_EQ(out.signature.pattern["mix_grade"].get<std::string>(),
              std::string("C50/60"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillPierPour, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::pier_pour::apply(*stock, goodInput());

    auto cands = skill::pier_pour::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["pier_id"].get<std::string>(),
              std::string("P3"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::pier_pour::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "pier_pour cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: cycle-time scales with placed volume + overhead ─────────
TEST(SkillPierPour, CycleTimeScalesWithVolume)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto inA = goodInput();
    inA.volume_m3 = 25.0;     // 25 m3 / 25 m3/h × 3600 + 1800 = 3600 + 1800 = 5400 s
    auto outA = skill::pier_pour::apply(*stock, inA);
    EXPECT_NEAR(outA.signature.tooling.est_cycle_time_s, 5400.0, 1.0);

    auto inB = goodInput();
    inB.volume_m3 = 50.0;     // 50/25 × 3600 + 1800 = 7200 + 1800 = 9000 s
    auto outB = skill::pier_pour::apply(*stock, inB);
    EXPECT_NEAR(outB.signature.tooling.est_cycle_time_s, 9000.0, 1.0);

    EXPECT_EQ(outA.signature.tooling.tool_type, std::string("concrete_pump"));
}
