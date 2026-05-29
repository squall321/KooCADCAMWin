// @lat: [[process/test-strategy#skill round-trip]]
//
// asphalt_lay skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of invalid mix_class, metadata-only recognition, and the
// specific assertion that cycle-time scales with paved area + finishing.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/asphalt_lay.hpp"

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

skill::asphalt_lay::Input goodInput()
{
    skill::asphalt_lay::Input in;
    in.mix_class        = "wearing";
    in.lay_thickness_mm = 50.0;
    in.area_m2          = 1200.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillAsphaltLay, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::asphalt_lay::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("asphalt_lay"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ────────────────────────────────────────
TEST(SkillAsphaltLay, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.mix_class = "tack_coat";     // not in {binder, wearing}
    auto r = skill::asphalt_lay::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-ASPH-CLASS"));
    EXPECT_THROW(skill::asphalt_lay::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.lay_thickness_mm = 0.0;
    auto r2 = skill::asphalt_lay::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));

    auto in3 = goodInput();
    in3.area_m2 = -10.0;
    auto r3 = skill::asphalt_lay::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillAsphaltLay, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.mix_class        = "binder";
    in.lay_thickness_mm = 70.0;
    in.area_m2          = 2000.0;
    auto out = skill::asphalt_lay::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("asphalt_lay"));
    EXPECT_EQ(out.signature.pattern["mix_class"].get<std::string>(),
              std::string("binder"));
    EXPECT_NEAR(out.signature.pattern["lay_thickness_mm"].get<double>(),
                70.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["area_m2"].get<double>(),
                2000.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillAsphaltLay, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::asphalt_lay::apply(*stock, goodInput());

    auto cands = skill::asphalt_lay::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["mix_class"].get<std::string>(),
              std::string("wearing"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::asphalt_lay::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "asphalt_lay cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: cycle-time scales with paved area (+30 min finishing) ───
TEST(SkillAsphaltLay, CycleTimeScalesWithArea)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto inA = goodInput();
    inA.area_m2 = 600.0;       // 600/600 × 3600 + 1800 = 3600 + 1800 = 5400 s
    auto outA = skill::asphalt_lay::apply(*stock, inA);
    EXPECT_NEAR(outA.signature.tooling.est_cycle_time_s, 5400.0, 1.0);

    auto inB = goodInput();
    inB.area_m2 = 1200.0;      // 1200/600 × 3600 + 1800 = 7200 + 1800 = 9000 s
    auto outB = skill::asphalt_lay::apply(*stock, inB);
    EXPECT_NEAR(outB.signature.tooling.est_cycle_time_s, 9000.0, 1.0);

    EXPECT_EQ(outA.signature.tooling.tool_type, std::string("asphalt_paver"));
}
