// @lat: [[process/test-strategy#skill round-trip]]
//
// generator_winding skill — 5-case sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/generator_winding.hpp"

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

skill::generator_winding::Input goodInput()
{
    skill::generator_winding::Input in;
    in.conductor_dia_mm = 1.2;
    in.slot_count       = 24;
    in.turns_per_coil   = 50;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillGeneratorWinding, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::generator_winding::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("generator_winding"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input: zero slot count, negative turns ──────
TEST(SkillGeneratorWinding, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.slot_count     = 0;
    in.turns_per_coil = -3;

    auto r = skill::generator_winding::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::generator_winding::apply(*stock, in), skill::SkillError);

    // conductor diameter out of range
    auto in2 = goodInput();
    in2.conductor_dia_mm = 7.0;     // > 5.0
    auto r2 = skill::generator_winding::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-WIND-DIA"));
}

// ─── 3. Signature records kind + dia + slots + turns ─────────────────────
TEST(SkillGeneratorWinding, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.conductor_dia_mm = 0.8;
    in.slot_count       = 36;
    in.turns_per_coil   = 75;

    auto out = skill::generator_winding::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("generator_winding"));
    EXPECT_NEAR(out.signature.pattern["conductor_dia_mm"].get<double>(),
                0.8, 1e-9);
    EXPECT_EQ(out.signature.pattern["slot_count"].get<int>(), 36);
    EXPECT_EQ(out.signature.pattern["turns_per_coil"].get<int>(), 75);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillGeneratorWinding, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::generator_winding::apply(*stock, goodInput());

    auto cands = skill::generator_winding::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["slot_count"].get<int>(), 24);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::generator_winding::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "generator_winding cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: cycle time scales with slots × turns ───────────────────
TEST(SkillGeneratorWinding, CycleTimeScalesWithSlotsAndTurns)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto smallIn = goodInput();
    smallIn.slot_count     = 12;
    smallIn.turns_per_coil = 20;
    auto smallOut = skill::generator_winding::apply(*stock, smallIn);

    auto bigIn = goodInput();
    bigIn.slot_count     = 48;
    bigIn.turns_per_coil = 80;
    auto bigOut = skill::generator_winding::apply(*stock, bigIn);

    EXPECT_GT(bigOut.signature.tooling.est_cycle_time_s,
              smallOut.signature.tooling.est_cycle_time_s)
        << "more slots × turns must take longer to wind";
    EXPECT_EQ(smallOut.signature.tooling.tool_type,
              std::string("coil_winder"));
}
