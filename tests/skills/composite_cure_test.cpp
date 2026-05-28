// @lat: [[process/test-strategy#skill round-trip]]
//
// composite_cure skill — metadata-only cure cycle recording.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/composite_cure.hpp"

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

// ─── 1. Apply handles input + geometry unchanged ──────────────────────────
TEST(SkillCompositeCure, ApplyHandlesInput)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 1.5);
    const double volBefore = volumeOf(stock->shape());
    const int    edgeBefore = stock->edgeCount();

    skill::composite_cure::Input in;
    in.temp_c       = 120.0;
    in.time_min     = 120.0;
    in.pressure_kpa = 600.0;
    in.vacuum       = true;

    auto out = skill::composite_cure::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("composite_cure"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. Validate rejects bad input (under-cure time) ──────────────────────
TEST(SkillCompositeCure, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 1.0);

    skill::composite_cure::Input in;
    in.temp_c       = 120.0;
    in.time_min     = 1.0;          // < 5 → DFM-CURE-TIME error
    in.pressure_kpa = 600.0;
    in.vacuum       = true;

    auto r = skill::composite_cure::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CURE-TIME") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::composite_cure::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + key params + cure method ─────────────────
TEST(SkillCompositeCure, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 1.0);

    skill::composite_cure::Input in;
    in.temp_c       = 180.0;
    in.time_min     = 60.0;
    in.pressure_kpa = 700.0;        // > 100 → autoclave
    in.vacuum       = true;

    auto out = skill::composite_cure::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("composite_cure"));
    EXPECT_NEAR(out.signature.pattern["temp_c"].get<double>(), 180.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["time_min"].get<double>(), 60.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["pressure_kpa"].get<double>(), 700.0, 1e-9);
    EXPECT_TRUE(out.signature.pattern["vacuum"].get<bool>());
    EXPECT_EQ(out.signature.pattern["cure_method"].get<std::string>(), "autoclave");
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("autoclave"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillCompositeCure, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 1.0);

    skill::composite_cure::Input in;
    in.temp_c       = 120.0;
    in.time_min     = 90.0;
    in.pressure_kpa = 500.0;
    in.vacuum       = true;

    auto out = skill::composite_cure::apply(*stock, in);
    auto cands = skill::composite_cure::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["temp_c"].get<double>(), 120.0, 1e-9);
    EXPECT_TRUE(cands[0].recovered_params["vacuum"].get<bool>());

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::composite_cure::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific assertion: low pressure infers "oven" not "autoclave" ────
TEST(SkillCompositeCure, LowPressureInfersOvenCure)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 1.0);

    skill::composite_cure::Input in;
    in.temp_c       = 100.0;
    in.time_min     = 240.0;
    in.pressure_kpa = 50.0;         // ≤ 100 → oven
    in.vacuum       = true;

    auto out = skill::composite_cure::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["cure_method"].get<std::string>(), "oven");
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("oven"));
}
