// @lat: [[process/test-strategy#skill round-trip]]
//
// temper_glass skill — metadata-only thermal-tempering recording.  Covers:
//   1. apply clones geometry unchanged
//   2. validate rejects bad input (quench_temp_c ≥ peak_temp_c)
//   3. signature records kind + key params
//   4. recognize via metadata replay
//   5. fully_tempered flag flips to false for σ_surf < 69 MPa (info only)

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/temper_glass.hpp"

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
TEST(SkillTemperGlass, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(300.0, 300.0, 6.0, "soda_lime_glass");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::temper_glass::Input in;
    in.peak_temp_c        = 650.0;
    in.quench_temp_c      = 20.0;
    in.surface_stress_mpa = 100.0;

    auto out = skill::temper_glass::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("temper_glass"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. Validate rejects bad input (quench_temp_c ≥ peak_temp_c) ──────────
TEST(SkillTemperGlass, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 6.0, "soda_lime_glass");

    skill::temper_glass::Input in;
    in.peak_temp_c        = 650.0;
    in.quench_temp_c      = 660.0;     // ≥ peak → DFM-TEMPER-QUENCH error
    in.surface_stress_mpa = 100.0;

    auto r = skill::temper_glass::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-TEMPER-QUENCH") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::temper_glass::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillTemperGlass, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 8.0, "soda_lime_glass");

    skill::temper_glass::Input in;
    in.peak_temp_c        = 680.0;
    in.quench_temp_c      = 25.0;
    in.surface_stress_mpa = 120.0;

    auto out = skill::temper_glass::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("temper_glass"));
    EXPECT_NEAR(out.signature.pattern["peak_temp_c"].get<double>(),         680.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["quench_temp_c"].get<double>(),        25.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["surface_stress_mpa"].get<double>(),  120.0, 1e-9);
    EXPECT_TRUE(out.signature.pattern["fully_tempered"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("tempering_furnace"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillTemperGlass, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(150.0, 150.0, 6.0, "soda_lime_glass");

    skill::temper_glass::Input in;
    in.peak_temp_c        = 640.0;
    in.quench_temp_c      = 22.0;
    in.surface_stress_mpa = 90.0;

    auto out = skill::temper_glass::apply(*stock, in);
    auto cands = skill::temper_glass::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["peak_temp_c"].get<double>(), 640.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::temper_glass::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "temper_glass leaves no B-Rep trace";
}

// ─── 5. Specific: σ_surf < 69 MPa → heat-strengthened (not fully tempered) ─
TEST(SkillTemperGlass, BelowSeventyMpaIsNotFullyTempered)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 4.0, "soda_lime_glass");

    skill::temper_glass::Input in;
    in.peak_temp_c        = 620.0;
    in.quench_temp_c      = 20.0;
    in.surface_stress_mpa = 50.0;     // < 69 → info finding, not error

    auto r = skill::temper_glass::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "stress below the 69 MPa fully-tempered threshold is info only";

    auto out = skill::temper_glass::apply(*stock, in);
    EXPECT_FALSE(out.signature.pattern["fully_tempered"].get<bool>());
}
